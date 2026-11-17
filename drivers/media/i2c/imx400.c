// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX400 (Motion Eye, Xperia XZ Premium rear camera) sensor driver.
 *
 * The sensor needs a large vendor register setup that is not public.
 * It is loaded at stream start from a firmware file
 * ("sony/imx400-<mode>.bin") that the user extracts from their own
 * device; this driver ships no vendor register data.
 *
 * Firmware format: "IMX4" magic, u32 LE count, then count entries of
 * { u16 LE address, u8 value, u8 reserved }.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX400_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX400_CHIP_ID			0x0400
#define IMX400_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX400_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define IMX400_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX400_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX400_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX400_REG_DIGITAL_GAIN		CCI_REG16(0x020e)

#define IMX400_XCLK_FREQ		8000000
/* OP PLL: 8 MHz * 169 = 1352 Mbit/s per lane -> 676 MHz DDR link */
#define IMX400_LINK_FREQ		676000000ULL
/* measured: 8920 x 5744 frames at 52 fps; IVTPXCK 333.33 MHz x 8 */
#define IMX400_PIXEL_RATE		2666666667ULL
#define IMX400_EXPOSURE_MARGIN		48
#define IMX400_ANA_GAIN_MAX		960
#define IMX400_FLL_MAX			0xfff0

/* pixel array (from the vendor module description) */
#define IMX400_NATIVE_W			5520
#define IMX400_NATIVE_H			4032
#define IMX400_ACTIVE_LEFT		8
#define IMX400_ACTIVE_TOP		176
#define IMX400_ACTIVE_W			5504
#define IMX400_ACTIVE_H			3808

struct imx400_mode {
	const char *fw;
	u32 width, height;
	u32 line_length;	/* 0x0342 */
	u32 frame_length;	/* 0x0340 default */
	struct v4l2_rect crop;	/* in native coordinates */
};

static const struct imx400_mode imx400_modes[] = {
	{
		/* 2x2 binned 4:3, full field of view */
		.fw = "sony/imx400-2528x1896.bin",
		.width = 2528, .height = 1896,
		.line_length = 5744, .frame_length = 8920,
		.crop = { .left = IMX400_ACTIVE_LEFT + 216 + 8,
			  .top = IMX400_ACTIVE_TOP + 8,
			  .width = 5056, .height = 3792 },
	},
};

static const char * const imx400_supplies[] = { "vdig", "vio", "vaf" };

struct imx400 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx400_supplies)];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *vblank, *hblank, *exposure;

	const struct imx400_mode *mode;
};

static inline struct imx400 *to_imx400(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx400, sd);
}

static int imx400_write_fw(struct imx400 *imx400, const char *name)
{
	struct device *dev = imx400->sd.dev;
	const struct firmware *fw;
	const u8 *p;
	u32 n, i, start;
	u8 buf[8];
	int ret, len;

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		dev_err(dev, "missing register firmware %s\n", name);
		return ret;
	}
	if (fw->size < 8 || memcmp(fw->data, "IMX4", 4)) {
		ret = -EINVAL;
		goto out;
	}
	n = get_unaligned_le32(fw->data + 4);
	if (fw->size < 8 + (size_t)n * 4) {
		ret = -EINVAL;
		goto out;
	}
	p = fw->data + 8;

	/* coalesce runs of consecutive addresses into short burst writes */
	for (i = 0; i < n; ) {
		start = get_unaligned_le16(p + i * 4);
		len = 0;
		while (i < n && len < (int)sizeof(buf) &&
		       get_unaligned_le16(p + i * 4) == start + len) {
			buf[len++] = p[i * 4 + 2];
			i++;
		}
		ret = regmap_bulk_write(imx400->regmap, start, buf, len);
		if (ret) {
			dev_err(dev, "write 0x%04x failed: %d\n", start, ret);
			goto out;
		}
	}
	dev_dbg(dev, "%s: %u registers\n", name, n);
out:
	release_firmware(fw);
	return ret;
}

static int imx400_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx400 *imx400 = container_of(ctrl->handler, struct imx400, ctrls);
	struct device *dev = imx400->sd.dev;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int max = imx400->mode->height + ctrl->val - IMX400_EXPOSURE_MARGIN;

		__v4l2_ctrl_modify_range(imx400->exposure, imx400->exposure->minimum,
					 max, 1, min(imx400->exposure->val, max));
	}

	if (!pm_runtime_get_if_in_use(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx400->regmap, IMX400_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx400->regmap, IMX400_REG_ANALOG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN: {
		/* Gr, R, B, Gb at 0x020e..0x0215, as the vendor stack writes them */
		u8 g[8];
		int i;

		for (i = 0; i < 4; i++) {
			g[2 * i] = ctrl->val >> 8;
			g[2 * i + 1] = ctrl->val & 0xff;
		}
		ret = regmap_bulk_write(imx400->regmap, 0x020e, g, sizeof(g));
		break;
	}
	case V4L2_CID_VBLANK:
		cci_write(imx400->regmap, IMX400_REG_FRAME_LENGTH,
			  imx400->mode->height + ctrl->val, &ret);
		break;
	default:
		break;
	}

	pm_runtime_put(dev);
	return ret;
}

static const struct v4l2_ctrl_ops imx400_ctrl_ops = {
	.s_ctrl = imx400_set_ctrl,
};

static int imx400_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx400 *imx400 = to_imx400(sd);
	struct device *dev = sd->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = imx400_write_fw(imx400, imx400->mode->fw);
	if (ret)
		goto err;

	ret = __v4l2_ctrl_handler_setup(&imx400->ctrls);
	if (ret)
		goto err;

	ret = cci_write(imx400->regmap, IMX400_REG_MODE_SELECT, 1, NULL);
	if (ret)
		goto err;
	return 0;
err:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int imx400_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx400 *imx400 = to_imx400(sd);
	int ret;

	ret = cci_write(imx400->regmap, IMX400_REG_MODE_SELECT, 0, NULL);
	pm_runtime_put_autosuspend(sd->dev);
	return ret;
}

static void imx400_fill_fmt(const struct imx400_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx400_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int imx400_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10 ||
	    fse->index >= ARRAY_SIZE(imx400_modes))
		return -EINVAL;
	fse->min_width = fse->max_width = imx400_modes[fse->index].width;
	fse->min_height = fse->max_height = imx400_modes[fse->index].height;
	return 0;
}

static int imx400_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx400 *imx400 = to_imx400(sd);
	const struct imx400_mode *mode;

	mode = v4l2_find_nearest_size(imx400_modes, ARRAY_SIZE(imx400_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	imx400_fill_fmt(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx400->mode = mode;
		__v4l2_ctrl_modify_range(imx400->vblank, IMX400_EXPOSURE_MARGIN,
					 IMX400_FLL_MAX - mode->height, 1,
					 mode->frame_length - mode->height);
		__v4l2_ctrl_s_ctrl(imx400->vblank, mode->frame_length - mode->height);
		__v4l2_ctrl_modify_range(imx400->hblank,
					 mode->line_length - mode->width,
					 mode->line_length - mode->width, 1,
					 mode->line_length - mode->width);
	}
	return 0;
}

static int imx400_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = (struct v4l2_rect){ 0, 0, IMX400_NATIVE_W, IMX400_NATIVE_H };
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){ IMX400_ACTIVE_LEFT, IMX400_ACTIVE_TOP,
					     IMX400_ACTIVE_W, IMX400_ACTIVE_H };
		return 0;
	}
	return -EINVAL;
}

static int imx400_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = { .width = imx400_modes[0].width,
			    .height = imx400_modes[0].height },
	};

	return imx400_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx400_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx400_pad_ops = {
	.enum_mbus_code = imx400_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx400_set_fmt,
	.get_selection = imx400_get_selection,
	.enum_frame_size = imx400_enum_frame_size,
	.enable_streams = imx400_enable_streams,
	.disable_streams = imx400_disable_streams,
};

static const struct v4l2_subdev_ops imx400_subdev_ops = {
	.video = &imx400_video_ops,
	.pad = &imx400_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx400_internal_ops = {
	.init_state = imx400_init_state,
};

static int imx400_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx400 *imx400 = to_imx400(sd);
	int ret;

	/* vendor order: vdig, vio, vaf, mclk, reset release, 1 ms apart */
	ret = regulator_bulk_enable(ARRAY_SIZE(imx400->supplies), imx400->supplies);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = clk_prepare_enable(imx400->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(imx400->supplies), imx400->supplies);
		return ret;
	}
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(imx400->reset_gpio, 0);
	usleep_range(10000, 12000);
	return 0;
}

static int imx400_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx400 *imx400 = to_imx400(sd);

	gpiod_set_value_cansleep(imx400->reset_gpio, 1);
	usleep_range(5000, 6000);
	clk_disable_unprepare(imx400->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx400->supplies), imx400->supplies);
	return 0;
}

static int imx400_init_controls(struct imx400 *imx400)
{
	struct v4l2_ctrl_handler *h = &imx400->ctrls;
	const struct imx400_mode *mode = imx400->mode;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	static const s64 link_freq[] = { IMX400_LINK_FREQ };
	int ret;

	v4l2_ctrl_handler_init(h, 10);

	ctrl = v4l2_ctrl_new_int_menu(h, &imx400_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  IMX400_PIXEL_RATE, IMX400_PIXEL_RATE, 1, IMX400_PIXEL_RATE);
	imx400->vblank = v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_VBLANK,
					   IMX400_EXPOSURE_MARGIN,
					   IMX400_FLL_MAX - mode->height, 1,
					   mode->frame_length - mode->height);
	imx400->hblank = v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_HBLANK,
					   mode->line_length - mode->width,
					   mode->line_length - mode->width, 1,
					   mode->line_length - mode->width);
	if (imx400->hblank)
		imx400->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx400->exposure = v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_EXPOSURE,
					     4, mode->frame_length - IMX400_EXPOSURE_MARGIN,
					     1, 2000);
	v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  0, IMX400_ANA_GAIN_MAX, 1, 0);
	v4l2_ctrl_new_std(h, &imx400_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  0x100, 0xfff, 1, 0x100);

	ret = v4l2_fwnode_device_parse(imx400->sd.dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(h, &imx400_ctrl_ops, &props);

	if (h->error) {
		ret = h->error;
		v4l2_ctrl_handler_free(h);
		return ret;
	}
	imx400->sd.ctrl_handler = h;
	return 0;
}

static int imx400_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx400 *imx400;
	u64 id;
	int ret, i;

	imx400 = devm_kzalloc(dev, sizeof(*imx400), GFP_KERNEL);
	if (!imx400)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx400->sd, client, &imx400_subdev_ops);
	imx400->sd.internal_ops = &imx400_internal_ops;
	imx400->mode = &imx400_modes[0];

	imx400->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx400->regmap))
		return PTR_ERR(imx400->regmap);

	imx400->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx400->xclk))
		return dev_err_probe(dev, PTR_ERR(imx400->xclk), "no xclk\n");
	if (clk_get_rate(imx400->xclk) != IMX400_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "xclk must be 8 MHz, is %lu\n",
				     clk_get_rate(imx400->xclk));

	for (i = 0; i < ARRAY_SIZE(imx400_supplies); i++)
		imx400->supplies[i].supply = imx400_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx400->supplies),
				      imx400->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "regulators\n");

	imx400->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx400->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx400->reset_gpio), "reset gpio\n");

	ret = imx400_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(imx400->regmap, IMX400_REG_CHIP_ID, &id, NULL);
	if (ret || id != IMX400_CHIP_ID) {
		dev_err(dev, "chip id 0x%llx (ret %d), expected 0x%x\n", id, ret,
			IMX400_CHIP_ID);
		ret = ret ?: -ENODEV;
		goto err_power;
	}

	/*
	 * Power down again right away: keeping the module powered while the
	 * front camera probes on the other CCI master made its reads time out.
	 */
	imx400_power_off(dev);

	ret = imx400_init_controls(imx400);
	if (ret)
		return ret;

	imx400->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx400->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx400->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx400->sd.entity, 1, &imx400->pad);
	if (ret)
		goto err_ctrls;

	imx400->sd.state_lock = imx400->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&imx400->sd);
	if (ret)
		goto err_entity;

	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx400->sd);
	if (ret)
		goto err_subdev;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	dev_info(dev, "IMX400 found\n");
	return 0;

err_subdev:
	v4l2_subdev_cleanup(&imx400->sd);
	pm_runtime_disable(dev);
err_entity:
	media_entity_cleanup(&imx400->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx400->ctrls);
	return ret;
err_power:
	imx400_power_off(dev);
	return ret;
}

static void imx400_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx400 *imx400 = to_imx400(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx400->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		imx400_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static const struct of_device_id imx400_of_match[] = {
	{ .compatible = "sony,imx400" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx400_of_match);

static const struct dev_pm_ops imx400_pm_ops = {
	SET_RUNTIME_PM_OPS(imx400_power_off, imx400_power_on, NULL)
};

static struct i2c_driver imx400_driver = {
	.driver = {
		.name = "imx400",
		.of_match_table = imx400_of_match,
		.pm = &imx400_pm_ops,
	},
	.probe = imx400_probe,
	.remove = imx400_remove,
};
module_i2c_driver(imx400_driver);

MODULE_DESCRIPTION("Sony IMX400 sensor driver (vendor setup loaded as firmware)");
MODULE_LICENSE("GPL");
