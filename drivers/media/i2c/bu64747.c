// SPDX-License-Identifier: GPL-2.0-only
/*
 * ROHM BU64747GWZ closed-loop (Hall) VCM autofocus controller.
 *
 * The controller runs a small program that is downloaded at every power-up,
 * followed by a coefficient table; both are module specific and come from
 * the phone vendor (not redistributable), loaded with request_firmware().
 * Protocol (from the Sony Xperia camera stack):
 *   0x82 ef 80 00             prepare download
 *   0x80 + program            in 192-byte blocks, command byte low 6 bits = 0
 *   0x84 f7 -> rd 2           status, byte 1 bit 2 = program checksum ok
 *   0x88 + coefficients       same block layout
 *   0x8c 01 00                start
 *   0x84 30 0d 00, 0x84 86 00 00
 *   0x8a 00 lo hi|mode v1 v2 v3 (u16 LE each)   move, 10-bit DAC target
 *   0x8a 00 -> rd 2           current Hall position (byte 0 bits 1:0 = MSBs)
 * The controller stretches SCL while busy and needs repeated-start reads.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define BU64747_FW_PROG		"sony/bu64747-xfw.bin"
#define BU64747_FW_COEF		"sony/bu64747-xcf.bin"
#define BU64747_BLOCK		192
/*
 * Usable Hall range of the SOI20BS2 module, measured on one unit: below ~250
 * the servo faults and the controller stops answering until powered off;
 * 1023 still tracks. Infinity is near 333, 10 cm near 690.
 */
#define BU64747_MIN_POS		300
#define BU64747_MAX_POS		1000
#define BU64747_MAX_STEP	60	/* bigger jumps overshoot into the fault zone */
#define BU64747_DEFAULT_POS	420	/* about the hyperfocal distance */
#define BU64747_PARK_POS	BU64747_MIN_POS

/* the Hall servo also needs the module clock and core supply */
static const char * const bu64747_supply_names[] = { "vdig", "vio", "vdd" };

struct bu64747 {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
	struct regulator_bulk_data supplies[ARRAY_SIZE(bu64747_supply_names)];
	struct clk *mclk;
	const struct firmware *prog, *coef;
	bool running;
	int cur;
};

static inline struct bu64747 *to_bu64747(struct v4l2_subdev *sd)
{
	return container_of(sd, struct bu64747, sd);
}

static int bu64747_send(struct bu64747 *af, const u8 *buf, int len)
{
	int ret = i2c_master_send(af->client, buf, len);

	return ret < 0 ? ret : ret == len ? 0 : -EIO;
}

static int bu64747_read(struct bu64747 *af, u8 cmd, u8 sub, u8 *val)
{
	u8 wr[2] = { cmd, sub };
	struct i2c_msg msgs[2] = {
		{ .addr = af->client->addr, .len = 2, .buf = wr },
		{ .addr = af->client->addr, .flags = I2C_M_RD, .len = 2, .buf = val },
	};
	int ret = i2c_transfer(af->client->adapter, msgs, 2);

	return ret < 0 ? ret : ret == 2 ? 0 : -EIO;
}

static int bu64747_download(struct bu64747 *af, u8 cmd, const struct firmware *fw)
{
	u8 buf[BU64747_BLOCK + 1];
	size_t off, n;
	int ret;

	for (off = 0; off < fw->size; off += n) {
		n = min_t(size_t, BU64747_BLOCK, fw->size - off);
		buf[0] = cmd + off;	/* as the vendor driver: only bits 5:0 matter */
		memcpy(buf + 1, fw->data + off, n);
		ret = bu64747_send(af, buf, n + 1);
		if (ret)
			return ret;
	}
	return 0;
}

static int bu64747_move_raw(struct bu64747 *af, u16 pos)
{
	u8 buf[10] = { 0x8a, 0x00, pos & 0xff, (pos >> 8) & 0x03 };

	return bu64747_send(af, buf, sizeof(buf));
}

static int bu64747_move(struct bu64747 *af, int pos)
{
	int ret, next;

	pos = clamp(pos, BU64747_MIN_POS, BU64747_MAX_POS);
	do {
		next = pos;
		if (next > af->cur + BU64747_MAX_STEP)
			next = af->cur + BU64747_MAX_STEP;
		else if (next < af->cur - BU64747_MAX_STEP)
			next = af->cur - BU64747_MAX_STEP;
		ret = bu64747_move_raw(af, next);
		if (ret)
			return ret;
		af->cur = next;
		if (next != pos)
			usleep_range(3000, 4000);
	} while (next != pos);
	return 0;
}

static int bu64747_start(struct bu64747 *af)
{
	static const u8 prep[] = { 0x82, 0xef, 0x80, 0x00 };
	static const u8 run[] = { 0x8c, 0x01, 0x00 };
	static const u8 cfg1[] = { 0x84, 0x30, 0x0d, 0x00 };
	static const u8 cfg2[] = { 0x84, 0x86, 0x00, 0x00 };
	struct device *dev = &af->client->dev;
	u8 st[2];
	int ret;

	if (!af->prog) {
		ret = request_firmware(&af->prog, BU64747_FW_PROG, dev);
		if (ret)
			return ret;
	}
	if (!af->coef) {
		ret = request_firmware(&af->coef, BU64747_FW_COEF, dev);
		if (ret)
			return ret;
	}

	ret = bu64747_send(af, prep, sizeof(prep));
	if (!ret)
		ret = bu64747_download(af, 0x80, af->prog);
	if (!ret)
		ret = bu64747_read(af, 0x84, 0xf7, st);
	if (ret)
		return ret;
	if (!(st[1] & BIT(2))) {
		dev_err(dev, "program download failed (status %02x %02x)\n",
			st[0], st[1]);
		return -EIO;
	}
	ret = bu64747_download(af, 0x88, af->coef);
	if (!ret)
		ret = bu64747_send(af, run, sizeof(run));
	if (!ret)
		ret = bu64747_send(af, cfg1, sizeof(cfg1));
	if (!ret)
		ret = bu64747_send(af, cfg2, sizeof(cfg2));
	if (ret)
		return ret;
	usleep_range(10000, 12000);
	af->running = true;
	/* ramp from wherever the servo holds the lens now (512 after start) */
	af->cur = 512;
	if (!bu64747_read(af, 0x8a, 0x00, st))
		af->cur = st[1] | (st[0] & 0x03) << 8;
	return bu64747_move(af, af->focus->val);
}

static int bu64747_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu64747 *af = to_bu64747(sd);

	/* land the lens gently instead of letting it drop */
	if (af->running) {
		bu64747_move(af, BU64747_PARK_POS);
		usleep_range(10000, 12000);
	}
	af->running = false;
	clk_disable_unprepare(af->mclk);
	return regulator_bulk_disable(ARRAY_SIZE(af->supplies), af->supplies);
}

static int bu64747_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct bu64747 *af = to_bu64747(sd);
	int ret, tries;

	for (tries = 0; tries < 2; tries++) {
		ret = regulator_bulk_enable(ARRAY_SIZE(af->supplies), af->supplies);
		if (ret)
			return ret;
		ret = clk_prepare_enable(af->mclk);
		if (ret) {
			regulator_bulk_disable(ARRAY_SIZE(af->supplies), af->supplies);
			return ret;
		}
		usleep_range(10000, 12000);

		ret = bu64747_start(af);
		if (!ret)
			return 0;
		dev_warn(dev, "start failed: %d\n", ret);
		clk_disable_unprepare(af->mclk);
		regulator_bulk_disable(ARRAY_SIZE(af->supplies), af->supplies);
		msleep(20);
	}
	/* not a sticky runtime PM error: the next open tries again */
	return -EAGAIN;
}

static int bu64747_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bu64747 *af = container_of(ctrl->handler, struct bu64747, ctrls);
	struct device *dev = &af->client->dev;
	int ret = 0;

	if (!pm_runtime_get_if_in_use(dev))
		return 0;
	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		ret = bu64747_move(af, ctrl->val);
	pm_runtime_put(dev);
	return ret;
}

static const struct v4l2_ctrl_ops bu64747_ctrl_ops = {
	.s_ctrl = bu64747_set_ctrl,
};

static int bu64747_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int bu64747_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops bu64747_internal_ops = {
	.open = bu64747_open,
	.close = bu64747_close,
};

static const struct v4l2_subdev_ops bu64747_ops = { };

static int bu64747_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct bu64747 *af;
	unsigned int i;
	int ret;

	af = devm_kzalloc(dev, sizeof(*af), GFP_KERNEL);
	if (!af)
		return -ENOMEM;
	af->client = client;

	for (i = 0; i < ARRAY_SIZE(af->supplies); i++)
		af->supplies[i].supply = bu64747_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(af->supplies), af->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "getting supplies\n");

	af->mclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(af->mclk))
		return dev_err_probe(dev, PTR_ERR(af->mclk), "getting mclk\n");

	v4l2_i2c_subdev_init(&af->sd, client, &bu64747_ops);
	af->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	af->sd.internal_ops = &bu64747_internal_ops;

	v4l2_ctrl_handler_init(&af->ctrls, 1);
	af->focus = v4l2_ctrl_new_std(&af->ctrls, &bu64747_ctrl_ops,
				      V4L2_CID_FOCUS_ABSOLUTE, BU64747_MIN_POS,
				      BU64747_MAX_POS, 1, BU64747_DEFAULT_POS);
	if (af->ctrls.error) {
		ret = af->ctrls.error;
		goto err_ctrls;
	}
	af->sd.ctrl_handler = &af->ctrls;

	ret = media_entity_pads_init(&af->sd.entity, 0, NULL);
	if (ret)
		goto err_ctrls;
	af->sd.entity.function = MEDIA_ENT_F_LENS;

	/* firmware lives in the root filesystem: power up on first open only */
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev(&af->sd);
	if (ret)
		goto err_pm;
	return 0;

err_pm:
	pm_runtime_disable(dev);
	media_entity_cleanup(&af->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&af->ctrls);
	return ret;
}

static void bu64747_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bu64747 *af = to_bu64747(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&af->ctrls);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		bu64747_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	release_firmware(af->prog);
	release_firmware(af->coef);
}

static DEFINE_RUNTIME_DEV_PM_OPS(bu64747_pm_ops, bu64747_power_off,
				 bu64747_power_on, NULL);

static const struct of_device_id bu64747_of_match[] = {
	{ .compatible = "rohm,bu64747" },
	{ }
};
MODULE_DEVICE_TABLE(of, bu64747_of_match);

static struct i2c_driver bu64747_i2c_driver = {
	.driver = {
		.name = "bu64747",
		.pm = pm_ptr(&bu64747_pm_ops),
		.of_match_table = bu64747_of_match,
	},
	.probe = bu64747_probe,
	.remove = bu64747_remove,
};
module_i2c_driver(bu64747_i2c_driver);

MODULE_DESCRIPTION("ROHM BU64747GWZ closed-loop VCM autofocus driver");
MODULE_LICENSE("GPL");
