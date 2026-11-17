// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bring-up helper for the Sony Xperia XZ Premium cameras: power a camera module
 * up the way Sony's downstream driver does (supplies, 24 MHz MCLK, reset), read
 * the SMIA/CCS model ID register 0x0016 over CCI and power it down again.
 * It proves the CCI bus, clocks, supplies and reset lines before a real sensor
 * driver exists. Re-read through /sys/bus/i2c/devices/<dev>/model_id.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

static const char * const camprobe_supplies[] = { "vdig", "vio" };

struct camprobe {
	struct i2c_client *client;
	struct regulator_bulk_data supplies[ARRAY_SIZE(camprobe_supplies)];
	struct clk *mclk;
	struct gpio_desc *reset;
};

static int camprobe_read16(struct i2c_client *c, u16 reg, u16 *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	u8 buf[2];
	struct i2c_msg msgs[2] = {
		{ .addr = c->addr, .flags = 0, .len = 2, .buf = addr },
		{ .addr = c->addr, .flags = I2C_M_RD, .len = 2, .buf = buf },
	};
	int ret = i2c_transfer(c->adapter, msgs, 2);

	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = (buf[0] << 8) | buf[1];
	return 0;
}

static int camprobe_power(struct camprobe *cp, bool on)
{
	int ret;

	if (!on) {
		gpiod_set_value_cansleep(cp->reset, 1);
		clk_disable_unprepare(cp->mclk);
		regulator_bulk_disable(ARRAY_SIZE(cp->supplies), cp->supplies);
		return 0;
	}

	dev_info(&cp->client->dev, "power up: supplies\n");
	ret = regulator_bulk_enable(ARRAY_SIZE(cp->supplies), cp->supplies);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	dev_info(&cp->client->dev, "power up: mclk %lu Hz\n", clk_get_rate(cp->mclk));
	ret = clk_prepare_enable(cp->mclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(cp->supplies), cp->supplies);
		return ret;
	}
	usleep_range(1000, 2000);
	dev_info(&cp->client->dev, "power up: reset release\n");
	gpiod_set_value_cansleep(cp->reset, 0);
	msleep(10);
	return 0;
}

static int camprobe_read_id(struct camprobe *cp, u16 *id)
{
	int ret = camprobe_power(cp, true);

	if (ret)
		return ret;
	dev_info(&cp->client->dev, "reading 0x0016\n");
	ret = camprobe_read16(cp->client, 0x0016, id);
	dev_info(&cp->client->dev, "read: %d id 0x%04x\n", ret, ret ? 0 : *id);
	camprobe_power(cp, false);
	return ret;
}

static ssize_t model_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct camprobe *cp = dev_get_drvdata(dev);
	u16 id;
	int ret = camprobe_read_id(cp, &id);

	return ret ? ret : sysfs_emit(buf, "0x%04x\n", id);
}
static DEVICE_ATTR_RO(model_id);

static int camprobe_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct camprobe *cp;
	int i, ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->client = client;
	i2c_set_clientdata(client, cp);

	for (i = 0; i < ARRAY_SIZE(camprobe_supplies); i++)
		cp->supplies[i].supply = camprobe_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(cp->supplies), cp->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	cp->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(cp->mclk))
		return dev_err_probe(dev, PTR_ERR(cp->mclk), "mclk\n");
	ret = clk_set_rate(cp->mclk, 24000000);
	if (ret)
		dev_warn(dev, "mclk rate: %d\n", ret);

	cp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cp->reset))
		return dev_err_probe(dev, PTR_ERR(cp->reset), "reset gpio\n");

	dev_info(dev, "ready; read model_id to power the module up\n");

	return device_create_file(dev, &dev_attr_model_id);
}

static void camprobe_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_model_id);
}

static const struct of_device_id camprobe_of_match[] = {
	{ .compatible = "sony,maple-camprobe" },
	{ }
};
MODULE_DEVICE_TABLE(of, camprobe_of_match);

static struct i2c_driver camprobe_driver = {
	.driver = {
		.name = "maple-camprobe",
		.of_match_table = camprobe_of_match,
	},
	.probe = camprobe_probe,
	.remove = camprobe_remove,
};
module_i2c_driver(camprobe_driver);

MODULE_DESCRIPTION("Xperia XZ Premium camera module probe (bring-up helper)");
MODULE_LICENSE("GPL");
