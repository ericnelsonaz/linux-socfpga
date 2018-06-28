/*
 * rtc-ab-mk-t3 - Driver for Abracon AB-RTCMK-32.768Khz-T3
 *                  I2C RTC chip
 *
 * Copyright (C) 2018, Michael Fiorenza <mfiorenza@criticallink.com>
 *
 * Detailed datasheet of the chip is available here:
 *
 *  http://www.abracon.com/realtimeclock/AB-RTCMK-32.768kHz-Application-Manual.pdf
 *
 * This work is based on AB-RTCMC-B5ZE-S3 driver (drivers/rtc/rtc-ab-b5ze-s3.c).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#define DRV_NAME "rtc-ab-mk-t3"

/* RTC section */
#define ABMKT3_REG_RTC_SC	   0x00	   /* RTC Seconds register */
#define ABMKT3_REG_RTC_MN	   0x01	   /* RTC Minutes register */
#define ABMKT3_REG_RTC_HR	   0x02	   /* RTC Hours register */
#define ABMKT3_REG_RTC_DW	   0x03	   /* RTC Day of the week register */
#define ABMKT3_REG_RTC_DT	   0x04	   /* RTC Date register */
#define ABMKT3_REG_RTC_MO	   0x05	   /* RTC Month register */
#define ABMKT3_REG_RTC_YR	   0x06	   /* RTC Year register */

#define ABMKT3_RTC_SEC_LEN	   7

/* Alarm section (enable bits are all active low) */
#define ABMKT3_REG_ALRM_MN	   0x07	   /* Alarm - minute register */
#define ABMKT3_REG_ALRM_MN_AE	   BIT(7)  /* Minute enable */
#define ABMKT3_REG_ALRM_HR	   0x08	   /* Alarm - hours register */
#define ABMKT3_REG_ALRM_HR_AE	   BIT(7)  /* Hour enable */
#define ABMKT3_REG_ALRM_DT	   0x09	   /* Alarm - date register */
#define ABMKT3_REG_ALRM_DT_AE	   BIT(7)  /* Date (day of the month) enable */
#define ABMKT3_REG_ALRM_DW	   0x09	   /* Alarm - day of the week reg. */
#define ABMKT3_REG_ALRM_DW_AE	   BIT(7)  /* Day of the week enable */

#define ABMKT3_ALRM_SEC_LEN	   3

/* Timer Counter section */
#define ABMKT3_REG_TIM_CNT	   0x0A    /* Timer counter register */

#define ABMKT3_TIM_CNT_SEC_LEN	   1

/* Select section */
#define ABMKT3_REG_SEL		   0x0B    /* Select register */
#define ABMKT3_REG_SEL_UTS 	   BIT(0)  /* Update time select */
#define ABMKT3_REG_SEL_AS	   BIT(1)  /* Alarm select */
#define ABMKT3_REG_SEL_TSS0	   BIT(2)  /* Timer source clock bit 0 */
#define ABMKT3_REG_SEL_TSS1	   BIT(3)  /* Timer source clock bit 1 */
#define ABMKT3_REG_SEL_CFS0	   BIT(4)  /* Clkout Freq bit 0 */
#define ABMKT3_REG_SEL_CFS1	   BIT(5)  /* Clkout Freq bit 1 */
#define ABMKT3_REG_SEL_TCS0	   BIT(6)  /* Temperature comp bit 0 */
#define ABMKT3_REG_SEL_TCS1	   BIT(7)  /* Temperature comp bit 1 */

#define ABMKT3_SEL_SEC_LEN	   1

/* Flag section */
#define ABMKT3_REG_FLAG		   0x0C	   /* Flag register */
#define ABMKT3_REG_FLAG_UTF	   BIT(0)  /* Update time flag */
#define ABMKT3_REG_FLAG_AF	   BIT(1)  /* Alarm flag */
#define ABMKT3_REG_FLAG_TF	   BIT(2)  /* Timer flag */
#define ABMKT3_REG_FLAG_VDLF	   BIT(4)  /* Voltage detect low voltage flag */
#define ABMKT3_REG_FLAG_VDHF	   BIT(5)  /* Voltage detect low voltage flag */

#define ABMKT3_FLAG_SEC_LEN	   1

/* Control section */
#define ABMKT3_REG_CTRL	   0x0D	   /* Control register */
#define ABMKT3_REG_CTRL_UTIE	   BIT(0)  /* Update time interrupt enable */
#define ABMKT3_REG_CTRL_AIE	   BIT(1)  /* Alarm interrupt enable */
#define ABMKT3_REG_CTRL_TIE	   BIT(2)  /* Time interrupt enable */
#define ABMKT3_REG_CTRL_TE	   BIT(3)  /* Timer enable */
#define ABMKT3_REG_CTRL_FIE	   BIT(4)  /* Frequency interrupt enable */
#define ABMKT3_REG_CTRL_RAM	   BIT(5)  /* General purpose RAM bit */
#define ABMKT3_REG_CTRL_TEST	   BIT(6)  /* Manufacturer test bit */
#define ABMKT3_REG_CTRL_RESET	   BIT(7)  /* Reset bit */

#define ABMKT3_CTRL_SEC_LEN	   1

#define ABMKT3_MEM_MAP_LEN	   0x0E

struct abmkt3_rtc_data {
	struct rtc_device *rtc;
	struct regmap *regmap;
	struct mutex lock;
};

/*
 * Try and match register bits w/ fixed null values to see whether we
 * are dealing with an abmkt3. Note: this function is called early
 * during init and hence does need mutex protection.
 */
static int abmkt3_i2c_validate_chip(struct regmap *regmap)
{
	u8 regs[ABMKT3_MEM_MAP_LEN];
	static const u8 mask[ABMKT3_MEM_MAP_LEN] = { 0x80, 0x80, 0xC0, 0xF8,
						       0xC0, 0xE0, 0x00, 0x00,
						       0x00, 0x00, 0x00, 0x00,
						       0xC8, 0x00 };
	int ret, i;

	ret = regmap_bulk_read(regmap, 0, regs, ABMKT3_MEM_MAP_LEN);
	if (ret)
		return ret;

	for (i = 0; i < ABMKT3_MEM_MAP_LEN; ++i) {
		if (regs[i] & mask[i]) /* check if bits are cleared */
			return -ENODEV;
	}

	return 0;
}

/*
 * Note: we only read, so regmap inner lock protection is sufficient, i.e.
 * we do not need driver's main lock protection.
 */
static int _abmkt3_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct abmkt3_rtc_data *data = dev_get_drvdata(dev);
	u8 regs[ABMKT3_RTC_SEC_LEN];
	int ret;

	ret = regmap_bulk_read(data->regmap, ABMKT3_REG_RTC_SC, regs,
			       sizeof(regs));
	if (ret) {
		dev_err(dev, "%s: reading RTC time failed (%d)\n",
			__func__, ret);
		goto err;
	}

	tm->tm_sec = bcd2bin(regs[ABMKT3_REG_RTC_SC]);
	tm->tm_min = bcd2bin(regs[ABMKT3_REG_RTC_MN]);
	tm->tm_hour = bcd2bin(regs[ABMKT3_REG_RTC_HR]);
	tm->tm_wday = bcd2bin(regs[ABMKT3_REG_RTC_DW]);
	tm->tm_mday = bcd2bin(regs[ABMKT3_REG_RTC_DT]);
	tm->tm_mon  = bcd2bin(regs[ABMKT3_REG_RTC_MO]) - 1; /* starts at 1 */
	tm->tm_year = bcd2bin(regs[ABMKT3_REG_RTC_YR]) + 100;

	ret = rtc_valid_tm(tm);

err:
	return ret;
}

static int abmkt3_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct abmkt3_rtc_data *data = dev_get_drvdata(dev);
	u8 regs[ABMKT3_RTC_SEC_LEN];
	int ret;

	/*
	 * Year register is 8-bit wide and bcd-coded, i.e records values
	 * between 0 and 99. tm_year is an offset from 1900 and we are
	 * interested in the 2000-2099 range, so any value less than 100
	 * is invalid.
	 */
	if (tm->tm_year < 100)
		return -EINVAL;

	regs[ABMKT3_REG_RTC_SC] = bin2bcd(tm->tm_sec);
	regs[ABMKT3_REG_RTC_MN] = bin2bcd(tm->tm_min);
	regs[ABMKT3_REG_RTC_HR] = bin2bcd(tm->tm_hour);
	regs[ABMKT3_REG_RTC_DT] = bin2bcd(tm->tm_mday);
	regs[ABMKT3_REG_RTC_DW] = bin2bcd(tm->tm_wday);
	regs[ABMKT3_REG_RTC_MO] = bin2bcd(tm->tm_mon + 1);
	regs[ABMKT3_REG_RTC_YR] = bin2bcd(tm->tm_year - 100);

	mutex_lock(&data->lock);
	ret = regmap_bulk_write(data->regmap, ABMKT3_REG_RTC_SC,
				regs, ABMKT3_RTC_SEC_LEN);
	mutex_unlock(&data->lock);


	return ret;
}

/*
 * Check current RTC status and enable/disable what needs to be. Return 0 if
 * everything went ok and a negative value upon error. Note: this function
 * is called early during init and hence does need mutex protection.
 */
static int abmkt3_rtc_check_setup(struct device *dev)
{
	struct abmkt3_rtc_data *data = dev_get_drvdata(dev);
	struct regmap *regmap = data->regmap;
	int ret;
	u32 val;
	u8 mask;

	/*
	 * Disable interrupts
	 */
	mask = (ABMKT3_REG_CTRL_UTIE | ABMKT3_REG_CTRL_TIE | 
		ABMKT3_REG_CTRL_AIE | ABMKT3_REG_CTRL_TE | ABMKT3_REG_CTRL_FIE);
	ret = regmap_update_bits(regmap, ABMKT3_REG_CTRL, mask, 0);
	if (ret < 0) {
		dev_err(dev, "%s: unable to initialize ctrl register (%d)\n",
			__func__, ret);
		return ret;
	}

	/*
	 * Each component of the alarm (MN, HR, DT and DW) can be enabled/disabled
	 * individually by clearing/setting MSB of each associated register. So,
	 * we set all alarm enable bits to disable current alarm setting.
	 */
	mask = (ABMKT3_REG_ALRM_MN_AE); 
	ret = regmap_update_bits(regmap, ABMKT3_REG_ALRM_MN, mask, mask);
	if (ret < 0) {
		dev_err(dev, "%s: unable to disable alarm setting (%d)\n",
			__func__, ret);
		return ret;
	}
	
	mask = (ABMKT3_REG_ALRM_HR_AE);
	ret = regmap_update_bits(regmap, ABMKT3_REG_ALRM_HR, mask, mask);
	if (ret < 0) {
		dev_err(dev, "%s: unable to disable alarm setting (%d)\n",
			__func__, ret);
		return ret;
	}

	mask = (ABMKT3_REG_ALRM_DT_AE); 
	ret = regmap_update_bits(regmap, ABMKT3_REG_ALRM_DT, mask, mask);
	if (ret < 0) {
		dev_err(dev, "%s: unable to disable alarm setting (%d)\n",
			__func__, ret);
		return ret;
	}

	/*
	 * Clear flags register
	 */
	mask = (ABMKT3_REG_FLAG_UTF | ABMKT3_REG_FLAG_AF |
		ABMKT3_REG_FLAG_TF | ABMKT3_REG_FLAG_VDLF | 
		ABMKT3_REG_FLAG_VDHF);
	ret = regmap_update_bits(regmap, ABMKT3_REG_FLAG, mask, 0);
	if (ret < 0) {
		dev_err(dev, "%s: unable to initialize flags register (%d)\n",
			__func__, ret);
		return ret;
	}

	/*
	 * Set temperature compensation operating interval
	 */
	mask = (ABMKT3_REG_SEL_TCS0 | ABMKT3_REG_SEL_TCS1);
	ret = of_property_read_u32(dev->of_node, "temp-interval", &val);
	if (ret)
		val = 0; // Default value

	ret = regmap_update_bits(regmap, ABMKT3_REG_SEL, mask, val << 6);
	if (ret < 0) {
		dev_err(dev, "%s: unable to initialize sel register (%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static const struct rtc_class_ops rtc_ops = {
	.read_time = _abmkt3_rtc_read_time,
	.set_time = abmkt3_rtc_set_time,
};

static const struct regmap_config abmkt3_rtc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int abmkt3_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct abmkt3_rtc_data *data = NULL;
	struct device *dev = &client->dev;
	struct regmap *regmap;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK)) {
		ret = -ENODEV;
		goto err;
	}

	regmap = devm_regmap_init_i2c(client, &abmkt3_rtc_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "%s: regmap allocation failed: %d\n",
			__func__, ret);
		goto err;
	}

	ret = abmkt3_i2c_validate_chip(regmap);
	if (ret)
		goto err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&data->lock);
	data->regmap = regmap;
	dev_set_drvdata(dev, data);

	ret = abmkt3_rtc_check_setup(dev);
	if (ret)
		goto err;

	data->rtc = devm_rtc_device_register(dev, DRV_NAME, &rtc_ops,
					     THIS_MODULE);
	ret = PTR_ERR_OR_ZERO(data->rtc);
	if (ret) {
		dev_err(dev, "%s: unable to register RTC device (%d)\n",
			__func__, ret);
		goto err;
	}

err:
	return ret;
}

static int abmkt3_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id abmkt3_dt_match[] = {
	{ .compatible = "abracon,abmkt3" },
	{ },
};
MODULE_DEVICE_TABLE(of, abmkt3_dt_match);
#endif

static const struct i2c_device_id abmkt3_id[] = {
	{ "abmkt3", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, abmkt3_id);

static struct i2c_driver abmkt3_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(abmkt3_dt_match),
	},
	.probe	  = abmkt3_probe,
	.remove	  = abmkt3_remove,
	.id_table = abmkt3_id,
};
module_i2c_driver(abmkt3_driver);

MODULE_AUTHOR("Michael Fiorenza <mfiorenza@criticallink.com>");
MODULE_DESCRIPTION("Abracon AB-RTCMK-32.768kHz-T3 RTC driver");
MODULE_LICENSE("GPL");
