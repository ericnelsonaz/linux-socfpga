/*
 * Hardware monitoring driver for EM21XX
 *
 * Copyright (c) 2018 Michael Fiorenza <mfiorenza@criticallink.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include "pmbus.h"

#define DRV_NAME "em21xx"

static int em21xx_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int retval;
	struct pmbus_driver_info *info;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -ENODEV;

	info = devm_kzalloc(&client->dev, sizeof(struct pmbus_driver_info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Try to access device over i2c */
	retval = i2c_smbus_read_byte_data(client, PMBUS_OPERATION);
	if (0 > retval)
		return retval;

	info->pages = 1;
	info->func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IOUT 
			| PMBUS_HAVE_POUT | PMBUS_HAVE_TEMP2;

	return pmbus_do_probe(client, id, info);
}

static int em21xx_remove(struct i2c_client *client)
{
	return pmbus_do_remove(client);
}

static const struct of_device_id em21xx_dt_match[] = {
	{ .compatible = "intel,em21xx" },
	{ },
};
MODULE_DEVICE_TABLE(of, em21xx_dt_match);

static const struct i2c_device_id em21xx_id[] = {
	{ "em21xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, em21xx_id);

static struct i2c_driver em21xx_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(em21xx_dt_match),
	},
	.probe	  = em21xx_probe,
	.remove	  = em21xx_remove,
	.id_table = em21xx_id,
};
module_i2c_driver(em21xx_driver);

MODULE_AUTHOR("Michael Fiorenza <mfiorenza@criticallink.com>");
MODULE_DESCRIPTION("Intel EM21XX PowerSoC driver");
MODULE_LICENSE("GPL");
