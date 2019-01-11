/*
 * Driver for Atmel MXT1664 Multiple Touch Controller
 *
 * Copyright (C) 2017 Embedded Artists AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>

#define MAX_SUPPORT_POINTS		5

#define EVENT_VALID_OFFSET	7
#define EVENT_VALID_MASK	(0x1 << EVENT_VALID_OFFSET)
#define EVENT_ID_OFFSET		2
#define EVENT_ID_MASK		(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE		(0x1 << 1)
#define EVENT_DOWN_UP		(0X1 << 0)

#define MAX_I2C_DATA_LEN	20

#define MXT1664_MAX_X	 	4095
#define MXT1664_MAX_Y	 	4095
#define MXT1664_MAX_TRIES 	 100

struct mxt1664_ts {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
};

static irqreturn_t mxt1664_ts_read_one_set(struct input_dev *input_dev,
	struct i2c_client *client, u8 *buf)
{
	int id, ret, x, y, z;
	int tries = 0;
	bool down, valid;
	u8 state;

	do {
		ret = i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);
	} while (ret == -EAGAIN && tries++ < MXT1664_MAX_TRIES);

	if (ret < 0)
		return IRQ_HANDLED;

	state = buf[1];
	x = (buf[6] << 8) | buf[5];
	y = (buf[10] << 8) | buf[9];
	z = (buf[14] << 8) | buf[13];

	valid = 1;//state & EVENT_VALID_MASK;
	id = buf[4];//(state & EVENT_ID_MASK) >> EVENT_ID_OFFSET;
	down = buf[3]==1;//state & EVENT_DOWN_UP;

	if (!valid || id > MAX_SUPPORT_POINTS) {
		dev_dbg(&client->dev, "point invalid\n");
		return IRQ_HANDLED;
	}

	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, down);

	//dev_err(&client->dev, "%s id:%d x:%d y:%d z:%d",
	//	down ? "down" : "up", id, x, y, z);
	/*dev_err(&client->dev, "%02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x\n",
		buf[0],
		buf[1],
		buf[2],
		buf[3],
		buf[4],
		buf[5],
		buf[6],
		buf[7],
		buf[8],
		buf[9],
		buf[10],
		buf[11],
		buf[12],
		buf[13],
		buf[14],
		buf[15],
		buf[16],
		buf[17],
		buf[18],
		buf[19]);*/

	if (down) {
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, z);
	}

	input_mt_report_pointer_emulation(input_dev, true);
	input_sync(input_dev);

	return IRQ_HANDLED;
}

static irqreturn_t mxt1664_ts_interrupt(int irq, void *dev_id)
{
	struct mxt1664_ts *ts = dev_id;
	struct input_dev *input_dev = ts->input_dev;
	struct i2c_client *client = ts->client;
	u8 buf[MAX_I2C_DATA_LEN];
	unsigned gpio = (2-1)*32 + 15;//irq_to_gpio(irq);

	while (gpio_get_value(gpio) == 0) {
		mxt1664_ts_read_one_set(input_dev, client, buf);
	}

	return IRQ_HANDLED;
}

/* wake up controller with a pulse on the reset gpio.  */
static int mxt1664_wake_up_device(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	int gpio;
	int ret;

	if (!np)
		return -ENODEV;

	gpio = of_get_named_gpio(np, "wakeup-gpios", 0);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	ret = gpio_request(gpio, "mxt1664_wakeup");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot wake up controller: %d\n",
			ret);
		return ret;
	}

	/* wake up controller via an 100ms low period on reset gpio. */
	gpio_direction_output(gpio, 0);
	msleep(100);
	gpio_set_value(gpio, 1);
	msleep(500);

	/* controller should be waken up, return gpio.  */
	//gpio_direction_input(gpio);
	//gpio_free(gpio);

	return 0;
}

static int mxt1664_ts_enter_app_mode(struct i2c_client *client)
{
	char info[7];
	int loop;
	int ret;
	int original_addr = client->addr;

	for (loop = 1; loop <= 3; loop++) {
		/* read the 7 byte information header from reg 0 */
		info[0] = info[1] = 0x00;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to set read address, ret=%d\n", ret);
		} else {
			ret = i2c_master_recv(client, info, 7);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to read information, ret=%d\n", ret);
			} else {
				/* could read the information */
				break;
			}
		}

		/* Check CRC_FAIL bit from bootloader addr 1*/
		client->addr = 0x27;
		ret = i2c_master_recv(client, info, 1);
		if (ret < 0) {
			/* no response from primary address, test secondary */
			dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
			client->addr = 0x28;
			ret = i2c_master_recv(client, info, 1);
			if (ret < 0) {
				/* Neither primary nor secondary answers */
				dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
				dev_err(&client->dev, "No response from bootloaders, aborting\n");
				client->addr = original_addr;
				return ret;
			} else {
				dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
			}
		} else {
			dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
		}

		dev_err(&client->dev, "Going into APP mode..\n");
		info[0] = info[1] = 0x01;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to send 0x0101 to 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
		}
		msleep(3000);
	}

	if (loop == 4) {
		dev_err(&client->dev, "Failed 3 attempts, aborting\n");
		return -ENODEV;
	}

	dev_err(&client->dev, "Info: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		info[0], info[1], info[2], info[3], info[4], info[5], info[6]);

	return 0;
}

static int mxt1664_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct mxt1664_ts *ts;
	struct input_dev *input_dev;
	int error;

	ts = devm_kzalloc(&client->dev, sizeof(struct mxt1664_ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->input_dev = input_dev;

	/* controller may be in sleep, wake it up. */
	error = mxt1664_wake_up_device(client);
	if (error) {
		dev_err(&client->dev, "Failed to wake up the controller\n");
		return error;
	}

	error = mxt1664_ts_enter_app_mode(client);
	if (error) {
		dev_err(&client->dev, "Failed to put controller in app mode\n");
		return error;
	}

	input_dev->name = "Atmel MXT1664 Touch Screen";
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_mt_init_slots(input_dev, MAX_SUPPORT_POINTS, 0);

	input_set_drvdata(input_dev, ts);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					  mxt1664_ts_interrupt,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "mxt1664_ts", ts);
	if (error < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error)
		return error;

	i2c_set_clientdata(client, ts);
	return 0;
}

static const struct i2c_device_id mxt1664_ts_id[] = {
	{ "mxt1664_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt1664_ts_id);

static int __maybe_unused mxt1664_ts_suspend(struct device *dev)
{
	/*static const u8 suspend_cmd[MAX_I2C_DATA_LEN] = {
		0x3, 0x6, 0xa, 0x3, 0x36, 0x3f, 0x2, 0, 0, 0
	};
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = i2c_master_send(client, suspend_cmd, MAX_I2C_DATA_LEN);
	return ret > 0 ? 0 : ret;
	*/
	return 0;
}

static int __maybe_unused mxt1664_ts_resume(struct device *dev)
{
	/*struct i2c_client *client = to_i2c_client(dev);

	return mxt1664_wake_up_device(client);*/
	return 0;
}

static SIMPLE_DEV_PM_OPS(mxt1664_ts_pm_ops, mxt1664_ts_suspend, mxt1664_ts_resume);

static const struct of_device_id mxt1664_ts_dt_ids[] = {
	{ .compatible = "atmel,mxt1664_ts" },
	{ /*  */ }
};

static struct i2c_driver mxt1664_ts_driver = {
	.driver = {
		.name	= "mxt1664_ts",
		.owner	= THIS_MODULE,
		.pm	= &mxt1664_ts_pm_ops,
		.of_match_table	= mxt1664_ts_dt_ids,
	},
	.id_table	= mxt1664_ts_id,
	.probe		= mxt1664_ts_probe,
};

module_i2c_driver(mxt1664_ts_driver);

MODULE_AUTHOR("Embedded Artists AB");
MODULE_DESCRIPTION("Touchscreen driver for Atmel MXT1664 touch controller");
MODULE_LICENSE("GPL");
