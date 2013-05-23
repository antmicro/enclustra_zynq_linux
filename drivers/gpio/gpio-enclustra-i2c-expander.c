#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define DRIVER_NAME		                "enclustra_i2"
#define DRIVER_DESC	                    "Enclustra I2C Expander driver"
#define DRIVER_VERSION                  "1.00"

static int available = 0;
module_param(available, int, 0);
/*****************************************************************************/

#define ENCLUSTRA_I2C_EXPANDER_OUT_MSK  0x80
#define ENCLUSTRA_I2C_EXPANDER_OUT_DATA 0x80



/* Register Set */
#define ENCLUSTRA_I2C_EXPANDER_DATA_REG 0x00
#define ENCLUSTRA_I2C_EXPANDER_DIR_REG  0x01

/*****************************************************************************/
static int sx150x_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int err = i2c_smbus_write_byte_data(client, reg, val);

	if (err < 0) {
		dev_warn(&client->dev, "i2c write fail: can't write %02x to %02x: %d\n", val, reg, err);
	    return err;
    } else {
        return 0;
    }
}

static int sx150x_i2c_read(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	int err = i2c_smbus_read_byte_data(client, reg);

	if (err < 0) {
		dev_warn(&client->dev, "i2c read fail: can't read from %02x: %d\n", reg, err);
	    return err;
    } else {
        *val = err;
        return 0;
    }
}

/*****************************************************************************/
static int __devinit gpio_enclustra_i2c_expander_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	unsigned char val = 0;
    int i;

    printk(KERN_ERR "==========================SX1505I087==========================\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
        goto err_check_funct;
    }
    

    /* set the direction for output */
    ret = sx150x_i2c_read(client, ENCLUSTRA_I2C_EXPANDER_DIR_REG, &val);
    if(ret != 0) {
		ret = -EINVAL;
        goto err_read_dir;
    }
    
    val = val & ~ENCLUSTRA_I2C_EXPANDER_OUT_MSK;

    ret = sx150x_i2c_write(client, ENCLUSTRA_I2C_EXPANDER_DIR_REG, val);
    if(ret != 0) {
		ret = -EINVAL;
        goto err_write_dir;
    }

    /* set the values for output */
    ret = sx150x_i2c_read(client, ENCLUSTRA_I2C_EXPANDER_DATA_REG, &val);
    if(ret != 0) {
		ret = -EINVAL;
        goto err_read_data;
    }
    
    val = val & ~ENCLUSTRA_I2C_EXPANDER_OUT_MSK;
    val = val | (ENCLUSTRA_I2C_EXPANDER_OUT_MSK & ENCLUSTRA_I2C_EXPANDER_OUT_DATA);

    ret = sx150x_i2c_write(client, ENCLUSTRA_I2C_EXPANDER_DATA_REG, val);
    if(ret != 0) {
		ret = -EINVAL;
        goto err_write_data;
    }

    for(i=0; i<2; i++) {
        ret = sx150x_i2c_read(client, i, &val);
        printk(KERN_ERR "REG:%d val: %02x\n", i, val);
        if(ret != 0) {
            printk(KERN_ERR "READ_FAILED:%d",i);    
        }
    }
    return 0;

err_write_data:
err_read_data:
err_write_dir:
err_read_dir:
err_check_funct:
    return ret;}

/*****************************************************************************/
static int __devexit gpio_enclustra_i2c_expander_remove(struct i2c_client *client)
{
    return 0;    
}

/*****************************************************************************/
static const struct i2c_device_id gpio_enclustra_i2c_expander_id[] = {
	{ "enclustra_i2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, gpio_enclustra_i2c_expander_id);

/*****************************************************************************/
static struct i2c_driver gpio_enclustra_i2c_expander_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = gpio_enclustra_i2c_expander_id,
	.probe    = gpio_enclustra_i2c_expander_probe,
	.remove   = __devexit_p(gpio_enclustra_i2c_expander_remove),
};

module_i2c_driver(gpio_enclustra_i2c_expander_driver);

/*****************************************************************************/
MODULE_AUTHOR("Enclustra GmbH, Sven Meier <sven.meier@enclustra.com>");
MODULE_DESCRIPTION("Enclustra I2C Expander driver");
MODULE_LICENSE("GPL");


