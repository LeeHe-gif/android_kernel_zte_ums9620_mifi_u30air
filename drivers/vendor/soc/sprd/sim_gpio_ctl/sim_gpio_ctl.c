#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define GPIO_DEV_NAME "sim_gpio_ctrl"
#define GPIO_MAJOR 0

struct gpio_ctrl_data {
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct gpio_desc *gpios[3];
    dev_t devt;
	int psim_gpio_irq;
	struct delayed_work wq_detcable;
};

static struct gpio_ctrl_data *gpio_data;

static int gpio_ctrl_open(struct inode *inode, struct file *file)
{
    file->private_data = gpio_data;
    return 0;
}

static int gpio_ctrl_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t gpio_ctrl_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct gpio_ctrl_data *data = file->private_data;
    unsigned char state[2];
    int ret;
	pr_info("gpio_ctrl_read enter\n");
    ret = gpiod_get_value_cansleep(data->gpios[0]);
    if (ret < 0) {
        pr_err("Failed to read GPIO 0\n");
        return ret;
    }
    state[0] = ret;

    ret = gpiod_get_value_cansleep(data->gpios[1]);
    if (ret < 0) {
        pr_err("Failed to read GPIO 1\n");
        return ret;
    }
    state[1] = ret;

    if (copy_to_user(buf, state, 2))
        return -EFAULT;
	pr_info("gpio_ctrl_read completed\n");
    return 2;
}

static ssize_t gpio_ctrl_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct gpio_ctrl_data *data = file->private_data;
    unsigned char state[2];
    //int ret;
	pr_info("gpio_ctrl_write enter\n");
    if (count != 2)
        return -EINVAL;

	pr_info("gpio_ctrl_write copy_from_user\n");
    if (copy_from_user(state, buf, 2))
        return -EFAULT;

	pr_info("gpio_ctrl_write set value 0\n");
    gpiod_set_value_cansleep(data->gpios[0], state[0]);
    //if (ret < 0) {
    //    pr_err("Failed to set GPIO 0\n");
    //    return ret;
    //}

	pr_info("gpio_ctrl_write set value 1\n");
    gpiod_set_value_cansleep(data->gpios[1], state[1]);
    //if (ret < 0) {
    //    pr_err("Failed to set GPIO 1\n");
    //    return ret;
    //}
	pr_info("gpio_ctrl_write completed\n");
    return 2;
}

static int gpio_sim_ctrl_get(char *buffer,
								 const struct kernel_param *kp)
{
    int ret;
	int state[2];
	pr_info("gpio_sim_ctrl_get enter\n");
    ret = gpiod_get_value_cansleep(gpio_data->gpios[0]);
    if (ret < 0) {
        pr_err("Failed to read GPIO 0\n");
        return ret;
    }
    state[0] = ret;

    ret = gpiod_get_value_cansleep(gpio_data->gpios[1]);
    if (ret < 0) {
        pr_err("Failed to read GPIO 1\n");
        return ret;
    }
    state[1] = ret;

	pr_info("gpio_sim_ctrl_get completed\n");
	return sprintf(buffer, "%d%d\n", state[0], state[1]);
}

static int gpio_sim_ctrl_set(const char *str,
								 const struct kernel_param *kp)
{
	int ret;
	u8 val;
	ret = kstrtou8(str, 0, &val);
	pr_info("gpio_sim_ctrl_set set value 1\n");
	gpiod_set_value_cansleep(gpio_data->gpios[1], val);
	pr_info("gpio_sim_ctrl_set completed\n");
	return 0;
}

static const struct kernel_param_ops gpio_sim_ctrl_op = {
	.set = gpio_sim_ctrl_set,
	.get = gpio_sim_ctrl_get,
};

module_param_cb(gpio_sim_ctrl, &gpio_sim_ctrl_op, NULL, S_IWUSR | S_IRUGO);

static int gpio_sim_ctrl_psim_get(char *buffer,
								 const struct kernel_param *kp)
{
    int ret;
	pr_info("gpio_sim_ctrl_psim_get enter\n");
    ret = gpiod_get_value_cansleep(gpio_data->gpios[2]);
    if (ret < 0) {
        pr_err("Failed to read GPIO 2\n");
        return ret;
    }

	pr_info("gpio_sim_ctrl_psim_get completed\n");
	return sprintf(buffer, "%d\n", ret);
}

static int gpio_sim_ctrl_psim_set(const char *str,
								 const struct kernel_param *kp)
{
	int ret;
	u8 val;
	ret = kstrtou8(str, 0, &val);
	pr_info("gpio_sim_ctrl_psim_set set value\n");
	gpiod_set_value_cansleep(gpio_data->gpios[2], val);
	pr_info("gpio_sim_ctrl_psim_set completed\n");
	return 0;
}

static const struct kernel_param_ops gpio_sim_ctrl_psim_op = {
	.set = gpio_sim_ctrl_psim_set,
	.get = gpio_sim_ctrl_psim_get,
};

module_param_cb(gpio_sim_ctrl_psim, &gpio_sim_ctrl_psim_op, NULL, S_IWUSR | S_IRUGO);

static const struct file_operations gpio_ctrl_fops = {
    .owner = THIS_MODULE,
    .open = gpio_ctrl_open,
    .release = gpio_ctrl_release,
    .read = gpio_ctrl_read,
    .write = gpio_ctrl_write,
};

static int esim_switch_status() {
	struct device_node *cmdline_node;
    const char *cmdline, *result;
	char sim_switch[32];
	int ret;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);
	if (ret) {
		pr_err("Can not parse bootargs\n");
		return 1;
	}
	result =  strstr(cmdline, "androidboot.simswitch=");
	if (!result) {
		pr_err("cmdline no simswitch value\n");
		return 1;
	} else {
        if (!sscanf(result, "androidboot.simswitch=%s", sim_switch)) {
            pr_err("sscanf get sim_switch fail\n");
		    return 1;
		} else {
			if (sim_switch[0] == '1') {
				return 1;
			} else if (sim_switch[0] == '0') {
				return 0;
			} else {
				return 1;
			}
		}
	}
}

static void psim_gpio_detect_cable(struct work_struct *work)
{
	struct gpio_ctrl_data *data = container_of(to_delayed_work(work),
						    struct gpio_ctrl_data,
						    wq_detcable);

	char status[16] = { 0, };
	char *card = "ZTE_DRIVER=zte_psim";
	char *envp[3] = { card, status, NULL };
	int card_status;
    pr_info("psim_gpio_detect_cable enter\n");
	card_status = gpiod_get_value_cansleep(data->gpios[0]);
    pr_info("psim_gpio_detect_cable card_status: %s\n", card_status == 1 ? "false" : "true");
    gpiod_set_value_cansleep(gpio_data->gpios[2], card_status == 1 ? 0 : 1);
    pr_info("psim_gpio_detect_cable card_status=%d", card_status);
	snprintf(status, 16, "PSIM_STATUS=%d", card_status);

	kobject_uevent_env(&data->device->kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t psim_gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_ctrl_data *data = dev_id;

	queue_delayed_work(system_power_efficient_wq, &data->wq_detcable,
			   msecs_to_jiffies(100));

	return IRQ_HANDLED;
}


static int gpio_ctrl_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct gpio_ctrl_data *data;

    int ret, i;

    data = kzalloc(sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->gpios[0] = devm_gpiod_get(dev, "esim1-psim-switch", GPIOD_IN);
    if (IS_ERR(data->gpios[0])) {
        ret = PTR_ERR(data->gpios[0]);
        pr_err("Failed to get GPIO 0: %d\n", ret);
        goto err_free;
    }
    pr_info("probe gpio0 value =%d", gpiod_get_value_cansleep(data->gpios[0]));
    pr_info("probe gpio0 value =%s", gpiod_get_value_cansleep(data->gpios[0]) == 1 ? "false" : "true");
    data->gpios[2] = devm_gpiod_get(dev, "soft-psim-switch", gpiod_get_value_cansleep(data->gpios[0]) == 1 ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
    if (IS_ERR(data->gpios[2])) {
        ret = PTR_ERR(data->gpios[2]);
        pr_err("Failed to get GPIO 2: %d\n", ret);
        goto err_free;
    }

    if (esim_switch_status()) {
        data->gpios[1] = devm_gpiod_get(dev, "esim2-esim3-switch", GPIOD_OUT_HIGH);
        pr_info("switch to esim3\n");
	} else {
        data->gpios[1] = devm_gpiod_get(dev, "esim2-esim3-switch", GPIOD_OUT_LOW);
        pr_info("switch to esim2\n");
	}
    if (IS_ERR(data->gpios[1])) {
        ret = PTR_ERR(data->gpios[1]);
        pr_err("Failed to get GPIO 1: %d\n", ret);
        goto err_free;
    }

    cdev_init(&data->cdev, &gpio_ctrl_fops);
    data->cdev.owner = THIS_MODULE;

    ret = alloc_chrdev_region(&data->devt, 0, 1, GPIO_DEV_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate chrdev region\n");
        goto err_free;
    }

    ret = cdev_add(&data->cdev, data->devt, 1);
    if (ret < 0) {
        pr_err("Failed to add cdev\n");
        goto err_unregister;
    }

    data->class = class_create(THIS_MODULE, GPIO_DEV_NAME);
    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        pr_err("Failed to create class\n");
        goto err_cdev_del;
    }

    data->device = device_create(data->class, NULL, data->devt, NULL, GPIO_DEV_NAME);
    if (IS_ERR(data->device)) {
        ret = PTR_ERR(data->device);
        pr_err("Failed to create device\n");
        goto err_class_destroy;
    }
    gpio_data = data;
	INIT_DELAYED_WORK(&data->wq_detcable, psim_gpio_detect_cable);
	if (data->gpios[0]) {
		data->psim_gpio_irq = gpiod_to_irq(data->gpios[0]);
		if (data->psim_gpio_irq < 0) {
			pr_err("failed to get psim gpio IRQ\n");
			return data->psim_gpio_irq;
		}

		ret = devm_request_threaded_irq(dev, data->psim_gpio_irq, NULL,
						psim_gpio_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						pdev->name, data);
		if (ret < 0) {
			pr_err("failed to request handler for psim gpio IRQ\n");
			return ret;
		}
	}
	psim_gpio_detect_cable(&data->wq_detcable.work);

    platform_set_drvdata(pdev, data);

    pr_info("SIM GPIO control driver loaded\n");
    return 0;

err_class_destroy:
    class_destroy(data->class);
err_cdev_del:
    cdev_del(&data->cdev);
err_unregister:
    unregister_chrdev_region(data->devt, 1);
err_free:
    for (i = 0; i < 3; i++) {
        if (!IS_ERR_OR_NULL(data->gpios[i]))
            devm_gpiod_put(dev, data->gpios[i]);
    }
    kfree(data);
    return ret;
}

static int gpio_ctrl_remove(struct platform_device *pdev)
{
	int i;
    struct gpio_ctrl_data *data = platform_get_drvdata(pdev);

    device_destroy(data->class, data->devt);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->devt, 1);
    for (i = 0; i < 3; i++)
        devm_gpiod_put(&pdev->dev, data->gpios[i]);
    kfree(data);
    pr_info("SIM GPIO control driver removed\n");
    return 0;
}

static const struct of_device_id gpio_ctrl_of_match[] = {
    { .compatible = "zte,sim_gpio_ctrl" },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, gpio_ctrl_of_match);

static struct platform_driver gpio_ctrl_driver = {
    .probe = gpio_ctrl_probe,
    .remove = gpio_ctrl_remove,
    .driver = {
        .name = GPIO_DEV_NAME,
        .of_match_table = gpio_ctrl_of_match,
    },
};

module_platform_driver(gpio_ctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zte huduan");
MODULE_DESCRIPTION("SIM GPIO Control Driver");