/*
 * Allwinner sun8i hardware spinlock driver
 *
 * Copyright (C) 2016 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 */

#include <linux/clk.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "hwspinlock_internal.h"

/* Spinlock register offsets */
#define SYSSTATUS_OFFSET		0x0000
#define LOCK_BASE_OFFSET		0x0100

/* Possible values of SPINLOCK_LOCK_REG */
#define SPINLOCK_NOTTAKEN		0	/* free */

struct sun8i_hwspinlock_device {
	struct hwspinlock_device *bank;
	struct reset_control *rst;
	struct clk *ahb_clk;
};

static int sun8i_hwspinlock_trylock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;

	/* attempt to acquire the lock by reading its value */
	return readl(lock_addr) == SPINLOCK_NOTTAKEN;
}

static void sun8i_hwspinlock_unlock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;

	/* release the lock by writing 0 to it */
	writel(SPINLOCK_NOTTAKEN, lock_addr);
}

static const struct hwspinlock_ops sun8i_hwspinlock_ops = {
	.trylock = sun8i_hwspinlock_trylock,
	.unlock = sun8i_hwspinlock_unlock,
};

static int sun8i_hwspinlock_probe(struct platform_device *pdev)
{
	struct hwspinlock *hwlock;
	struct resource *res;
	int err, num_locks;
	unsigned int i;
	struct sun8i_hwspinlock_device *priv;
	size_t array_size;
	void __iomem *base;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		return err;
	}

	priv->ahb_clk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(priv->ahb_clk)) {
		err = PTR_ERR(priv->ahb_clk);
		dev_err(&pdev->dev, "Cannot get AHB clock err=%d\n", err);
		return err;
	}

	priv->rst = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(priv->rst)) {
		err = PTR_ERR(priv->rst);
		if (err == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(&pdev->dev, "No optional reset control found %d\n",
			 err);
		priv->rst = NULL;
	}

	if (priv->rst) {
		err = reset_control_deassert(priv->rst);
		if (err) {
			dev_err(&pdev->dev, "Cannot deassert reset control\n");
			return err;
		}
	}

	err = clk_prepare_enable(priv->ahb_clk);
	if (err) {
		dev_err(&pdev->dev, "Cannot prepare AHB clock %d\n", err);
		goto rst_fail;
	}

	/* Determine number of locks */
	i = readl(base + SYSSTATUS_OFFSET);
	i >>= 28;

	switch (i) {
	case 0x1:
		num_locks = 32;
		break;
	case 0x2:
		num_locks = 64;
		break;
	case 0x3:
		num_locks = 128;
		break;
	case 0x4:
		num_locks = 256;
		break;
	default:
		dev_err(&pdev->dev, "Invalid number of spinlocks %d\n", i);
		err = -EINVAL;
		goto clk_fail;
	}

	array_size = sizeof(*priv->bank) + num_locks * sizeof(*hwlock);
	priv->bank = devm_kzalloc(&pdev->dev, array_size, GFP_KERNEL);
	if (!priv->bank) {
		err = -ENOMEM;
		goto clk_fail;
	}

	for (i = 0; i < num_locks; i++) {
		hwlock = &priv->bank->lock[i];
		hwlock->priv = base + LOCK_BASE_OFFSET + sizeof(u32) * i;
	}

	err = hwspin_lock_register(priv->bank, &pdev->dev,
				   &sun8i_hwspinlock_ops, 0, num_locks);
	if (err)
		goto clk_fail;

	dev_info(&pdev->dev, "Sun8i hwspinlock driver loaded with %d locks\n",
		 num_locks);
	return 0;

clk_fail:
	clk_disable_unprepare(priv->ahb_clk);
rst_fail:
	if (priv->rst)
		reset_control_assert(priv->rst);
	return err;
}

static int sun8i_hwspinlock_remove(struct platform_device *pdev)
{
	struct sun8i_hwspinlock_device *priv = platform_get_drvdata(pdev);
	int ret;

	ret = hwspin_lock_unregister(priv->bank);
	if (ret)
		return ret;

	if (priv->rst)
		reset_control_assert(priv->rst);

	clk_disable_unprepare(priv->ahb_clk);

	return 0;
}

static const struct of_device_id sun8i_hwspinlock_of_match[] = {
	{ .compatible = "allwinner,sun8i-hwspinlock", },
	{ /* end */ },
};
MODULE_DEVICE_TABLE(of, sun8i_hwspinlock_of_match);

static struct platform_driver sun8i_hwspinlock_driver = {
	.probe		= sun8i_hwspinlock_probe,
	.remove		= sun8i_hwspinlock_remove,
	.driver		= {
		.name	= "sun8i_hwspinlock",
		.of_match_table = of_match_ptr(sun8i_hwspinlock_of_match),
	},
};

static int __init sun8i_hwspinlock_init(void)
{
	return platform_driver_register(&sun8i_hwspinlock_driver);
}
/* board init code might need to reserve hwspinlocks for predefined purposes */
postcore_initcall(sun8i_hwspinlock_init);

static void __exit sun8i_hwspinlock_exit(void)
{
	platform_driver_unregister(&sun8i_hwspinlock_driver);
}
module_exit(sun8i_hwspinlock_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hardware spinlock driver for Allwinner sun8i");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
