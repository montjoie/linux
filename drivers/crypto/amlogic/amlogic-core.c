// SPDX-License-Identifier: GPL-2.0
/*
 * meson-ce-core.c - hardware cryptographic accelerator for
 * Amlogic SoC
 *
 * Copyright (C) 2018 Corentin Labbe <clabbe@baylibre.com>
 *
 * Core file which registers crypto algorithms supported by the CryptoEngine.
 */
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <crypto/internal/skcipher.h>
#include <linux/dma-mapping.h>

#include "amlogic.h"

static const struct meson_variant gxl_variant = {
	.maxflow = 2,
};

static irqreturn_t meson_irq_handler(int irq, void *data)
{
	struct meson_dev *mc = (struct meson_dev *)data;
	int flow;
	u32 p;

	for (flow = 0; flow < mc->variant->maxflow; flow++) {
		if (mc->irqs[flow] == irq) {
			p = readl(mc->base + ((0x04 + flow) << 2));
			if (p) {
				writel_relaxed(0xF, mc->base + ((0x4 + flow) << 2));
				mc->chanlist[flow].status = 1;
				complete(&mc->chanlist[flow].complete);
				return IRQ_HANDLED;
			}
			dev_err(mc->dev, "%s %d Got irq for flow %d but ctrl is empty\n", __func__, irq, flow);
		}
	}

	dev_err(mc->dev, "%s %d from unknown irq\n", __func__, irq);
	return IRQ_HANDLED;
}

static struct meson_alg_template ce_algs[] = {
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.blockmode = MESON_OPMODE_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-meson",
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct meson_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
			.cra_init = meson_cipher_init,
			.cra_exit = meson_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= meson_aes_setkey,
		.encrypt	= meson_skencrypt,
		.decrypt	= meson_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.blockmode = MESON_OPMODE_ECB,
	.alg.skcipher = {
		.base = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-meson",
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct meson_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
			.cra_init = meson_cipher_init,
			.cra_exit = meson_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= meson_aes_setkey,
		.encrypt	= meson_skencrypt,
		.decrypt	= meson_skdecrypt,
	}
},
};

#ifdef CONFIG_CRYPTO_DEV_AMLOGIC_DEBUG
static int meson_dbgfs_read(struct seq_file *seq, void *v)
{
	struct meson_dev *mc = seq->private;
	int i;

	for (i = 0; i < mc->variant->maxflow; i++)
		seq_printf(seq, "Channel %d: req %lu\n", i, mc->chanlist[i].stat_req);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].mc = mc;
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			seq_printf(seq, "%s %s %lu %lu\n",
				   ce_algs[i].alg.skcipher.base.cra_driver_name,
				   ce_algs[i].alg.skcipher.base.cra_name,
				   ce_algs[i].stat_req, ce_algs[i].stat_fb);
			break;
		}
	}
	return 0;
}

static int meson_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, meson_dbgfs_read, inode->i_private);
}

static const struct file_operations meson_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = meson_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int meson_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct meson_dev *mc;
	int err, i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	mc = devm_kzalloc(&pdev->dev, sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return -ENOMEM;

	mc->dev = &pdev->dev;
	platform_set_drvdata(pdev, mc);

	dev_info(mc->dev, "Meson crypto driver v1.1\n");

	mc->variant = of_device_get_match_data(&pdev->dev);
	if (!mc->variant) {
		dev_err(&pdev->dev, "Missing Crypto Engine variant\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mc->base)) {
		err = PTR_ERR(mc->base);
		dev_err(&pdev->dev, "Cannot request MMIO err=%d\n", err);
		return err;
	}
	mc->busclk = devm_clk_get(&pdev->dev, "blkmv");
	if (IS_ERR(mc->busclk)) {
		err = PTR_ERR(mc->busclk);
		dev_err(&pdev->dev, "Cannot get clock err=%d\n", err);
		return err;
	}

	mc->irqs = devm_kcalloc(mc->dev, mc->variant->maxflow, sizeof(int), GFP_KERNEL);
	for (i = 0; i < mc->variant->maxflow; i++) {
		mc->irqs[i] = platform_get_irq(pdev, i);
		if (mc->irqs[i] < 0) {
			dev_err(mc->dev, "Cannot get IRQ for flow %d\n", i);
			return mc->irqs[i];
		}

		err = devm_request_irq(&pdev->dev, mc->irqs[i], meson_irq_handler, 0,
				       "meson-crypto", mc);
		if (err < 0) {
			dev_err(mc->dev, "Cannot request IRQ for flow %d\n", i);
			return err;
		}
	}

	mc->reset = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(mc->reset)) {
		if (PTR_ERR(mc->reset) == -EPROBE_DEFER)
			return PTR_ERR(mc->reset);
		dev_info(&pdev->dev, "No reset control found\n");
		mc->reset = NULL;
	}

	err = clk_prepare_enable(mc->busclk);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot prepare_enable busclk\n");
		return err;
	}

	err = reset_control_deassert(mc->reset);
	if (err) {
		dev_err(&pdev->dev, "Cannot deassert reset control\n");
		goto error_clk;
	}

	mc->chanlist = devm_kcalloc(mc->dev, mc->variant->maxflow,
				    sizeof(struct meson_flow), GFP_KERNEL);
	if (!mc->chanlist) {
		err = -ENOMEM;
		goto error_flow;
	}

	for (i = 0; i < mc->variant->maxflow; i++) {
		init_completion(&mc->chanlist[i].complete);

		mc->chanlist[i].engine = crypto_engine_alloc_init(mc->dev, 1);
		if (!mc->chanlist[i].engine) {
			dev_err(mc->dev, "Cannot allocate engine\n");
			i--;
			goto error_engine;
		}
		err = crypto_engine_start(mc->chanlist[i].engine);
		if (err) {
			dev_err(mc->dev, "Cannot request engine\n");
			goto error_engine;
		}
		mc->chanlist[i].tl = dma_alloc_coherent(mc->dev,
							sizeof(struct meson_desc) * 32,
							&mc->chanlist[i].t_phy,
							GFP_KERNEL);
		if (!mc->chanlist[i].tl) {
			dev_err(mc->dev, "Cannot get DMA memory for task %d\n",
				i);
			err = -ENOMEM;
			goto error_engine;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_AMLOGIC_DEBUG
	mc->dbgfs_dir = debugfs_create_dir("meson-crypto", NULL);
	if (IS_ERR_OR_NULL(mc->dbgfs_dir)) {
		dev_err(mc->dev, "Fail to create debugfs dir");
		err = -ENOMEM;
		goto error_engine;
	}
	mc->dbgfs_stats = debugfs_create_file("stats", 0444,
					      mc->dbgfs_dir, mc, &meson_debugfs_fops);
	if (IS_ERR_OR_NULL(mc->dbgfs_stats)) {
		dev_err(mc->dev, "Fail to create debugfs stat");
		err = -ENOMEM;
		goto error_debugfs;
	}
#endif
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].mc = mc;
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			err = crypto_register_skcipher(&ce_algs[i].alg.skcipher);
			if (err) {
				dev_err(mc->dev, "Fail to register %s\n",
					ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].mc = NULL;
				goto error_alg;
			}
			break;
		}
	}

	return 0;
error_alg:
	i--;
	for (; i >= 0; i--) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].mc)
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			break;
		}
	}
#ifdef CONFIG_CRYPTO_DEV_AMLOGIC_DEBUG
error_debugfs:
	debugfs_remove_recursive(mc->dbgfs_dir);
#endif
	i = mc->variant->maxflow;
error_engine:
	while (i >= 0) {
		if (mc->chanlist[i].tl)
			dma_free_coherent(mc->dev, sizeof(struct meson_desc),
					  mc->chanlist[i].tl, mc->chanlist[i].t_phy);
		i--;
	}
error_flow:
	reset_control_assert(mc->reset);
error_clk:
	clk_disable_unprepare(mc->busclk);
	return err;
}

static int meson_remove(struct platform_device *pdev)
{
	int i;
	struct meson_dev *mc = platform_get_drvdata(pdev);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].mc)
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			break;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_AMLOGIC_DEBUG
	debugfs_remove_recursive(mc->dbgfs_dir);
#endif

	reset_control_assert(mc->reset);
	clk_disable_unprepare(mc->busclk);
	return 0;
}

static const struct of_device_id meson_crypto_of_match_table[] = {
	{ .compatible = "amlogic,meson-gxl-crypto",
	  .data = &gxl_variant,
	},
	{}
};
MODULE_DEVICE_TABLE(of, meson_crypto_of_match_table);

static struct platform_driver meson_driver = {
	.probe		 = meson_probe,
	.remove		 = meson_remove,
	.driver		 = {
		.name		   = "meson-crypto",
		.of_match_table	= meson_crypto_of_match_table,
	},
};

module_platform_driver(meson_driver);

MODULE_DESCRIPTION("Amlogic Crypto Engine cryptographic accelerator");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin Labbe <clabbe@baylibre.com>");
