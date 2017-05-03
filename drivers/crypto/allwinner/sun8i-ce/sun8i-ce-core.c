// SPDX-License-Identifier: GPL-2.0
/*
 * sun8i-ce-core.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/R40 SoC
 *
 * Copyright (C) 2015-2019 Corentin Labbe <clabbe.montjoie@gmail.com>
 *
 * Core file which registers crypto algorithms supported by the CryptoEngine.
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <crypto/internal/skcipher.h>

#include "sun8i-ce.h"

static const struct ce_variant ce_h3_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES,
		CE_ID_NOTSUPP,
	},
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 4, 4, 4, 1, },
	.intreg = CE_ISR,
	.maxflow = 4,
	.clk_names = { "ahb", "mod" }
};

static const struct ce_variant ce_h5_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES,
		CE_ID_NOTSUPP,
	},
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 4, 4, 4, 4, },
	.intreg = CE_ISR,
	.maxflow = 4,
	.clk_freqs = { 200000000, 300000000, 0},
	.clk_names = { "ahb", "mod" }
};

static const struct ce_variant ce_h6_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES,
		CE_ALG_RAES,
	},
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP,
		CE_OP_XTS,
	},
	.op_len_align = { 0, 4, 4, 4, 4, 0, 0, 0, 4 },
	.model = CE_v2,
	.intreg = CE_ISR,
	.maxflow = 4,
	.clk_freqs = { 200000000, 300000000, 400000000},
	.clk_names = { "ahb", "mod", "mbus" }
};

static const struct ce_variant ce_a64_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES,
		CE_ID_NOTSUPP,
	},
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 4, 4, 4, 4, },
	.intreg = CE_ISR,
	.maxflow = 4,
	.clk_names = { "ahb", "mod" }
};

static const struct ce_variant ce_r40_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES,
		CE_ID_NOTSUPP,
	},
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 4, 4, 4, 1, },
	.intreg = CE_ISR,
	.maxflow = 4,
	.clk_freqs = { 200000000, 300000000, 0},
	.clk_names = { "ahb", "mod" }
};

int get_engine_number(struct sun8i_ce_dev *ce)
{
	return atomic_inc_return(&ce->flow) % ce->variant->maxflow;
}

int sun8i_ce_run_task(struct sun8i_ce_dev *ce, int flow, const char *name)
{
	u32 v;
	int err = 0;
	struct ce_task *cet = ce->chanlist[flow].tl;

	if (ce->chanlist[flow].bounce_iv) {
		cet->t_iv = dma_map_single(ce->dev,
					   ce->chanlist[flow].bounce_iv,
					   ce->chanlist[flow].ivlen,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(ce->dev, cet->t_iv)) {
			dev_err(ce->dev, "Cannot DMA MAP IV\n");
			return -EFAULT;
		}
	}

	if (ce->chanlist[flow].next_iv) {
		cet->t_ctr = dma_map_single(ce->dev,
					    ce->chanlist[flow].next_iv,
					    ce->chanlist[flow].ivlen,
					    DMA_FROM_DEVICE);
		if (dma_mapping_error(ce->dev, cet->t_ctr)) {
			dev_err(ce->dev, "Cannot DMA MAP NEXT IV\n");
			err = -EFAULT;
			goto err_next_iv;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	ce->chanlist[flow].stat_req++;
#endif

	mutex_lock(&ce->mlock);

	v = readl(ce->base + CE_ICR);
	v |= 1 << flow;
	writel(v, ce->base + CE_ICR);

	reinit_completion(&ce->chanlist[flow].complete);
	writel(ce->chanlist[flow].t_phy, ce->base + CE_TDQ);

	ce->chanlist[flow].status = 0;
	/* Be sure all data is written before enabling the task */
	wmb();

	v = 1 | (ce->chanlist[flow].tl->t_common_ctl & 0x7F) << 8;
	writel(v, ce->base + CE_TLR);
	mutex_unlock(&ce->mlock);

	wait_for_completion_interruptible_timeout(&ce->chanlist[flow].complete,
			msecs_to_jiffies(5000));

	if (ce->chanlist[flow].status == 0) {
		dev_err(ce->dev, "DMA timeout for %s\n", name);
		err = -EINVAL;
	}
	/* No need to lock for this read, the channel is locked so
	 * nothing could modify the error value for this channel
	 */
	v = readl(ce->base + CE_ESR);
	if (v) {
		v >>= (flow * 4);
		v &= 0xFF;
		if (v) {
			dev_err(ce->dev, "CE ERROR: %x for flow %x\n", v, flow);
			err = -EFAULT;
		}
		if (v & CE_ERR_ALGO_NOTSUP)
			dev_err(ce->dev, "CE ERROR: algorithm not supported\n");
		if (v & CE_ERR_DATALEN)
			dev_err(ce->dev, "CE ERROR: data length error\n");
		if (v & CE_ERR_KEYSRAM)
				dev_err(ce->dev, "CE ERROR: keysram access error for AES\n");
			if (v & CE_ERR_ADDR_INVALID)
				dev_err(ce->dev, "CE ERROR: address invalid\n");
		}

	if (ce->chanlist[flow].next_iv) {
		dma_unmap_single(ce->dev, cet->t_ctr,
				 ce->chanlist[flow].ivlen,
				 DMA_FROM_DEVICE);
	}
err_next_iv:
	if (ce->chanlist[flow].bounce_iv) {
		dma_unmap_single(ce->dev, cet->t_iv,
				 ce->chanlist[flow].ivlen,
				 DMA_TO_DEVICE);
	}

	return err;
}

static irqreturn_t ce_irq_handler(int irq, void *data)
{
	struct sun8i_ce_dev *ce = (struct sun8i_ce_dev *)data;
	int flow = 0;
	u32 p;

	p = readl(ce->base + ce->variant->intreg);
	for (flow = 0; flow < ce->variant->maxflow; flow++) {
		if (p & (BIT(flow))) {
			writel(BIT(flow), ce->base + ce->variant->intreg);
			ce->chanlist[flow].status = 1;
			complete(&ce->chanlist[flow].complete);
		}
	}

	return IRQ_HANDLED;
}

static struct sun8i_ce_alg_template ce_algs[] = {
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CTR,
	.alg.skcipher = {
		.base = {
			.cra_name = "ctr(aes)",
			.cra_driver_name = "ctr-aes-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ce_aes_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CTS,
	.alg.skcipher = {
		.base = {
			.cra_name = "cts(cbc(aes))",
			.cra_driver_name = "cts(cbc-aes-sun8i-ce)",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ce_aes_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ce_aes_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_ECB,
	.alg.skcipher = {
		.base = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= sun8i_ce_aes_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_RAES,
	.ce_blockmode = CE_ID_OP_XTS,
	.alg.skcipher = {
		.base = {
			.cra_name = "xts(aes)",
			.cra_driver_name = "xts-aes-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE * 2,
		.max_keysize	= AES_MAX_KEY_SIZE * 2,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ce_aes_xts_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_DES3,
	.ce_blockmode = CE_ID_OP_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= DES3_EDE_KEY_SIZE,
		.max_keysize	= DES3_EDE_KEY_SIZE,
		.ivsize		= DES3_EDE_BLOCK_SIZE,
		.setkey		= sun8i_ce_des3_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_DES3,
	.ce_blockmode = CE_ID_OP_ECB,
	.alg.skcipher = {
		.base = {
			.cra_name = "ecb(des3_ede)",
			.cra_driver_name = "ecb-des3-sun8i-ce",
			.cra_priority = 400,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ce_cipher_init,
			.cra_exit = sun8i_ce_cipher_exit,
		},
		.min_keysize	= DES3_EDE_KEY_SIZE,
		.max_keysize	= DES3_EDE_KEY_SIZE,
		.setkey		= sun8i_ce_des3_setkey,
		.encrypt	= sun8i_ce_skencrypt,
		.decrypt	= sun8i_ce_skdecrypt,
	}
},
};

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
static int sun8i_ce_dbgfs_read(struct seq_file *seq, void *v)
{
	struct sun8i_ce_dev *ce = seq->private;
	int i;

	for (i = 0; i < ce->variant->maxflow; i++)
		seq_printf(seq, "Channel %d: req %lu\n", i, ce->chanlist[i].stat_req);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].ce = ce;
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

static int sun8i_ce_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, sun8i_ce_dbgfs_read, inode->i_private);
}

static const struct file_operations sun8i_ce_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = sun8i_ce_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int sun8i_ce_probe(struct platform_device *pdev)
{
	struct resource *res;
	u32 v;
	int err, i, ce_method, id, irq;
	unsigned long cr;
	struct sun8i_ce_dev *ce;

	if (!pdev->dev.of_node)
		return -ENODEV;

	ce = devm_kzalloc(&pdev->dev, sizeof(*ce), GFP_KERNEL);
	if (!ce)
		return -ENOMEM;

	ce->variant = of_device_get_match_data(&pdev->dev);
	if (!ce->variant) {
		dev_err(&pdev->dev, "Missing Crypto Engine variant\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ce->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ce->base)) {
		err = PTR_ERR(ce->base);
		dev_err(&pdev->dev, "Cannot request MMIO err=%d\n", err);
		return err;
	}

	for (i = 0; i < CE_MAX_CLOCKS; i++) {
		if (!ce->variant->clk_names[i])
			continue;
		dev_info(&pdev->dev, "Get %s clock\n", ce->variant->clk_names[i]);
		ce->ceclks[i] = devm_clk_get(&pdev->dev, ce->variant->clk_names[i]);
		if (IS_ERR(ce->ceclks[i])) {
			err = PTR_ERR(ce->ceclks[i]);
			dev_err(&pdev->dev, "Cannot get %s CE clock err=%d\n",
				ce->variant->clk_names[i], err);
		}
		cr = clk_get_rate(ce->ceclks[i]);
		if (ce->variant->clk_freqs[i]) {
			dev_info(&pdev->dev, "Set %s clock to %lu (%lu Mhz) from %lu (%lu Mhz)\n",
				 ce->variant->clk_names[i],
				 ce->variant->clk_freqs[i],
				 ce->variant->clk_freqs[i] / 1000000,
				 cr,
				 cr / 1000000);
			err = clk_set_rate(ce->ceclks[i], ce->variant->clk_freqs[i]);
			if (err)
				dev_err(&pdev->dev, "Fail to set %s clk speed to %lu\n",
					ce->variant->clk_names[i],
					ce->variant->clk_freqs[i]);
		} else {
			dev_info(&pdev->dev, "%s run at %lu\n",
				 ce->variant->clk_names[i], cr);
		}
		err = clk_prepare_enable(ce->ceclks[i]);
		if (err) {
			dev_err(&pdev->dev, "Cannot prepare_enable %s\n",
				ce->variant->clk_names[i]);
			return err;
		}
	}

	/* Get Non Secure IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(ce->dev, "Cannot get NS IRQ\n");
		return irq;
	}

	err = devm_request_irq(&pdev->dev, irq, ce_irq_handler, 0,
			       "sun8i-ce-ns", ce);
	if (err < 0) {
		dev_err(ce->dev, "Cannot request NS IRQ\n");
		return err;
	}

	ce->reset = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(ce->reset)) {
		if (PTR_ERR(ce->reset) == -EPROBE_DEFER)
			return PTR_ERR(ce->reset);
		dev_info(&pdev->dev, "No reset control found\n");
		ce->reset = NULL;
	}

	err = reset_control_deassert(ce->reset);
	if (err) {
		dev_err(&pdev->dev, "Cannot deassert reset control\n");
		goto error_clk;
	}

	v = readl(ce->base + CE_CTR);
	v >>= 16;
	v &= 0x07;
	dev_info(&pdev->dev, "CE_NS Die ID %x\n", v);

	ce->dev = &pdev->dev;
	platform_set_drvdata(pdev, ce);

	mutex_init(&ce->mlock);

	ce->chanlist = devm_kcalloc(ce->dev, ce->variant->maxflow,
				    sizeof(struct sun8i_ce_flow), GFP_KERNEL);
	if (!ce->chanlist) {
		err = -ENOMEM;
		goto error_flow;
	}

	for (i = 0; i < ce->variant->maxflow; i++) {
		init_completion(&ce->chanlist[i].complete);
		mutex_init(&ce->chanlist[i].lock);

		ce->chanlist[i].engine = crypto_engine_alloc_init(ce->dev, true);
		if (!ce->chanlist[i].engine) {
			dev_err(ce->dev, "Cannot allocate engine\n");
			i--;
			goto error_engine;
		}
		err = crypto_engine_start(ce->chanlist[i].engine);
		if (err) {
			dev_err(ce->dev, "Cannot start engine\n");
			goto error_engine;
		}
		ce->chanlist[i].tl = dma_alloc_coherent(ce->dev,
							sizeof(struct ce_task),
							&ce->chanlist[i].t_phy,
							GFP_KERNEL);
		if (!ce->chanlist[i].tl) {
			dev_err(ce->dev, "Cannot get DMA memory for task %d\n",
				i);
			err = -ENOMEM;
			goto error_engine;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	ce->dbgfs_dir = debugfs_create_dir("sun8i-ce", NULL);
	if (IS_ERR_OR_NULL(ce->dbgfs_dir)) {
		dev_err(ce->dev, "Fail to create debugfs dir");
		err = -ENOMEM;
		goto error_engine;
	}
	ce->dbgfs_stats = debugfs_create_file("stats", 0444,
					      ce->dbgfs_dir, ce,
					      &sun8i_ce_debugfs_fops);
	if (IS_ERR_OR_NULL(ce->dbgfs_stats)) {
		dev_err(ce->dev, "Fail to create debugfs stat");
		err = -ENOMEM;
		goto error_debugfs;
	}
#endif
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].ce = ce;
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			id = ce_algs[i].ce_algo_id;
			ce_method = ce->variant->alg_cipher[id];
			if (ce_method == CE_ID_NOTSUPP) {
				dev_info(ce->dev,
					 "DEBUG: Algo of %s not supported\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ce = NULL;
				break;
			}
			id = ce_algs[i].ce_blockmode;
			ce_method = ce->variant->op_mode[id];
			if (ce_method == CE_ID_NOTSUPP) {
				dev_info(ce->dev, "DEBUG: Blockmode of %s not supported\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ce = NULL;
				break;
			}
			dev_info(ce->dev, "DEBUG: Register %s\n",
				 ce_algs[i].alg.skcipher.base.cra_name);
			err = crypto_register_skcipher(&ce_algs[i].alg.skcipher);
			if (err) {
				dev_err(ce->dev, "Fail to register %s\n",
					ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ce = NULL;
				goto error_alg;
			}
			break;
		default:
			dev_err(ce->dev, "ERROR: tryed to register an unknown algo\n");
		}
	}
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		dev_info(ce->dev, "Validate %d %p\n", i, ce_algs[i].ce);
	}

	return 0;
error_alg:
	i--;
	for (; i >= 0; i--) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].ce)
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			break;
		}
	}
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
error_debugfs:
	debugfs_remove_recursive(ce->dbgfs_dir);
#endif
	i = ce->variant->maxflow;
error_engine:
	while (i >= 0) {
		crypto_engine_exit(ce->chanlist[i].engine);
		if (ce->chanlist[i].tl)
			dma_free_coherent(ce->dev, sizeof(struct ce_task),
					  ce->chanlist[i].tl,
					  ce->chanlist[i].t_phy);
		i--;
	}
error_flow:
	reset_control_assert(ce->reset);
error_clk:
	for (i = 0; i < CE_MAX_CLOCKS; i++)
		clk_disable_unprepare(ce->ceclks[i]);
	return err;
}

static int sun8i_ce_remove(struct platform_device *pdev)
{
	int i, timeout;
	struct sun8i_ce_dev *ce = platform_get_drvdata(pdev);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].ce) {
				dev_info(ce->dev, "Unregister %d %s\n", i, ce_algs[i].alg.skcipher.base.cra_name);
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			}
			break;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	debugfs_remove_recursive(ce->dbgfs_dir);
#endif

	for (i = 0; i < ce->variant->maxflow; i++) {
		crypto_engine_exit(ce->chanlist[i].engine);
		timeout = 0;
		while (mutex_is_locked(&ce->chanlist[i].lock) && timeout < 10) {
			dev_info(ce->dev, "Wait for %d %d\n", i, timeout);
			timeout++;
			msleep(20);
		}
		dma_free_coherent(ce->dev, sizeof(struct ce_task),
				  ce->chanlist[i].tl,
				  ce->chanlist[i].t_phy);
	}

	reset_control_assert(ce->reset);
	for (i = 0; i < CE_MAX_CLOCKS; i++)
		clk_disable_unprepare(ce->ceclks[i]);
	return 0;
}

static const struct of_device_id sun8i_ce_crypto_of_match_table[] = {
	{ .compatible = "allwinner,sun8i-h3-crypto",
	  .data = &ce_h3_variant },
	{ .compatible = "allwinner,sun50i-h5-crypto",
	  .data = &ce_h5_variant },
	{ .compatible = "allwinner,sun50i-h6-crypto",
	  .data = &ce_h6_variant },
	{ .compatible = "allwinner,sun50i-a64-crypto",
	  .data = &ce_a64_variant },
	{ .compatible = "allwinner,sun8i-r40-crypto",
	  .data = &ce_r40_variant },
	{}
};
MODULE_DEVICE_TABLE(of, sun8i_ce_crypto_of_match_table);

static struct platform_driver sun8i_ce_driver = {
	.probe		 = sun8i_ce_probe,
	.remove		 = sun8i_ce_remove,
	.driver		 = {
		.name		   = "sun8i-ce",
		.of_match_table	= sun8i_ce_crypto_of_match_table,
	},
};

module_platform_driver(sun8i_ce_driver);

MODULE_DESCRIPTION("Allwinner Crypto Engine cryptographic accelerator");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin Labbe <clabbe.montjoie@gmail.com>");
