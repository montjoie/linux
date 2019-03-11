// SPDX-License-Identifier: GPL-2.0
/*
 * sun8i-ss-core.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T/R40 SoC
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

#include "sun8i-ss.h"

static const struct ce_variant ce_a80_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, SS_ALG_AES, SS_ALG_DES, SS_ALG_3DES,
	},
	.op_mode = { CE_ID_NOTSUPP, SS_OP_ECB, SS_OP_CBC, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 16, 16 },
	.maxflow = 2,
	.clk_names = { "ahb", "mod" }
};

static const struct ce_variant ce_a83t_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, SS_ALG_AES, SS_ALG_DES, SS_ALG_3DES,
	},
	.op_mode = { CE_ID_NOTSUPP, SS_OP_ECB, SS_OP_CBC, CE_ID_NOTSUPP,
		CE_ID_NOTSUPP,
	},
	.op_len_align = { 0, 16, 16 },
	.maxflow = 2,
	.clk_names = { "ahb", "mod" }
};

int get_engine_number(struct sun8i_ss_dev *ss)
{
	return atomic_inc_return(&ss->flow) % ss->variant->maxflow;
}

int sun8i_ss_run_task(struct sun8i_ss_dev *ss, int flow, const char *name)
{
	int err = 0;
	struct ce_task *cet = ss->chanlist[flow].tl;
	u32 v = 1;
	int i;

	if (ss->chanlist[flow].bounce_iv) {
		cet->t_iv = dma_map_single(ss->dev,
					   ss->chanlist[flow].bounce_iv,
					   ss->chanlist[flow].ivlen,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(ss->dev, cet->t_iv)) {
			dev_err(ss->dev, "Cannot DMA MAP IV\n");
			return -EFAULT;
		}
	}

	if (ss->chanlist[flow].next_iv) {
		cet->t_ctr = dma_map_single(ss->dev,
					    ss->chanlist[flow].next_iv,
					    ss->chanlist[flow].ivlen,
					    DMA_FROM_DEVICE);
		if (dma_mapping_error(ss->dev, cet->t_ctr)) {
			dev_err(ss->dev, "Cannot DMA MAP NEXT IV\n");
			err = -EFAULT;
			goto err_next_iv;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
	ss->chanlist[flow].stat_req++;
#endif

	mutex_lock(&ss->mlock);
	/* choose between stream0/stream1 */
	if (flow)
		v |= SS_FLOW1;
	else
		v |= SS_FLOW0;

	v |= ss->chanlist[flow].op_mode;
	v |= ss->chanlist[flow].method;

	/* dir bit is different on SS */
	if (ss->chanlist[flow].op_dir)
		v |= SS_DECRYPTION;

	switch (ss->chanlist[flow].keylen) {
	case 128 / 8:
		v |= SS_AES_128BITS << 7;
	break;
	case 192 / 8:
		v |= SS_AES_192BITS << 7;
	break;
	case 256 / 8:
		v |= SS_AES_256BITS << 7;
	break;
	}

	/* enable INT for this flow */
	writel(BIT(flow), ss->base + SS_INT_CTL_REG);

	if (cet->t_key)
		writel(cet->t_key, ss->base + SS_KEY_ADR_REG);

	if (cet->t_iv)
		writel(cet->t_iv, ss->base + SS_IV_ADR_REG);

	for (i = 0; i < MAX_SG; i++) {
		if (!cet->t_dst[i].addr)
			break;
		if (i > 0 && ss->chanlist[flow].bounce_iv) {
			writel(cet->t_key, ss->base + SS_KEY_ADR_REG);
			/*writel(BIT(flow), ss->base + SS_INT_CTL_REG);*/
			if (ss->chanlist[flow].op_dir == 0) {
				writel(cet->t_dst[i - 1].addr + cet->t_dst[i - 1].len * 4 - ss->chanlist[flow].ivlen, ss->base + SS_IV_ADR_REG);
			} else
				writel(cet->t_src[i - 1].addr + cet->t_src[i - 1].len * 4 - ss->chanlist[flow].ivlen, ss->base + SS_IV_ADR_REG);
			wmb();
		}
		dev_dbg(ss->dev,
			"Processing SG %d on flow %d %s ctl=%x %d to %d method=%x opmode=%x opdir=%x srclen=%d\n",
			i, flow, name, v,
			cet->t_src[i].len, cet->t_dst[i].len,
			ss->chanlist[flow].method,
			ss->chanlist[flow].op_mode,
			ss->chanlist[flow].op_dir,
			cet->t_src[i].len);

		writel(cet->t_src[i].addr, ss->base + SS_SRC_ADR_REG);
		writel(cet->t_dst[i].addr, ss->base + SS_DST_ADR_REG);
		writel(cet->t_src[i].len, ss->base + SS_LEN_ADR_REG);

		reinit_completion(&ss->chanlist[flow].complete);
		ss->chanlist[flow].status = 0;
		wmb();

		writel(v, ss->base + SS_CTL_REG);
		wait_for_completion_interruptible_timeout(&ss->chanlist[flow].complete,
							  msecs_to_jiffies(2000));
		if (ss->chanlist[flow].status == 0) {
			dev_err(ss->dev, "DMA timeout for %s\n", name);
			err = -EFAULT;
			goto theend;
		}

	}
theend:
	mutex_unlock(&ss->mlock);

	if (ss->chanlist[flow].next_iv) {
		dma_unmap_single(ss->dev, cet->t_ctr,
				 ss->chanlist[flow].ivlen,
				 DMA_FROM_DEVICE);
	}
err_next_iv:
	if (ss->chanlist[flow].bounce_iv) {
		dma_unmap_single(ss->dev, cet->t_iv,
				 ss->chanlist[flow].ivlen,
				 DMA_TO_DEVICE);
	}

	return err;
}

static irqreturn_t ce_irq_handler(int irq, void *data)
{
	struct sun8i_ss_dev *ss = (struct sun8i_ss_dev *)data;
	int flow = 0;
	u32 p;

	p = readl(ss->base + SS_INT_STA_REG);
	for (flow = 0; flow < ss->variant->maxflow; flow++) {
		if (p & (BIT(flow))) {
			writel(BIT(flow), ss->base + SS_INT_STA_REG);
			ss->chanlist[flow].status = 1;
			complete(&ss->chanlist[flow].complete);
		}
	}

	return IRQ_HANDLED;
}

static struct sun8i_ss_alg_template ce_algs[] = {
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CTR,
	.alg.skcipher = {
		.base = {
			.cra_name = "ctr(aes)",
			.cra_driver_name = "ctr-aes-sun8i-ss",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ss_aes_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CTS,
	.alg.skcipher = {
		.base = {
			.cra_name = "cts(cbc(aes))",
			.cra_driver_name = "cts(cbc-aes-sun8i-ss)",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ss_aes_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-sun8i-ss",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= sun8i_ss_aes_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_AES,
	.ce_blockmode = CE_ID_OP_ECB,
	.alg.skcipher = {
		.base = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-sun8i-ss",
			.cra_priority = 400,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= sun8i_ss_aes_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_DES3,
	.ce_blockmode = CE_ID_OP_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-sun8i-ss",
			.cra_priority = 400,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= DES3_EDE_KEY_SIZE,
		.max_keysize	= DES3_EDE_KEY_SIZE,
		.ivsize		= DES3_EDE_BLOCK_SIZE,
		.setkey		= sun8i_ss_des3_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
{
	.type = CRYPTO_ALG_TYPE_SKCIPHER,
	.ce_algo_id = CE_ID_CIPHER_DES3,
	.ce_blockmode = CE_ID_OP_ECB,
	.alg.skcipher = {
		.base = {
			.cra_name = "ecb(des3_ede)",
			.cra_driver_name = "ecb-des3-sun8i-ss",
			.cra_priority = 400,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_cipher_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 0xf,
			.cra_init = sun8i_ss_cipher_init,
			.cra_exit = sun8i_ss_cipher_exit,
		},
		.min_keysize	= DES3_EDE_KEY_SIZE,
		.max_keysize	= DES3_EDE_KEY_SIZE,
		.setkey		= sun8i_ss_des3_setkey,
		.encrypt	= sun8i_ss_skencrypt,
		.decrypt	= sun8i_ss_skdecrypt,
	}
},
};

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
static int sun8i_ss_dbgfs_read(struct seq_file *seq, void *v)
{
	struct sun8i_ss_dev *ss = seq->private;
	int i;

	for (i = 0; i < ss->variant->maxflow; i++)
		seq_printf(seq, "Channel %d: req %lu\n", i, ss->chanlist[i].stat_req);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].ss = ss;
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

static int sun8i_ss_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, sun8i_ss_dbgfs_read, inode->i_private);
}

static const struct file_operations sun8i_ss_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = sun8i_ss_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int sun8i_ss_probe(struct platform_device *pdev)
{
	struct resource *res;
	u32 v;
	int err, i, ce_method, id, irq;
	unsigned long cr;
	struct sun8i_ss_dev *ss;

	if (!pdev->dev.of_node)
		return -ENODEV;

	ss = devm_kzalloc(&pdev->dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->variant = of_device_get_match_data(&pdev->dev);
	if (!ss->variant) {
		dev_err(&pdev->dev, "Missing Crypto Engine variant\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ss->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ss->base)) {
		err = PTR_ERR(ss->base);
		dev_err(&pdev->dev, "Cannot request MMIO err=%d\n", err);
		return err;
	}

	for (i = 0; i < CE_MAX_CLOCKS; i++) {
		if (!ss->variant->clk_names[i])
			continue;
		dev_info(&pdev->dev, "Get %s clock\n", ss->variant->clk_names[i]);
		ss->ceclks[i] = devm_clk_get(&pdev->dev, ss->variant->clk_names[i]);
		if (IS_ERR(ss->ceclks[i])) {
			err = PTR_ERR(ss->ceclks[i]);
			dev_err(&pdev->dev, "Cannot get %s CE clock err=%d\n",
				ss->variant->clk_names[i], err);
		}
		cr = clk_get_rate(ss->ceclks[i]);
		if (ss->variant->clk_freqs[i]) {
			dev_info(&pdev->dev, "Set %s clock to %lu (%lu Mhz) from %lu (%lu Mhz)\n",
				 ss->variant->clk_names[i],
				 ss->variant->clk_freqs[i],
				 ss->variant->clk_freqs[i] / 1000000,
				 cr,
				 cr / 1000000);
			err = clk_set_rate(ss->ceclks[i], ss->variant->clk_freqs[i]);
			if (err)
				dev_err(&pdev->dev, "Fail to set %s clk speed to %lu\n",
					ss->variant->clk_names[i],
					ss->variant->clk_freqs[i]);
		} else {
			dev_info(&pdev->dev, "%s run at %lu\n",
				 ss->variant->clk_names[i], cr);
		}
		err = clk_prepare_enable(ss->ceclks[i]);
		if (err) {
			dev_err(&pdev->dev, "Cannot prepare_enable %s\n",
				ss->variant->clk_names[i]);
			return err;
		}
	}

	/* Get Non Secure IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(ss->dev, "Cannot get NS IRQ\n");
		return irq;
	}

	err = devm_request_irq(&pdev->dev, irq, ce_irq_handler, 0,
			       "sun8i-ss-ns", ss);
	if (err < 0) {
		dev_err(ss->dev, "Cannot request NS IRQ\n");
		return err;
	}

	ss->reset = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(ss->reset)) {
		if (PTR_ERR(ss->reset) == -EPROBE_DEFER)
			return PTR_ERR(ss->reset);
		dev_info(&pdev->dev, "No reset control found\n");
		ss->reset = NULL;
	}

	err = reset_control_deassert(ss->reset);
	if (err) {
		dev_err(&pdev->dev, "Cannot deassert reset control\n");
		goto error_clk;
	}


	v = readl(ss->base + SS_CTL_REG);
	v >>= 20;
	v &= 0x07;
	dev_info(&pdev->dev, "SS_NS Die ID %x\n", v);

	ss->dev = &pdev->dev;
	platform_set_drvdata(pdev, ss);

	mutex_init(&ss->mlock);

	ss->chanlist = devm_kcalloc(ss->dev, ss->variant->maxflow,
				    sizeof(struct sun8i_ss_flow), GFP_KERNEL);
	if (!ss->chanlist) {
		err = -ENOMEM;
		goto error_flow;
	}

	for (i = 0; i < ss->variant->maxflow; i++) {
		init_completion(&ss->chanlist[i].complete);
		mutex_init(&ss->chanlist[i].lock);

		ss->chanlist[i].engine = crypto_engine_alloc_init(ss->dev, true);
		if (!ss->chanlist[i].engine) {
			dev_err(ss->dev, "Cannot allocate engine\n");
			i--;
			goto error_engine;
		}
		err = crypto_engine_start(ss->chanlist[i].engine);
		if (err) {
			dev_err(ss->dev, "Cannot start engine\n");
			goto error_engine;
		}
		ss->chanlist[i].tl = dma_alloc_coherent(ss->dev,
							sizeof(struct ce_task),
							&ss->chanlist[i].t_phy,
							GFP_KERNEL);
		if (!ss->chanlist[i].tl) {
			dev_err(ss->dev, "Cannot get DMA memory for task %d\n",
				i);
			err = -ENOMEM;
			goto error_engine;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
	ss->dbgfs_dir = debugfs_create_dir("sun8i-ss", NULL);
	if (IS_ERR_OR_NULL(ss->dbgfs_dir)) {
		dev_err(ss->dev, "Fail to create debugfs dir");
		err = -ENOMEM;
		goto error_engine;
	}
	ss->dbgfs_stats = debugfs_create_file("stats", 0444,
					      ss->dbgfs_dir, ss,
					      &sun8i_ss_debugfs_fops);
	if (IS_ERR_OR_NULL(ss->dbgfs_stats)) {
		dev_err(ss->dev, "Fail to create debugfs stat");
		err = -ENOMEM;
		goto error_debugfs;
	}
#endif
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].ss = ss;
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			id = ce_algs[i].ce_algo_id;
			ce_method = ss->variant->alg_cipher[id];
			if (ce_method == CE_ID_NOTSUPP) {
				dev_info(ss->dev,
					 "DEBUG: Algo of %s not supported\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				break;
			}
			id = ce_algs[i].ce_blockmode;
			ce_method = ss->variant->op_mode[id];
			if (ce_method == CE_ID_NOTSUPP) {
				dev_info(ss->dev, "DEBUG: Blockmode of %s not supported\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				break;
			}
			dev_info(ss->dev, "DEBUG: Register %s\n",
				 ce_algs[i].alg.skcipher.base.cra_name);
			err = crypto_register_skcipher(&ce_algs[i].alg.skcipher);
			if (err) {
				dev_err(ss->dev, "Fail to register %s\n",
					ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				goto error_alg;
			}
			break;
		default:
			dev_err(ss->dev, "ERROR: tryed to register an unknown algo\n");
		}
	}
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		dev_info(ss->dev, "Validate %d %p\n", i, ce_algs[i].ss);
	}

	return 0;
error_alg:
	i--;
	for (; i >= 0; i--) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].ss)
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			break;
		}
	}
#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
error_debugfs:
	debugfs_remove_recursive(ss->dbgfs_dir);
#endif
	i = ss->variant->maxflow;
error_engine:
	while (i >= 0) {
		crypto_engine_exit(ss->chanlist[i].engine);
		if (ss->chanlist[i].tl)
			dma_free_coherent(ss->dev, sizeof(struct ce_task),
					  ss->chanlist[i].tl,
					  ss->chanlist[i].t_phy);
		i--;
	}
error_flow:
	reset_control_assert(ss->reset);
error_clk:
	for (i = 0; i < CE_MAX_CLOCKS; i++)
		clk_disable_unprepare(ss->ceclks[i]);
	return err;
}

static int sun8i_ss_remove(struct platform_device *pdev)
{
	int i, timeout;
	struct sun8i_ss_dev *ss = platform_get_drvdata(pdev);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].ss) {
				dev_info(ss->dev, "Unregister %d %s\n", i, ce_algs[i].alg.skcipher.base.cra_name);
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			}
			break;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
	debugfs_remove_recursive(ss->dbgfs_dir);
#endif

	for (i = 0; i < ss->variant->maxflow; i++) {
		crypto_engine_exit(ss->chanlist[i].engine);
		timeout = 0;
		while (mutex_is_locked(&ss->chanlist[i].lock) && timeout < 10) {
			dev_info(ss->dev, "Wait for %d %d\n", i, timeout);
			timeout++;
			msleep(20);
		}
		dma_free_coherent(ss->dev, sizeof(struct ce_task),
				  ss->chanlist[i].tl,
				  ss->chanlist[i].t_phy);
	}

	reset_control_assert(ss->reset);
	for (i = 0; i < CE_MAX_CLOCKS; i++)
		clk_disable_unprepare(ss->ceclks[i]);
	return 0;
}

static const struct of_device_id sun8i_ss_crypto_of_match_table[] = {
	{ .compatible = "allwinner,sun8i-a83t-crypto",
	  .data = &ce_a83t_variant },
	{ .compatible = "allwinner,sun9i-a80-crypto",
	  .data = &ce_a80_variant },
	{}
};
MODULE_DEVICE_TABLE(of, sun8i_ss_crypto_of_match_table);

static struct platform_driver sun8i_ss_driver = {
	.probe		 = sun8i_ss_probe,
	.remove		 = sun8i_ss_remove,
	.driver		 = {
		.name		   = "sun8i-ss",
		.of_match_table	= sun8i_ss_crypto_of_match_table,
	},
};

module_platform_driver(sun8i_ss_driver);

MODULE_DESCRIPTION("Allwinner Crypto Engine cryptographic accelerator");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin Labbe <clabbe.montjoie@gmail.com>");
