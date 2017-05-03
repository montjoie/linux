/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sun8i-ce-core.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
 *
 * Copyright (C) 2015-2018 Corentin Labbe <clabbe.montjoie@gmail.com>
 *
 * Core file which registers crypto algorithms supported by the CryptoEngine.
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/skcipher.h>
#include <linux/dma-mapping.h>

#include "sun8i-ce.h"

static const struct ce_variant ce_h3_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES, },
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP
	},
	.intreg = CE_ISR,
	.maxflow = 4,
	.prng = CE_ALG_PRNG,
	.maxrsakeysize = 2048,
	.rsa_op_mode = { CE_OP_RSA_512, CE_OP_RSA_1024, CE_OP_RSA_2048,
			CE_OP_RSA_3072, CE_OP_RSA_4096, },
	.alg_akcipher = { CE_ID_NOTSUPP, CE_ALG_RSA, },
};

static const struct ce_variant ce_h5_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES, },
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP
	},
	.intreg = CE_ISR,
	.maxflow = 4,
	.prng = CE_ALG_PRNG,
	.maxrsakeysize = 4096,
	.rsa_op_mode = { CE_OP_RSA_512, CE_OP_RSA_1024, CE_OP_RSA_2048,
			CE_OP_RSA_3072, CE_OP_RSA_4096, },
	.alg_akcipher = { CE_ID_NOTSUPP, CE_ALG_RSA, },
};

static const struct ce_variant ce_a64_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, CE_ALG_AES, CE_ALG_DES, CE_ALG_3DES, },
	.op_mode = { CE_ID_NOTSUPP, CE_OP_ECB, CE_OP_CBC, CE_OP_CTR,
		CE_OP_CTS, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP
	},
	.intreg = CE_ISR,
	.maxflow = 4,
	.prng = CE_ALG_PRNG,
	.maxrsakeysize = 2048,
	.rsa_op_mode = { CE_OP_RSA_512, CE_OP_RSA_1024, CE_OP_RSA_2048,
			CE_ID_NOTSUPP, CE_ID_NOTSUPP, },
	.alg_akcipher = { CE_ID_NOTSUPP, CE_ALG_RSA, },
};

static const struct ce_variant ce_a83t_variant = {
	.alg_cipher = { CE_ID_NOTSUPP, SS_ALG_AES, SS_ALG_DES, SS_ALG_3DES, },
	.op_mode = { CE_ID_NOTSUPP, SS_OP_ECB, SS_OP_CBC, SS_OP_CTR,
		CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP, CE_ID_NOTSUPP
	},
	.is_ss = true,
	.intreg = SS_INT_STA_REG,
	.maxflow = 2,
	.prng = SS_ALG_PRNG,
	.maxrsakeysize = 3072,
	.rsa_op_mode = { SS_OP_RSA_512, SS_OP_RSA_1024, SS_OP_RSA_2048,
			CE_ID_NOTSUPP, CE_ID_NOTSUPP, },
	.alg_akcipher = { CE_ID_NOTSUPP, SS_ALG_RSA, },
};

int get_engine_number(struct sun8i_ss_ctx *ss)
{
	int e = ss->flow;

	ss->flow++;
	if (ss->flow >= ss->variant->maxflow)
		ss->flow = 0;

	return e;
}

static int sun8i_ss_run_task(struct sun8i_ss_ctx *ss, int flow,
			     const char *name)
{
	int err = 0;
	u32 v = 1;
	struct ce_task *cet = ss->chanlist[flow].tl;
	int i;
	u32 *iv;

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

	if (ss->chanlist[flow].method == SS_ALG_PRNG) {
		/* grab continue mode */
		v |= SS_RNG_CONTINUE;
		/* TODO */
		cet->t_src[0].len = 5;
	}

	switch (ss->chanlist[flow].keylen) {
	case 128 / 8:
		v |= CE_AES_128BITS << 7;
	break;
	case 192 / 8:
		v |= CE_AES_192BITS << 7;
	break;
	case 256 / 8:
		v |= CE_AES_256BITS << 7;
	break;
	}

	/* enable INT for this flow */
	writel(BIT(flow), ss->base + SS_INT_CTL_REG);

	if (cet->t_key)
		writel(cet->t_key, ss->base + SS_KEY_ADR_REG);

	/* hash arbitrary IV */
	if (cet->t_common_ctl & BIT(16)) {
		v |= BIT(17);
		dev_info(ss->dev, "Need to set IV from %p\n",
			 ss->chanlist[flow].bounce_iv);
		writel(cet->t_iv, ss->base + SS_KEY_ADR_REG);
	}

	/* For PRNG the IV is set ... in key :) */
	if (ss->chanlist[flow].method == SS_ALG_PRNG)
		writel(cet->t_iv, ss->base + SS_KEY_ADR_REG);
	else
		writel(cet->t_iv, ss->base + SS_IV_ADR_REG);

	for (i = 0; i < MAX_SG; i++) {
		if (!cet->t_dst[i].addr)
			break;
		dev_info(ss->dev,
			 "Processing SG %d %s ctl=%x %d to %d method=%x op=%x opdir=%x\n",
			 i, name, v,
			 cet->t_src[i].len, cet->t_dst[i].len,
			 ss->chanlist[flow].method,
			 ss->chanlist[flow].op_mode,
			 ss->chanlist[flow].op_dir);

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
			err = -EINVAL;
			goto theend;
		}
		/*print_hex_dump(KERN_INFO, "IV ", DUMP_PREFIX_NONE, 16, 1, ss->chanlist[flow].bounce_iv,
			ss->chanlist[flow].ivlen, false);*/

	}
	/* copy next IV */
	if (ss->chanlist[flow].next_iv) {
		iv = ss->chanlist[flow].next_iv;
		for (i = 0; i < 4; i++) {
			if (flow)
				*iv = readl(ss->base + SS_CTR_REG1 + i * 4);
			else
				*iv = readl(ss->base + SS_CTR_REG0 + i * 4);
			iv++;
		}
	}
theend:
	mutex_unlock(&ss->mlock);

	return err;
}

int sun8i_ce_run_task(struct sun8i_ss_ctx *ss, int flow, const char *name)
{
	u32 v;
	int err = 0;
	struct ce_task *cet = ss->chanlist[flow].tl;

	if (ss->chanlist[flow].bounce_iv) {
		cet->t_iv = dma_map_single(ss->dev,
					   ss->chanlist[flow].bounce_iv,
					   ss->chanlist[flow].ivlen,
					   DMA_BIDIRECTIONAL);
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
			dev_err(ss->dev, "Cannot DMA MAP IV\n");
			err = -EFAULT;
			goto err_next_iv;
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	ss->chanlist[flow].stat_req++;
#endif

	if (ss->variant->is_ss) {
		err = sun8i_ss_run_task(ss, flow, name);
	} else {
		mutex_lock(&ss->mlock);

		v = readl(ss->base + CE_ICR);
		v |= 1 << flow;
		writel(v, ss->base + CE_ICR);

		reinit_completion(&ss->chanlist[flow].complete);
		writel(ss->chanlist[flow].t_phy, ss->base + CE_TDQ);

		ss->chanlist[flow].status = 0;
		/* Be sure all data is written before enabling the task */
		wmb();

		writel(1, ss->base + CE_TLR);
		mutex_unlock(&ss->mlock);

		wait_for_completion_interruptible_timeout(&ss->chanlist[flow].complete,
							  msecs_to_jiffies(5000));

		if (ss->chanlist[flow].status == 0) {
			dev_err(ss->dev, "DMA timeout for %s\n", name);
			err = -EINVAL;
		}
		/* No need to lock for this read, the channel is locked so
		 * nothing could modify the error value for this channel
		 */
		v = readl(ss->base + CE_ESR);
		if (v) {
			dev_err(ss->dev, "CE ERROR %x for flow %x\n", v, flow);
			err = -EFAULT;
			v >>= (flow * 4);
			switch (v) {
			case 1:
				dev_err(ss->dev, "CE ERROR: algorithm not supported\n");
			break;
			case 2:
				dev_err(ss->dev, "CE ERROR: data length error\n");
			break;
			case 4:
				dev_err(ss->dev, "CE ERROR: keysram access error for AES\n");
			break;
			default:
				dev_err(ss->dev, "CE ERROR: invalid error\n");
			}
		}
	}

	if (ss->chanlist[flow].next_iv) {
		dma_unmap_single(ss->dev, cet->t_ctr,
				 ss->chanlist[flow].ivlen,
				 DMA_FROM_DEVICE);
	}
err_next_iv:
	if (ss->chanlist[flow].bounce_iv) {
		dma_unmap_single(ss->dev, cet->t_iv,
				 ss->chanlist[flow].ivlen,
				 DMA_BIDIRECTIONAL);
	}

	return err;
}

static irqreturn_t ce_irq_handler(int irq, void *data)
{
	u32 p;
	struct sun8i_ss_ctx *ss = (struct sun8i_ss_ctx *)data;
	int flow = 0;

	p = readl(ss->base + ss->variant->intreg);
	for (flow = 0; flow < ss->variant->maxflow; flow++) {
		if (p & (BIT(flow))) {
			writel(BIT(flow), ss->base + ss->variant->intreg);
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
			.cra_driver_name = "ctr-aes-sun8i-ce",
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
			.cra_priority = 300,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
	.ce_algo_id = CE_ID_CIPHER_DES3,
	.ce_blockmode = CE_ID_OP_CBC,
	.alg.skcipher = {
		.base = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-sun8i-ce",
			.cra_priority = 300,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
			.cra_priority = 300,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_flags = CRYPTO_ALG_TYPE_SKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
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
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_PRNG
{
	.type = CRYPTO_ALG_TYPE_RNG,
	.alg.rng = {
		.base = {
			.cra_name		= "stdrng",
			.cra_driver_name	= "sun8i_ce_rng",
			.cra_priority		= 100,
			.cra_ctxsize		= sizeof(struct sun8i_ce_prng_ctx),
			.cra_module		= THIS_MODULE,
			.cra_init		= sun8i_ce_prng_init,
		},
		.generate               = sun8i_ce_prng_generate,
		.seed                   = sun8i_ce_prng_seed,
		.seedsize               = PRNG_SEED_SIZE,
	}
},
#endif
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_RSA
{
	.type = CRYPTO_ALG_TYPE_AKCIPHER,
	.ce_algo_id = CE_ID_AKCIPHER_RSA,
	.alg.rsa = {
		.encrypt = sun8i_rsa_encrypt,
		.decrypt = sun8i_rsa_decrypt,
		.sign = sun8i_rsa_sign,
		.verify = sun8i_rsa_verify,
		.set_priv_key = sun8i_rsa_set_priv_key,
		.set_pub_key = sun8i_rsa_set_pub_key,
		.max_size = sun8i_rsa_max_size,
		.init = sun8i_rsa_init,
		.exit = sun8i_rsa_exit,
		.base = {
			.cra_name = "rsa",
			.cra_driver_name = "rsa-sun8i-ce",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AKCIPHER |
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_ctxsize = sizeof(struct sun8i_tfm_rsa_ctx),
			.cra_module = THIS_MODULE,
			.cra_alignmask = 3,
		}
	}
},
#endif
};

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
static int sun8i_ce_dbgfs_read(struct seq_file *seq, void *v)
{
	struct sun8i_ss_ctx *ss = seq->private;
	int i;

	for (i = 0; i < ss->variant->maxflow; i++) {
		seq_printf(seq, "Channel %d: req %lu\n", i, ss->chanlist[i].stat_req);
	}
	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		ce_algs[i].ss = ss;
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			seq_printf(seq, "%s %s %lu %lu\n",
				   ce_algs[i].alg.skcipher.base.cra_driver_name,
				   ce_algs[i].alg.skcipher.base.cra_name,
				   ce_algs[i].stat_req, ce_algs[i].stat_fb);
			break;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_PRNG
		case CRYPTO_ALG_TYPE_RNG:
			seq_printf(seq, "%s %s %lu %lu\n",
				   ce_algs[i].alg.rng.base.cra_driver_name,
				   ce_algs[i].alg.rng.base.cra_name,
				   ce_algs[i].stat_req, ce_algs[i].stat_fb);
			break;
#endif
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_RSA
		case CRYPTO_ALG_TYPE_AKCIPHER:
			seq_printf(seq, "%s %s %lu %lu\n",
				   ce_algs[i].alg.rsa.base.cra_driver_name,
				   ce_algs[i].alg.rsa.base.cra_name,
				   ce_algs[i].stat_req, ce_algs[i].stat_fb);
			break;
#endif
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
	int err, i, ce_method, id;
	struct sun8i_ss_ctx *ss;

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
		dev_err(&pdev->dev, "Cannot request MMIO %d\n", err);
		return err;
	}

	ss->busclk = devm_clk_get(&pdev->dev, "ahb1_ce");
	if (IS_ERR(ss->busclk)) {
		err = PTR_ERR(ss->busclk);
		dev_err(&pdev->dev, "Cannot get AHB SS clock err=%d\n", err);
		return err;
	}
	dev_dbg(&pdev->dev, "clock ahb_ss acquired\n");

	ss->ssclk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(ss->ssclk)) {
		err = PTR_ERR(ss->ssclk);
		dev_err(&pdev->dev, "Cannot get SS clock err=%d\n", err);
		return err;
	}

	/* Get Non Secure IRQ */
	ss->ns_irq = platform_get_irq(pdev, 0);
	if (ss->ns_irq < 0) {
		dev_err(ss->dev, "Cannot get NS IRQ\n");
		return ss->ns_irq;
	}

	err = devm_request_irq(&pdev->dev, ss->ns_irq, ce_irq_handler, 0,
			       "sun8i-ce-ns", ss);
	if (err < 0) {
		dev_err(ss->dev, "Cannot request NS IRQ\n");
		return err;
	}

	ss->reset = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(ss->reset)) {
		if (PTR_ERR(ss->reset) == -EPROBE_DEFER)
			return PTR_ERR(ss->reset);
		dev_info(&pdev->dev, "no reset control found\n");
		ss->reset = NULL;
	}
#ifdef SUN8I_CE_OVERVLOCK
	err = clk_set_rate(ss->ssclk, 400 * 1000 * 1000);
	dev_info(&pdev->dev, "clk_set_rate %d\n", err);
#endif

	err = clk_prepare_enable(ss->busclk);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot prepare_enable busclk\n");
		return err;
	}

	err = clk_prepare_enable(ss->ssclk);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot prepare_enable ssclk\n");
		goto error_clk;
	}

	err = reset_control_deassert(ss->reset);
	if (err) {
		dev_err(&pdev->dev, "Cannot deassert reset control\n");
		goto error_ssclk;
	}

	v = readl(ss->base + CE_CTR);
	v >>= 16;
	v &= 0x07;
	dev_info(&pdev->dev, "CE_NS Die ID %x\n", v);

	ss->dev = &pdev->dev;
	platform_set_drvdata(pdev, ss);

	mutex_init(&ss->mlock);

	ss->chanlist = kcalloc(ss->variant->maxflow, sizeof(struct sun8i_ce_flow), GFP_KERNEL);
	if (!ss->chanlist) {
		err = -ENOMEM;
		goto error_flow;
	}

	for (i = 0; i < ss->variant->maxflow; i++) {
		init_completion(&ss->chanlist[i].complete);
		mutex_init(&ss->chanlist[i].lock);

		ss->chanlist[i].engine = crypto_engine_alloc_init(ss->dev, 1);
		if (!ss->chanlist[i].engine) {
			dev_err(ss->dev, "Cannot request engine\n");
			goto error_engine;
		}
		err = crypto_engine_start(ss->chanlist[i].engine);
		if (err) {
			dev_err(ss->dev, "Cannot request engine\n");
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
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	ss->dbgfs_dir = debugfs_create_dir("sun8i-ce", NULL);
	if (IS_ERR_OR_NULL(ss->dbgfs_dir)) {
		dev_err(ss->dev, "Fail to create debugfs dir");
		err = -ENOMEM;
		goto error_engine;
	}
	ss->dbgfs_stats = debugfs_create_file("stats", 0444,
		ss->dbgfs_dir, ss, &sun8i_ce_debugfs_fops);
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
				dev_info(ss->dev, "DEBUG: Algo of %s not supp\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				break;
			}
			id = ce_algs[i].ce_blockmode;
			ce_method = ss->variant->op_mode[id];
			if (ce_method == CE_ID_NOTSUPP) {
				dev_info(ss->dev, "DEBUG: Blockmode of %s not supp\n",
					 ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				break;
			}
			err = crypto_register_skcipher(&ce_algs[i].alg.skcipher);
			if (err) {
				dev_err(ss->dev, "Fail to register %s\n",
					ce_algs[i].alg.skcipher.base.cra_name);
				ce_algs[i].ss = NULL;
				goto error_alg;
			}
			break;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_PRNG
		case CRYPTO_ALG_TYPE_RNG:
			ce_method = ss->variant->prng;
			if (ce_method == CE_ID_NOTSUPP) {
				ce_algs[i].ss = NULL;
				break;
			}
			err = crypto_register_rng(&ce_algs[i].alg.rng);
			if (err) {
				dev_err(ss->dev, "Fail to register %s\n",
					ce_algs[i].alg.rng.base.cra_name);
				goto error_alg;
			}
			break;
#endif
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_RSA
		case CRYPTO_ALG_TYPE_AKCIPHER:
			err = crypto_register_akcipher(&ce_algs[i].alg.rsa);
			if (err != 0) {
				dev_err(ss->dev, "Fail to register RSA %s\n",
					ce_algs[i].alg.rsa.base.cra_name);
				goto error_alg;
			}
			break;
#endif
		}
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
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_PRNG
		case CRYPTO_ALG_TYPE_RNG:
			if (ce_algs[i].ss)
				crypto_unregister_rng(&ce_algs[i].alg.rng);
			break;
#endif
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_RSA
		case CRYPTO_ALG_TYPE_AKCIPHER:
			if (ce_algs[i].ss)
				crypto_unregister_akcipher(&ce_algs[i].alg.rsa);
			break;
#endif
		}
	}
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
error_debugfs:
	debugfs_remove_recursive(ss->dbgfs_dir);
#endif
error_engine:
	while (i >= 0) {
		crypto_engine_exit(ss->chanlist[i].engine);
		if (ss->chanlist[i].tl)
			dma_free_coherent(ss->dev, sizeof(struct ce_task),
				ss->chanlist[i].tl, ss->chanlist[i].t_phy);
		i--;
	}
	kfree(ss->chanlist);
error_flow:
	reset_control_assert(ss->reset);
error_ssclk:
	clk_disable_unprepare(ss->ssclk);
error_clk:
	clk_disable_unprepare(ss->busclk);
	return err;
}

static int sun8i_ce_remove(struct platform_device *pdev)
{
	int i, timeout;
	struct sun8i_ss_ctx *ss = platform_get_drvdata(pdev);

	for (i = 0; i < ARRAY_SIZE(ce_algs); i++) {
		switch (ce_algs[i].type) {
		case CRYPTO_ALG_TYPE_SKCIPHER:
			if (ce_algs[i].ss)
				crypto_unregister_skcipher(&ce_algs[i].alg.skcipher);
			break;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_PRNG
		case CRYPTO_ALG_TYPE_RNG:
			if (ce_algs[i].ss)
				crypto_unregister_rng(&ce_algs[i].alg.rng);
			break;
#endif
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_RSA
		case CRYPTO_ALG_TYPE_AKCIPHER:
			if (ce_algs[i].ss)
				crypto_unregister_akcipher(&ce_algs[i].alg.rsa);
			break;
#endif
		}
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
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
	}

	/* TODO check that any request are still under work */

	reset_control_assert(ss->reset);
	clk_disable_unprepare(ss->busclk);
	return 0;
}

static const struct of_device_id sun8i_ce_crypto_of_match_table[] = {
	{ .compatible = "allwinner,sun8i-h3-crypto",
	  .data = &ce_h3_variant },
	{ .compatible = "allwinner,sun50i-h5-crypto",
	  .data = &ce_h5_variant },
	{ .compatible = "allwinner,sun50i-a64-crypto",
	  .data = &ce_a64_variant },
	{ .compatible = "allwinner,sun8i-a83t-crypto",
	  .data = &ce_a83t_variant },
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
