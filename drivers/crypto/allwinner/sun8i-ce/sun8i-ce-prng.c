/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sun8i-ce-prng.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
 *
 * Copyright (C) 2016-2017 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This file adds support for the PRNG present in the CryptoEngine
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */

#include <crypto/internal/rng.h>
#include "sun8i-ce.h"

int sun8i_ce_prng_generate(struct crypto_rng *tfm, const u8 *src,
			   unsigned int slen, u8 *dst, unsigned int dlen)
{
	struct sun8i_ce_prng_ctx *ctx = crypto_rng_ctx(tfm);
	struct ce_task *cet;
	int flow, ret = 0;
	void *data;
	size_t len;
	int antifail = 0;
	struct sun8i_ss_ctx *ss;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	struct rng_alg *alg = crypto_rng_alg(tfm);
	struct sun8i_ss_alg_template *algt;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rng);
	algt->stat_req++;
#endif

	ss = ctx->ss;
	if (!ctx->seed) {
		dev_err(ss->dev, "PRNG is un-seeded\n");
		return -EINVAL;
	}
	dev_dbg(ss->dev, "%s %u %u\n", __func__, slen, dlen);

	data = kmalloc(PRNG_DATA_SIZE, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	flow = get_engine_number(ss);
	mutex_lock(&ss->chanlist[flow].lock);
	cet = ss->chanlist[flow].tl;
	memset(cet, 0, sizeof(struct ce_task));
	cet->t_id = flow;
	cet->t_common_ctl = ctx->op | BIT(31);
	cet->t_dlen = PRNG_DATA_SIZE / 4;
	ss->chanlist[flow].op_mode = 0;
	ss->chanlist[flow].op_dir = 0;
	ss->chanlist[flow].method = ctx->op;

/*	print_hex_dump(KERN_INFO, "RNG IV ", DUMP_PREFIX_NONE, 16, 1, ss->seed,
		PRNG_SEED_SIZE, false);*/

	ss->chanlist[flow].next_iv = kmalloc(PRNG_SEED_SIZE, GFP_KERNEL | GFP_DMA);
	if (!ss->chanlist[flow].next_iv) {
		ret = -ENOMEM;
		goto fail;
	}

rebegin:
	len = min_t(size_t, dlen, PRNG_DATA_SIZE);
	dev_dbg(ss->dev, "%s Rebegin %u dlen=%u steplen=%lu\n", __func__, slen, dlen, len);

	cet->t_dst[0].addr = dma_map_single(ss->dev, data, PRNG_DATA_SIZE,
					    DMA_FROM_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_dst[0].addr)) {
		dev_err(ss->dev, "Cannot DMA MAP DST DATA\n");
		ret = -EFAULT;
		goto fail;
	}
	cet->t_dst[0].len = PRNG_DATA_SIZE / 4;

	cet->t_key = cet->t_dst[0].addr;
	cet->t_iv = dma_map_single(ss->dev, ctx->seed, PRNG_SEED_SIZE,
				DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_iv)) {
		dev_err(ss->dev, "Cannot DMA MAP SEED\n");
		ret = -EFAULT;
		goto ce_rng_iv_err;
	}

	ret = sun8i_ce_run_task(ss, flow, "PRNG");

	dma_unmap_single(ss->dev, cet->t_iv, PRNG_SEED_SIZE, DMA_TO_DEVICE);
ce_rng_iv_err:
	dma_unmap_single(ss->dev, cet->t_dst[0].addr, PRNG_DATA_SIZE,
			 DMA_FROM_DEVICE);
fail:
	memcpy(ctx->seed, ss->chanlist[flow].next_iv, PRNG_SEED_SIZE);
/*	print_hex_dump(KERN_INFO, "RNG NIV ", DUMP_PREFIX_NONE, 16, 1, ss->seed,
		PRNG_SEED_SIZE, false);*/

	if (!ret) {
		memcpy(dst, data, len);
		dst += len;
		dlen -= len;
		if (dlen > 4 && antifail++ < 10)
			goto rebegin;
	}

	kfree(ss->chanlist[flow].next_iv);
	mutex_unlock(&ss->chanlist[flow].lock);
	memzero_explicit(data, PRNG_DATA_SIZE);
	kfree(data);

	return ret;
}

int sun8i_ce_prng_seed(struct crypto_rng *tfm, const u8 *seed,
		       unsigned int slen)
{
	struct sun8i_ce_prng_ctx *ctx = crypto_rng_ctx(tfm);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	struct rng_alg *alg = crypto_rng_alg(tfm);
	struct sun8i_ss_alg_template *algt;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rng);
#endif
	if (slen < PRNG_SEED_SIZE) {
		dev_err(ctx->ss->dev, "ERROR: Invalid seedsize get %u instead of %u\n", slen, PRNG_SEED_SIZE);
		return -EINVAL;
	}
	if (!ctx->seed)
		ctx->seed = kmalloc(slen, GFP_KERNEL | GFP_DMA);
	if (!ctx->seed)
		return -ENOMEM;

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_fb++;
#endif
	memcpy(ctx->seed, seed, PRNG_SEED_SIZE);

	return 0;
}

int sun8i_ce_prng_init(struct crypto_tfm *tfm)
{
	struct sun8i_ce_prng_ctx *ctx = crypto_tfm_ctx(tfm);
	struct sun8i_ss_alg_template *algt;
	struct crypto_rng *rngtfm = __crypto_rng_cast(tfm);
	struct rng_alg *alg = crypto_rng_alg(rngtfm);

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rng);
	ctx->ss = algt->ss;
	ctx->op = ctx->ss->variant->prng;

	return 0;
}
