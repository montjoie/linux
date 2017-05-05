/*
 *
 */

#include <crypto/internal/rng.h>
#include "sun8i-ce.h"

int sun8i_ce_prng_generate(struct crypto_rng *tfm, const u8 *src,
			   unsigned int slen, u8 *dst, unsigned int dlen)
{
	struct sun8i_ce_prng_ctx *ctx = crypto_rng_ctx(tfm);
	struct rng_alg *alg = crypto_rng_alg(tfm);
	struct sun8i_ss_alg_template *algt;
	struct ce_task *cet;
	int flow, ret = 0;
	void *data;
	size_t len;
	int antifail = 0;
	struct sun8i_ss_ctx *ss;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rng);
	ss = ctx->ss;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_req++;
#endif
	dev_dbg(ss->dev, "%s %u %u\n", __func__, slen, dlen);

	data = kmalloc(PRNG_DATA_SIZE, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

rebegin:
	len = min_t(size_t, dlen, PRNG_DATA_SIZE);
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

	ss->chanlist[flow].next_iv = kmalloc(PRNG_SEED_SIZE, GFP_KERNEL);
	if (!ss->chanlist[flow].next_iv) {
		ret = -ENOMEM;
		goto fail;
	}

	cet->t_dst[0].addr = dma_map_single(ss->dev, data, PRNG_DATA_SIZE,
					    DMA_FROM_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_dst[0].addr)) {
		dev_err(ss->dev, "Cannot DMA MAP DST DATA\n");
		ret = -EFAULT;
		goto fail;
	}
	cet->t_dst[0].len = PRNG_DATA_SIZE / 4;

	cet->t_key = cet->t_dst[0].addr;
	cet->t_iv = dma_map_single(ss->dev, ss->seed, PRNG_SEED_SIZE,
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
	memcpy(ss->seed, ss->chanlist[flow].next_iv, PRNG_SEED_SIZE);
/*	print_hex_dump(KERN_INFO, "RNG NIV ", DUMP_PREFIX_NONE, 16, 1, ss->seed,
		PRNG_SEED_SIZE, false);*/
	mutex_unlock(&ss->chanlist[flow].lock);

	if (!ret) {
		memcpy(dst, data, len);
		dst += len;
		dlen -= len;
		if (dlen > 4 && antifail++ < 10)
			goto rebegin;
	}
	memzero_explicit(data, PRNG_DATA_SIZE);
	kfree(data);
	kfree(ss->chanlist[flow].next_iv);

	return ret;
}

int sun8i_ce_prng_seed(struct crypto_rng *tfm, const u8 *seed,
		       unsigned int slen)
{
	struct sun8i_ce_prng_ctx *ctx = crypto_rng_ctx(tfm);
	struct rng_alg *alg = crypto_rng_alg(tfm);
	struct sun8i_ss_alg_template *algt;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rng);

	dev_dbg(ctx->ss->dev, "%s %u\n", __func__, slen);
	if (!ctx->ss->seed)
		ctx->ss->seed = kmalloc(slen, GFP_KERNEL | GFP_DMA);
	if (!ctx->ss->seed)
		return -ENOMEM;

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_fb++;
#endif
	memcpy(ctx->ss->seed, seed, slen);
	ctx->ss->seedsize = slen;

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
