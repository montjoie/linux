// SPDX-License-Identifier: GPL-2.0
/*
 * sun8i-ce-cipher.c - hardware cryptographic offloader for
 * Allwinner H3/A64/H5/H2+/H6/R40 SoC
 *
 * Copyright (C) 2016-2019 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This file add support for AES cipher with 128,192,256 bits keysize in
 * CBC and ECB mode.
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */

#include <linux/crypto.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/skcipher.h>
#include <crypto/xts.h>
#include "sun8i-ce.h"

static int sun8i_ce_cipher(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ce_dev *ce = op->ce;
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);
	struct sun8i_ce_alg_template *algt;
	struct sun8i_ce_flow *chan;
	struct ce_task *cet;
	struct scatterlist *sg;
	bool need_fallback = false;
	unsigned int todo, len, offset, ivsize;
	void *backup_iv = NULL;
	int flow, i;
	int nr_sgs = 0;
	int nr_sgd = 0;
	int err = 0;

	algt = container_of(alg, struct sun8i_ce_alg_template, alg.skcipher);

	dev_dbg(ce->dev, "%s %s %u %x IV(%p %u) key=%u\n", __func__,
		crypto_tfm_alg_name(areq->base.tfm),
		areq->cryptlen,
		rctx->op_dir, areq->iv, crypto_skcipher_ivsize(tfm),
		op->keylen);

	if (sg_nents(areq->src) > 8 || sg_nents(areq->dst) > 8)
		need_fallback = true;

	len = ce->variant->op_len_align[algt->ce_blockmode];
	if (len <= 0) {
		dev_info(ce->dev, "Invalid len align\n");
		len = 4;
	}

	if (areq->cryptlen == 0)
		need_fallback = true;

	sg = areq->src;
	while (sg && !need_fallback) {
		if ((sg->length % len) != 0) {
			need_fallback = true;
			break;
		}
		if ((sg_dma_len(sg) % 4) != 0)
			need_fallback = true;

		if (!IS_ALIGNED(sg->offset, sizeof(u32)))
			need_fallback = true;
		sg = sg_next(sg);
	}
	sg = areq->dst;
	while (sg && !need_fallback) {
		if ((sg->length % len) != 0) {
			need_fallback = true;
			break;
		}
		if ((sg_dma_len(sg) % 4) != 0) {
			need_fallback = true;
			dev_info(ce->dev, "DMA len %u\n", sg_dma_len(sg));
		}
		if (!IS_ALIGNED(sg->offset, sizeof(u32)))
			need_fallback = true;
		sg = sg_next(sg);
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_req++;
#endif

	if (need_fallback) {
		SYNC_SKCIPHER_REQUEST_ON_STACK(subreq, op->fallback_tfm);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
		algt->stat_fb++;
#endif
		skcipher_request_set_sync_tfm(subreq, op->fallback_tfm);
		skcipher_request_set_callback(subreq, areq->base.flags, NULL,
					      NULL);
		skcipher_request_set_crypt(subreq, areq->src, areq->dst,
					   areq->cryptlen, areq->iv);
		if (rctx->op_dir & CE_DECRYPTION)
			err = crypto_skcipher_decrypt(subreq);
		else
			err = crypto_skcipher_encrypt(subreq);
		skcipher_request_zero(subreq);
		return err;
	}

	flow = rctx->flow;

	chan = &ce->chanlist[flow];
	mutex_lock(&chan->lock);

	cet = chan->tl;
	memset(cet, 0, sizeof(struct ce_task));

	cet->t_id = flow;
	cet->t_common_ctl = ce->variant->alg_cipher[algt->ce_algo_id];
	cet->t_common_ctl |= rctx->op_dir | CE_COMM_INT;
	cet->t_dlen = areq->cryptlen / 4;
	/* CTS and recent CE (H6) need length in bytes, in word otherwise */
	if (algt->ce_blockmode == CE_ID_OP_CTS || ce->variant->model == CE_v2)
		cet->t_dlen = areq->cryptlen;

	cet->t_sym_ctl = ce->variant->op_mode[algt->ce_blockmode];
	len = op->keylen;
	if (algt->ce_blockmode == CE_ID_OP_XTS)
		len = op->keylen / 2;
	switch (len) {
	case 128 / 8:
		cet->t_sym_ctl |= CE_AES_128BITS;
		break;
	case 192 / 8:
		cet->t_sym_ctl |= CE_AES_192BITS;
		break;
	case 256 / 8:
		cet->t_sym_ctl |= CE_AES_256BITS;
		break;
	}

	if (algt->ce_blockmode == CE_ID_OP_CTR)
		cet->t_sym_ctl |= CE_CTR_128;
	if (algt->ce_blockmode == CE_ID_OP_CTS)
		cet->t_sym_ctl |= CE_CTS;
	if (algt->ce_blockmode == CE_ID_OP_XTS)
		cet->t_sym_ctl |= CE_XTS_FIRST | CE_XTS_LAST;

	cet->t_asym_ctl = 0;

	chan->op_mode = ce->variant->op_mode[algt->ce_blockmode];
	chan->op_dir = rctx->op_dir;
	chan->method = ce->variant->alg_cipher[algt->ce_algo_id];
	chan->keylen = op->keylen;

	cet->t_key = dma_map_single(ce->dev, op->key, op->keylen,
				    DMA_TO_DEVICE);
	if (dma_mapping_error(ce->dev, cet->t_key)) {
		dev_err(ce->dev, "Cannot DMA MAP KEY\n");
		err = -EFAULT;
		goto theend;
	}

	ivsize = crypto_skcipher_ivsize(tfm);
	if (areq->iv && crypto_skcipher_ivsize(tfm) > 0) {
		chan->next_iv = NULL;
		chan->ivlen = ivsize;
		chan->bounce_iv = kzalloc(ivsize, GFP_KERNEL | GFP_DMA);
		if (!chan->bounce_iv) {
			err = -ENOMEM;
			goto theend_key;
		}
		if (algt->ce_blockmode == CE_ID_OP_CTR) {
			chan->next_iv = kzalloc(ivsize, GFP_KERNEL | GFP_DMA);
			if (!chan->next_iv) {
				err = -ENOMEM;
				goto theend_iv;
			}
		}
		if (rctx->op_dir & CE_DECRYPTION) {
			backup_iv = kzalloc(ivsize, GFP_KERNEL);
			if (!backup_iv) {
				err = -ENOMEM;
				goto theend_key;
			}
			offset = areq->cryptlen - ivsize;
			scatterwalk_map_and_copy(backup_iv, areq->src, offset,
						 ivsize, 0);
		}
		memcpy(chan->bounce_iv, areq->iv, ivsize);
	}

	if (areq->src == areq->dst) {
		nr_sgs = dma_map_sg(ce->dev, areq->src, sg_nents(areq->src),
				    DMA_BIDIRECTIONAL);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ce->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend_iv;
		}
		nr_sgd = nr_sgs;
	} else {
		nr_sgs = dma_map_sg(ce->dev, areq->src, sg_nents(areq->src),
				    DMA_TO_DEVICE);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ce->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend_iv;
		}
		nr_sgd = dma_map_sg(ce->dev, areq->dst, sg_nents(areq->dst),
				    DMA_FROM_DEVICE);
		if (nr_sgd <= 0 || nr_sgd > 8) {
			dev_err(ce->dev, "Invalid sg number %d\n", nr_sgd);
			err = -EINVAL;
			goto theend_sgs;
		}
	}

	len = areq->cryptlen;
	for_each_sg(areq->src, sg, nr_sgs, i) {
		cet->t_src[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_src[i].len = todo / 4;
		dev_dbg(ce->dev, "%s total=%u SG(%d %u off=%d) todo=%u\n", __func__,
			areq->cryptlen, i, cet->t_src[i].len, sg->offset, todo);
		len -= todo;
	}
	if (len > 0)
		dev_err(ce->dev, "remaining len %d\n", len);

	len = areq->cryptlen;
	for_each_sg(areq->dst, sg, nr_sgd, i) {
		cet->t_dst[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_dst[i].len = todo / 4;
		dev_dbg(ce->dev, "%s total=%u SG(%d %u off=%d) todo=%u\n", __func__,
			areq->cryptlen, i, cet->t_dst[i].len, sg->offset, todo);
		len -= todo;
	}
	if (len > 0)
		dev_err(ce->dev, "remaining len %d\n", len);

	err = sun8i_ce_run_task(ce, flow, "cipher");

theend_sgs:
	if (areq->src == areq->dst) {
		dma_unmap_sg(ce->dev, areq->src, nr_sgs, DMA_BIDIRECTIONAL);
	} else {
		if (nr_sgs > 0)
			dma_unmap_sg(ce->dev, areq->src, nr_sgs, DMA_TO_DEVICE);
		dma_unmap_sg(ce->dev, areq->dst, nr_sgd, DMA_FROM_DEVICE);
	}

theend_iv:
	if (areq->iv && ivsize > 0) {
		offset = areq->cryptlen - ivsize;
		if (rctx->op_dir & CE_DECRYPTION) {
			memcpy(areq->iv, backup_iv, ivsize);
			kzfree(backup_iv);
		} else {
			scatterwalk_map_and_copy(areq->iv, areq->dst, offset,
					ivsize, 0);
		}
		if (chan->next_iv) {
			memcpy(areq->iv, chan->next_iv, ivsize);
			memzero_explicit(chan->next_iv, ivsize);
		}
		kfree(chan->bounce_iv);
		kfree(chan->next_iv);
	}

theend_key:
	dma_unmap_single(ce->dev, cet->t_key, op->keylen, DMA_TO_DEVICE);

theend:
	mutex_unlock(&chan->lock);

	return err;
}

static int handle_cipher_request(struct crypto_engine *engine, void *areq)
{
	int err;
	struct skcipher_request *breq = container_of(areq, struct skcipher_request, base);

	err = sun8i_ce_cipher(breq);
	crypto_finalize_skcipher_request(engine, breq, err);

	return 0;
}

int sun8i_ce_skdecrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ce);
	struct crypto_engine *engine = op->ce->chanlist[e].engine;

	rctx->op_dir = CE_DECRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ce_skencrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ce);
	struct crypto_engine *engine = op->ce->chanlist[e].engine;

	rctx->op_dir = CE_ENCRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ce_cipher_init(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);
	struct sun8i_ce_alg_template *algt;
	const char *name = crypto_tfm_alg_name(tfm);
	struct crypto_skcipher *sktfm = __crypto_skcipher_cast(tfm);
	struct skcipher_alg *alg = crypto_skcipher_alg(sktfm);

	memset(op, 0, sizeof(struct sun8i_cipher_tfm_ctx));

	algt = container_of(alg, struct sun8i_ce_alg_template, alg.skcipher);
	op->ce = algt->ce;

	sktfm->reqsize = sizeof(struct sun8i_cipher_req_ctx);

	op->fallback_tfm = crypto_alloc_sync_skcipher(name, 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(op->fallback_tfm)) {
		dev_err(op->ce->dev, "ERROR: Cannot allocate fallback for %s %ld\n",
			name, PTR_ERR(op->fallback_tfm));
		return PTR_ERR(op->fallback_tfm);
	}

	dev_info(op->ce->dev, "Fallback is %s\n", crypto_tfm_alg_driver_name(crypto_skcipher_tfm(&op->fallback_tfm->base)));

	op->enginectx.op.do_one_request = handle_cipher_request;
	op->enginectx.op.prepare_request = NULL;
	op->enginectx.op.unprepare_request = NULL;

	return 0;
}

void sun8i_ce_cipher_exit(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);

	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	crypto_free_sync_skcipher(op->fallback_tfm);
}

int sun8i_ce_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ce_dev *ce = op->ce;

	switch (keylen) {
	case 128 / 8:
		break;
	case 192 / 8:
		break;
	case 256 / 8:
		break;
	default:
		dev_err(ce->dev, "ERROR: Invalid keylen %u\n", keylen);
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	op->keylen = keylen;
	op->key = kmalloc(keylen, GFP_KERNEL | GFP_DMA);
	if (!op->key)
		return -ENOMEM;
	memcpy(op->key, key, keylen);

	crypto_sync_skcipher_clear_flags(op->fallback_tfm, CRYPTO_TFM_REQ_MASK);
	crypto_sync_skcipher_set_flags(op->fallback_tfm, tfm->base.crt_flags & CRYPTO_TFM_REQ_MASK);

	return crypto_sync_skcipher_setkey(op->fallback_tfm, key, keylen);
}

int sun8i_ce_aes_xts_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ce_dev *ce = op->ce;
	int ret;

	ret = xts_check_key(&tfm->base, key, keylen);
	if (ret)
		return ret;

	switch (keylen / 2) {
	case 128 / 8:
		break;
	case 192 / 8:
		break;
	case 256 / 8:
		break;
	default:
		dev_err(ce->dev, "ERROR: Invalid keylen %u\n", keylen);
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	op->keylen = keylen;
	op->key = kmalloc(keylen, GFP_KERNEL | GFP_DMA);
	if (!op->key)
		return -ENOMEM;
	memcpy(op->key, key, keylen);

	crypto_sync_skcipher_clear_flags(op->fallback_tfm, CRYPTO_TFM_REQ_MASK);
	crypto_sync_skcipher_set_flags(op->fallback_tfm, tfm->base.crt_flags & CRYPTO_TFM_REQ_MASK);

	return crypto_sync_skcipher_setkey(op->fallback_tfm, key, keylen);
}

int sun8i_ce_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			 unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ce_dev *ce = op->ce;

	if (unlikely(keylen != 3 * DES_KEY_SIZE)) {
		dev_err(ce->dev, "Invalid keylen %u\n", keylen);
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	op->keylen = keylen;
	op->key = kmalloc(keylen, GFP_KERNEL | GFP_DMA);
	if (!op->key)
		return -ENOMEM;
	memcpy(op->key, key, keylen);

	return crypto_sync_skcipher_setkey(op->fallback_tfm, key, keylen);
}
