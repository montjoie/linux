// SPDX-License-Identifier: GPL-2.0
/*
 * sun8i-ss-cipher.c - hardware cryptographic offloader for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
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
#include "sun8i-ss.h"

static int sun8i_ss_cipher(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_dev *ss = op->ss;
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);
	struct sun8i_ss_alg_template *algt;
	struct sun8i_ss_flow *chan;
	struct ce_task *cet;
	struct scatterlist *in_sg = areq->src;
	struct scatterlist *out_sg = areq->dst;
	struct scatterlist *sg;
	bool need_fallback = false;
	unsigned int todo, len, offset, ivsize;
	void *backup_iv = NULL;
	int flow, i;
	int nr_sgs = 0;
	int nr_sgd = 0;
	int err = 0;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.skcipher);

	dev_dbg(ss->dev, "%s %s %u %x IV(%p %u) key=%u\n", __func__,
		crypto_tfm_alg_name(areq->base.tfm),
		areq->cryptlen,
		rctx->op_dir, areq->iv, crypto_skcipher_ivsize(tfm),
		op->keylen);

	if (sg_nents(areq->src) > 8 || sg_nents(areq->dst) > 8)
		need_fallback = true;

	len = ss->variant->op_len_align[algt->ce_blockmode];
	if (len <= 0) {
		dev_info(ss->dev, "Invalid len align\n");
		len = 4;
	}

	sg = areq->src;
	while (sg && !need_fallback) {
		if ((sg->length % len) != 0) {
			need_fallback = true;
			break;
		}
		if ((sg_dma_len(sg) % 4) != 0) {
			need_fallback = true;
			dev_info(ss->dev, "DMA len %u\n", sg_dma_len(sg));
		}
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
			dev_info(ss->dev, "DMA len %u\n", sg_dma_len(sg));
		}
		if (!IS_ALIGNED(sg->offset, sizeof(u32)))
			need_fallback = true;
		sg = sg_next(sg);
	}

	/* SS need same numbers of SG (with same length) for source and destination */
	in_sg = areq->src;
	out_sg = areq->dst;
	while ((in_sg || out_sg) && !need_fallback) {
		if (in_sg->length != out_sg->length) {
			need_fallback = true;
			dev_dbg(ss->dev, "Fallback due to different sglen\n");
		}
		in_sg = sg_next(in_sg);
		out_sg = sg_next(out_sg);
	}
	if (!need_fallback && (in_sg || out_sg)) {
		need_fallback = true;
		dev_info(ss->dev, "Fallback due to SS\n");
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
	algt->stat_req++;
#endif

	if (need_fallback) {
		SYNC_SKCIPHER_REQUEST_ON_STACK(subreq, op->fallback_tfm);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_SS_DEBUG
		algt->stat_fb++;
#endif
		skcipher_request_set_sync_tfm(subreq, op->fallback_tfm);
		skcipher_request_set_callback(subreq, areq->base.flags, NULL,
					      NULL);
		skcipher_request_set_crypt(subreq, areq->src, areq->dst,
					   areq->cryptlen, areq->iv);
		if (rctx->op_dir & SS_DECRYPTION)
			err = crypto_skcipher_decrypt(subreq);
		else
			err = crypto_skcipher_encrypt(subreq);
		skcipher_request_zero(subreq);
		return err;
	}

	flow = rctx->flow;

	chan = &ss->chanlist[flow];
	mutex_lock(&chan->lock);

	cet = chan->tl;
	memset(cet, 0, sizeof(struct ce_task));

	cet->t_id = flow;
	cet->t_common_ctl = ss->variant->alg_cipher[algt->ce_algo_id];
	cet->t_common_ctl |= rctx->op_dir;
	cet->t_dlen = areq->cryptlen / 4;
	/* CTS and recent CE (H6) need length in bytes, in word otherwise */
	if (algt->ce_blockmode == CE_ID_OP_CTS)
		cet->t_dlen = areq->cryptlen;

	cet->t_sym_ctl = ss->variant->op_mode[algt->ce_blockmode];
	len = op->keylen;
	switch (len) {
	case 128 / 8:
		cet->t_sym_ctl |= SS_AES_128BITS;
		break;
	case 192 / 8:
		cet->t_sym_ctl |= SS_AES_192BITS;
		break;
	case 256 / 8:
		cet->t_sym_ctl |= SS_AES_256BITS;
		break;
	}

	cet->t_asym_ctl = 0;

	chan->op_mode = ss->variant->op_mode[algt->ce_blockmode];
	if (algt->ce_blockmode == CE_ID_OP_CTR)
		chan->op_mode |= SS_CTR_128;
	chan->op_dir = rctx->op_dir;
	chan->method = ss->variant->alg_cipher[algt->ce_algo_id];
	chan->keylen = op->keylen;

	cet->t_key = dma_map_single(ss->dev, op->key, op->keylen,
				    DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_key)) {
		dev_err(ss->dev, "Cannot DMA MAP KEY\n");
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
		if (rctx->op_dir & SS_DECRYPTION) {
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
		nr_sgs = dma_map_sg(ss->dev, areq->src, sg_nents(areq->src),
				    DMA_BIDIRECTIONAL);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend_iv;
		}
		nr_sgd = nr_sgs;
	} else {
		nr_sgs = dma_map_sg(ss->dev, areq->src, sg_nents(areq->src),
				    DMA_TO_DEVICE);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend_iv;
		}
		nr_sgd = dma_map_sg(ss->dev, areq->dst, sg_nents(areq->dst),
				    DMA_FROM_DEVICE);
		if (nr_sgd <= 0 || nr_sgd > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgd);
			err = -EINVAL;
			goto theend_sgs;
		}
	}

	len = areq->cryptlen;
	for_each_sg(areq->src, sg, nr_sgs, i) {
		cet->t_src[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_src[i].len = todo / 4;
		dev_dbg(ss->dev, "%s total=%u SG(%d %u off=%d) todo=%u\n", __func__,
			areq->cryptlen, i, cet->t_src[i].len, sg->offset, todo);
		len -= todo;
	}
	if (len > 0)
		dev_err(ss->dev, "remaining len %d\n", len);

	len = areq->cryptlen;
	for_each_sg(areq->dst, sg, nr_sgd, i) {
		cet->t_dst[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_dst[i].len = todo / 4;
		dev_dbg(ss->dev, "%s total=%u SG(%d %u off=%d) todo=%u\n", __func__,
			areq->cryptlen, i, cet->t_dst[i].len, sg->offset, todo);
		len -= todo;
	}
	if (len > 0)
		dev_err(ss->dev, "remaining len %d\n", len);

	err = sun8i_ss_run_task(ss, flow, "cipher");

theend_sgs:
	if (areq->src == areq->dst) {
		dma_unmap_sg(ss->dev, areq->src, nr_sgs, DMA_BIDIRECTIONAL);
	} else {
		if (nr_sgs > 0)
			dma_unmap_sg(ss->dev, areq->src, nr_sgs, DMA_TO_DEVICE);
		dma_unmap_sg(ss->dev, areq->dst, nr_sgd, DMA_FROM_DEVICE);
	}

theend_iv:
	if (areq->iv && ivsize > 0) {
		offset = areq->cryptlen - ivsize;
		if (rctx->op_dir & SS_DECRYPTION) {
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
	dma_unmap_single(ss->dev, cet->t_key, op->keylen, DMA_TO_DEVICE);

theend:
	mutex_unlock(&chan->lock);

	return err;
}

static int handle_cipher_request(struct crypto_engine *engine, void *areq)
{
	int err;
	struct skcipher_request *breq = container_of(areq, struct skcipher_request, base);

	err = sun8i_ss_cipher(breq);
	crypto_finalize_skcipher_request(engine, breq, err);

	return 0;
}

int sun8i_ss_skdecrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ss);
	struct crypto_engine *engine = op->ss->chanlist[e].engine;

	rctx->op_dir = SS_DECRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ss_skencrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ss);
	struct crypto_engine *engine = op->ss->chanlist[e].engine;

	rctx->op_dir = SS_ENCRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ss_cipher_init(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);
	struct sun8i_ss_alg_template *algt;
	const char *name = crypto_tfm_alg_name(tfm);
	struct crypto_skcipher *sktfm = __crypto_skcipher_cast(tfm);
	struct skcipher_alg *alg = crypto_skcipher_alg(sktfm);

	memset(op, 0, sizeof(struct sun8i_cipher_tfm_ctx));

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.skcipher);
	op->ss = algt->ss;

	sktfm->reqsize = sizeof(struct sun8i_cipher_req_ctx);

	op->fallback_tfm = crypto_alloc_sync_skcipher(name, 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(op->fallback_tfm)) {
		dev_err(op->ss->dev, "ERROR: Cannot allocate fallback for %s %ld\n",
			name, PTR_ERR(op->fallback_tfm));
		return PTR_ERR(op->fallback_tfm);
	}

	dev_info(op->ss->dev, "Fallback is %s\n", crypto_tfm_alg_driver_name(crypto_skcipher_tfm(&op->fallback_tfm->base)));

	op->enginectx.op.do_one_request = handle_cipher_request;
	op->enginectx.op.prepare_request = NULL;
	op->enginectx.op.unprepare_request = NULL;

	return 0;
}

void sun8i_ss_cipher_exit(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);

	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	crypto_free_sync_skcipher(op->fallback_tfm);
}

int sun8i_ss_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_dev *ss = op->ss;

	switch (keylen) {
	case 128 / 8:
		break;
	case 192 / 8:
		break;
	case 256 / 8:
		break;
	default:
		dev_err(ss->dev, "ERROR: Invalid keylen %u\n", keylen);
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

int sun8i_ss_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			 unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_dev *ss = op->ss;

	if (unlikely(keylen != 3 * DES_KEY_SIZE)) {
		dev_err(ss->dev, "Invalid keylen %u\n", keylen);
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
