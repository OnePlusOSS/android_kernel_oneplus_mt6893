// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6833-mt6359.c  --  mt6833 mt6359 ALSA SoC machine driver
 *
 *  Copyright (c) 2020 MediaTek Inc.
 *  Author: Eason Yen <eason.yen@mediatek.com>
 */

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../common/mtk-afe-platform-driver.h"
#include "mt6833-afe-common.h"
#include "mt6833-afe-clk.h"
#include "mt6833-afe-gpio.h"
#include "../../codecs/mt6359.h"
#include "../common/mtk-sp-spk-amp.h"

#ifdef CONFIG_SND_SOC_AW87339
#include "aw87339.h"
#endif

#ifdef CONFIG_SND_SOC_SIA8109
#include "sia81xx_common.h"
#endif

#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
#include "../../codecs/audio/oplus_speaker_manager/oplus_speaker_manager_platform.h"
#include "../../codecs/audio/oplus_speaker_manager/oplus_speaker_manager_codec.h"
#endif /*CONFIG_SND_SOC_OPLUS_PA_MANAGER*/

/*
 * if need additional control for the ext spk amp that is connected
 * after Lineout Buffer / HP Buffer on the codec, put the control in
 * mt6833_mt6359_spk_amp_event()
 */
#define EXT_SPK_AMP_W_NAME "Ext_Speaker_Amp"

#ifdef OPLUS_BUG_COMPATIBILITY
#define EXT_RCV_AMP_W_NAME "Ext_Reciver_Amp"
#endif /*OPLUS_BUG_COMPATIBILITY*/

#ifdef OPLUS_BUG_COMPATIBILITY
#include "../../codecs/audio/sia81xx/sia81xx_aux_dev_if.h"
#endif  /*OPLUS_BUG_COMPATIBILITY*/

static const char *const mt6833_spk_type_str[] = {MTK_SPK_NOT_SMARTPA_STR,
						  MTK_SPK_RICHTEK_RT5509_STR,
						  MTK_SPK_MEDIATEK_MT6660_STR,
						  MTK_SPK_NXP_TFA98XX_STR
						  };
static const char *const
	mt6833_spk_i2s_type_str[] = {MTK_SPK_I2S_0_STR,
				     MTK_SPK_I2S_1_STR,
				     MTK_SPK_I2S_2_STR,
				     MTK_SPK_I2S_3_STR,
				     MTK_SPK_I2S_5_STR,
				     };

static const struct soc_enum mt6833_spk_type_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mt6833_spk_type_str),
			    mt6833_spk_type_str),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mt6833_spk_i2s_type_str),
			    mt6833_spk_i2s_type_str),
};
#ifdef OPLUS_BUG_COMPATIBILITY


enum oplus_pa_type_def {
	OPLUS_PA_NXP = 0,
	OPLUS_PA_AWINIC,
	OPLUS_PA_SIA,
	OPLUS_PA_TYPE_NUM
};
static int oplus_pa_type = OPLUS_PA_NXP;

//if more config values, set a bigger number
#define AUDIO_EXTERN_CONFIG_MAX_NUM  4
#define OPLUS_PA_TYPE_OFFSET 0
int audio_extern[AUDIO_EXTERN_CONFIG_MAX_NUM] = {0};

static int read_audio_extern_config_dts(struct platform_device *pdev)
{
	int ret;
	int count, i;
	count = of_property_count_u32_elems(pdev->dev.of_node, "audio_extern_config");
	if (count <= 0) {
		dev_err(&pdev->dev, "%s: no property match audio_extern_config\n", __func__);
		return -ENODATA;
	} else if (count > AUDIO_EXTERN_CONFIG_MAX_NUM) {
		dev_err(&pdev->dev, "%s: audio_extern_config num=%d > %d(max numbers)\n",
				__func__, count, AUDIO_EXTERN_CONFIG_MAX_NUM);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node, "audio_extern_config",
			audio_extern, count);
	if (ret) {
		dev_err(&pdev->dev, "%s: read audio_extern_config error = %d\n", __func__, ret);
		return ret;
	}
	for (i = 0; i < count; i++) {
		dev_info(&pdev->dev, "%s: audio_extern[%d] = %d\n",
				__func__, i ,audio_extern[i]);
	}

	if (OPLUS_PA_TYPE_OFFSET < count) {
		oplus_pa_type = audio_extern[OPLUS_PA_TYPE_OFFSET];
		dev_info(&pdev->dev, "%s: pa_type = audio_extern[%d] = %d\n",
				__func__, OPLUS_PA_TYPE_OFFSET , oplus_pa_type);
	}

	return ret;
}

static int mt6853_audio_extern_config_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < AUDIO_EXTERN_CONFIG_MAX_NUM; i++) {
		ucontrol->value.integer.value[i] = audio_extern[i];
		pr_info("%s(), OPLUS_AUDIO_EXTERN_CONFIG get value(%d) = %d",
				__func__, i, audio_extern[i]);
	}

	return 0;
}

static int mt6853_audio_extern_config_ctl(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = AUDIO_EXTERN_CONFIG_MAX_NUM;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x7fffffff; /* 32 bit value,  */

	return 0;
}
#endif  /*OPLUS_BUG_COMPATIBILITY*/
static int mt6833_spk_type_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

static int mt6833_spk_i2s_out_type_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_i2s_out_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

static int mt6833_spk_i2s_in_type_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_i2s_in_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

#ifdef OPLUS_FEATURE_SPEAKER_MUTE
static int speaker_mute_control = 0;
static int kspk_enable_spk_pa_state = 0;

static const char *const spk_mute_function[] = { "Off", "On" };

static const struct soc_enum spkmute_snd_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_mute_function), spk_mute_function),
};

static int speaker_mute_get_status(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = speaker_mute_control;
	pr_err("%s(), speaker_mute_control = %d\n", __func__, speaker_mute_control);
	return 0;
}

static int speaker_mute_put_status(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	if(ucontrol->value.integer.value[0] == speaker_mute_control)
		return 1;
	speaker_mute_control = ucontrol->value.integer.value[0];

	if(speaker_mute_control)
	{
#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
		oplus_ext_amp_l_enable(false);
		oplus_ext_amp_r_enable(false);
#else
		if (OPLUS_PA_AWINIC == oplus_pa_type) {
                #ifdef CONFIG_SND_SOC_CODEC_AW87339
			aw87339_audio_spk_if_off();
                #endif  /*CONFIG_SND_SOC_CODEC_AW87339*/
		} else if (OPLUS_PA_SIA == oplus_pa_type) {
			sia81xx_stop();
		}
#endif //CONFIG_SND_SOC_OPLUS_PA_MANAGER
	} else {
#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
		oplus_ext_amp_l_enable(true);
		oplus_ext_amp_r_enable(true);
#else
		if((kspk_enable_spk_pa_state)&&(OPLUS_PA_AWINIC == oplus_pa_type)) {
                #ifdef CONFIG_SND_SOC_CODEC_AW87339
			aw87339_audio_spk_if_kspk();
                #endif  /*CONFIG_SND_SOC_CODEC_AW87339*/
		} else if ((kspk_enable_spk_pa_state)&&(OPLUS_PA_SIA == oplus_pa_type)) {
			sia81xx_start();
		}
#endif //CONFIG_SND_SOC_OPLUS_PA_MANAGER
	}

	pr_err("%s(), speaker_mute_control = %d\n", __func__, ucontrol->value.integer.value[0]);
	return 0;
}
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */

#ifdef OPLUS_BUG_COMPATIBILITY
static int rcv_amp_mode;
static const char *rcv_amp_type_str[] = {"SPEAKER_MODE", "RECIEVER_MODE"};
static const struct soc_enum rcv_amp_type_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rcv_amp_type_str), rcv_amp_type_str);

static int mt6833_rcv_amp_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s() = %d\n", __func__, rcv_amp_mode);
	ucontrol->value.integer.value[0] = rcv_amp_mode;
	return 0;
}

static int mt6833_rcv_amp_mode_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	if (ucontrol->value.enumerated.item[0] >= e->items)
		return -EINVAL;

	rcv_amp_mode = ucontrol->value.integer.value[0];
	pr_info("%s() = %d\n", __func__, rcv_amp_mode);
	return 0;
}
#endif /*OPLUS_BUG_COMPATIBILITY*/

static int mt6833_mt6359_spk_amp_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_info(card->dev, "%s(), event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* spk amp on control */
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
		if(speaker_mute_control){
			dev_err(card->dev, "%s(), speaker force mute\n", __func__);
			return 0;
		}
		kspk_enable_spk_pa_state = 1;
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
#ifdef CONFIG_SND_SOC_AW87339
		aw87339_spk_enable_set(true);
#endif

#ifdef CONFIG_SND_SOC_SIA8109
		sia81xx_start();
#endif

#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
		if (rcv_amp_mode == 1) {
			oplus_ext_amp_recv_l_enable(true);
                } else {
			oplus_ext_amp_l_enable(true);
		}
#endif  /*CONFIG_SND_SOC_OPLUS_PA_MANAGER*/
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* spk amp off control */
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
		kspk_enable_spk_pa_state = 0;
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
#ifdef CONFIG_SND_SOC_AW87339
		aw87339_spk_enable_set(false);
#endif

#ifdef CONFIG_SND_SOC_SIA8109
		sia81xx_stop();
#endif

#ifdef CONFIG_SND_SOC_OPLUS_PA_MANAGER
		if (rcv_amp_mode == 1) {
			oplus_ext_amp_recv_l_enable(false);
		} else {
			oplus_ext_amp_l_enable(false);
		}
#endif  /*CONFIG_SND_SOC_OPLUS_PA_MANAGER*/

		break;
	default:
		break;
	}

	return 0;
};

#ifdef OPLUS_BUG_COMPATIBILITY
static int mt6833_mt6359_rcv_amp_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_info(card->dev, "%s(), event %d, rcv_amp_mode %d\n",
		__func__, event, rcv_amp_mode);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
		if (speaker_mute_control) {
			dev_err(card->dev, "%s(), speaker force mute%d\n", __func__);
			return 0;
		}
		kspk_enable_spk_pa_state = 1;
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
		if(rcv_amp_mode == 1)
			oplus_ext_amp_recv_enable(true);
		else
			oplus_ext_amp_r_enable(true);
		break;
	case SND_SOC_DAPM_PRE_PMD:
#ifdef OPLUS_FEATURE_SPEAKER_MUTE
		kspk_enable_spk_pa_state = 0;
#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
		if(rcv_amp_mode == 1)
			oplus_ext_amp_recv_enable(false);
		else
			oplus_ext_amp_r_enable(false);
		break;
	default:
		break;
	}

	return 0;
};
#endif /*OPLUS_BUG_COMPATIBILITY*/

static const struct snd_soc_dapm_widget mt6833_mt6359_widgets[] = {
	SND_SOC_DAPM_SPK(EXT_SPK_AMP_W_NAME, mt6833_mt6359_spk_amp_event),
#ifdef OPLUS_BUG_COMPATIBILITY
	SND_SOC_DAPM_SPK(EXT_RCV_AMP_W_NAME, mt6833_mt6359_rcv_amp_event),
#endif /*OPLUS_BUG_COMPATIBILITY*/
};

static const struct snd_soc_dapm_route mt6833_mt6359_routes[] = {
	{EXT_SPK_AMP_W_NAME, NULL, "LINEOUT L"},
#ifdef OPLUS_BUG_COMPATIBILITY
	{EXT_RCV_AMP_W_NAME, NULL, "Receiver"},
#endif /*OPLUS_BUG_COMPATIBILITY*/
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone L Ext Spk Amp"},
#ifdef OPLUS_BUG_COMPATIBILITY
	{EXT_RCV_AMP_W_NAME, NULL, "Headphone R Ext Spk Amp"},
#else
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone R Ext Spk Amp"},
#endif /*OPLUS_BUG_COMPATIBILITY*/
};

#ifdef CONFIG_OPLUS_FEATURE_MM_FEEDBACK
#define HAL_FEEDBACK_MAX_BYTES         (256)
extern int hal_feedback_config_get(struct snd_kcontrol *kcontrol,
			unsigned int __user *bytes,
			unsigned int size);
extern int hal_feedback_config_set(struct snd_kcontrol *kcontrol,
			const unsigned int __user *bytes,
			unsigned int size);
#endif  /*CONFIG_OPLUS_FEATURE_MM_FEEDBACK*/

static const struct snd_kcontrol_new mt6833_mt6359_controls[] = {
	SOC_DAPM_PIN_SWITCH(EXT_SPK_AMP_W_NAME),
#ifdef OPLUS_BUG_COMPATIBILITY
	SOC_DAPM_PIN_SWITCH(EXT_RCV_AMP_W_NAME),
	SOC_ENUM_EXT("RCV_AMP_MODE", rcv_amp_type_enum,
		     mt6833_rcv_amp_mode_get, mt6833_rcv_amp_mode_set),
#endif /*OPLUS_BUG_COMPATIBILITY*/
	SOC_ENUM_EXT("MTK_SPK_TYPE_GET", mt6833_spk_type_enum[0],
		     mt6833_spk_type_get, NULL),
	SOC_ENUM_EXT("MTK_SPK_I2S_OUT_TYPE_GET", mt6833_spk_type_enum[1],
		     mt6833_spk_i2s_out_type_get, NULL),
	SOC_ENUM_EXT("MTK_SPK_I2S_IN_TYPE_GET", mt6833_spk_type_enum[1],
		     mt6833_spk_i2s_in_type_get, NULL),
	#ifdef OPLUS_BUG_COMPATIBILITY
	#ifdef OPLUS_FEATURE_SPEAKER_MUTE
	SOC_ENUM_EXT("Speaker_Mute_Switch", spkmute_snd_enum[0], speaker_mute_get_status, speaker_mute_put_status),
	#endif /* OPLUS_FEATURE_SPEAKER_MUTE */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "OPLUS_AUDIO_EXTERN_CONFIG",
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = mt6853_audio_extern_config_ctl,
		.get = mt6853_audio_extern_config_get
	},
	#endif //OPLUS_BUG_COMPATIBILITY
	#ifdef CONFIG_OPLUS_FEATURE_MM_FEEDBACK
	SND_SOC_BYTES_TLV("HAL FEEDBACK",
			  HAL_FEEDBACK_MAX_BYTES,
			  hal_feedback_config_get, hal_feedback_config_set),
	#endif //CONFIG_OPLUS_FEATURE_MM_FEEDBACK
};

/*
 * define mtk_spk_i2s_mck node in dts when need mclk,
 * BE i2s need assign snd_soc_ops = mt6833_mt6359_i2s_ops
 */
static int mt6833_mt6359_i2s_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int rate = params_rate(params);
	unsigned int mclk_fs_ratio = 128;
	unsigned int mclk_fs = rate * mclk_fs_ratio;

	return snd_soc_dai_set_sysclk(rtd->cpu_dai,
				      0, mclk_fs, SND_SOC_CLOCK_OUT);
}

static const struct snd_soc_ops mt6833_mt6359_i2s_ops = {
	.hw_params = mt6833_mt6359_i2s_hw_params,
};

static int mt6833_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_component *codec_component =
		snd_soc_rtdcom_lookup(rtd, CODEC_MT6359_NAME);
	int phase;
	unsigned int monitor;
	int test_done_1, test_done_2, test_done_3;
	int cycle_1, cycle_2, cycle_3;
	int prev_cycle_1, prev_cycle_2, prev_cycle_3;
	int counter;
	int mtkaif_calib_ok;
	bool need_calib_miso0, need_calib_miso1, need_calib_miso2;

	dev_info(afe->dev, "%s(), start\n", __func__);

	pm_runtime_get_sync(afe->dev);

	need_calib_miso0 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO0_ON);
	need_calib_miso1 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO1_ON);
	need_calib_miso2 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO2_ON);
	mt6833_afe_gpio_request(afe, true, MT6833_DAI_ADDA, 1);
	mt6833_afe_gpio_request(afe, true, MT6833_DAI_ADDA, 0);

	mt6359_mtkaif_calibration_enable(codec_component);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen,
			   CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	mtkaif_calib_ok = true;
	afe_priv->mtkaif_calibration_num_phase = 42;	/* mt6359: 0 ~ 42 */
	afe_priv->mtkaif_chosen_phase[0] = -1;
	afe_priv->mtkaif_chosen_phase[1] = -1;
	afe_priv->mtkaif_chosen_phase[2] = -1;

	for (phase = 0;
	     phase <= afe_priv->mtkaif_calibration_num_phase &&
	     mtkaif_calib_ok;
	     phase++) {
		mt6359_set_mtkaif_calibration_phase(codec_component,
						    phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done_1 = need_calib_miso0 ? 0 : -1;
		test_done_2 = need_calib_miso1 ? 0 : -1;
		test_done_3 = need_calib_miso2 ? 0 : -1;
		cycle_1 = -1;
		cycle_2 = -1;
		cycle_3 = -1;
		counter = 0;
		while (test_done_1 == 0 ||
		       test_done_2 == 0 ||
		       test_done_3 == 0) {
			regmap_read(afe_priv->topckgen,
				    CKSYS_AUD_TOP_MON, &monitor);
			/* get calibration done state */
			if (test_done_1 == 0)
				test_done_1 = (monitor >> 28) & 0x1;

			if (test_done_2 == 0)
				test_done_2 = (monitor >> 29) & 0x1;

			if (test_done_3 == 0)
				test_done_3 = (monitor >> 30) & 0x1;

			/* get delay cycle */
			if (test_done_1 == 1)
				cycle_1 = monitor & 0xf;

			if (test_done_2 == 1)
				cycle_2 = (monitor >> 4) & 0xf;

			if (test_done_3 == 1)
				cycle_3 = (monitor >> 8) & 0xf;

			/* handle if never test done */
			if (++counter > 10000) {
				dev_err(afe->dev, "%s(), test fail, cycle_1 %d, cycle_2 %d, cycle_3 %d, monitor 0x%x\n",
					__func__,
					cycle_1, cycle_2, cycle_3, monitor);
				mtkaif_calib_ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle_1 = cycle_1;
			prev_cycle_2 = cycle_2;
			prev_cycle_3 = cycle_3;
		}

		if (cycle_1 != prev_cycle_1 && need_calib_miso0) {
			afe_priv->mtkaif_chosen_phase[0] = phase - 1;
			afe_priv->mtkaif_phase_cycle[0] = prev_cycle_1;
			need_calib_miso0 = false;
		}

		if (cycle_2 != prev_cycle_2 && need_calib_miso1) {
			afe_priv->mtkaif_chosen_phase[1] = phase - 1;
			afe_priv->mtkaif_phase_cycle[1] = prev_cycle_2;
			need_calib_miso1 = false;
		}

		if (cycle_3 != prev_cycle_3 && need_calib_miso2) {
			afe_priv->mtkaif_chosen_phase[2] = phase - 1;
			afe_priv->mtkaif_phase_cycle[2] = prev_cycle_3;
			need_calib_miso2 = false;
		}

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x0);

		if (!need_calib_miso0 && !need_calib_miso1 && !need_calib_miso2)
			break;
	}

	mt6359_set_mtkaif_calibration_phase(codec_component,
		(afe_priv->mtkaif_chosen_phase[0] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[0],
		(afe_priv->mtkaif_chosen_phase[1] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[1],
		(afe_priv->mtkaif_chosen_phase[2] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[2]);

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(codec_component);

	mt6833_afe_gpio_request(afe, false, MT6833_DAI_ADDA, 1);
	mt6833_afe_gpio_request(afe, false, MT6833_DAI_ADDA, 0);

	/* disable syncword if miso pin not prepared */
	if (!mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO0_ON)) {
		dev_info(afe->dev, "%s(), miso0 syncword disabled\n",
			 __func__);
		regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_SYNCWORD_CFG,
				   0x1 << 11, 0x1 << 11);
	}

	if (!mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO1_ON)) {
		dev_info(afe->dev, "%s(), miso1 syncword disabled\n",
			 __func__);
		regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_SYNCWORD_CFG,
				   0x1 << 15, 0x1 << 15);
	}

	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "%s(), mtkaif_chosen_phase[0/1/2]:%d/%d/%d\n",
		 __func__,
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2]);

	return 0;
}

static int mt6833_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct mt6359_codec_ops ops;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_dapm_context *dapm = &rtd->card->dapm;
	struct snd_soc_component *codec_component =
		snd_soc_rtdcom_lookup(rtd, CODEC_MT6359_NAME);

	ops.enable_dc_compensation = mt6833_enable_dc_compensation;
	ops.set_lch_dc_compensation = mt6833_set_lch_dc_compensation;
	ops.set_rch_dc_compensation = mt6833_set_rch_dc_compensation;
	ops.adda_dl_gain_control = mt6833_adda_dl_gain_control;
	mt6359_set_codec_ops(codec_component, &ops);

	/* set mtkaif protocol */
	mt6359_set_mtkaif_protocol(codec_component,
				   MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	/* mtkaif calibration */
	if (afe_priv->mtkaif_protocol == MTKAIF_PROTOCOL_2_CLK_P2)
		mt6833_mt6359_mtkaif_calibration(rtd);

	/* disable ext amp connection */
	snd_soc_dapm_disable_pin(dapm, EXT_SPK_AMP_W_NAME);

	return 0;
}

static int mt6833_i2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	dev_info(rtd->dev, "%s(), fix format to 32bit\n", __func__);

	/* fix BE i2s format to 32bit, clean param mask first */
	snd_mask_reset_range(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			     0, SNDRV_PCM_FORMAT_LAST);

	params_set_format(params, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

#ifdef CONFIG_MTK_VOW_SUPPORT
static const struct snd_pcm_hardware mt6833_mt6359_vow_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.period_bytes_min = 256,
	.period_bytes_max = 2 * 1024,
	.periods_min = 2,
	.periods_max = 4,
	.buffer_bytes_max = 2 * 2 * 1024,
};

static int mt6833_mt6359_vow_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_component *comp = NULL;
	struct snd_soc_rtdcom_list *rtdcom = NULL;

	dev_info(afe->dev, "%s(), start\n", __func__);
	snd_soc_set_runtime_hwparams(substream, &mt6833_mt6359_vow_hardware);

	mt6833_afe_gpio_request(afe, true, MT6833_DAI_VOW, 0);

	/* ASoC will call pm_runtime_get, but vow don't need */
	for_each_rtdcom(rtd, rtdcom) {
		comp = rtdcom->component;
		pm_runtime_put_autosuspend(comp->dev);
	}

	return 0;
}

static void mt6833_mt6359_vow_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_component *comp = NULL;
	struct snd_soc_rtdcom_list *rtdcom = NULL;

	dev_info(afe->dev, "%s(), end\n", __func__);
	mt6833_afe_gpio_request(afe, false, MT6833_DAI_VOW, 0);

	/* restore to fool ASoC */
	for_each_rtdcom(rtd, rtdcom) {
		comp = rtdcom->component;
		pm_runtime_get_sync(comp->dev);
	}
}

static const struct snd_soc_ops mt6833_mt6359_vow_ops = {
	.startup = mt6833_mt6359_vow_startup,
	.shutdown = mt6833_mt6359_vow_shutdown,
};
#endif  // #ifdef CONFIG_MTK_VOW_SUPPORT

static struct snd_soc_dai_link mt6833_mt6359_dai_links[] = {
	/* Front End DAI links */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.cpu_dai_name = "DL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_12",
		.stream_name = "Playback_12",
		.cpu_dai_name = "DL12",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.cpu_dai_name = "DL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_3",
		.stream_name = "Playback_3",
		.cpu_dai_name = "DL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_4",
		.stream_name = "Playback_4",
		.cpu_dai_name = "DL4",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_5",
		.stream_name = "Playback_5",
		.cpu_dai_name = "DL5",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_6",
		.stream_name = "Playback_6",
		.cpu_dai_name = "DL6",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_7",
		.stream_name = "Playback_7",
		.cpu_dai_name = "DL7",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_8",
		.stream_name = "Playback_8",
		.cpu_dai_name = "DL8",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_9",
		.stream_name = "Playback_9",
		.cpu_dai_name = "DL9",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.cpu_dai_name = "UL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.cpu_dai_name = "UL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_3",
		.stream_name = "Capture_3",
		.cpu_dai_name = "UL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_4",
		.stream_name = "Capture_4",
		.cpu_dai_name = "UL4",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_5",
		.stream_name = "Capture_5",
		.cpu_dai_name = "UL5",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_6",
		.stream_name = "Capture_6",
		.cpu_dai_name = "UL6",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_7",
		.stream_name = "Capture_7",
		.cpu_dai_name = "UL7",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_8",
		.stream_name = "Capture_8",
		.cpu_dai_name = "UL8",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_1",
		.stream_name = "Capture_Mono_1",
		.cpu_dai_name = "UL_MONO_1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_2",
		.stream_name = "Capture_Mono_2",
		.cpu_dai_name = "UL_MONO_2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_3",
		.stream_name = "Capture_Mono_3",
		.cpu_dai_name = "UL_MONO_3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},

	{
		.name = "Hostless_LPBK",
		.stream_name = "Hostless_LPBK",
		.cpu_dai_name = "Hostless LPBK DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_FM",
		.stream_name = "Hostless_FM",
		.cpu_dai_name = "Hostless FM DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.cpu_dai_name = "Hostless Speech DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Sph_Echo_Ref",
		.stream_name = "Hostless_Sph_Echo_Ref",
		.cpu_dai_name = "Hostless_Sph_Echo_Ref_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Spk_Init",
		.stream_name = "Hostless_Spk_Init",
		.cpu_dai_name = "Hostless_Spk_Init_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_ADDA_DL_I2S_OUT",
		.stream_name = "Hostless_ADDA_DL_I2S_OUT",
		.cpu_dai_name = "Hostless_ADDA_DL_I2S_OUT DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_1",
		.stream_name = "Hostless_SRC_1",
		.cpu_dai_name = "Hostless_SRC_1_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_Bargein",
		.stream_name = "Hostless_SRC_Bargein",
		.cpu_dai_name = "Hostless_SRC_Bargein_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* Back End DAI links */
	{
		.name = "Primary Codec",
		.cpu_dai_name = "ADDA",
		.codec_dai_name = "mt6359-snd-codec-aif1",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.init = mt6833_mt6359_init,
	},
	{
		.name = "AP_DMIC",
		.cpu_dai_name = "AP_DMIC",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "I2S3",
		.cpu_dai_name = "I2S3",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S0",
		.cpu_dai_name = "I2S0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S1",
		.cpu_dai_name = "I2S1",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S2",
		.cpu_dai_name = "I2S2",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S5",
		.cpu_dai_name = "I2S5",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "HW Gain 1",
		.cpu_dai_name = "HW Gain 1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW Gain 2",
		.cpu_dai_name = "HW Gain 2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW_SRC_1",
		.cpu_dai_name = "HW_SRC_1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW_SRC_2",
		.cpu_dai_name = "HW_SRC_2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "CONNSYS_I2S",
		.cpu_dai_name = "CONNSYS_I2S",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "PCM 1",
		.cpu_dai_name = "PCM 1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "PCM 2",
		.cpu_dai_name = "PCM 2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* dummy BE for ul memif to record from dl memif */
	{
		.name = "Hostless_UL1",
		.cpu_dai_name = "Hostless_UL1 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL2",
		.cpu_dai_name = "Hostless_UL2 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL3",
		.cpu_dai_name = "Hostless_UL3 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL6",
		.cpu_dai_name = "Hostless_UL6 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_HW_Gain_AAudio",
		.stream_name = "Hostless_HW_Gain_AAudio",
		.cpu_dai_name = "Hostless HW Gain AAudio DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_AAudio",
		.stream_name = "Hostless_SRC_AAudio",
		.cpu_dai_name = "Hostless SRC AAudio DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* BTCVSD */
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	{
		.name = "BTCVSD",
		.stream_name = "BTCVSD",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = "18050000.mtk-btcvsd-snd",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
#ifdef CONFIG_MTK_VOW_SUPPORT
	{
		.name = "VOW_Capture",
		.stream_name = "VOW_Capture",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_dai_name = "mt6359-snd-codec-vow",
		.ignore_suspend = 1,
		.ops = &mt6833_mt6359_vow_ops,
	},
#endif  // #ifdef CONFIG_MTK_VOW_SUPPORT
#if defined(CONFIG_SND_SOC_MTK_SCP_SMARTPA)
	{
		.name = "SCP_SPK_Playback",
		.stream_name = "SCP_SPK_Playback",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd_scp_spk",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
#if defined(CONFIG_MTK_ULTRASND_PROXIMITY)
	{
		.name = "SCP_ULTRA_Playback",
		.stream_name = "SCP_ULTRA_Playback",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd_scp_ultra",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
};

static struct snd_soc_card mt6833_mt6359_soc_card = {
	.name = "mt6833-mt6359",
	.owner = THIS_MODULE,
	.dai_link = mt6833_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6833_mt6359_dai_links),

	.controls = mt6833_mt6359_controls,
	.num_controls = ARRAY_SIZE(mt6833_mt6359_controls),
	.dapm_widgets = mt6833_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6833_mt6359_widgets),
	.dapm_routes = mt6833_mt6359_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6833_mt6359_routes),
};

static int mt6833_mt6359_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6833_mt6359_soc_card;
	struct device_node *platform_node, *codec_node, *spk_node;
	struct snd_soc_dai_link *spk_out_dai_link, *spk_iv_dai_link;
	int ret, i;
	int spk_out_dai_link_idx, spk_iv_dai_link_idx;

#ifdef OPLUS_BUG_COMPATIBILITY
	read_audio_extern_config_dts(pdev);
#endif  /*OPLUS_BUG_COMPATIBILITY*/
	dev_info(&pdev->dev, "%s(), ++\n", __func__);

	ret = mtk_spk_update_info(card, pdev,
				  &spk_out_dai_link_idx, &spk_iv_dai_link_idx,
				  &mt6833_mt6359_i2s_ops);
	if (ret) {
		dev_err(&pdev->dev, "%s(), mtk_spk_update_info error\n",
			__func__);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "%s(), update spk dai\n", __func__);

	spk_out_dai_link = &mt6833_mt6359_dai_links[spk_out_dai_link_idx];
	spk_iv_dai_link = &mt6833_mt6359_dai_links[spk_iv_dai_link_idx];
	if (!spk_out_dai_link->codec_dai_name &&
	    !spk_iv_dai_link->codec_dai_name) {
		spk_node = of_get_child_by_name(pdev->dev.of_node,
					"mediatek,speaker-codec");
		if (!spk_node) {
			dev_err(&pdev->dev,
				"spk_codec of_get_child_by_name fail\n");
			return -EINVAL;
		}
		ret = snd_soc_of_get_dai_link_codecs(
				&pdev->dev, spk_node, spk_out_dai_link);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"i2s out get_dai_link_codecs fail\n");
			return -EINVAL;
		}
		ret = snd_soc_of_get_dai_link_codecs(
				&pdev->dev, spk_node, spk_iv_dai_link);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"i2s in get_dai_link_codecs fail\n");
			return -EINVAL;
		}
	}

	dev_info(&pdev->dev, "%s(), update platform dai\n", __func__);

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}
	for (i = 0; i < card->num_links; i++) {
		if (mt6833_mt6359_dai_links[i].platform_name)
			continue;
		mt6833_mt6359_dai_links[i].platform_of_node = platform_node;
	}

	dev_info(&pdev->dev, "%s(), update audio-codec dai\n", __func__);

	codec_node = of_parse_phandle(pdev->dev.of_node,
				      "mediatek,audio-codec", 0);
	if (!codec_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		return -EINVAL;
	}
	for (i = 0; i < card->num_links; i++) {
		if (mt6833_mt6359_dai_links[i].codec_name ||
		    i == spk_out_dai_link_idx ||
		    i == spk_iv_dai_link_idx)
			continue;
		mt6833_mt6359_dai_links[i].codec_of_node = codec_node;
	}

	card->dev = &pdev->dev;
#ifdef OPLUS_BUG_COMPATIBILITY
	ret = soc_aux_init_only_sia81xx(pdev, card);
        dev_err(&pdev->dev, "%s soc_aux_init_only_sia8108 %d\n",
            __func__, ret);
	if (ret)
		dev_err(&pdev->dev, "%s soc_aux_init_only_sia8108 fail %d\n",
			__func__, ret);
#endif  /*OPLUS_BUG_COMPATIBILITY*/
	dev_info(&pdev->dev, "%s(), devm_snd_soc_register_card\n", __func__);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id mt6833_mt6359_dt_match[] = {
	{.compatible = "mediatek,mt6833-mt6359-sound",},
	{}
};
#endif

static const struct dev_pm_ops mt6833_mt6359_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver mt6833_mt6359_driver = {
	.driver = {
		.name = "mt6833-mt6359",
#ifdef CONFIG_OF
		.of_match_table = mt6833_mt6359_dt_match,
#endif
		.pm = &mt6833_mt6359_pm_ops,
	},
	.probe = mt6833_mt6359_dev_probe,
};

module_platform_driver(mt6833_mt6359_driver);

/* Module information */
MODULE_DESCRIPTION("MT6833 MT6359 ALSA SoC machine driver");
MODULE_AUTHOR("Eason Yen <eason.yen@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mt6833 mt6359 soc card");
