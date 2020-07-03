/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Based on imx-sgtl5000.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/mfd/syscon.h>

#include "../codecs/wm8731.h"
#include "imx-audmux.h"
#include "imx-ssi.h"

#define DAI_NAME_SIZE	32

struct imx_wm8731_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct i2c_client *codec_dev;
	struct clk *codec_clk;
	struct clk *mclk_parent_clk;
	long sysclk;
};

static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd);
static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
                                       struct snd_pcm_hw_params *params);

struct imx_priv {
	struct platform_device *pdev;
	struct imx_wm8731_data *data;
};

static struct imx_priv card_priv;

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params 	= imx_hifi_hw_params_slv_mode,
};


/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack",       NULL),
	SND_SOC_DAPM_SPK("Ext Spk",             NULL),
	SND_SOC_DAPM_LINE("Line Jack",          NULL),
	SND_SOC_DAPM_MIC("Mic Jack",            NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack",     NULL,   "LHPOUT" },
	{ "Headphone Jack",     NULL,   "RHPOUT" },

	{ "Ext Spk",            NULL,   "LOUT" },
	{ "Ext Spk",            NULL,   "ROUT" },

	{ "LLINEIN",            NULL,   "Line Jack" },
	{ "RLINEIN",            NULL,   "Line Jack" },

	{ "MICIN",              NULL,   "Mic Bias" },
	{ "Mic Bias",           NULL,   "Mic Jack"},
};


static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);

	u32 dai_format;
	snd_pcm_format_t sample_format;
	unsigned int channels;
	unsigned int sampling_rate;
	unsigned int div_2, div_psr, div_pm;
	int ret;

	sampling_rate = params_rate(params);
	sample_format = params_format(params);

	channels = params_channels(params);
	/*printk("%s:%s  sampling rate = %u  channels = %u \n", __FUNCTION__,
		   (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture"),
		   sampling_rate, channels);*/

	/* set CPU DAI configuration */
	switch (sampling_rate) {
		case 8000:
		case 32000:
		case 48000:
		case 96000:
			data->sysclk = 12288000;
			break;

		case 44100:
		case 88200:
			data->sysclk = 11289600;
			break;

		default:
			return -EINVAL;
	}

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	/* S[TR]CCR:DC */
	snd_soc_dai_set_tdm_slot(cpu_dai, 0x3, 0x3, 2, 32);

	/*
	 * SSI sysclk divider:
	 * div_2:	/1 or /2
	 * div_psr:	/1 or /8
	 * div_pm:	/1 .. /256
	 */
	div_2	= 0;
	div_psr	= 0;
	switch (sampling_rate) {
		case 8000:
			// 1x1x12
			div_pm	= 11;
			break;
		case 32000:
			// 1x1x3
			div_pm	= 2;
			break;
		case 48000:
			// 1x1x2
			div_pm	= 1;
			break;
		case 96000:
			// 1x1x1
			div_pm	= 0;
			break;
		case 44100:
			// 1x1x2
			div_pm	= 1;
			break;
		case 88200:
			// 1x1x1
			div_pm	= 0;
			break;
		default:
			return -EINVAL;
	}

	/* sync mode: a single clock controls both playback and capture */
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, (div_2 ? SSI_STCCR_DIV2 : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, (div_psr ? SSI_STCCR_PSR : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, div_pm);

	/* set codec DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	if (data->mclk_parent_clk)
	{
		/*
		 * This is an attempt to support several sample rates (and mclk's
		 * The mclk rate can only be set if the parent clock is a multiple
		 * of requested mclk. clk_get_parent isn't used since we want this
		 * to be configurable from the dts. If mclk_parent isn't set in
		 * the dts the rate won't be changed for the parent.
		 *
		 * This was introduced for iMX6 SoloX
		 */
		clk_set_rate(data->mclk_parent_clk, 32*data->sysclk);
	}

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     WM8731_SYSCLK_MCLK,
				     data->sysclk,
				     SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("Failed to set codec master clock to %lu: %d \n",
		       data->sysclk, ret);
		return ret;
	}

	return 0;
}

static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(rtd->codec_dai->component);

	/* Add imx specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, imx_dapm_widgets,
									ARRAY_SIZE(imx_dapm_widgets));
	if (ret)
			goto out_retcode;

	/* Set up imx specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	if (ret)
			goto out_retcode;

	ret = snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	if (ret)
			goto out_retcode;

	ret = snd_soc_dapm_nc_pin(dapm, "Ext Spk");
	if (ret)
			goto out_retcode;

out_retcode:

	if (ret)
			pr_err("%s: failed with error code: %d \n", __FUNCTION__, ret);
	else
			pr_info("%s: success \n", __FUNCTION__);

	return ret;
}

/**
 * Configure AUDMUX interconnection between
 * _slave (CPU side) and _master (codec size)
 *
 * When SSI operates in master mode, 5-wire interconnect with
 * audio codec is required:
 * TXC  - BCLK
 * TXD  - DAC data
 * RXD  - ADC data
 * TXFS - {DAC|ADC}LRC, i.e. word clock
 * RXC  - MCLK, i.e. oversampling clock
 * Audmux is operated in asynchronous mode to enable 6-wire
 * interface (as opposed to 4-wire interface in sync mode).
 */
static int imx_audmux_config_slv_mode(int _slave, int _master)
{
	unsigned int ptcr, pdcr;
	int slave = _slave - 1;
	int master = _master - 1;

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
		IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(slave) |
		IMX_AUDMUX_V2_PTCR_RCLKDIR |
		IMX_AUDMUX_V2_PTCR_RCSEL(slave | 0x8) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(slave);

	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);
	imx_audmux_v2_configure_port(master, ptcr, pdcr);
	ptcr = ptcr & ~IMX_AUDMUX_V2_PTCR_SYN;
	imx_audmux_v2_configure_port(master, ptcr, pdcr);

	ptcr = IMX_AUDMUX_V2_PTCR_SYN |
		IMX_AUDMUX_V2_PTCR_RCLKDIR |
		IMX_AUDMUX_V2_PTCR_RCSEL(master | 0x8) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(master);

	pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);
	imx_audmux_v2_configure_port(slave, ptcr, pdcr);
	ptcr = ptcr & ~IMX_AUDMUX_V2_PTCR_SYN;
	imx_audmux_v2_configure_port(slave, ptcr, pdcr);

	return 0;
}


static int imx_wm8731_probe(struct platform_device *pdev)
{
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_wm8731_data *data;
	unsigned int src_port, ext_port;

	int ret;
	u32 out_val[3];
	phandle phandle;
	struct regmap *gpr;
	struct device_node *node;

	priv->pdev = pdev;

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	do {
		ret = of_property_read_u32_array(pdev->dev.of_node, "gpr", out_val, 3);
		if (ret) break;

		phandle = *out_val;
		node = of_find_node_by_phandle(phandle);
		if (!node) break;

		gpr = syscon_node_to_regmap(node);
		if (IS_ERR(gpr)) break;

		of_node_put(node);

		/*
		 * set SAI2_MCLK_DIR to enable codec MCLK
		 * out_val[1] is the register offset
		 * out_val[2] is the bit field
		 */
		regmap_update_bits(gpr, out_val[1], (1<<out_val[2]), (1<<out_val[2]));


	} while(0);

	card_priv.data = data;

	data->codec_dev = codec_dev;

	data->codec_clk = devm_clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		dev_err(&codec_dev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;	
	}

//	data->clk_frequency = clk_get_rate(data->codec_clk);	
	ret = clk_prepare_enable(data->codec_clk);

	if (ret) {
		dev_err(&codec_dev->dev, "failed to enable codec clk: %d\n", ret);
		goto fail;
	}

	data->mclk_parent_clk = devm_clk_get(&codec_dev->dev, "mclk_parent");
	if (IS_ERR(data->mclk_parent_clk)) {
		data->mclk_parent_clk = NULL;
	}

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "wm8731-hifi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->dai_name = dev_name(&ssi_pdev->dev);
	data->dai.platforms->of_node = ssi_np;
	data->dai.ops = &imx_hifi_ops;
	data->dai.init = &imx_wm8731_init;

	ret = of_property_read_u32(pdev->dev.of_node, "src-port", &src_port);
	if (ret) {
		dev_err(&pdev->dev, "failed to get \"src-port\" value\n");
		ret = -EINVAL;
		goto clk_fail;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "failed to get \"ext-port\" value\n");
		ret = -EINVAL;
		goto clk_fail;
	}

	imx_audmux_config_slv_mode(src_port, ext_port);

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto clk_fail;

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto clk_fail;


	data->card.num_links = 1;
	data->card.dai_link = &data->dai;

	data->card.dapm_widgets = imx_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto clk_fail;

	}

	return 0;

clk_fail:
	clk_disable_unprepare(data->codec_clk);

fail:

	if (ssi_np)
		of_node_put(ssi_np);

	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_wm8731_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_wm8731_data *data = snd_soc_card_get_drvdata(card);
 
	if (!IS_ERR(data->codec_clk))
		clk_disable_unprepare(data->codec_clk);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_wm8731_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8731", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8731_dt_ids);

static struct platform_driver imx_wm8731_driver = {
	.driver = {
		.name = "imx-wm8731",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8731_dt_ids,
	},
	.probe = imx_wm8731_probe,
	.remove = imx_wm8731_remove,
};
module_platform_driver(imx_wm8731_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8731 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8731");

 
