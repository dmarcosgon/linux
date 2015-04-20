/*
 *  Copyright (C) 2015, Grupo de Radiacion
 *	Author: David Marcos <dmarcosgon@gr.ssr.upm.es>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>


static const struct snd_soc_dapm_widget te0720_pcm1754_widgets[] = {
	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_OUTPUT("OUTR"),
};

static const struct snd_soc_dapm_route te0720_pcm1754_routes[] = {
	{ "OUTL", NULL, "Playback" },
	{ "OUTR", NULL, "Playback" },
};


static struct snd_soc_dai_link te0720_dai_pcm1754 = {
	.name		= "PCM1754", 
	.stream_name	= "Playback",
	.codec_dai_name	= "pcm1754-hifi", 
	.dai_fmt 	= (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),
};


static struct snd_soc_card te0720_pcm1754_card = {
	.name = "TE0720 PCM1754",
	.owner = THIS_MODULE,
	.dai_link = &te0720_dai_pcm1754,
	.num_links = 1,
	.dapm_widgets = te0720_pcm1754_widgets,
	.num_dapm_widgets = ARRAY_SIZE(te0720_pcm1754_widgets),
	.dapm_routes = te0720_pcm1754_routes,
	.num_dapm_routes = ARRAY_SIZE(te0720_pcm1754_routes),
	.fully_routed = true,
};

static int te0720_pcm1754_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &te0720_pcm1754_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	te0720_dai_pcm1754.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	te0720_dai_pcm1754.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	te0720_dai_pcm1754.platform_of_node = te0720_dai_pcm1754.cpu_of_node;

	if (!te0720_dai_pcm1754.codec_of_node ||
		!te0720_dai_pcm1754.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int te0720_pcm1754_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id te0720_pcm1754_of_match[] = {
	{ .compatible = "trenz,te0720_pcm1754", },
	{},
};
MODULE_DEVICE_TABLE(of, te0720_pcm1754_of_match);

static struct platform_driver te0720_pcm1754_card_driver = {
	.driver = {
		.name = "te0720-pcm1754-snd",
		.owner = THIS_MODULE,
		.of_match_table = te0720_pcm1754_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = te0720_pcm1754_probe,
	.remove = te0720_pcm1754_remove,
};
module_platform_driver(te0720_pcm1754_card_driver);

MODULE_DESCRIPTION("ASoC TE0720 PCM1754 driver");
MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:te0720-pcm1754-snd");
