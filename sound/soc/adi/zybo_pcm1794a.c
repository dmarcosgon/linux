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


static const struct snd_soc_dapm_widget zybo_pcm1794a_widgets[] = {
	SND_SOC_DAPM_OUTPUT("IOUTL+"),
	SND_SOC_DAPM_OUTPUT("IOUTL-"),
	SND_SOC_DAPM_OUTPUT("IOUTR+"),
	SND_SOC_DAPM_OUTPUT("IOUTR-"),
};

static const struct snd_soc_dapm_route zybo_pcm1794a_routes[] = {
	{ "IOUTL+", NULL, "Playback" },
	{ "IOUTL-", NULL, "Playback" },
	{ "IOUTR+", NULL, "Playback" },
	{ "IOUTR-", NULL, "Playback" },
};


static struct snd_soc_dai_link zybo_dai_pcm1794a = {
	.name		= "PCM1794A", 
	.stream_name	= "Playback",
	.codec_dai_name	= "pcm1794a-hifi", 
	.dai_fmt 	= (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),
};


static struct snd_soc_card zybo_pcm1794a_card = {
	.name = "ZYBO PCM1794A",
	.owner = THIS_MODULE,
	.dai_link = &zybo_dai_pcm1794a,
	.num_links = 1,
	.dapm_widgets = zybo_pcm1794a_widgets,
	.num_dapm_widgets = ARRAY_SIZE(zybo_pcm1794a_widgets),
	.dapm_routes = zybo_pcm1794a_routes,
	.num_dapm_routes = ARRAY_SIZE(zybo_pcm1794a_routes),
	.fully_routed = true,
};

static int zybo_pcm1794a_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &zybo_pcm1794a_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	zybo_dai_pcm1794a.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	zybo_dai_pcm1794a.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	zybo_dai_pcm1794a.platform_of_node = zybo_dai_pcm1794a.cpu_of_node;

	if (!zybo_dai_pcm1794a.codec_of_node ||
		!zybo_dai_pcm1794a.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int zybo_pcm1794a_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id zybo_pcm1794a_of_match[] = {
	{ .compatible = "digilent,zybo_pcm1794a", },
	{},
};
MODULE_DEVICE_TABLE(of, zybo_pcm1794a_of_match);

static struct platform_driver zybo_pcm1794a_card_driver = {
	.driver = {
		.name = "zybo-pcm1794a-snd",
		.owner = THIS_MODULE,
		.of_match_table = zybo_pcm1794a_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = zybo_pcm1794a_probe,
	.remove = zybo_pcm1794a_remove,
};
module_platform_driver(zybo_pcm1794a_card_driver);

MODULE_DESCRIPTION("ASoC ZYBO PCM1794A driver");
MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:zybo-pcm1794a-snd");
