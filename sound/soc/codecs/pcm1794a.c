/*
 * PCM1794A ASoC codec driver
 *
 * Copyright (c) Grupo de Radiacion UPM 2015
 *
 *     David Marcos <dmarcosgon@gr.ssr.upm.es>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>

#define DRV_NAME "pcm1794a"
#define RATES	SNDRV_PCM_RATE_8000_192000
#define FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE| SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dapm_widget pcm1794a_widgets[] = {
	SND_SOC_DAPM_OUTPUT("IOUTL+"),
	SND_SOC_DAPM_OUTPUT("IOUTL-"),
	SND_SOC_DAPM_OUTPUT("IOUTR+"),
	SND_SOC_DAPM_OUTPUT("IOUTR-"),
};

static const struct snd_soc_dapm_route pcm1794a_routes[] = {
	{ "IOUTL+", NULL, "Playback" },
	{ "IOUTL-", NULL, "Playback" },
	{ "IOUTR+", NULL, "Playback" },
	{ "IOUTR-", NULL, "Playback" },
};

struct pcm1794a_private {
	unsigned int format;
	unsigned int rate;
};


static struct snd_soc_codec_driver soc_codec_pcm1794a_dit = {
	.dapm_widgets		= pcm1794a_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(pcm1794a_widgets),
	.dapm_routes		= pcm1794a_routes,
	.num_dapm_routes	= ARRAY_SIZE(pcm1794a_routes),
};

static struct snd_soc_dai_driver pcm1794a_dai = {
	.name		= "pcm1794a-hifi",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RATES,
		.formats	= FORMATS,
	},
};
static int pcm1794a_dit_probe(struct platform_device *pdev)

{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_pcm1794a_dit,
			&pcm1794a_dai, 1);

}

static int pcm1794a_dit_remove(struct platform_device *pdev)

{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id pcm1794a_dt_ids[] = {
	{ .compatible = "ti,pcm1794a", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm1794a_dt_ids);
#endif

static struct platform_driver pcm1794a_dit_driver = {

	.probe		= pcm1794a_dit_probe,
	.remove		= pcm1794a_dit_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(pcm1794a_dt_ids),
	},

};

module_platform_driver(pcm1794a_dit_driver);
MODULE_AUTHOR("David Marcos");
MODULE_DESCRIPTION("PCM1794A codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
