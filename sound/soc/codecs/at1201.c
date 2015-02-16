/*
 * AT1201 ASoC codec driver
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

#define DRV_NAME "AT1201"
#define RATES	SNDRV_PCM_RATE_8000_192000
#define FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE| SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dapm_widget at1201_widgets[] = {
	SND_SOC_DAPM_INPUT("INL+"),
	SND_SOC_DAPM_INPUT("INL-"),
	SND_SOC_DAPM_INPUT("INR+"),
	SND_SOC_DAPM_INPUT("INR-"),
};

static const struct snd_soc_dapm_route at1201_routes[] = {
	{ "Capture", NULL, "INL+" },
	{ "Capture", NULL, "INL-" },
	{ "Capture", NULL, "INR+" },
	{ "Capture", NULL, "INR-" },
};


static struct snd_soc_codec_driver soc_codec_at1201_dit = {
	.dapm_widgets		= at1201_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(at1201_widgets),
	.dapm_routes		= at1201_routes,
	.num_dapm_routes	= ARRAY_SIZE(at1201_routes),
};

static struct snd_soc_dai_driver at1201_dai = {
	.name		= "at1201-hifi",
	.capture 	= {
		.stream_name	= "Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= RATES,
		.formats	= FORMATS,
	},
};
static int at1201_dit_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_at1201_dit,
			&at1201_dai, 1);

}

static int at1201_dit_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id at1201_dt_ids[] = {
	{ .compatible = "arda,at1201", },
	{ }
};
MODULE_DEVICE_TABLE(of, at1201_dt_ids);
#endif

static struct platform_driver at1201_dit_driver = {
	.probe		= at1201_dit_probe,
	.remove		= at1201_dit_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(at1201_dt_ids),
	},
};

module_platform_driver(at1201_dit_driver);
MODULE_AUTHOR("David Marcos");
MODULE_DESCRIPTION("AT1201 codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
