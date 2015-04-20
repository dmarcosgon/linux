/*
 * PCM1754 ASoC codec driver
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

#define DRV_NAME "pcm1754"
#define RATES	SNDRV_PCM_RATE_8000_192000
#define FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE|SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dapm_widget pcm1754_widgets[] = {
    SND_SOC_DAPM_OUTPUT("OUTL"),
    SND_SOC_DAPM_OUTPUT("OUTR"),
};

static const struct snd_soc_dapm_route pcm1754_routes[] = {
    { "OUTL", NULL, "Playback"},
    { "OUTR", NULL, "Playback"},
};

struct pcm1754_private {
    unsigned int format;
    unsigned int rate;
};


static struct snd_soc_codec_driver soc_codec_pcm1754_dit = {
    .dapm_widgets = pcm1754_widgets,
    .num_dapm_widgets = ARRAY_SIZE(pcm1754_widgets),
    .dapm_routes = pcm1754_routes,
    .num_dapm_routes = ARRAY_SIZE(pcm1754_routes),
};

static struct snd_soc_dai_driver pcm1754_dai = {
    .name = "pcm1754-hifi",
    .playback =
    {
        .stream_name = "Playback",
        .channels_min = 1,
        .channels_max = 2,
        .rates = RATES,
        .formats = FORMATS,
    },
};

static int pcm1754_dit_probe(struct platform_device *pdev) {
    return snd_soc_register_codec(&pdev->dev, &soc_codec_pcm1754_dit,
            &pcm1754_dai, 1);

}

static int pcm1754_dit_remove(struct platform_device *pdev) {
    snd_soc_unregister_codec(&pdev->dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id pcm1754_dt_ids[] = {
    { .compatible = "ti,pcm1754",},
    {}
};
MODULE_DEVICE_TABLE(of, pcm1754_dt_ids);
#endif

static struct platform_driver pcm1754_dit_driver = {

    .probe = pcm1754_dit_probe,
    .remove = pcm1754_dit_remove,
    .driver =
    {
        .name = DRV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(pcm1754_dt_ids),
    },

};

module_platform_driver(pcm1754_dit_driver);
MODULE_AUTHOR("David Marcos");
MODULE_DESCRIPTION("PCM1754 codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
