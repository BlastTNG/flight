/*
 * bias_tone.c:
 *
 * This software is copyright
 *  (C) 2013-2015 University of Pennsylvania
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: Nov 25, 2015 by vagrant
 */
#include <stdio.h>
#include <string.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <alsa/asoundlib.h>
#include "phenom/job.h"
#include "phenom/log.h"
#include "phenom/sysutil.h"

#include "bias_tone.h"
#include "blast.h"
#include "blast_time.h"
#include "command_struct.h"
#include "mcp.h"

void nameThread(const char*);		/* mcp.c */

static char *device = "plughw:0,0";                     /* playback device */
static char *card = "default";                          /* sound card for mixer */
static char *selem_name = "Master";
static snd_pcm_format_t format = SND_PCM_FORMAT_S16;    /* sample format */
static unsigned int rate = 44100;                       /* stream rate */
static unsigned int channels = 2;                       /* count of channels */
static unsigned int buffer_time = 500000;               /* ring buffer length in us */
static unsigned int period_time = 100000;               /* period time in us */
static double freq = 200;                                /* sinusoidal wave frequency in Hz */
static int resample = 1;                                /* enable alsa-lib resampling */
static snd_pcm_t *handle = NULL;
static snd_mixer_t *handle_mx = NULL;

static snd_pcm_sframes_t buffer_size;
static snd_pcm_sframes_t period_size;
static snd_output_t *output = NULL;

struct async_private_data {
        int16_t *samples;
        snd_pcm_channel_area_t *areas;
        double phase;
};
static struct async_private_data data;
static snd_async_handler_t *ahandler;
int bias_tone_shutting_down = 0;

#define BIAS_WAIT_BEFORE_RECONNECTING 10000000

static void generate_sine(const snd_pcm_channel_area_t *areas,
                          snd_pcm_uframes_t offset,
                          int count, double *_phase)
{
        static double max_phase = 2. * M_PI;
        double phase = *_phase;
        double step = max_phase*freq/(double)rate;
        unsigned char *samples[channels];
        int steps[channels];
        unsigned int chn;
        int format_bits = snd_pcm_format_width(format);
        unsigned int maxval = (1 << (format_bits - 1)) - 1;
        int bps = format_bits / 8;  /* bytes per sample */

        int to_unsigned = snd_pcm_format_unsigned(format) == 1;
        int is_float = (format == SND_PCM_FORMAT_FLOAT_LE ||
                        format == SND_PCM_FORMAT_FLOAT_BE);
        /* verify and prepare the contents of areas */
        for (chn = 0; chn < channels; chn++) {
                if ((areas[chn].first % 8) != 0) {
                        blast_err("areas[%i].first == %i, aborting...", chn, areas[chn].first);
                        exit(EXIT_FAILURE);
                }
                samples[chn] = /*(signed short *)*/(((unsigned char *)areas[chn].addr) + (areas[chn].first / 8));
                if ((areas[chn].step % 16) != 0) {
                        blast_err("areas[%i].step == %i, aborting...", chn, areas[chn].step);
                        exit(EXIT_FAILURE);
                }
                steps[chn] = areas[chn].step / 8;
                samples[chn] += offset * steps[chn];
        }
        /* fill the channel areas */
        while (count-- > 0) {
                union {
                        float f;
                        int i;
                } fval;
                int res, i;
                if (is_float) {
                        fval.f = sin(phase);
                        res = fval.i;
                } else {
                        res = sin(phase) * maxval;
                }
                if (to_unsigned)
                        res ^= 1U << (format_bits - 1);
                for (chn = 0; chn < channels; chn++) {
                        for (i = 0; i < bps; i++) {
                                *(samples[chn] + i) = (res >>  i * 8) & 0xff;
                        }
                        samples[chn] += steps[chn];
                        res = -res;
                }
                phase += step;
                if (phase >= max_phase)
                        phase -= max_phase;
        }
        *_phase = phase;
}

int set_mixer_params(void)
{
    int64_t min, max;
    snd_mixer_selem_id_t *sid;
    int retval = 0;

    snd_mixer_selem_id_alloca(&sid);

    snd_mixer_selem_id_set_index(sid, 0);
    if (retval < 0) {
        blast_err("Selem id set failed: %s", snd_strerror(retval));
        return retval;
    }

    snd_mixer_selem_id_set_name(sid, selem_name);
    if (retval < 0) {
        blast_err("Selem id set name failed: %s", snd_strerror(retval));
        return retval;
    }

    snd_mixer_elem_t* elem = snd_mixer_find_selem(handle_mx, sid);
    if (retval < 0) {
        blast_err("Find Selem failed: %s", snd_strerror(retval));
        return retval;
    }

    retval = snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
    if (retval < 0) {
        blast_err("Get playback volume failed: %s", snd_strerror(retval));
        return retval;
    } else {
        blast_info("Mixer playback volume max = %li, min = %li", max, min);
    }

    retval = snd_mixer_selem_set_playback_volume_all(elem, CommandData.rox_bias.amp);
    if (retval < 0) {
        blast_err("Find Selem failed: %s", snd_strerror(retval));
        return retval;
    } else {
        blast_info("Mixer volume set to %i", CommandData.rox_bias.amp);
    }
    return 0;
}
static int set_hwparams(snd_pcm_t *handle)
{
    unsigned int rrate;
    snd_pcm_uframes_t size;
    int retval, dir = 0;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    /* choose all parameters */
    retval = snd_pcm_hw_params_any(handle, hw_params);
    if (retval < 0) {
        blast_err("Broken configuration for playback: no configurations available: %s", snd_strerror(retval));
        return retval;
    }
    /* set hardware resampling */
    retval = snd_pcm_hw_params_set_rate_resample(handle, hw_params, resample);
    if (retval < 0) {
        blast_err("Resampling setup failed for playback: %s", snd_strerror(retval));
        return retval;
    }
    /* set the interleaved read/write format */
    retval = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (retval < 0) {
        blast_err("Access type not available for playback: %s", snd_strerror(retval));
        return retval;
    }
    /* set the sample format */
    retval = snd_pcm_hw_params_set_format(handle, hw_params, format);
    if (retval < 0) {
        blast_err("Sample format not available for playback: %s", snd_strerror(retval));
        return retval;
    }
    /* set the count of channels */
    retval = snd_pcm_hw_params_set_channels(handle, hw_params, channels);
    if (retval < 0) {
        blast_err("Channels count (%i) not available for playbacks: %s", channels, snd_strerror(retval));
        return retval;
    }
    /* set the stream rate */
    rrate = rate;
    retval = snd_pcm_hw_params_set_rate_near(handle, hw_params, &rrate, 0);
    if (retval < 0) {
        blast_err("Rate %iHz not available for playback: %s", rate, snd_strerror(retval));
        return retval;
    }
    if (rrate != rate) {
        blast_err("Rate doesn't match (requested %iHz, get %iHz)", rate, retval);
        return -EINVAL;
    }
    /* set the buffer time */
    retval = snd_pcm_hw_params_set_buffer_time_near(handle, hw_params, &buffer_time, &dir);
    if (retval < 0) {
        blast_err("Unable to set buffer time %i for playback: %s", buffer_time, snd_strerror(retval));
        return retval;
    }
    retval = snd_pcm_hw_params_get_buffer_size(hw_params, &size);
    if (retval < 0) {
        blast_err("Unable to get buffer size for playback: %s", snd_strerror(retval));
        return retval;
    }
    buffer_size = size;
    /* set the period time */
    retval = snd_pcm_hw_params_set_period_time_near(handle, hw_params, &period_time, &dir);
    if (retval < 0) {
        blast_err("Unable to set period time %i for playback: %s", period_time, snd_strerror(retval));
        return retval;
    }
    retval = snd_pcm_hw_params_get_period_size(hw_params, &size, &dir);
    if (retval < 0) {
        blast_err("Unable to get period size for playback: %s", snd_strerror(retval));
        return retval;
    }
    period_size = size;
    /* write the parameters to device */
    retval = snd_pcm_hw_params(handle, hw_params);
    if (retval < 0) {
        blast_err("Unable to set hw params for playback: %s", snd_strerror(retval));
        return retval;
    }
    return 0;
}
static int set_swparams(snd_pcm_t *handle)
{
    snd_pcm_sw_params_t *swparams;
    int retval;

    snd_pcm_sw_params_alloca(&swparams);
    /* get the current swparams */
    retval = snd_pcm_sw_params_current(handle, swparams);
    if (retval < 0) {
        blast_err("Unable to determine current swparams for playback: %s", snd_strerror(retval));
        return retval;
    }
    /* start the transfer when the buffer is almost full:  (buffer_size / avail_min) * avail_min */
    retval = snd_pcm_sw_params_set_start_threshold(handle, swparams, (buffer_size / period_size) * period_size);
    if (retval < 0) {
        blast_err("Unable to set start threshold mode for playback: %s", snd_strerror(retval));
        return retval;
    }
    /* allow the transfer when at least period_size samples can be processed */
    retval = snd_pcm_sw_params_set_avail_min(handle, swparams, buffer_size);
    if (retval < 0) {
        blast_err("Unable to set avail min for playback: %s", snd_strerror(retval));
        return retval;
    }

    /* write the parameters to the playback device */
    retval = snd_pcm_sw_params(handle, swparams);
    if (retval < 0) {
        blast_err("Unable to set sw params for playback: %s", snd_strerror(retval));
        return retval;
    }
    return 0;
}

static void bias_tone_callback(snd_async_handler_t *ahandler)
{
    snd_pcm_t *handle = snd_async_handler_get_pcm(ahandler);
    struct async_private_data *data = snd_async_handler_get_callback_private(ahandler);
    int16_t *samples = data->samples;
    snd_pcm_channel_area_t *areas = data->areas;
    snd_pcm_sframes_t avail;
    int retval;

    avail = snd_pcm_avail_update(handle);
    while (avail >= period_size) {
        generate_sine(areas, 0, period_size, &data->phase);
        retval = snd_pcm_writei(handle, samples, period_size);
        if (retval < 0) {
            blast_err("Write error: %s", snd_strerror(retval));
            return;
        }
        if (retval != period_size) {
            blast_err("Write error: written %i expected %li", retval, period_size);
        }
        avail = snd_pcm_avail_update(handle);
    }
}

int reset_rox_bias(snd_pcm_t* handle) {
    shutdown_bias_tone();
// Commenting this out for now because I need to finish writing the function.
//    usleep(BIAS_WAIT_BEFORE_RECONNECTING);
// TODO(laura): Figure out a way to monitor the whether the bias has actually shut down.
}

void *bias_monitor(void *param)
{
	snd_pcm_state_t state;
	snd_pcm_status_t* 	status;
	int have_warned, have_warned_status = 0;
    int ret = 0;
    int err;
    int first_time = 1;
    uint16_t bias_state = 0;
    uint8_t reset_counter = 0;
    static channel_t *bias_alsa_state_rox_channel;
	char *channel_name = "bias_alsa_state_rox";
    bias_alsa_state_rox_channel = channels_find_by_name(channel_name);
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 1,
                                    .tv_nsec = 0}; /// 1HZ interval

    nameThread("BiasMonitor");
    blast_info("BiasMonitor thread startup.");
    clock_gettime(CLOCK_REALTIME, &ts);
    snd_pcm_status_alloca(&status);

    while (!bias_tone_shutting_down) {
        bias_state = 0;
        state = snd_pcm_state(handle);
        if (first_time) {
            blast_info("Attempting to read bias pcm state.");
        }
        switch (state) {
            // The PCM device is in the open state.
            // After the snd_pcm_open() open call, the device is in this state.
            case SND_PCM_STATE_OPEN:
               bias_state = BIAS_PCM_STATE_OPEN;
               have_warned = 0;
               break;
            case SND_PCM_STATE_SETUP:
               bias_state = BIAS_PCM_STATE_SETUP;
               have_warned = 0;
               break;
            case SND_PCM_STATE_PREPARED:
               bias_state = BIAS_PCM_STATE_PREPARED;
               have_warned = 0;
               break;
            case SND_PCM_STATE_RUNNING:
               bias_state = BIAS_PCM_STATE_RUNNING;
               have_warned = 0;
               break;
            case SND_PCM_STATE_XRUN:
               bias_state = BIAS_PCM_STATE_XRUN;
               break;
            case SND_PCM_STATE_DRAINING:
               bias_state = BIAS_PCM_STATE_DRAINING;
               have_warned = 0;
               break;
            case SND_PCM_STATE_PAUSED:
               bias_state = BIAS_PCM_STATE_PAUSED;
               if (!have_warned) blast_warn("bias pcm state is paused.");
               have_warned = 1;
               break;
            case SND_PCM_STATE_SUSPENDED:
               bias_state = BIAS_PCM_STATE_SUSPENDED;
               if (!have_warned) blast_err("bias pcm state is suspended!");
               have_warned = 1;
               break;
            case SND_PCM_STATE_DISCONNECTED:
               blast_err("Steam status error received: %s",  snd_strerror(err));
               bias_state = BIAS_PCM_STATE_DISCONNECTED;
               if (!have_warned) blast_warn("bias pcm state is disconnected!");
               have_warned = 1;
               break;
            default:
               blast_err("Could not parse the snd_pcm_state return value: %s",  snd_strerror(err));
               have_warned = 1;
        }
        if (first_time) {
            blast_info("bias_state = %i.", bias_state);
            first_time = 0;
        }
        if (((err = snd_pcm_status(handle, status)) < 0) && !have_warned_status) {
                blast_info("Stream status error: %s\n", snd_strerror(err));
                have_warned_status = 1;
        }

        SET_SCALED_VALUE(bias_alsa_state_rox_channel, bias_state);

        if (CommandData.rox_bias.reset) {
            ret = reset_rox_bias(handle);
        }
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (ret && ret != -EINTR) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }
    }
}

void shutdown_bias_tone(void)
{
    bias_tone_shutting_down = 1;
    BLAST_SAFE_FREE(data.areas);
    BLAST_SAFE_FREE(data.samples);
    if (handle) snd_pcm_close(handle);
    handle = NULL;
}

int initialize_bias_tone(void)
{
    int retval;
    ph_thread_t *bias_thread = NULL;

    retval = snd_output_stdio_attach(&output, stdout, 0);
    if (retval < 0) {
        blast_err("Output failed: %s", snd_strerror(retval));
        return -1;
    }
    blast_startup("Playback device is %s", device);
    blast_startup("Sine wave rate is %.4fHz", freq);
    if ((retval = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        blast_err("Playback open error: %s", snd_strerror(retval));
        goto init_err;
    }

    if ((retval = set_hwparams(handle)) < 0) {
        blast_err("Setting of sound hardware params failed: %s", snd_strerror(retval));
        goto init_err;
    }
    if ((retval = set_swparams(handle)) < 0) {
        blast_err("Setting of software params failed: %s", snd_strerror(retval));
        goto init_err;
    }

    if (!(data.samples = balloc(err, (period_size * channels * snd_pcm_format_physical_width(format)) / 8))) {
        blast_err("Not enough memory");
        goto init_err;
    }

    if (!(data.areas = calloc(channels, sizeof(snd_pcm_channel_area_t)))) {
        blast_err("Not enough memory");
        goto init_err;
    }
    for (int chn = 0; chn < channels; chn++) {
        data.areas[chn].addr = data.samples;
        data.areas[chn].first = chn * snd_pcm_format_physical_width(format);
        data.areas[chn].step = channels * snd_pcm_format_physical_width(format);
    }

    data.phase = 0;
    retval = snd_async_add_pcm_handler(&ahandler, handle, bias_tone_callback, &data);
    if (retval < 0) {
        blast_err("Unable to register async handler");
        goto init_err;
    }
    blast_startup("Registered ASync sound handler");

    for (int count = 0; count < 2; count++) {
        generate_sine(data.areas, 0, period_size, &data.phase);
        retval = snd_pcm_writei(handle, data.samples, period_size);
        if (retval < 0) {
            blast_err("Initial write error: %s", snd_strerror(retval));
            goto init_err;
        }
        if (retval != period_size) {
            blast_err("Initial write error: written %i expected %li", retval, period_size);
            goto init_err;
        }
    }
    blast_startup("Generated Sine Wave");

    if (snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) {
        retval = snd_pcm_start(handle);
        if (retval < 0) {
            blast_err("Start error: %s", snd_strerror(retval));
            goto init_err;
        }
    }
    blast_startup("Started Sine Wave");

    if ((retval = snd_mixer_open(&handle_mx, 0)) < 0) {
        blast_err("Mixer open error: %s", snd_strerror(retval));
        goto init_err;
    }
    blast_startup("Opened Mixer");

    if ((retval = snd_mixer_attach(handle_mx, card)) < 0) {
        blast_err("Mixer attach error: %s", snd_strerror(retval));
        goto init_err;
    }
    blast_startup("Mixer attached");

    if ((retval = snd_mixer_selem_register(handle_mx, NULL, NULL)) < 0) {
        blast_err("Mixer selem register error: %s", snd_strerror(retval));
        goto init_err;
    }
    blast_startup("Mixer Selem register set");

    if ((retval = snd_mixer_load(handle_mx)) < 0) {
        blast_err("Mixer load error: %s", snd_strerror(retval));
        goto init_err;
    }
    blast_startup("Mixer loaded");

    if ((retval = set_mixer_params()) < 0) {
        blast_err("Could not send mixer parameters.: %s", snd_strerror(retval));
        goto init_err;
    }
    blast_startup("Mixer parameters sent!");

    bias_thread = ph_thread_spawn(bias_monitor, NULL);

    return 0;

init_err:
    blast_warn("Initialization error!");
    shutdown_bias_tone();
    return -1;
}

/* 
Bias signal monitoring thread.
void *bias_monitor_thread(void *m_arg)
{
    
}
 */
int set_rox_bias() {
    int retval;
    if ((retval = set_mixer_params()) < 0) {
        blast_err("Could not send mixer parameters.: %s", snd_strerror(retval));
        return -1;
    }
    blast_startup("Mixer parameters re-sent!");

    return 0;
}
