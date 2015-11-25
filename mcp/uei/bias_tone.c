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
#include <ao/ao.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "blast.h"
#include "blast_time.h"

#define BUF_SIZE 4096

static ao_device *device;
static ao_sample_format format;

char *buffer;
static bool ao_closing = false;

int initialize_ao_driver(void)
{
	int sample;
	float freq = 200.0;
	int i;
	int default_driver;
	int buf_size;

	ao_initialize();
	default_driver = ao_default_driver_id();

    memset(&format, 0, sizeof(format));
	format.bits = 16;
	format.channels = 2;
	format.rate = 44100;
	format.byte_format = AO_FMT_LITTLE;

	/* -- Open driver -- */
	device = ao_open_live(default_driver, &format, NULL);
	if (device == NULL) {
		fprintf(stderr, "Error opening device.\n");
		return -1;
	}

	/**
	 * Buffer 1 second worth of Sine wave data
	 */
	buf_size = (int) format.bits/8 * format.channels * format.rate;
	buffer = calloc(buf_size,
			sizeof(char));

	for (i = 0; i < format.rate; i++) {
		sample = (int)(0.55 * INT16_MAX *
			sin(2 * M_PI * freq * ((float) i/format.rate)));

		/**
		 * Technically we don't need both channels but output them anyway
		 */
		buffer[4*i] = buffer[4*i+2] = sample & 0xff;
		buffer[4*i+1] = buffer[4*i+3] = (sample >> 8) & 0xff;
	}
	return 0;
}

void *ao_play_sine_wave(void *arg)
{

    static struct timespec prev_ts = {0,0};
    static int buffer_offset = 0;
    struct timespec ts;
    double delta_t;
    int buffer_size = (int) format.bits/8 * format.channels * format.rate;

    blast_dbg("Playing Sine wave on %p", device);
    if (!device) return NULL;

    while (!ao_closing) {
    clock_gettime(CLOCK_REALTIME, &ts);

    if (buffer_offset >= buffer_size) buffer_offset = 0;

    delta_t = ts.tv_sec - prev_ts.tv_sec + (double) (ts.tv_nsec - prev_ts.tv_nsec)/ (NSEC_PER_SEC);
    /**
     * First buffer any whole multiple of seconds of audio data, wrapping the buffer as needed
     */
    if (delta_t > 2.0) delta_t = 2.0;
    while (delta_t > 1.0) {
    	ao_play(device, buffer + buffer_offset, buffer_size - buffer_offset);
    	if (buffer_offset) ao_play(device, buffer, buffer_offset);

    	delta_t -= 1.0;
    }

    /**
     * After wrapping the buffer, play the fraction of a whole second of data remaining, filling the
     * audio buffer as much as we can.
     */
    if (delta_t > 1.0 / (double)format.rate) {
    	int samples_to_play = (int)(delta_t * buffer_size);
    	int samples_to_end = min(samples_to_play, buffer_size - buffer_offset);
    	ao_play(device, buffer + buffer_offset, samples_to_end);
    	samples_to_play -= samples_to_end;
    	if (samples_to_play) {
    		buffer_offset = 0;
    		ao_play(device, buffer, samples_to_play);
    	}
    	buffer_offset += samples_to_play;
    }

	prev_ts.tv_sec = ts.tv_sec;
	prev_ts.tv_nsec = ts.tv_nsec;
	clock_gettime(CLOCK_REALTIME, &ts);
	blast_info("Took %f seconds to execute", ts.tv_sec - prev_ts.tv_sec + (double)(ts.tv_nsec - prev_ts.tv_nsec)/(NSEC_PER_SEC));
    }

    return NULL;
}

void shutdown_ao_driver(void)
{
	ao_closing=true;
	ao_close(device);
	ao_shutdown();
}
