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
#include <portaudio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "blast.h"
#include "blast_time.h"

#define SAMPLE_RATE   (44100)
#define FRAMES_PER_BUFFER  (512)

#define TABLE_SIZE   (441) /* 2 x 200Hz cycles at 44100Hz sample rate */

static float sine[TABLE_SIZE];
static int sine_index = 0;

static bool ao_closing = false;
static PaStream *bias_stream = NULL;

static int bias_tone_callback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData )
{

    float *out = (float*)outputBuffer;
    unsigned long i;

    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) inputBuffer;

    for( i=0; i<framesPerBuffer; i++ )
    {
        *out++ = sine[sine_index];  /* left */
        *out++ = sine[sine_index];  /* right */

        if (++ sine_index == TABLE_SIZE) sine_index = 0;
    }

    if (ao_closing) return paAbort;

    return paContinue;
}

int initialize_bias_tone(void)
{
    PaStreamParameters outputParameters;

	int retval;

    /* initialise sinusoidal wavetable */
    for(int i=0; i<TABLE_SIZE; i++ )
    {
        sine[i] = (float) sin( ((double)i/(double)TABLE_SIZE) * M_PI * 4.0); // There are two full waves in the TABLE_SIZE
    }

    retval = Pa_Initialize();
    if( retval != paNoError ) goto init_err;

    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters.device == paNoDevice) {
      fprintf(stderr,"Error: No default output device.\n");
      goto init_err;
    }
    outputParameters.channelCount = 2;       /* stereo output */
    outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultHighOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    retval = Pa_OpenStream(
              &bias_stream,
              NULL, /* no input */
              &outputParameters,
              SAMPLE_RATE,
              FRAMES_PER_BUFFER,
              paClipOff,      /* we won't output out of range samples so don't bother clipping them */
              bias_tone_callback,
              NULL );
    if( retval != paNoError ) goto init_err;

    retval = Pa_StartStream( bias_stream );
    if( retval != paNoError ) goto init_err;

    return retval;

init_err:
    Pa_Terminate();
    blast_err("An error occurred while initializing the portaudio stream: %s", Pa_GetErrorText( retval ) );
    return retval;
}


void shutdown_bias_tone(void)
{
	ao_closing=true;
	Pa_AbortStream(bias_stream);
    Pa_Terminate();

}
