/*
 * bias_tone.h:
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

#ifndef INCLUDE_BIAS_TONE_H_
#define INCLUDE_BIAS_TONE_H_
#include <alsa/asoundlib.h>

#define BIAS_PCM_STATE_OPEN          0x0001
#define BIAS_PCM_STATE_SETUP         0x0002
#define BIAS_PCM_STATE_PREPARED      0x0004
#define BIAS_PCM_STATE_RUNNING       0x0008
#define BIAS_PCM_STATE_XRUN          0x0010
#define BIAS_PCM_STATE_DRAINING      0x0020
#define BIAS_PCM_STATE_PAUSED        0x0040
#define BIAS_PCM_STATE_SUSPENDED     0x0080
#define BIAS_PCM_STATE_DISCONNECTED  0x0100

#define BIAS_THREAD_WAIT_SEC              1 // Check the bias loop status every second.
int initialize_bias_tone(void);
void shutdown_bias_tone(void);
int set_mixer_params(void);
int set_rox_bias(void);

#endif /* INCLUDE_BIAS_TONE_H_ */
