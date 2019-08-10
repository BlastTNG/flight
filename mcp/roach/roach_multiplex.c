/*
 * roach_multiplex.c
 *
 * This software is copyright (C) 2013-2014 University of Pennsylvania
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
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Last edited: August 2018
 *      Author: javier
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/statvfs.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <openssl/md5.h>

#include "blast.h"
#include "command_list.h"
#include "command_struct.h"
#include "derived.h"
#include "mcp.h"
#include "roach.h"

extern char * ROACH_TYPES[NUM_RTYPES];
extern roach_state_t roach_state_table[NUM_ROACHES]; /* NUM_ROACHES = 5 */

// distributes and multiplexes commanded roach channels to compressed telemetry fields

void add_roach_tlm_488hz()
{
  int i, j;

  static channel_t * tlm[NUM_ROACH_TLM] = {NULL};
  static channel_t * tlm_index[NUM_ROACH_TLM] = {NULL};
  static unsigned int prev_roach_index[NUM_ROACH_TLM] = {0};
  static int have_warned = 0;
  static int first_time = 1;

  roach_tlm_t * r_tlm = NULL;

  if (first_time) {
    for (i = 0; i < NUM_ROACH_TLM; i++) {
      char tlm_name[64] = {0};
      snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN", 65+i);
      tlm[i] = channels_find_by_name(tlm_name);
      snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN_index", 65+i);
      tlm_index[i] = channels_find_by_name(tlm_name);
    }
    for (i = 0; i < NUM_ROACHES; i++) {
      for (j = 0; j < MAX_CHANNELS_PER_ROACH; j++) {
        roach_df_telem[i][j].first_call = 1; // Tell mcp to initialize
      }
    }
    memset(prev_roach_index, 0xff, NUM_ROACH_TLM*sizeof(unsigned int));
    first_time = 0;
  }

  // -----------------------------------
  // Set multiplex for all channels here
  // -----------------------------------
  // Multiplex is set by command, where the commanded number of channels per roach will be sent
  // multiplexed at 488 Hz. If multiplex number is zero, then the channel is set to the roach,
  // kid, and rtype specified by roach channels command in CommandData.
  // -----------------------------------

  static unsigned int kid_counter[NUM_ROACHES] = {0};

  static unsigned int roach_tlm[NUM_ROACH_TLM] = {0};
  static unsigned int kid_tlm[NUM_ROACH_TLM] = {0};
  static unsigned int rtype_tlm[NUM_ROACH_TLM] = {0};

  for (int j = 0; j < NUM_ROACHES; j++) {
    int multiplexing = 0;

    if (CommandData.num_channels_all_roaches[j]) { // if non-zero, multiplex that number of roach channels
      // set kid and roach counters for I, Q, and df multiplex
      unsigned int wrap = MIN(CommandData.num_channels_all_roaches[j], MAX_CHANNELS_PER_ROACH);
      kid_counter[j] = (kid_counter[j]+1)%wrap;
      multiplexing = 1;
    }

		for (int i = 0; i < 3; i++) { // loop through I, Q, and df
			int tlm_ind = j*3+i;
			if (tlm_ind >= NUM_ROACH_TLM) continue;
			r_tlm = &(CommandData.roach_tlm[tlm_ind]);

      if (multiplexing) {
				// force multiplexing to assigned indices
				roach_tlm[tlm_ind] = j+1;
				rtype_tlm[tlm_ind] = i;
				kid_tlm[tlm_ind] = (r_tlm->kid+kid_counter[j])%MAX_CHANNELS_PER_ROACH;
      } else {
        // set the commanded values to those set by roach channels
				roach_tlm[tlm_ind] = r_tlm->roach;
				rtype_tlm[tlm_ind] = r_tlm->rtype;
				kid_tlm[tlm_ind] = r_tlm->kid;
      }

      // compute the roach index
			r_tlm->index = get_roach_index(roach_tlm[tlm_ind], kid_tlm[tlm_ind], rtype_tlm[tlm_ind]);
		}
  }

  // ------------------------------------------------
  // --------- Multiplexed Channel selection --------
  // ------------------------------------------------
  // Channels are selected by commands and multiplexed based on a unique roach channel Id.
  // Channels are identified in getdata/kst from an indexed string array.
  for (i = 0; i < NUM_ROACH_TLM; i++) {
    r_tlm = &CommandData.roach_tlm[i];
    int i_roach = roach_tlm[i]-1;

    prev_roach_index[i] = r_tlm->index;

    // invalid roach selection
    if (roach_tlm[i] > NUM_ROACHES) continue;

    unsigned int i_udp_read = GETREADINDEX(roach_udp[i_roach].index);
    data_udp_packet_t *m_packet = &(roach_udp[i_roach].last_pkts[i_udp_read]);

    // write the roach data to the multiplexed field
    int mode = CommandData.roach_tlm_mode;
    if (tlm[i]) {
      double value = -3.14159;
      if ((i_roach >= NUM_ROACHES) || (kid_tlm[i] >= MAX_CHANNELS_PER_ROACH)) {
          if (!have_warned) {
            blast_err("Indexing error: roach_index %d, ind_rtype %d conflict with NUM_ROACHES %d, NUM_RTYPES %d",
                    i_roach, kid_tlm[i], NUM_ROACHES, MAX_CHANNELS_PER_ROACH);
          }
          have_warned = 1;
      } else { // valid indices, so proceed
        if (strcmp(ROACH_TYPES[rtype_tlm[i]], "i") == 0) { // I comes from the UDP packet directly
          if (mode == ROACH_TLM_IQDF) value = m_packet->Ival[kid_tlm[i]];
          else if (mode == ROACH_TLM_DELTA) value = roach_state_table[i_roach].I_diff[kid_tlm[i]];

        } else if (strcmp(ROACH_TYPES[rtype_tlm[i]], "q") == 0) { // Q comes from the UDP packet directly
          if (mode == ROACH_TLM_IQDF) value = m_packet->Qval[kid_tlm[i]];
          else if (mode == ROACH_TLM_DELTA) value = roach_state_table[i_roach].Q_diff[kid_tlm[i]];

        } else if (strcmp(ROACH_TYPES[rtype_tlm[i]], "df") == 0) { // df comes from the frame
          if (CommandData.roach_tlm_mode == ROACH_TLM_IQDF) {
                value = roach_df_continuous(&(roach_df_telem[i_roach][kid_tlm[i]]),
                                   m_packet->Ival[kid_tlm[i]], m_packet->Qval[kid_tlm[i]],
                                   i_roach, kid_tlm[i]);
          } else if (mode == ROACH_TLM_DELTA) {
            value = roach_state_table[i_roach].df_diff[kid_tlm[i]];
          }
        }
        have_warned = 0;
      }

      SET_FLOAT(tlm[i], value);
    }
    // write the multiplex index
    if (tlm_index[i]) {
      SET_INT32(tlm_index[i], r_tlm->index);
    }
  }
}

