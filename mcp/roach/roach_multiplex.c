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

// distributes and multiplexes commanded roach channels to compressed telemetry fields

void add_roach_tlm_488hz()
{
  static channel_t * tlm[NUM_ROACH_TLM] = {NULL};
  static channel_t * tlm_index[NUM_ROACH_TLM] = {NULL};
  static int first_time = 1;
  static unsigned int RoachId[NUM_ROACH_TLM] = {0};
  static unsigned int KidId[NUM_ROACH_TLM] = {0};
  static unsigned int TypeId[NUM_ROACH_TLM] = {0};
  static unsigned int roach_indices[NUM_ROACH_TLM] = {0};
  static int have_warned = 0;
  int i;

  if (first_time) {
    for (i = 0; i < NUM_ROACH_TLM; i++) {
			char tlm_name[64] = {0};
			snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN", 65+i);
      tlm[i] = channels_find_by_name(tlm_name);
			snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN_index", 65+i);
      tlm_index[i] = channels_find_by_name(tlm_name);
    }
    for (i = 0; i < NUM_ROACHES; i++) roach_df_telem[i].first_call = 1; // Tell mcp to initialize
                                                                        // the roach_df_telem struct.
    memset(roach_indices, 0xff, NUM_ROACH_TLM*sizeof(unsigned int));
    first_time = 0;
  }

  // Set multiplex for all channels here
  // -----------------------------------

  for (int j = 0; j < NUM_ROACHES; j++) {
		if (CommandData.num_channels_all_roaches[j]) {
			for (int i = 0; i < 3; i++) {
        int tlm_ind = j*3+i;
        if (tlm_ind >= NUM_ROACH_TLM) continue;
				// set kid and roach counters for I, Q, and df multiplex
        RoachId[tlm_ind] = j;
				KidId[tlm_ind] = (KidId[tlm_ind]+1)%MIN(CommandData.num_channels_all_roaches[j], MAX_CHANNELS_PER_ROACH);
        TypeId[tlm_ind] = i;
			}
		}
  }

  // ------------------------------------------------
  // --------- Multiplexed Channel selection --------
  // ------------------------------------------------
  // Channels are selected by commands and multiplexed based on a unique roach channel Id.
  // Channels are identified in getdata/kst from an indexed string array.
  for (i = 0; i < NUM_ROACH_TLM; i++) {
    int ind_rtype = i % NUM_RTYPES;
    int ind_roach = i / NUM_RTYPES;
    if ((roach_indices[i] != CommandData.roach_tlm[i].index) && (strlen(CommandData.roach_tlm[i].name))) {
      roach_indices[i] = CommandData.roach_tlm[i].index;
      read_roach_index(&RoachId[i], &KidId[i], &TypeId[i], roach_indices[i]);
      RoachId[i] -= 1; // switch from 1 to 0 indexing

      if (tlm[i]) blast_info("Telemetering \"%s\" -> \"%s\"", CommandData.roach_tlm[i].name, tlm[i]->field);
    }

    // invalid roach selection
    if (RoachId[i] >= NUM_ROACHES) continue;

    unsigned int i_udp_read = GETREADINDEX(roach_udp[RoachId[i]].index);
    data_udp_packet_t *m_packet = &(roach_udp[RoachId[i]].last_pkts[i_udp_read]);

    // Calculate the df incorporating the new packet data
    if ((ind_roach >= NUM_ROACHES) || (ind_rtype >= NUM_RTYPES)) {
        blast_err("Df indexing error: roach_index %d, ind_rtype %d conflict with NUM_ROACHES %d, NUM_RTYPES %d",
                  ind_roach, ind_rtype, NUM_ROACHES, NUM_RTYPES);
        have_warned = 1;
    } else {
        if (CommandData.roach_tlm_mode & ROACH_TLM_IQDF) {
            switch (ind_rtype) {
                case 0: // I values
                    roach_df_telem[ind_roach].i_cur = m_packet->Ival[KidId[i]];
                    break;
                case 1: // Q values
                    roach_df_telem[ind_roach].q_cur = m_packet->Qval[KidId[i]];
                    break;
                case 2: // calc df values
                    roach_df_telem[ind_roach].ind_kid = KidId[i];
                    roach_df_telem[ind_roach].ind_roach = ind_roach;
                    roach_df_continuous(&(roach_df_telem[ind_roach]));
                    break;
            }
        }
        have_warned = 0;
    }
    // write the roach data to the multiplexed field
    if (tlm[i]) {
      double value = -3.14159;
      if (strcmp(ROACH_TYPES[TypeId[i]], "i") == 0) { // I comes from the UDP packet directly
        value = m_packet->Ival[KidId[i]];
      } else if (strcmp(ROACH_TYPES[TypeId[i]], "q") == 0) { // Q comes from the UDP packet directly
        value = m_packet->Qval[KidId[i]];
      } else if (strcmp(ROACH_TYPES[TypeId[i]], "df") == 0) { // df comes from the frame
        if (CommandData.roach_tlm_mode == ROACH_TLM_IQDF) {
          value = roach_df_telem[ind_roach].df;
        }
      }

      SET_FLOAT(tlm[i], value);
    }
    // write the multiplex index
    if (tlm_index[i]) {
      SET_INT32(tlm_index[i], roach_indices[i]);
    }
  }
}

