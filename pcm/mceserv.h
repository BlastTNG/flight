/* pcm: the Spider master control program
 *
 * mceserv.c: the MCE flight computer network server
 *
 * This software is copyright (C) 2012-2013 D. V. Wiebe
 *
 * pcm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * pcm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pcm; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef MCESERV_H
#define MCESERV_H

#include "mpc_proto.h"
#include "mce_counts.h"
#include "bset.h"
#include "mce_blob.h"
#include "tes.h"

/* Timeout on MPC-induced MCE power state changes */
#define MPC_POWER_VETO 200 /* slow frames */

extern int mce_slow_index[NUM_MCE];
extern struct mpc_slow_data mce_slow_dat[NUM_MCE][3];
extern int request_ssdata;

/* super slow data */
extern uint32_t mce_param[N_MCE_STAT * NUM_MCE];

/* array statistics */
extern uint8_t array_statistics[NUM_ARRAY_STAT];

/* mcc status */
extern uint16_t mccs_alive, mccs_reporting;

/* blobs */
extern uint16_t mce_blob_envelope[MCE_BLOB_ENVELOPE_MAX];
extern size_t mce_blob_size;

/* entry points */
void *mcesend(void*);
void *mcerecv(void*);

/* TES data FIFO read-side functions */

extern int empty_tes_fifo;

/* the TES data frame */
struct tes_frame {
  uint32_t frameno;
  uint16_t data[MAX_BSET];

  int present; /* only used during frame reconstruction */
};

/* length of the TES FIFO -- FIFO overfilling results in data droppage */
#define TES_FIFO_DEPTH 101

/* returns the number of records in the fifo; in the range [0:TES_FIFO_DEPTH] */
int tes_nfifo(void);

/* returns a pointer to the oldest record in the fifo or NULL if the FIFO is
 * empty. */
const struct tes_frame *tes_data(void);

/* pops and discards the oldest record in the FIFO; does nothing if the FIFO
 * is empty.  Always succeeds. */
void tes_pop(void);

#endif
