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
#include "mpc_proto.h"
#include "mce_counts.h"
#include "bset.h"
#include "tes.h"

extern int mce_slow_index[NUM_MCE];
extern struct mpc_slow_data mce_slow_dat[NUM_MCE][3];
extern int request_ssdata;

/* super slow data */
extern uint32_t mce_param[N_MCE_STAT * NUM_MCE];

/* array statistics */
extern uint8_t array_statistics[NUM_ARRAY_STAT];

/* blobs */

/* 0xEB90 is the 16-bit Maury-Style optimum synchronisation sequence; see:
 *
 *   Maury, J. L. and Style, F. J., "Development of optimum frame
 *   synchronisation codes for Goddard space flight center PCM telemetry
 *   standards" in Proceedings of the National Telemetring Conference,
 *   Los Angeles, June 1964.
 *
 * 0xFAF320 is the 24-bit Maury-Style optimum synchronisation sequence.
 *
 * 0x146F is the inverse of 0xEB90.
 *
 * After the leadin is a 16-bit CRC of the payload (repeated before the
 * leadout)
 */
#define BLOB_LEADIN_LEN 12
#define BLOB_LEADIN {0, 0, 0, 0, 0, 0, 0, 0, 0xEB90, 0xFAF3, 0x2000, 0x146F}

/* 0xFE6B2840 is the 32-bit Muary-Style optimum synchronisation sequence.
 *
 * Before the leadout is a 16-bit CRC of the payload (a duplicate of the one
 * following the leadin)
 */
#define BLOB_LEADOUT_LEN 2
#define BLOB_LEADOUT {0xFE6B, 0x2840};

/* MCE_BLOB_MAX (defined in mpc_proto.h) is the payload size from MPC's point of
 * view.  The PCM payload is two words larger (for the type and size)
 */
#define MCE_BLOB_PAYLOAD_MAX (MCE_BLOB_MAX + 2)

/* In addition to the payload, the blob envelope contains the leadin and leadout
 * plus two CRCs
 */
#define MCE_BLOB_ENVELOPE_MAX (MCE_BLOB_PAYLOAD_MAX + 2 + BLOB_LEADIN_LEN + \
    BLOB_LEADOUT_LEN)
extern volatile int mce_blob_pos;
extern uint16_t mce_blob_envelope[MCE_BLOB_ENVELOPE_MAX];
extern size_t mce_blob_size;

/* entry points */
void *mcesend(void*);
void *mcerecv(void*);

/* TES data FIFO read-side functions */

/* the TES data frame */
struct tes_frame {
  uint32_t frameno;
  uint16_t data[MAX_BSET];

  int present; /* only used during frame reconstruction */
};

/* length of the TES FIFO -- FIFO overfilling results in data droppage */
#define TES_FIFO_DEPTH 11

/* returns the number of records in the fifo; in the range [0:TES_FIFO_DEPTH] */
int tes_nfifo(void);

/* returns a pointer to the oldest record in the fifo or NULL if the FIFO is
 * empty. */
const struct tes_frame *tes_data(void);

/* pops and discards the oldest record in the FIFO; does nothing if the FIFO
 * is empty.  Always succeeds. */
void tes_pop(void);
