/* fcp: the EBEX flight control program
 *
 * This software is copyright (C) 2009 Columbia University
 *                            (C) 2016 University of Pennsylvania
 *
 * This file is part of fcp.
 *
 * fcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#include <termios.h>

#include <blast.h>
#include <command_struct.h>

#include "bi0.h"
#include "crc.h"
#include "channels_tng.h"
#include "mputs.h"
#include "bbc_pci.h"
#include "biphase_hardware.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_FRAME_SIZE_NOCRC_BYTES (BI0_FRAME_SIZE*2 - 2)
#define BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES (BI0_FRAME_SIZE*2 - 4)

extern int16_t SouthIAm;


/******************** Main Biphase Loop **************************/

void biphase_writer(void)
{
    // TODO(Joy): Verify what error checks are performed in BLAST code

    uint16_t    biphase_linklist_chunk[BI0_FRAME_SIZE];
    uint16_t    sync_word = 0xEB90;

    struct timeval begin, end;
    bool mpsse_hardware = true;

    struct mpsse_ctx *ctx = NULL;
    const char *serial = NULL;
    uint8_t direction = 0xFF; // all pins set to write
    // For synclink
    int rc;
    uint16_t lsb_biphase_linklist_chunk[BI0_FRAME_SIZE];
    int synclink_fd = get_synclink_fd();

    nameThread("Biphase");

    if (mpsse_hardware) {
        if (!SouthIAm) {
            serial = "FC1"; // "FC1NS9HU"
        } else {
            serial = "?"; // "FC2"
        }
        setup_mpsse(&ctx, serial, direction);
    } else {
        rc = setup_synclink();
    }

    // blast_info("used biphase_linklist_chunk is %zd, biphase frame size is %d, biphase frame size for data is %d (bytes)",
    //            (size_t) (frame_size[RATE_100HZ]+2*frame_size[RATE_200HZ]+ceil(frame_size[RATE_1HZ]/100.0)),
    //            BIPHASE_FRAME_SIZE_BYTES, (BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES));

    while (true) {
        if (CommandData.biphase_bw_changed) {
            CommandData.biphase_bw_changed = false;
            get_num_frames_per_superframe(num_frames_per_superframe);
            if (mpsse_hardware) {
                mpsse_reset_purge_close(ctx);
                usleep(1000);
                setup_mpsse(&ctx, serial, direction);
            } else {
                rc = setup_synclink();
            }
        }

        // TODO (Javier): place the correct chunk of linklist into biphase_linklist_chunk
        // memcpy(biphase_linklist_chunk, your_linklist)

         // This used to be when we used this to compute the CRC before alternation
        // biphase_linklist_chunk[0] = 0xEB90;
        // biphase_linklist_chunk[BI0_FRAME_SIZE-1] = crc16(CRC16_SEED, biphase_linklist_chunk, BIPHASE_FRAME_SIZE_NOCRC_BYTES);
        // blast_info("The computed CRC is %04x\n", biphase_linklist_chunk[BI0_FRAME_SIZE - 1]);

        biphase_linklist_chunk[0] = sync_word;
        sync_word = ~sync_word;

        gettimeofday(&begin, NULL);
        if (mpsse_hardware) {
            mpsse_biphase_write_data(ctx, biphase_linklist_chunk, BIPHASE_FRAME_SIZE_BYTES);
            mpsse_flush(ctx); // This should be a blocking call until data is written
            if (ctx->retval != ERROR_OK) {
                blast_err("Error writing frame to Biphase, discarding.");
            }
        } else {
            reverse_bits(BIPHASE_FRAME_SIZE_BYTES, biphase_linklist_chunk, lsb_biphase_linklist_chunk);
            rc = write(synclink_fd, lsb_biphase_linklist_chunk, BIPHASE_FRAME_SIZE_BYTES);
            if (rc < 0) {
                blast_err("Synclink write error=%d %s", errno, strerror(errno));
                usleep(5000);
            } else {
                blast_info("Wrote %d bytes through synclink", rc);
            }
            rc = tcdrain(synclink_fd);
        }
        gettimeofday(&end, NULL);
        usleep(10); // This should not be needed
    }
}
