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

#include "linklist.h"
#include "linklist_compress.h"
#include "bi0.h"
#include "blast.h"
#include "command_struct.h"
#include "FIFO.h"
#include "bitserver.h"
#include "bbc_pci.h"
#include "mputs.h"
#include "biphase_hardware.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_FRAME_SIZE_NOCRC_BYTES (BI0_FRAME_SIZE*2 - 2)
#define BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES (BI0_FRAME_SIZE*2 - 4)

extern int16_t SouthIAm;
uint8_t bi0_idle = 0;

/******************** Main Biphase Loop **************************/

void biphase_writer(void * arg)
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

    // setup linklists
    linklist_t * ll = NULL;
    linklist_t ** ll_array = arg;
    int allframe_count = 0;
    uint8_t * compbuffer = calloc(1, BI0_MAX_BUFFER_SIZE);

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

    while (true) {
        if (CommandData.biphase_bw_changed) {
            CommandData.biphase_bw_changed = false;
            if (mpsse_hardware) {
                mpsse_reset_purge_close(ctx);
                usleep(1000);
                setup_mpsse(&ctx, serial, direction);
            } else {
                rc = setup_synclink();
            }
        }

        // TODO(Javier): place the correct chunk of linklist into biphase_linklist_chunk
        // get the current linklist
        ll = ll_array[1];

        if (ll->data_ready & SUPERFRAME_READY) { // data is ready to be sent
            // unset the ready bit
            ll->data_ready &= ~SUPERFRAME_READY;

						// send allframe if necessary
						if (!allframe_count) {
						//  write_allframe(compbuffer, ll->superframe);
						//  sendToBITSender(&pilotsender, compbuffer, allframe_size, 0);
						}

            // compress the linklist
            int retval = compress_linklist(compbuffer, ll, NULL);

            bi0_idle = 1; // set the FIFO flag in mcp
            if (!retval) continue;

            // packetize the linklist and send the chunks
            uint16_t i_pkt = 0;
            uint16_t n_pkt = 1;
            uint8_t * chunk = NULL;
            uint32_t chunksize = BIPHASE_FRAME_SIZE_BYTES-PACKET_HEADER_SIZE-2;
            uint32_t sendsize = 0;

            while (i_pkt < n_pkt) {
                // TODO(javier): make the size commandable for bw
                chunk = packetizeBuffer(compbuffer, ll->blk_size, &chunksize,
                                         &i_pkt, &n_pkt);
                sendsize = chunksize+PACKET_HEADER_SIZE+2;

                // copy the data, prepending with the 2 byte syncword and 12 byte header
                biphase_linklist_chunk[0] = sync_word;
                // TODO(javier): put a sensible framenumber (perhaps mcp_1hz_framecount?)
                writeHeader(((uint8_t *) biphase_linklist_chunk)+2, *(uint32_t *) ll->serial,
                             0, i_pkt++, n_pkt);
                memcpy(((uint8_t *) biphase_linklist_chunk)+PACKET_HEADER_SIZE+2, chunk, chunksize);

                // invert the syncword
                sync_word = ~sync_word;

                // send the data to mpsse chip
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
                memset(biphase_linklist_chunk, 0, BIPHASE_FRAME_SIZE_BYTES);
								gettimeofday(&end, NULL);
								usleep(10); // This should not be needed
            }
        } else { // zzzzz.....
						if (mpsse_hardware) {
								mpsse_biphase_write_data(ctx, biphase_linklist_chunk, BI0_ZERO_PADDING);
								mpsse_flush(ctx); // This should be a blocking call until data is written
								if (ctx->retval != ERROR_OK) {
										blast_err("Error writing frame to Biphase, discarding.");
								}
						} else {
								reverse_bits(BIPHASE_FRAME_SIZE_BYTES, biphase_linklist_chunk, lsb_biphase_linklist_chunk);
								rc = write(synclink_fd, lsb_biphase_linklist_chunk, BI0_ZERO_PADDING);
								if (rc < 0) {
										blast_err("Synclink write error=%d %s", errno, strerror(errno));
										usleep(5000);
								} else {
										blast_info("Wrote %d bytes through synclink", rc);
								}
								rc = tcdrain(synclink_fd);
						}
            usleep(10000);
        }
    }
}
