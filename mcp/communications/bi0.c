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
#include <libusb-1.0/libusb.h>

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

int counter = 0;
struct Fifo libusb_fifo = {0}; 

extern int16_t SouthIAm;
uint8_t bi0_idle = 0;

/******************** Main Biphase Loop **************************/

static LIBUSB_CALL void demo_cb(struct libusb_transfer * write_transfer) {
    printf("Data that was sent\n");
    int i = 0;
    for (i = 0; i < BIPHASE_FRAME_SIZE_BYTES+3; i++) {
       if (i % 32 == 0) printf("\n");
       printf("0x%.2x ", write_transfer->buffer[i]);
    }
    printf("\n");

    printf("I am demo_cb, about to resubmit transfer\n");
    libusb_submit_transfer(write_transfer);

}


void biphase_writer(void * arg)
{
    // send buffers
    uint16_t lsb_biphase_linklist_chunk[BI0_FRAME_SIZE];
    uint16_t biphase_linklist_chunk[BI0_FRAME_SIZE];
    uint8_t * doublerbuffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
    uint16_t sync_word = 0xeb90;

    int counter = 0;
    int frequency = 0;
    uint16_t frame_counter = 0;

    // mpsse variables
    struct mpsse_ctx *ctx = NULL;
    const char *serial = NULL;
    uint8_t direction = 0xFF; // all pins set to write
    bool mpsse_hardware = true;

    // synclink variables
    int rc;
    int synclink_fd = get_synclink_fd();
 
    struct timeval begin, end;

    nameThread("Biphase");

    // setup for hardware
    if (mpsse_hardware) {
        if (!SouthIAm) {
            serial = "FC1BIPHASE"; // "FC1NS9HU"
        } else {
            serial = NULL; // "FC2"
            //serial = "?"; // "FC2"
        }
        setup_mpsse(&ctx, serial, direction);
        //setup_mpsse(&ctx, NULL, direction);
    } else {
        rc = setup_synclink();
    }

    // libubs setup
    struct libusb_transfer *write_transfer = NULL;
    write_transfer = libusb_alloc_transfer(0);

    // libusb variables
    allocFifo(&libusb_fifo, 32, BIPHASE_FRAME_SIZE_BYTES+3);
    static int first_time = 1;
    uint8_t * write_buf = NULL;

    // setup linklists
    linklist_t ** ll_array = arg;
    linklist_t * ll = NULL;
    int allframe_count = 0;
    uint8_t * compbuffer = calloc(1, BI0_MAX_BUFFER_SIZE);
    int retval = 0;
  
    // packetization variables
    uint16_t i_pkt = 0;
    uint16_t n_pkt = 0;
    unsigned int count = 0; 
 
    while (1) {
        // check if commanding data changed the bandwidth
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

        // check if superframe is ready and compress if so
        if (ll->data_ready & SUPERFRAME_READY) { // a superframe is ready 
            // unset the ready bit
            ll->data_ready &= ~SUPERFRAME_READY;
 
						// send allframe if necessary
						if (!allframe_count) {
						//  write_allframe(compbuffer, ll->superframe);
						//  sendToBITSender(&pilotsender, compbuffer, allframe_size, 0);
						}

            blast_info("Biphase send %d\n", count++);

            // compress the linklist to compbuffer
            compress_linklist(compbuffer, ll, NULL);
            bi0_idle = 1; // set the FIFO flag in mcp

            // set initialization for packetization
            i_pkt = 0;
            n_pkt = 1;
        }

        // packetization temporatry variables
				// TODO(javier): make the size commandable for bw
        uint8_t * chunk = NULL;
        uint32_t chunksize = BIPHASE_FRAME_SIZE_BYTES-PACKET_HEADER_SIZE-2;
        
        // packetize the linklist and send the chunks if there is data to packetize
        if ((i_pkt < n_pkt) && (chunk = packetizeBuffer(compbuffer, ll->blk_size,
                                                         &chunksize, &i_pkt, &n_pkt))) {
						// copy the data, prepending with header
						// TODO(javier): put a sensible framenumber (perhaps mcp_1hz_framecount?)
						writeHeader(((uint8_t *) biphase_linklist_chunk)+2, *(uint32_t *) ll->serial,
						             count, i_pkt, n_pkt);
						memcpy(((uint8_t *) biphase_linklist_chunk)+PACKET_HEADER_SIZE+2, chunk, chunksize);
						i_pkt++;
            printf("Send compressed packet %d of %d\n", i_pkt, n_pkt);
        } else {
            // printf("Send zero packet\n");
        }

        // syncword header for the packet and invert syncword for next send
        biphase_linklist_chunk[0] = sync_word;
        sync_word = ~sync_word; 

        // queue the packets to be sent
        if (mpsse_hardware) {
						mpsse_biphase_write_data(ctx, biphase_linklist_chunk, 
																			BIPHASE_FRAME_SIZE_BYTES, doublerbuffer);
						// libusb calls
						write_buf = getFifoWrite(&libusb_fifo);
						write_buf[0] = NEG_EDGE_OUT | MSB_FIRST | 0x10;
						write_buf[1] = (BIPHASE_FRAME_SIZE_BYTES-1) & 0xff;
						write_buf[2] = (BIPHASE_FRAME_SIZE_BYTES-1) >> 8;
						memcpy(write_buf+3, doublerbuffer, BIPHASE_FRAME_SIZE_BYTES); // data

						// increment the fifo and libusb_handle_events
						incrementFifo(&libusb_fifo); // push data to fifo
						if (first_time) {
								// initial fill_bulk_transfer and submit_transfer to start things off
								libusb_fill_bulk_transfer(write_transfer, ctx->usb_dev, ctx->out_ep, 
                      (void *) getFifoRead(&libusb_fifo), BIPHASE_FRAME_SIZE_BYTES+3, 
                      demo_cb, write_transfer, 2000);
							  libusb_submit_transfer(write_transfer);
                first_time = 0;
						} else {
                decrementFifo(&libusb_fifo); // remove read data from fifo
						    write_transfer->buffer = getFifoRead(&libusb_fifo); // get next read in fifo
            }

            // handle usb events
            gettimeofday(&begin, NULL);
						retval = libusb_handle_events(ctx->usb_ctx); // wait for data to be clocked
						printf("Handle events returned: %s\n", libusb_strerror(retval));
						gettimeofday(&end, NULL);
						if ((counter % 1) == 0) {
								blast_dbg("It took %f seconds", (end.tv_sec+(end.tv_usec/1000000.0) - 
                                                  (begin.tv_sec+(begin.tv_usec/1000000.0))));
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
				memset(doublerbuffer, 0, BIPHASE_FRAME_SIZE_BYTES);
        counter += 1;
        frame_counter++;
    }
    libusb_free_transfer(write_transfer);
}
