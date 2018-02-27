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
#include "watchdog.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_FRAME_SIZE_NOCRC_BYTES (BI0_FRAME_SIZE*2 - 2)
#define BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES (BI0_FRAME_SIZE*2 - 4)

int counter = 0;
struct Fifo libusb_fifo = {0}; 

extern int16_t SouthIAm;
struct Fifo bi0_fifo = {0};

/******************** Main Biphase Loop **************************/

struct mpsse_ctx *ctx = NULL;
uint16_t sync_word = 0xeb90;

bool reader_done = false;

static LIBUSB_CALL void watchdog_read_cb(struct libusb_transfer * watchdog_read_transfer) {
    // blast_info("I am read_cb and I just read 0x%.2x, 0x%.2x, 0x%.2x", watchdog_read_transfer->buffer[0], watchdog_read_transfer->buffer[1], watchdog_read_transfer->buffer[2]);
    reader_done = true;
}


static LIBUSB_CALL void biphase_write_cb(struct libusb_transfer * biphase_write_transfer) {
    static uint8_t * doublerbuffer;
    static uint8_t * zerobuffer;
    static int first_time = 1;
    static int counter = 0;

    static struct libusb_transfer *watchdog_read_transfer = NULL;
    static struct libusb_transfer *watchdog_write_transfer = NULL;
    static uint8_t watchdog_read_buffer[3] = {0x00, 0x00, 0x00};
    static uint8_t watchdog_commands[5] = {0x81, 0x87, 0x80, 0x84, 0xFB};
    static struct timeval begin, end;

    uint8_t *read_buf = NULL;
    int data_present = 0;
    int in_charge_from_wd = -1;
    double dt = 0;

    if (first_time) {
        // allocate memory for callback function
        doublerbuffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
        zerobuffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
        first_time = 0;

        // WATCHDOG READ 
        watchdog_read_transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(watchdog_read_transfer, ctx->usb_dev, ctx->in_ep, (void *) watchdog_read_buffer,
                                  3, watchdog_read_cb, watchdog_read_transfer, 2000);
        libusb_submit_transfer(watchdog_read_transfer);

        // WATCHDOG PING
        watchdog_write_transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(watchdog_write_transfer, ctx->usb_dev, ctx->out_ep, (void *) watchdog_commands,
                                  5, NULL, NULL, 2000);
        libusb_submit_transfer(watchdog_write_transfer);
        gettimeofday(&begin, NULL);
        gettimeofday(&end, NULL);
    } else {
/*
        printf("Data that was sent\n");
        int i = 0;
        for (i = 0; i < BIPHASE_FRAME_SIZE_BYTES+3; i++) {
             if (i % 32 == 0) printf("\n");
             printf("0x%.2x ", biphase_write_transfer->buffer[i]);
        }
        printf("\n");
*/
       if (biphase_write_transfer->actual_length != (BIPHASE_FRAME_SIZE_BYTES+3)) {
           blast_dbg("Transfer %d != %d", biphase_write_transfer->actual_length, BIPHASE_FRAME_SIZE_BYTES+3);
        }
    }

    if (!first_time) {
        gettimeofday(&end, NULL);
        dt = (end.tv_sec+(end.tv_usec/1000000.0) - (begin.tv_sec+(begin.tv_usec/1000000.0)));
    }

    if (!fifoIsEmpty(&libusb_fifo)) { // data to be sent from main thread
        // syncword header for the packet and invert syncword for next send
        read_buf = getFifoRead(&libusb_fifo);
        // printf("Start %d, end %d\n", libusb_fifo.start, libusb_fifo.end);
        data_present = 1;
    } else { // nothing in the fifo from the main thread, so send zeros
        read_buf = zerobuffer;
        data_present = 0;
    }

    // WATCHDOG READ IN CHARGE AND TOGGLE 
    if (reader_done && (dt > 1.0)){
        gettimeofday(&begin, NULL);
        gettimeofday(&end, NULL);
        libusb_submit_transfer(watchdog_read_transfer);
        watchdog_commands[3] = (watchdog_read_buffer[2])^(1 << 7);
        libusb_submit_transfer(watchdog_write_transfer);
        reader_done = false;
        in_charge_from_wd = watchdog_read_buffer[2] & 0x40;
        set_incharge(in_charge_from_wd);
    }

    // add headers and doubling data
    *(uint16_t *) read_buf = sync_word;
    sync_word = ~sync_word;
    mpsse_biphase_write_data(ctx, (void *) read_buf, BIPHASE_FRAME_SIZE_BYTES, doublerbuffer);
    if (data_present) decrementFifo(&libusb_fifo);

    // build the header for mpsse
    biphase_write_transfer->buffer[0] = NEG_EDGE_OUT | MSB_FIRST | 0x10;
    biphase_write_transfer->buffer[1] = (BIPHASE_FRAME_SIZE_BYTES-1) & 0xff;
    biphase_write_transfer->buffer[2] = (BIPHASE_FRAME_SIZE_BYTES-1) >> 8;

    // copy the data to the send_buffer
    memcpy(biphase_write_transfer->buffer+3, doublerbuffer, BIPHASE_FRAME_SIZE_BYTES); // data

    // zero the buffers for the next time around
    memset(doublerbuffer, 0, BIPHASE_FRAME_SIZE_BYTES);
    memset(zerobuffer, 0, BIPHASE_FRAME_SIZE_BYTES);

    libusb_submit_transfer(biphase_write_transfer);
}

void libusb_handle_all_events(libusb_context * usb_ctx)
{
    struct timeval begin, end;
    // struct timeval notime = {0};

    nameThread("HandleEvents");

    while (true) {
            // handle usb events
            gettimeofday(&begin, NULL);
            int retval = libusb_handle_events(usb_ctx); // wait for data to be clocked
            // int retval = libusb_handle_events_timeout(usb_ctx, &notime); // wait for data to be clocked
            gettimeofday(&end, NULL);
            if (retval != LIBUSB_SUCCESS) {
                printf("Handle events returned: %s\n", libusb_strerror(retval));
                  blast_dbg("It took %f seconds", (end.tv_sec+(end.tv_usec/1000000.0) -
                           (begin.tv_sec+(begin.tv_usec/1000000.0))));
            }
    }
}

void biphase_writer(void * arg)
{
    // mpsse variables
    const char *serial = NULL;
    uint8_t direction = 0xFB; // 0b11111011 leaving pin 6 to read for in_charge
    // 0x83=0b10000011, 0xC1=0b11000001 
    bool mpsse_hardware = true;

/*
    // synclink variables
    int rc;
    int synclink_fd = get_synclink_fd();

    // send buffers
    uint16_t lsb_biphase_linklist_chunk[BI0_FRAME_SIZE];
    uint16_t biphase_linklist_chunk[BI0_FRAME_SIZE];
*/

    nameThread("Biphase");

    // setup for hardware
    if (mpsse_hardware) {
        if (!SouthIAm) {
            serial = "FC1BIPHASE"; // "FC1NS9HU"
        } else {
        // TODO(joy): set this to the proper serial number fo FC2
            serial = NULL; // "FC2"
            //serial = "?"; // "FC2"
        }
        setup_mpsse(&ctx, serial, direction);
        // setup_mpsse(&ctx, NULL, direction);
    } else {
        // rc = setup_synclink();
    }

    // libubs setup
    struct libusb_transfer *biphase_write_transfer = NULL;
    biphase_write_transfer = libusb_alloc_transfer(0);

    // libusb variables
    allocFifo(&libusb_fifo, 128, BIPHASE_FRAME_SIZE_BYTES);

    // setup linklists
    linklist_t ** ll_array = arg;
    linklist_t * ll = NULL;
    int allframe_count = 0;
    uint8_t * compbuffer = calloc(1, BI0_MAX_BUFFER_SIZE);
  
    // packetization variables
    uint16_t i_pkt = 0;
    uint16_t n_pkt = 0;
    unsigned int count = 0; 
 
    // threading for libusb handles
    pthread_t libusb_thread;
    uint8_t * send_buffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES+3);



    // initial fill_bulk_transfer and callback to start things off
    libusb_fill_bulk_transfer(biphase_write_transfer, ctx->usb_dev, ctx->out_ep, (void *) send_buffer,
                                BIPHASE_FRAME_SIZE_BYTES+3, biphase_write_cb, biphase_write_transfer, 2000);
    biphase_write_cb(biphase_write_transfer); // call this once to initialize transfer loop


    pthread_create(&libusb_thread, NULL, (void *) libusb_handle_all_events, (void *) ctx->usb_ctx);

    while (1) {
        // check if commanding data changed the bandwidth
        if (CommandData.biphase_bw_changed) {
            CommandData.biphase_bw_changed = false;
            if (mpsse_hardware) {
                mpsse_reset_purge_close(ctx);
                usleep(1000);
                setup_mpsse(&ctx, serial, direction);
            } else {
                // rc = setup_synclink();
            }
        }
       
        // TODO(Javier): place the correct chunk of linklist into biphase_linklist_chunk
        // get the current linklist
        ll = ll_array[1];

        // check if superframe is ready and compress if so
        if (!fifoIsEmpty(&bi0_fifo) && ll) { // a superframe is ready 
        // send allframe if necessary
        if (!allframe_count) {
            //  write_allframe(compbuffer, getFifoRead(&bi0_fifo));
            //  sendToBITSender(&pilotsender, compbuffer, allframe_size, 0);
        }

            // compress the linklist to compbuffer
            compress_linklist(compbuffer, ll, getFifoRead(&bi0_fifo));
            decrementFifo(&bi0_fifo);

            // set initialization for packetization
            i_pkt = 0;
            n_pkt = 1;

            // packetization temporatry variables
        // TODO(javier): make the size commandable for bw
        uint8_t * chunk = NULL;
        uint32_t chunksize = BIPHASE_FRAME_SIZE_BYTES-BI0_ZERO_PADDING-PACKET_HEADER_SIZE-2;
                    
        // packetize the linklist and send the chunks if there is data to packetize
        while ((i_pkt < n_pkt) && (chunk = packetizeBuffer(compbuffer, ll->blk_size,
                                       &chunksize, &i_pkt, &n_pkt))) {
            // copy the data, prepending with header
                uint8_t * write_buf = getFifoWrite(&libusb_fifo); 
            writeHeader(write_buf+2, *(uint32_t *) ll->serial, count, i_pkt, n_pkt);
            memcpy(write_buf+PACKET_HEADER_SIZE+2, chunk, chunksize);
                incrementFifo(&libusb_fifo);
                count++;
            i_pkt++;
            //printf("Send compressed packet %d of %d\n", i_pkt, n_pkt);
                usleep(1000);
            }

/*
            // queue the packets to be sent
            if (mpsse_hardware) {
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
            counter += 1;
            frame_counter++;
*/
        } else { // sleep until the next superframe
            usleep(10000);
        }
    }
    // free the libusb transfer on thread exit
    libusb_free_transfer(biphase_write_transfer);
}
