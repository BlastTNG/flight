/* fcp: the eBEX flight control program
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
#include <sys/ioctl.h>
#include <libusb-1.0/libusb.h>

#include "phenom/job.h"
#include "phenom/log.h"
#include "phenom/sysutil.h"

#include <linklist.h>
#include <linklist_compress.h>

#include "bi0.h"
#include "blast.h"
#include "mcp.h"
#include "command_struct.h"
#include "FIFO.h"
#include "bitserver.h"
#include "bbc_pci.h"
#include "mputs.h"
#include "biphase_hardware.h"
#include "watchdog.h"

#define WATCHDOG_PING_TIMEOUT 0.25 // seconds between pings to the watchdog card

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_FRAME_SIZE_NOCRC_BYTES (BI0_FRAME_SIZE*2 - 2)
#define BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES (BI0_FRAME_SIZE*2 - 4)

struct Fifo libusb_fifo = {0}; 

extern int16_t SouthIAm;
extern int16_t InCharge;
extern bool shutdown_mcp;
struct Fifo bi0_fifo = {0};

/******************** Main Biphase Loop **************************/

struct mpsse_ctx *ctx = NULL;
uint16_t sync_word = 0xeb90;
bool mpsse_closing = false;
bool just_reopened_mpsse = false;
bool mpsse_disconnected = false;
bool use_synclink = true;

// callback function that is called when the read transfer for the watchdog is complete
static LIBUSB_CALL void watchdog_read_cb(struct libusb_transfer * watchdog_read_transfer) {
    // blast_info("I am read_cb and I just read 0x%.6x", *(uint32_t *) watchdog_read_transfer->buffer);
    bool * reader_done = watchdog_read_transfer->user_data;
    *reader_done = true;
}

// callback function that is called when the write transfer for the watchdog is complete
static LIBUSB_CALL void watchdog_write_cb(struct libusb_transfer * watchdog_write_transfer) {
    //blast_info("I am write_cb");
    bool * writer_done = watchdog_write_transfer->user_data;
    *writer_done = true;
}

static LIBUSB_CALL void biphase_write_cb(struct libusb_transfer * biphase_write_transfer) {

    static uint8_t *reverse_buffer = NULL;
    static uint8_t *zerobuffer = NULL;

    static struct libusb_transfer *watchdog_read_transfer = NULL;
    static struct libusb_transfer *watchdog_write_transfer = NULL;
    static struct libusb_transfer *watchdog_read_req_transfer = NULL;

    static uint8_t watchdog_read_buffer[64] = {0x00};
    static uint8_t watchdog_commands[3] = {0x80, 0x84, 0xBF}; // pin state cmd with immediate return
    static uint8_t watchdog_read_req_commands[2] = {0x81, 0x87}; // set pin state cmd 
    static struct timeval begin = {0}, end = {0};
    static bool reader_done = false;
    static bool writer_done = false;

    uint8_t *read_buf = NULL;
    int data_present = 0;
    int in_charge_from_wd = -1;
    double dt = 0;
    enum libusb_error retval;

    static int first_time = 1;

    if (!mpsse_closing) {
        if (first_time) {
            // allocate memory for callback function
            reverse_buffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
            zerobuffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
            first_time = 0;

            // WATCHDOG READ SETUP
            watchdog_read_transfer = libusb_alloc_transfer(0);
            if (watchdog_read_transfer) {
                libusb_fill_bulk_transfer(watchdog_read_transfer, ctx->usb_dev, ctx->in_ep, 
                                            (void *) watchdog_read_buffer, 64, watchdog_read_cb,
                                            &reader_done, 2000);
            } else {
                blast_err("Couldn't allocate watchdog_read_transfer");
                first_time = 1;
                return;
            }

            // WATCHDOG READ REQUEST SETUP
            watchdog_read_req_transfer = libusb_alloc_transfer(0);
            if (watchdog_read_req_transfer) {
                libusb_fill_bulk_transfer(watchdog_read_req_transfer, ctx->usb_dev, ctx->out_ep, 
                                            (void *) watchdog_read_req_commands, 2, NULL, NULL, 2000);
            } else {
                blast_err("Couldn't allocate watchdog_read_req_transfer");
                first_time = 1;
                return;
            }

            // WATCHDOG PING SETUP
            watchdog_write_transfer = libusb_alloc_transfer(0);
            if (watchdog_write_transfer) {
                libusb_fill_bulk_transfer(watchdog_write_transfer, ctx->usb_dev, ctx->out_ep, 
                                            (void *) watchdog_commands, 3, watchdog_write_cb, 
                                            &writer_done, 2000);
            } else {
                blast_err("Couldn't allocate watchdog_write_transfer");
                first_time = 1;
                return;
            }

            // get initial time and sync clock markers
            gettimeofday(&end, NULL);
            memcpy(&begin, &end, sizeof(struct timeval));
        } else {
            if (false) {
                // debugging
                printf("Data that was sent\n");
                int i = 0;
                for (i = 0; i < BIPHASE_FRAME_SIZE_BYTES+3; i++) {
                     if (i % 32 == 0) printf("\n");
                     printf("0x%.2x ", biphase_write_transfer->buffer[i]);
                }
                printf("\n");
            }
            if (biphase_write_transfer->actual_length != (BIPHASE_FRAME_SIZE_BYTES+3)) {
                blast_dbg("Transfer %d != %d", biphase_write_transfer->actual_length,
                                                 BIPHASE_FRAME_SIZE_BYTES+3);
            }
            gettimeofday(&end, NULL);
            dt = (end.tv_sec+(end.tv_usec/1000000.0)) - (begin.tv_sec+(begin.tv_usec/1000000.0));
        }
        if (just_reopened_mpsse & (!first_time)){
            // usb_dev changed when reopned chip, need to update
            libusb_fill_bulk_transfer(watchdog_read_transfer, ctx->usb_dev, ctx->in_ep, 
                                        (void *) watchdog_read_buffer, 64, watchdog_read_cb, 
                                        &reader_done, 2000);
            libusb_fill_bulk_transfer(watchdog_read_req_transfer, ctx->usb_dev, ctx->out_ep, 
                                        (void *) watchdog_read_req_commands, 2, NULL, NULL, 2000);
            libusb_fill_bulk_transfer(watchdog_write_transfer, ctx->usb_dev, ctx->out_ep, 
                                        (void *) watchdog_commands, 3, watchdog_write_cb, 
                                        &writer_done, 2000);
            just_reopened_mpsse = false;
        }

        // commented out to have SYNCLINK
        if (!use_synclink && !fifoIsEmpty(&libusb_fifo)) { // data to be sent from main thread
            read_buf = getFifoRead(&libusb_fifo);
            data_present = 1;
        } else { // nothing in the fifo from the main thread, so send zeros
            read_buf = zerobuffer;
            data_present = 0;
        }

        // enough time has elapsed, so request another read
        if (dt > WATCHDOG_PING_TIMEOUT) {
            // submit transfer for the read pin state cmd
            retval = libusb_submit_transfer(watchdog_read_transfer);
            if ((retval == LIBUSB_ERROR_NO_DEVICE) | (retval == LIBUSB_ERROR_IO)) {
                mpsse_disconnected = true;
                return;
            }
            retval = libusb_submit_transfer(watchdog_read_req_transfer);
            if ((retval == LIBUSB_ERROR_NO_DEVICE) | (retval == LIBUSB_ERROR_IO)) {
                mpsse_disconnected = true;
                return;
            }
            // sync clocks
            memcpy(&begin, &end, sizeof(struct timeval));
        }

        // reader has finished retrieving state, get in_charge and toggle wd
        if (reader_done) { 
            // set in charge based on latest read
            in_charge_from_wd = (watchdog_read_buffer[2] & 0x40) >> 6; // get pin 6 state
            set_incharge(in_charge_from_wd);
            // blast_info("in charge from watchdog reads %d", in_charge_from_wd);

            // toggle pin 7 for the watchdog ping and write new state
            watchdog_commands[1] = (watchdog_read_buffer[2]) ^ (1 << 7);
            retval = libusb_submit_transfer(watchdog_write_transfer);
            if ((retval == LIBUSB_ERROR_NO_DEVICE) | (retval == LIBUSB_ERROR_IO)) {
                mpsse_disconnected = true;
                return;
            }
            reader_done = false;
            memset(watchdog_read_buffer, 0 , 3);
            // blast_info("Just wrote to wd: 0x%.2x, with toggling pin set to %d", watchdog_commands[1], ((watchdog_commands[1]&(0x80))>>7));
        }

        // syncword header for the packet and invert syncword for next send
        *(uint16_t *) read_buf = sync_word;
        sync_word = ~sync_word;

        // switch byte endianness
        biphase_reverse_bytes((void *) read_buf, BIPHASE_FRAME_SIZE_BYTES, reverse_buffer);
        if (data_present) decrementFifo(&libusb_fifo);

        // build the header for mpsse
        biphase_write_transfer->buffer[0] = NEG_EDGE_OUT | MSB_FIRST | 0x10;
        biphase_write_transfer->buffer[1] = (BIPHASE_FRAME_SIZE_BYTES-1) & 0xff;
        biphase_write_transfer->buffer[2] = (BIPHASE_FRAME_SIZE_BYTES-1) >> 8;

        // copy the data to the send_buffer
        memcpy(biphase_write_transfer->buffer+3, reverse_buffer, BIPHASE_FRAME_SIZE_BYTES);

        // zero the buffers for the next time around
        memset(reverse_buffer, 0, BIPHASE_FRAME_SIZE_BYTES);
        memset(zerobuffer, 0, BIPHASE_FRAME_SIZE_BYTES);

        retval = libusb_submit_transfer(biphase_write_transfer);
        if ((retval == LIBUSB_ERROR_NO_DEVICE) | (retval == LIBUSB_ERROR_IO)) {
            mpsse_disconnected = true;
            return;
        }
    }
}

void * libusb_handle_all_events(void * arg)
{
    struct libusb_transfer *biphase_write_transfer = (struct libusb_transfer *) arg;
    struct timeval begin, end;
    nameThread("HandleEvents");
    enum libusb_error retval;

    while (true) {
        if ((!mpsse_closing) & (!mpsse_disconnected)) {
            gettimeofday(&begin, NULL);
            retval = libusb_handle_events(ctx->usb_ctx); // blocking call: wait for data to be clocked
            gettimeofday(&end, NULL);
            if (retval != LIBUSB_SUCCESS) {
                    blast_err("Handle events returned: %s", libusb_strerror(retval));
                    blast_dbg("It took %f seconds", (end.tv_sec+(end.tv_usec/1000000.0) -
                                                                (begin.tv_sec+(begin.tv_usec/1000000.0))));
            }
            if ((retval == LIBUSB_ERROR_NO_DEVICE) | (retval == LIBUSB_ERROR_IO)) {
                mpsse_disconnected = true;
            }
        } else {
            biphase_write_transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER; // This also frees send_buffer
            libusb_free_transfer(biphase_write_transfer);
            // pthread_exit(0);
            break;
        }
    }
    return NULL;
}

void setup_libusb_transfers(void) 
{
    struct libusb_transfer *biphase_write_transfer = NULL;
    biphase_write_transfer = libusb_alloc_transfer(0);

    // pthread_t libusb_thread;
    ph_thread_t *libusb_thread = NULL;

    uint8_t *send_buffer = calloc(1, BIPHASE_FRAME_SIZE_BYTES+3);

    // initial fill_bulk_transfer and callback to start things off
    libusb_fill_bulk_transfer(biphase_write_transfer, ctx->usb_dev, ctx->out_ep, (void *) send_buffer,
                                BIPHASE_FRAME_SIZE_BYTES+3, biphase_write_cb, biphase_write_transfer, 2000);
    // call this once to initialize transfer loop
    biphase_write_cb(biphase_write_transfer);
    // start the handle events parallel loop
    // pthread_create(&libusb_thread, NULL, (void *) libusb_handle_all_events, (void *) biphase_write_transfer);
    libusb_thread = ph_thread_spawn(libusb_handle_all_events, (void *) biphase_write_transfer);
}

void * setup_synclink_transfers(void * arg)
{
  // wait until InCharge
  while (!InCharge) {
    if (shutdown_mcp) return NULL;
    usleep(10000);
  }

  int rc = setup_synclink();
  int synclink_fd = get_synclink_fd();

  // send buffers
  uint8_t lsb_buffer[BIPHASE_FRAME_SIZE_BYTES] = {0};
  uint8_t reverse_buffer[BIPHASE_FRAME_SIZE_BYTES] = {0};
  uint8_t zerobuffer[BIPHASE_FRAME_SIZE_BYTES] = {0};
  uint8_t * read_buf = NULL;
  int data_present = 0;
  int counter = 0;
  uint16_t sync = 0xeb90;
  int sleeptime = 1;

  while (!shutdown_mcp) {
    while ((synclink_fd = get_synclink_fd()) < 0) { // keep trying to connect
      rc = setup_synclink();
      blast_err("Unable to connect to synclink. Will try again in %d seconds\n", sleeptime);
      sleep(sleeptime);
      sleeptime = MIN(sleeptime*2, 60);
    }
    sleeptime = 1;

    // wait for the buffer to get below 2x the frame size
    int count = 2*BIPHASE_FRAME_SIZE_BYTES;
    while (count >= 2*BIPHASE_FRAME_SIZE_BYTES) {  
      ioctl(synclink_fd, TIOCOUTQ, &count);
      usleep(500);
    }

    // get data from the fifo, or stuff with zeros
    if (!fifoIsEmpty(&libusb_fifo)) { // data to be sent from the main thread
      read_buf = getFifoRead(&libusb_fifo);
      data_present = 1;
    } else { // send zeros
      read_buf = zerobuffer;
      *(uint16_t *) (read_buf+BIPHASE_FRAME_SIZE_BYTES-2) = counter++;
      data_present = 0;
    }
    // set syncword and invert for next send
    *(uint16_t *) read_buf = sync;
    sync = ~sync;

    // reverse bits and switch to big endian
    reverse_bits(BIPHASE_FRAME_SIZE_BYTES, (uint16_t *) read_buf, (uint16_t *) lsb_buffer); // reverse_buffer);
    // biphase_reverse_bytes((void *) lsb_buffer, BIPHASE_FRAME_SIZE_BYTES, reverse_buffer);
    if (data_present) {
       decrementFifo(&libusb_fifo);
    }
 
    // write the data to the frame
    rc = write(synclink_fd, lsb_buffer, BIPHASE_FRAME_SIZE_BYTES);
    if (rc < 0) {
      blast_err("Synclink write error %d: %s\n", errno, strerror(errno));
      usleep(1000);
    }

    // clear buffers 
    memset(lsb_buffer, 0, BIPHASE_FRAME_SIZE_BYTES);
    memset(reverse_buffer, 0, BIPHASE_FRAME_SIZE_BYTES);
    memset(zerobuffer, 0, BIPHASE_FRAME_SIZE_BYTES);
  }  

  synclink_close();
  return NULL;  
}

void biphase_writer(void * arg)
{
    nameThread("Biphase");

    // mpsse setup
    // 0xFB = 0b11111011 leaving pin 6 to read for in_charge
    // 0x83 = 0b10000011, 0xC1 = 0b11000001 
    const char *serial = NULL;
    uint8_t direction = 0xBF; 
    uint32_t previous_clock_speed = CommandData.biphase_clk_speed;

    static unsigned int warned_mpsse = 0;

    while (!setup_mpsse(&ctx, serial, direction) && !shutdown_mcp) {
        InCharge = DEFAULT_INCHARGE;

        if (!warned_mpsse) {
          blast_warn("Error opening mpsse. Will retry every 5s");
          if (InCharge) blast_info("Defaulting to in charge");
          else blast_info("Defaulting to not in charge");
        }
        warned_mpsse = 1;
        sleep(5);
    }
    warned_mpsse = 0;

    // biphase fifo
    allocFifo(&libusb_fifo, 2048, BIPHASE_FRAME_SIZE_BYTES);

    // setup linklists
    linklist_t ** ll_array = arg;
    linklist_t * ll = NULL, * ll_old = NULL;
    unsigned int allframe_bytes = 0; 
    double bandwidth = 0; 
    uint32_t transmit_size = 0;
    uint8_t * compbuffer = calloc(1, BI0_MAX_BUFFER_SIZE); 

    // packetization variables
    uint16_t i_pkt = 0;
    uint16_t n_pkt = 0;
    unsigned int count = 0; 

    // Start the loop of mpsse communication
    setup_libusb_transfers(); 
    if (use_synclink) {
      ph_thread_spawn(setup_synclink_transfers, NULL);
    }
 
    while (!shutdown_mcp) {
        // if changing mpsse clock, close and reopen the chip, restart transfer loops
        if (mpsse_disconnected) {
            blast_err("MPSSE is disconnected. Will try reconnecting");
            mpsse_closing = true;
            mpsse_close(ctx);
            while (!setup_mpsse(&ctx, serial, direction) && !shutdown_mcp) {
                InCharge = DEFAULT_INCHARGE;

                if (!warned_mpsse) {
                  blast_warn("Error opening mpsse. Will retry every 5s");
                  if (InCharge) blast_info("Defaulting to in charge");
                  else blast_info("Defaulting to not in charge");
                }
                warned_mpsse = 1;
                sleep(5);
            }
            warned_mpsse = 0;
            usleep(1000);
            mpsse_closing = false;
            mpsse_disconnected = false;
            just_reopened_mpsse = true;
            setup_libusb_transfers(); 
        }
        if (previous_clock_speed != CommandData.biphase_clk_speed) {
            mpsse_closing = true;
            sleep(1);
            mpsse_reset_purge_close(ctx);
            usleep(1000);
            while (!setup_mpsse(&ctx, serial, direction) && !shutdown_mcp) {
                blast_warn("Error opening mpsse. Will retry in 5s");

                InCharge = DEFAULT_INCHARGE;
                blast_info("Defaulting to fc1 in charge");

                sleep(5);
            }
            previous_clock_speed = CommandData.biphase_clk_speed;
            usleep(1000);
            mpsse_closing = false;
            just_reopened_mpsse = true;
            setup_libusb_transfers(); 
        }
       
        // get the current linklist
        ll = ll_array[BI0_TELEMETRY_INDEX];
        if (ll != ll_old) {
            if (ll) blast_info("BI0 linklist set to \"%s\"", ll->name);
            else blast_info("BI0 linklist set to NULL");
        }
        ll_old = ll;

        // get the current bandwidth
        if ((bandwidth != CommandData.biphase_bw) ||
             (CommandData.biphase_allframe_fraction < 0.0001)) allframe_bytes = 0;
        bandwidth = CommandData.biphase_bw;

        // check if superframe is ready and compress if so
        if (!fifoIsEmpty(&bi0_fifo) && ll && InCharge) { // a superframe is ready 

            if (!strcmp(CommandData.bi0_linklist_name, LOS_FILE_LINKLIST)) { // special file downlinking
                // use the full bandwidth
                transmit_size = bandwidth;

                // fill the downlink buffer as much as the downlink will allow 
                unsigned int bytes_packed = 0;
                while ((bytes_packed+ll->blk_size) <= transmit_size) {
                    compress_linklist(compbuffer+bytes_packed, ll, getFifoRead(&bi0_fifo));
                    bytes_packed += ll->blk_size;
                } 
                decrementFifo(&bi0_fifo);
            
            } else { // normal linklist
                // send allframe if necessary
                if (allframe_bytes >= superframe->allframe_size) {
                    transmit_size = write_allframe(compbuffer, superframe, getFifoRead(&bi0_fifo));
                    allframe_bytes = 0;
                } else {
                    // bandwidth limit; frames are 1 Hz, so bandwidth == size
                    transmit_size = MIN(ll->blk_size, bandwidth*(1.0-CommandData.biphase_allframe_fraction));

                    // compress the linklist to compbuffer
                    compress_linklist(compbuffer, ll, getFifoRead(&bi0_fifo));

                    // bandwidth limit; frames are 1 Hz, so bandwidth == size
                    allframe_bytes += bandwidth*CommandData.biphase_allframe_fraction;
                    decrementFifo(&bi0_fifo);
                }
            }

            // no packetization if there is nothing to transmit
            if (!transmit_size) continue;
 
            // set initialization for packetization
            i_pkt = 0;
            n_pkt = 1;

            // packetization temporary variables
            uint8_t * chunk = NULL;
            uint32_t chunksize = BIPHASE_FRAME_SIZE_BYTES-BI0_ZERO_PADDING-PACKET_HEADER_SIZE-2;
                       
            // packetize the linklist and send the chunks if there is data to packetize
            while ((i_pkt < n_pkt) && (chunk = packetizeBuffer(compbuffer, transmit_size,
                                           &chunksize, &i_pkt, &n_pkt))) {
                // copy the data, prepending with header
                uint8_t * write_buf = getFifoWrite(&libusb_fifo); 
                writeHeader(write_buf+2, *(uint32_t *) ll->serial, transmit_size, i_pkt, n_pkt);
                memcpy(write_buf+PACKET_HEADER_SIZE+2, chunk, chunksize);
                incrementFifo(&libusb_fifo);
                count++;
                i_pkt++;
                //printf("Send compressed packet %d of %d\n", i_pkt, n_pkt);
                usleep(1000);
            }
        } else { // sleep until the next superframe
            usleep(10000);
        }
    }
}

