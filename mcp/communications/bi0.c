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
#include <math.h>
#include <sys/time.h>
#include <pthread.h>

#include <blast.h>

#include "bi0.h"
#include "mpsse.h"
#include "crc.h"
#include "channels_tng.h"
#include "mputs.h"
#include "bbc_pci.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)

extern int16_t SouthIAm;
extern int16_t InCharge;

bi0_buffer_t bi0_buffer; // This is passed to mpsse
uint8_t *biphase_frame; // This is pushed to bi0_buffer

void initialize_biphase_buffer(void)
{
    int i;
    // size_t max_rate_size = 0;

    bi0_buffer.i_in = 0;
    bi0_buffer.i_out = 0;
    for (i = 0; i < BI0_FRAME_BUFLEN; i++) {
        bi0_buffer.framelist[i] = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
        memset(bi0_buffer.framelist[i], 0, BIPHASE_FRAME_SIZE_BYTES);
    }
    biphase_frame = calloc(1,  BIPHASE_FRAME_SIZE_BYTES);
    memset(biphase_frame, 0, BIPHASE_FRAME_SIZE_BYTES);
}

void build_biphase_frame_200hz(const void *m_channel_data)
{
    static bool even = true;
    // Storing 2x200hz data in the biphase frames
    if ((2*frame_size[RATE_200HZ]) > BIPHASE_FRAME_SIZE_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) to hold 2x200Hz frames (byte 0 to %zu).",
                    BIPHASE_FRAME_SIZE_BYTES, (2*frame_size[RATE_200HZ]));
        return;
    }
    if (even) {
        memcpy(biphase_frame, m_channel_data, frame_size[RATE_200HZ]);
    } else {
        memcpy(biphase_frame+frame_size[RATE_200HZ], m_channel_data, frame_size[RATE_200HZ]);
    }
    even = !even;
}

void build_biphase_frame_100hz(const void *m_channel_data)
{
    // Storing one 100hz frame in the biphase frame after 2x200 Hz data
    size_t start = 2*frame_size[RATE_200HZ];
    size_t end = start + frame_size[RATE_100HZ];
    if (end > BIPHASE_FRAME_SIZE_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) to hold the 100Hz frame (byte %zu to %zu).",
                    BIPHASE_FRAME_SIZE_BYTES, start, end);
        return;
    }
    memcpy(biphase_frame+start, m_channel_data, frame_size[RATE_100HZ]);
}

void build_biphase_frame_1hz(const void *m_channel_data)
{
    // This function is called at 100 Hz
    static bool first_time = true;
    static uint8_t counter = 0;
    static channel_t *subframe_Addr = NULL;
    char *channel_ptr = NULL;

    channel_ptr = (char *) m_channel_data;

    // Storing 1/100th of 1hz frame in the biphase frame after 100 Hz data
    if (first_time) {
        subframe_Addr = channels_find_by_name("subframe_counter_1hz");
        first_time = false;
    }
    SET_SCALED_VALUE(subframe_Addr, counter);

    size_t one_hundredth_1hz_frame = (size_t) ceil(((float) frame_size[RATE_1HZ])/100.0);
    size_t start = 2*frame_size[RATE_200HZ] + frame_size[RATE_100HZ];
    size_t end = start + one_hundredth_1hz_frame;
    if (end > BIPHASE_FRAME_SIZE_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) for 1/10th of 100Hz frame (byte %zu to %zu).",
                    BIPHASE_FRAME_SIZE_BYTES, start, end);
        return;
    }
    if ((counter*one_hundredth_1hz_frame) < frame_size[RATE_1HZ]) {
        channel_ptr += counter*one_hundredth_1hz_frame;
        memcpy(biphase_frame+start, channel_ptr, one_hundredth_1hz_frame);
    }
    counter++;
    if (counter >= 100) {
        counter = 0;
    }
}

void push_bi0_buffer(void)
{
    int i_in;
    i_in = (bi0_buffer.i_in + 1) & BI0_FRAME_BUFMASK;
    bi0_buffer.framesize[i_in] = BIPHASE_FRAME_SIZE_BYTES;
    memcpy(bi0_buffer.framelist[i_in], biphase_frame, BIPHASE_FRAME_SIZE_BYTES);
    bi0_buffer.i_in = i_in;
    // blast_info("bi0_buffer.i_in = %d", i_in);
}

static void tickle(struct mpsse_ctx *ctx_passed_write) {
    // static uint8_t low = 0;
    // static uint8_t high = 1;
    static int tickled = 0;
    if (tickled == 0) {
        mpsse_watchdog_ping_low(ctx_passed_write);
        tickled = 1;
        mpsse_flush(ctx_passed_write);
    } else {
        mpsse_watchdog_ping_high(ctx_passed_write);
        tickled = 0;
        mpsse_flush(ctx_passed_write);
    }
}

static void set_incharge(struct mpsse_ctx *ctx_passed_read) {
    static int first_call = 1;
    static int in_charge;
    static channel_t* incharge_Addr;
    if (first_call == 1) {
        first_call = 0;
        incharge_Addr = channels_find_by_name("incharge");
    } else {
        in_charge = mpsse_watchdog_get_incharge(ctx_passed_read);
        // blast_dbg("the value is %d", in_charge);
        SET_SCALED_VALUE(incharge_Addr, in_charge+1);
//         if (in_charge && SouthIAm) {
//             // set incharge here to 1 if the && comes true
//            InCharge = 1;
//         } else {
//             InCharge = 0;
//         }
    }
}

void biphase_writer(void)
{
    // TODO(Joy): Verify what error checks are performed in BLAST code

    uint16_t    bi0_frame[BI0_FRAME_SIZE];
    uint16_t    sync_word = 0xEB90;

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct mpsse_ctx *ctx;
    const uint16_t vid = 1027;
    const uint16_t pid = 24593;
    const char *serial = NULL;
    const char *description = NULL;
    int channel = 0; // IFACE_A
    int frequency = 1000000; // 1 Mbps
    // int frequency = 100000; // 100 kbps

    // Setting pin direction. CLK, data, WD are output and pins 0, 1 and 7
    // 1=output, 0=input. 0x83 = 0b11000001 i.e. pin 0, 1 and 7 are output
    uint8_t direction = 0x83;
    uint8_t initial_value = 0x00;

    struct timeval begin, end;

    nameThread("Biphase");

    // The first open is hack, to check chip is there + properly reset it
    ctx = mpsse_open(&vid, &pid, description, serial, channel);
    if (!ctx) {
        blast_warn("Error Opening mpsse. Stopped Biphase Downlink Thread");
        pthread_exit(0);
    }
    mpsse_reset_purge_close(ctx);
    usleep(1000);

    // This is now the real open
	ctx = mpsse_open(&vid, &pid, description, serial, channel);
    if (!ctx) {
        blast_warn("Error Opening mpsse. Stopped Biphase Downlink Thread");
        pthread_exit(0);
    }
    usleep(1000);

    mpsse_set_data_bits_low_byte(ctx, initial_value, direction);
    mpsse_set_frequency(ctx, frequency);

    mpsse_flush(ctx);
    usleep(1000);

    blast_info("used biphase frame_size is %zd, biphase frame size is %d, biphase frame size for data is %d (bytes)",
               (size_t) (frame_size[RATE_100HZ]+2*frame_size[RATE_200HZ]+ceil(frame_size[RATE_1HZ]/100.0)),
               BIPHASE_FRAME_SIZE_BYTES, (BIPHASE_FRAME_SIZE_BYTES-4));

    while (true) {
        // blast_dbg("biphase buffer: read_frame is %d, write_frame is %d", read_frame, write_frame);
        write_frame = bi0_buffer.i_out;
        read_frame = bi0_buffer.i_in;
        if (read_frame == write_frame) {
            usleep(10000);
            continue;
        }

        while (read_frame != write_frame) {
            memcpy(bi0_frame, bi0_buffer.framelist[write_frame], BIPHASE_FRAME_SIZE_BYTES);
            write_frame = (write_frame + 1) & BI0_FRAME_BUFMASK;

            bi0_frame[0] = 0xEB90; // Isn't that going to overwrite the beginning of the frame??
            bi0_frame[BI0_FRAME_SIZE - 1] = crc16(CRC16_SEED, bi0_frame, BIPHASE_FRAME_SIZE_BYTES-2);
            // blast_info("The computed CRC is %04x\n", bi0_frame[BI0_FRAME_SIZE - 1]);

            bi0_frame[0] = sync_word;
            sync_word = ~sync_word;

            gettimeofday(&begin, NULL);
            mpsse_biphase_write_data(ctx, bi0_frame, BIPHASE_FRAME_SIZE_BYTES);
            mpsse_flush(ctx);
            gettimeofday(&end, NULL);
            // blast_info("Writing and flushing %d bytes of data to MPSSE took %f second",
            //          BIPHASE_FRAME_SIZE_BYTES, (end.tv_usec - begin.tv_usec)/1000000.0);
            if (ctx->retval != ERROR_OK) {
                blast_err("Error writing frame to Biphase, discarding.");
            }
            // Watchdog TODO (Joy/Ian): decide if dangerous to have the watchdog routines here
            tickle(ctx);
            set_incharge(ctx);
        }

        bi0_buffer.i_out = write_frame;
        usleep(10000);
    }

    // Currently we will never get here, but later we can implement a 'biphase is on' variable
    mpsse_close(ctx);
    blast_info("Stopped Biphase Downlink Thread");
}
