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

double num_frames_per_superframe[RATE_END];
biphase_frames_t biphase_frames; // This is passed to mpsse
uint8_t *biphase_superframe_in; // This is pushed to biphase_frames


/******************* Biphase Functions *******************/

void initialize_biphase_buffer(void)
{
    int i;
    // size_t max_rate_size = 0;

    // biphase_frames
    biphase_frames.i_in = 0;
    biphase_frames.i_out = 0;
    for (i = 0; i < BI0_FRAME_BUFLEN; i++) {
        biphase_frames.framelist[i] = calloc(1, BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);
        memset(biphase_frames.framelist[i], 0, BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);
    }

    // biphase_superframe
    biphase_superframe_in = calloc(1,  BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);
    memset(biphase_superframe_in, 0, BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);

    // define the superframe content
    get_num_frames_per_superframe(num_frames_per_superframe);
}

void get_num_frames_per_superframe(double num_frames_per_superframe[])
{
    if (CommandData.biphase_bw == 100000) {
        num_frames_per_superframe[RATE_200HZ] = 20;
        num_frames_per_superframe[RATE_100HZ] = 10;
        num_frames_per_superframe[RATE_5HZ] = 0.5;
        num_frames_per_superframe[RATE_1HZ] = 0.1;
    }
    if (CommandData.biphase_bw == 500000) {
        num_frames_per_superframe[RATE_200HZ] = 4;
        num_frames_per_superframe[RATE_100HZ] = 2;
        num_frames_per_superframe[RATE_5HZ] = 0.1;
        num_frames_per_superframe[RATE_1HZ] = 0.02;
    }
    if (CommandData.biphase_bw == 1000000) {
        num_frames_per_superframe[RATE_200HZ] = 2;
        num_frames_per_superframe[RATE_100HZ] = 1;
        num_frames_per_superframe[RATE_5HZ] = 0.05;
        num_frames_per_superframe[RATE_1HZ] = 0.01;
    }
    // TODO(Joy): Add 2 MBps case
}

void add_200hz_frame_to_biphase(void **m_channel_data)
{
    // Storing 200hz data in the biphase frames

    static uint8_t frame_position = 0;
    size_t total_frames = (size_t) num_frames_per_superframe[RATE_200HZ];

    if ((total_frames*frame_size[RATE_200HZ]) > BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) to hold 2x200Hz frames (byte 0 to %zu).",
                    BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES, (total_frames*frame_size[RATE_200HZ]));
        return;
    }
    // 200hz frames occupy the superframe position 0 to n_200*downlink_frame_size[RATE_200HZ]
    memcpy(biphase_superframe_in + frame_position*frame_size[RATE_200HZ],
           m_channel_data[RATE_200HZ], frame_size[RATE_200HZ]);
    frame_position++;
    if (frame_position >= total_frames) {
        // Filling in the partial frames from the slower framerates
        add_partial_5hz_frame_to_biphase(m_channel_data[RATE_5HZ]);
        add_partial_1hz_frame_to_biphase(m_channel_data[RATE_1HZ]);
        // TODO(Joy): Add partial_100hz for 2 MBps rate
        // Sending superframe to mpsse
        push_biphase_frames();
        frame_position = 0;
    }
}

void add_100hz_frame_to_biphase(const void *m_channel_data)
{
    // Storing 100hz frame in the superframe after nx200 Hz data
    static uint8_t frame_position = 0;
    size_t total_frames = (size_t) num_frames_per_superframe[RATE_100HZ];
    size_t start = ((size_t) num_frames_per_superframe[RATE_200HZ])*frame_size[RATE_200HZ];
    size_t end = start + total_frames*frame_size[RATE_100HZ];
    if (end > BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) to hold the 100Hz frame (byte %zu to %zu).",
                    BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES, start, end);
        return;
    }
    // 100hz frames occupy the superframe position from
    // n_200*downlink_frame_size[RATE_200HZ] to that + n_100*downlink_frame_size[RATE_100HZ]
    memcpy(biphase_superframe_in+start+frame_position*frame_size[RATE_100HZ],
           m_channel_data, frame_size[RATE_100HZ]);
    frame_position++;
    if (frame_position >= total_frames) {
        frame_position = 0;
    }
}

void add_partial_5hz_frame_to_biphase(const void *m_channel_data)
{
    static bool first_time = true;
    static uint8_t counter = 0;
    uint8_t max_counter;
    static channel_t *subframe_Addr = NULL;
    char *channel_ptr = NULL;
    double fraction_of_frame = num_frames_per_superframe[RATE_5HZ];

    channel_ptr = (char *) m_channel_data;

    if (fraction_of_frame > 0) {
        max_counter = (uint8_t) (1.0/fraction_of_frame);
    } else {
        // This should never happen
        max_counter = 1;
    }

    // Storing partial 5hz frame in the biphase frame after 100 Hz data
    if (first_time) {
        subframe_Addr = channels_find_by_name("subframe_counter_5hz");
        first_time = false;
    }
    SET_SCALED_VALUE(subframe_Addr, counter);

    size_t partial_5hz_frame_size = (size_t) ceil(((double) frame_size[RATE_5HZ])*fraction_of_frame);
    size_t start = ((size_t) num_frames_per_superframe[RATE_200HZ])*frame_size[RATE_200HZ]
                 + ((size_t) num_frames_per_superframe[RATE_100HZ])*frame_size[RATE_100HZ];
    size_t end = start + partial_5hz_frame_size;
    if (end > BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) for partial 1Hz frame (byte %zu to %zu).",
                    BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES, start, end);
        return;
    }
    // Selecting right part of 5hz frame and storing it to superframe
    if ((counter*partial_5hz_frame_size) < frame_size[RATE_5HZ]) {
        channel_ptr += counter*partial_5hz_frame_size;
        memcpy(biphase_superframe_in+start, channel_ptr, partial_5hz_frame_size);
    }
    counter++;
    if (counter >= max_counter) {
        counter = 0;
    }
}

void add_partial_1hz_frame_to_biphase(const void *m_channel_data)
{
    // This function is called at 100 Hz
    static bool first_time = true;
    static uint8_t counter = 0;
    uint8_t max_counter;
    static channel_t *subframe_Addr = NULL;
    char *channel_ptr = NULL;
    double fraction_of_frame = num_frames_per_superframe[RATE_1HZ];
    double fraction_of_frame_5Hz = num_frames_per_superframe[RATE_5HZ];

    channel_ptr = (char *) m_channel_data;

    if (fraction_of_frame > 0) {
        max_counter = (uint8_t) (1.0/fraction_of_frame);
    } else {
        // This should never happen
        max_counter = 1;
    }

    // Storing partial 1 Hz frame in the biphase frame after 5 Hz data
    if (first_time) {
        subframe_Addr = channels_find_by_name("subframe_counter_1hz");
        first_time = false;
    }
    SET_SCALED_VALUE(subframe_Addr, counter);

    size_t partial_1hz_frame_size = (size_t) ceil(((float) frame_size[RATE_1HZ])*fraction_of_frame);
    size_t partial_5hz_frame = (size_t) ceil(((float) frame_size[RATE_5HZ])*fraction_of_frame_5Hz);
    size_t start = ((size_t) num_frames_per_superframe[RATE_200HZ])*frame_size[RATE_200HZ]
                 + ((size_t) num_frames_per_superframe[RATE_100HZ])*frame_size[RATE_100HZ]
                 + partial_5hz_frame;
    size_t end = start + partial_1hz_frame_size;
    if (end > BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES) {
        blast_warn("Not enough space in biphase frame (%d bytes) for partial 1Hz frame (byte %zu to %zu).",
                    BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES, start, end);
        return;
    }
    // Selecting right part of 1 Hz frame and storing it to superframe
    if ((counter*partial_1hz_frame_size) < frame_size[RATE_1HZ]) {
        channel_ptr += counter*partial_1hz_frame_size;
        memcpy(biphase_superframe_in+start, channel_ptr, partial_1hz_frame_size);
    }
    counter++;
    if (counter >= max_counter) {
        counter = 0;
    }
}

void push_biphase_frames(void)
{
    int i_in;
    i_in = (biphase_frames.i_in + 1) & BI0_FRAME_BUFMASK;
    biphase_frames.framesize[i_in] = BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES;
    memcpy(biphase_frames.framelist[i_in], biphase_superframe_in, BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);
    biphase_frames.i_in = i_in;
    // blast_info("biphase_frames.i_in = %d", i_in);
}

/******************** Main Biphase Loop **************************/

void biphase_writer(void)
{
    // TODO(Joy): Verify what error checks are performed in BLAST code

    uint16_t    biphase_out[BI0_FRAME_SIZE];
    uint16_t    sync_word = 0xEB90;

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timeval begin, end;
    bool mpsse_hardware = true;

    struct mpsse_ctx *ctx = NULL;
    const char *serial = NULL;
    uint8_t direction = 0xFF; // all pins set to write
    // For synclink
    int rc;
    uint16_t lsb_biphase_out[BI0_FRAME_SIZE];
    int synclink_fd = get_synclink_fd();

    nameThread("Biphase");

    if (mpsse_hardware) {
        if (!SouthIAm) {
            serial = "FC1"; // "FC1NS9HU"
        } else {
            serial = "?";
        }
        setup_mpsse(&ctx, serial, direction);
    } else {
        rc = setup_synclink();
    }

    blast_info("used biphase frame_size is %zd, biphase frame size is %d, biphase frame size for data is %d (bytes)",
               (size_t) (frame_size[RATE_100HZ]+2*frame_size[RATE_200HZ]+ceil(frame_size[RATE_1HZ]/100.0)),
               BIPHASE_FRAME_SIZE_BYTES, (BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES));

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
        // blast_dbg("biphase buffer: read_frame is %d, write_frame is %d", read_frame, write_frame);
        write_frame = biphase_frames.i_out;
        read_frame = biphase_frames.i_in;
        if (read_frame == write_frame) {
            usleep(10000);
            continue;
        }

        while (read_frame != write_frame) {
            memcpy(&(biphase_out[1]), biphase_frames.framelist[write_frame], BIPHASE_FRAME_SIZE_NOCRC_NOSYNC_BYTES);
            write_frame = (write_frame + 1) & BI0_FRAME_BUFMASK;

            biphase_out[0] = 0xEB90;
            biphase_out[BI0_FRAME_SIZE-1] = crc16(CRC16_SEED, biphase_out, BIPHASE_FRAME_SIZE_NOCRC_BYTES);
            // blast_info("The computed CRC is %04x\n", biphase_out[BI0_FRAME_SIZE - 1]);

            biphase_out[0] = sync_word;
            sync_word = ~sync_word;

            gettimeofday(&begin, NULL);
            if (mpsse_hardware) {
                mpsse_biphase_write_data(ctx, biphase_out, BIPHASE_FRAME_SIZE_BYTES);
                mpsse_flush(ctx);
                if (ctx->retval != ERROR_OK) {
                    blast_err("Error writing frame to Biphase, discarding.");
                }
            } else {
                reverse_bits(BIPHASE_FRAME_SIZE_BYTES, biphase_out, lsb_biphase_out);
                rc = write(synclink_fd, lsb_biphase_out, BIPHASE_FRAME_SIZE_BYTES);
                if (rc < 0) {
                    blast_err("Synclink write error=%d %s", errno, strerror(errno));
                    usleep(5000);
                } else {
                    blast_info("Wrote %d bytes through synclink", rc);
                }
                rc = tcdrain(synclink_fd);
            }
            gettimeofday(&end, NULL);
        }
        biphase_frames.i_out = write_frame;
    }
}
