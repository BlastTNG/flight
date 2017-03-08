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
#include <sys/time.h>
#include <pthread.h>
// #include <packet_slinger.h>

#include <blast.h>

#include "bi0.h"
#include "bbc_pci.h"
#include "mpsse.h"
#include "crc.h"
#include "channels_tng.h"
#include "mputs.h"


bi0_buffer_t bi0_buffer;

void initialize_biphase_buffer(void)
{
    int i;
    size_t max_rate_size = 0;

    // for (int rate = RATE_100HZ; rate < RATE_END; rate++) {
    //    if (max_rate_size < frame_size[rate]) max_rate_size = frame_size[rate];
    // }
    max_rate_size = frame_size[RATE_100HZ]; // Temporary, for now only implementing biphase on the 100 Hz framerate
    bi0_buffer.i_in = 0;
    bi0_buffer.i_out = 0;
    for (i = 0; i < BI0_FRAME_BUFLEN; i++) {
        bi0_buffer.framelist[i] = calloc(1, max_rate_size);
        memset(bi0_buffer.framelist[i], 0, max_rate_size);
    }
}

void push_bi0_buffer(const void *m_frame)
{
    int i_in;
    i_in = (bi0_buffer.i_in + 1) & BI0_FRAME_BUFMASK;
    bi0_buffer.framesize[i_in] = frame_size[RATE_100HZ];
    // Joy added this, later will have to be modified for any rate
    memcpy(bi0_buffer.framelist[i_in], m_frame, frame_size[RATE_100HZ]);
    // Later will have to adapt to various sizes for various frequencies
    bi0_buffer.i_in = i_in;
}

void biphase_writer(void)
{
    // TODO(Joy): Verify what error checks are performed in BLAST code

    uint16_t    bi0_frame[BI0_FRAME_SIZE];
    uint16_t    sync_word = 0xEB90;
    const size_t    bi0_frame_bytes = (BI0_FRAME_SIZE - 1) * 2;

    uint16_t    read_frame;
    uint16_t    write_frame;

    // size_t slinger_frame_bytes = (BI0_FRAME_SIZE * sizeof(uint16_t)- frame_size[100HZ]);

    struct mpsse_ctx *ctx;
    const uint16_t vid = 1027;
    const uint16_t pid = 24593;
    const char *serial = NULL;
    const char *description = NULL;
    int channel = 0; // IFACE_A
    // int frequency = 1000000;
    int frequency = 500000;

    uint8_t initial_value = 0x00;
    // TODO(Joy): NEED TO CHANGE "direction" to only set the biphase pins to output!!
    uint8_t direction = 0xFF;  // 1=output, 0=input. 0xFF = output for all pins

    struct timeval begin, end;

    nameThread("Biphase");

    ctx = mpsse_open(&vid, &pid, description, serial, channel);
    if (!ctx) {
        blast_warn("Error Opening mpsse. Stopped Biphase Downlink Thread");
        pthread_exit(0);
    }
    mpsse_set_data_bits_low_byte(ctx, initial_value, direction);
    mpsse_set_data_bits_high_byte(ctx, initial_value, direction);
    mpsse_set_frequency(ctx, frequency);

    mpsse_flush(ctx);
    usleep(1000);

    blast_info("frame_size[100Hz] is %zd, biphase frame size is %zd, biphase frame size for data is %zd (bytes)",
               frame_size[RATE_100HZ], BI0_FRAME_SIZE*sizeof(uint16_t), bi0_frame_bytes);
    while (true) {
        write_frame = bi0_buffer.i_out;
        read_frame = bi0_buffer.i_in;
        if (read_frame == write_frame) {
            usleep(10000);
            continue;
        }

        blast_dbg("biphase buffer: read_frame is %d, write_frame is %d", read_frame, write_frame);
        while (read_frame != write_frame) {
            // Maybe later I'll be smarter and fill a partial frame to fill BI0_FRAME_SIZE
            memcpy(bi0_frame, bi0_buffer.framelist[write_frame], frame_size[RATE_100HZ]);
            // memset(&bi0_frame[frame_size[100_HZ]/2], 0xEE, slinger_frame_bytes); // What is that for ?
            write_frame = (write_frame + 1) & BI0_FRAME_BUFMASK;

            bi0_frame[0] = 0xEB90; // Isn't that going to overwrite the beginning of the frame??
            bi0_frame[BI0_FRAME_SIZE - 1] = crc16(CRC16_SEED, bi0_frame, bi0_frame_bytes);

            bi0_frame[0] = sync_word;
            sync_word = ~sync_word;

            gettimeofday(&begin, NULL);
            mpsse_biphase_write_data(ctx, bi0_frame, bi0_frame_bytes);
            mpsse_flush(ctx);
            gettimeofday(&end, NULL);
            // blast_info("Writing and flushing %zd bytes of data to MPSSE took %f second",
            //          BI0_FRAME_SIZE*sizeof(uint16_t), (end.tv_usec - begin.tv_usec)/1000000.0);

            if (ctx->retval != ERROR_OK) {
                blast_err("Error writing frame to Biphase, discarding.");
            }
        }

        bi0_buffer.i_out = write_frame;
        usleep(10000);
    }

    // Currently we will never get here, but later we can implement a 'biphase is on' variable
    mpsse_close(ctx);
    blast_info("Stopped Biphase Downlink Thread");
}
