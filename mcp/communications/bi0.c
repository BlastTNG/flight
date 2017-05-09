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

// Joy: all below is for synclink hardware
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/types.h>
//

#include <blast.h>

#include "bi0.h"
#include "mpsse.h"
#include "crc.h"
#include "channels_tng.h"
#include "mputs.h"
#include "bbc_pci.h"
// Joy: all below is for synclink hardware
#include "synclink.h"



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
        if (in_charge && SouthIAm) {
            // set incharge here to 1 if the && comes true
            InCharge = 1;
        } else {
            InCharge = 0;
        }
    }
}

void setup_mpsse(struct mpsse_ctx *ctx)
{
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
}

int setup_synclink(int *fd)
{
    int rc;
    int enable = 1;
    int sigs;
    MGSL_PARAMS params;

    *fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK, 0);
    usleep(1000);
    rc = ioctl(*fd, MGSL_IOCRXENABLE, enable);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCRXENABLE) error=%d %s", errno, strerror(errno));
        return rc;
    }

    /* Set parameters */
    rc = ioctl(*fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCGPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    params.mode = MGSL_MODE_RAW;
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_BRG + HDLC_FLAG_TXC_BRG;
    params.encoding = HDLC_ENCODING_BIPHASE_LEVEL;
    params.clock_speed = 100000;
    params.crc_type = HDLC_CRC_NONE;
    rc = ioctl(*fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    int mode = MGSL_INTERFACE_RS422;
    // mode += MGSL_INTERFACE_MSB_FIRST;
    rc = ioctl(*fd, MGSL_IOCSIF, mode);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSIF) error=%d %s", errno, strerror(errno));
        return rc;
    }
    // Blocking mode for read and writes
    fcntl(*fd, F_SETFL, fcntl(*fd, F_GETFL) & ~O_NONBLOCK);
    // Request to Send and Data Terminal Ready
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(*fd, TIOCMBIS, &sigs);
    if (rc < 0) {
        blast_err("assert DTR/RTS error = %d %s", errno, strerror(errno));
        return rc;
    }
    return rc;
}

void reverse_bits(const size_t bytes_to_write, const uint16_t *msb_data, uint16_t *lsb_data_out)
{
    static const unsigned char BitReverseTable256[] =
    {
      0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
      0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
      0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
      0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
      0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
      0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
      0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
      0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
      0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
      0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
      0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
      0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
      0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
      0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
      0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
      0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
    };
    uint16_t lsb;
    uint16_t msb;
    for (int i = 0; i < ((int) bytes_to_write/2); i++) {
        msb = *(msb_data+i);
        lsb = (BitReverseTable256[msb & 0xff] << 8) |
              (BitReverseTable256[(msb >> 8) & 0xff]);
        *(lsb_data_out+i) = lsb;
    }
}

void biphase_writer(void)
{
    // TODO(Joy): Verify what error checks are performed in BLAST code

    uint16_t    bi0_frame[BI0_FRAME_SIZE];
    uint16_t    sync_word = 0xEB90;

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timeval begin, end;

    struct mpsse_ctx *ctx;
    bool mpsse_hardware = true;
    // For synclink
    int fd;
    int rc;
    uint16_t lsb_bi0_frame[BI0_FRAME_SIZE];

    nameThread("Biphase");

    setup_mpsse(ctx);
    if (!mpsse_hardware) {
        rc = setup_synclink(&fd);
    }

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
            // Joy: until we figure out why decom card fails to read frames with alternate words
            // sync_word = ~sync_word;

            gettimeofday(&begin, NULL);
            if (mpsse_hardware) {
                mpsse_biphase_write_data(ctx, bi0_frame, BIPHASE_FRAME_SIZE_BYTES);
                mpsse_flush(ctx);
                if (ctx->retval != ERROR_OK) {
                    blast_err("Error writing frame to Biphase, discarding.");
                }
            } else {
                reverse_bits(BIPHASE_FRAME_SIZE_BYTES, bi0_frame, lsb_bi0_frame);
                rc = write(fd, lsb_bi0_frame, BIPHASE_FRAME_SIZE_BYTES);
                if (rc < 0) {
                    blast_err("Synclink write error=%d %s", errno, strerror(errno));
                }
            }
            gettimeofday(&end, NULL);
            // blast_info("Writing and flushing %d bytes of data to MPSSE took %f second",
            //          BIPHASE_FRAME_SIZE_BYTES, (end.tv_usec - begin.tv_usec)/1000000.0);
            // Watchdog TODO (Joy/Ian): decide if dangerous to have the watchdog routines here
            tickle(ctx);
            set_incharge(ctx);
        }
        bi0_buffer.i_out = write_frame;
        usleep(10000);
    }

    // Currently we will never get here, but later we can implement a 'biphase is on' variable
    mpsse_close(ctx);
    if (!mpsse_hardware) {
        close(fd);
    }
    blast_info("Stopped Biphase Downlink Thread");
}
