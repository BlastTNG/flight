/* mcp: the BLAST flight control program
 *
 * This software is copyright (C) 2009 Columbia University
 *                            (C) 2017 University of Pennsylvania
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/******************* MPSSE Functions *******************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>

#include <blast.h>
#include <command_struct.h>

// Joy: all below is for synclink hardware
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/types.h>
//
#include "biphase_hardware.h"
#include "mpsse.h"
#include "synclink.h"

static int synclink_fd = -1;

void setup_mpsse(struct mpsse_ctx **ctx_ptr, const char *serial, uint8_t direction)
{
    const uint16_t vid = 1027;
    const uint16_t pid = 24593;
    // const char *serial = NULL;
    const char *description = NULL;
    int channel = 0; // IFACE_A
    int frequency = (int) CommandData.biphase_bw;

    // Setting pin direction. CLK, data, WD are output and pins 0, 1 and 7
    // 1=output, 0=input. 0x83 = 0b10000011 i.e. pin 0, 1 and 7 are output
    // 1=output, 0=input. 0xBF = 0b10111111 i.e. pin 6 is input
    // uint8_t direction = 0xFF; // Hacking all pins to output for testing
    uint8_t initial_value = 0x00;

    // The first open is hack, to check chip is there + properly reset it
    *ctx_ptr = mpsse_open(&vid, &pid, description, serial, channel);
    if (!*ctx_ptr) {
        blast_warn("Error Opening mpsse. Stopped Biphase or Watchdog Thread");
        pthread_exit(0);
    }
    mpsse_reset_purge_close(*ctx_ptr);
    usleep(1000);

    // This is now the real open
	*ctx_ptr = mpsse_open(&vid, &pid, description, serial, channel);
    if (!*ctx_ptr) {
        blast_warn("Error Opening mpsse. Stopped Biphase or Watchdog Thread");
        pthread_exit(0);
    }
    usleep(1000);

    mpsse_set_data_bits_low_byte(*ctx_ptr, initial_value, direction);
    mpsse_set_frequency(*ctx_ptr, frequency);

    mpsse_flush(*ctx_ptr);
    usleep(1000);
}


/******************* Synclink Functions *******************/

/**
 * Close the synclink device before exiting mcp.
 */
void synclink_close(void)
{
    int rc = -1;
    int sigs = TIOCM_RTS + TIOCM_DTR;
    if (synclink_fd != -1) {
        rc = ioctl(synclink_fd, TIOCMBIC, &sigs);
        usleep(10000);
        rc = close(synclink_fd);
        blast_dbg("Closed synclink with return value %d", rc);
    }
    synclink_fd = -1;
}

int get_synclink_fd(void)
{
    return synclink_fd;
}

int setup_synclink()
{
    int rc;
    int enable = 1;
    int sigs;
    MGSL_PARAMS params;

    if (synclink_fd > 0) close(synclink_fd);
    synclink_fd = open("/dev/ttyMicrogate", O_RDWR | O_NONBLOCK, 0);
    if (synclink_fd < 0) {
        blast_warn("Error Opening synclink. Stopped Biphase Downlink Thread");
        pthread_exit(0);
    }
    usleep(1000);
    rc = ioctl(synclink_fd, MGSL_IOCRXENABLE, enable);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCRXENABLE) error=%d %s", errno, strerror(errno));
        return rc;
    }

    /* Set parameters */
    rc = ioctl(synclink_fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCGPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    params.mode = MGSL_MODE_RAW;
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_BRG + HDLC_FLAG_TXC_BRG;
    // params.encoding = HDLC_ENCODING_BIPHASE_LEVEL;
    params.encoding = HDLC_ENCODING_NRZ;
    params.clock_speed = 1000000;
    params.crc_type = HDLC_CRC_NONE;
    rc = ioctl(synclink_fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    int mode = MGSL_INTERFACE_RS422;
    // mode += MGSL_INTERFACE_MSB_FIRST;
    rc = ioctl(synclink_fd, MGSL_IOCSIF, mode);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSIF) error=%d %s", errno, strerror(errno));
        return rc;
    }
    // Blocking mode for read and writes
    fcntl(synclink_fd, F_SETFL, fcntl(synclink_fd, F_GETFL) & ~O_NONBLOCK);
    // Request to Send and Data Terminal Ready
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(synclink_fd, TIOCMBIS, &sigs);
    if (rc < 0) {
        blast_err("assert DTR/RTS error = %d %s", errno, strerror(errno));
        return rc;
    }
    return rc;
}

/*
 Synclink sends LSB first, but decom MSB, this is used to reverse the bits before sending
*/
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

