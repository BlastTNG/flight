/* 
 * store_data.c: 
 *
 * This software is copyright (C) 2013-2017 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 * Created on: May 4th, 2017 by Laura Fissel
 */
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <blast.h>
#include <blast_time.h>
#include <channels_tng.h>
#include <crc.h>
#include <derived.h>
#include <mputs.h>
#include <command_struct.h>
#include <store_data.h>
#include <diskmanager_tng.h>
#include <mcp.h>
#include <channel_macros.h>

#define MAX_NUM_FILENAME_CHARS 72

void get_write_file_name(char* fname, char* type, uint32_t index)
{
    static uint16_t extra_tag = 0;
    static int first_time = 1;
    if (first_time == 1) {
        srand48((uint64_t)time(NULL));
        extra_tag = (uint16_t) (lrand48());
		blast_info("Extra tag for writing to the harddrive is: %2x", extra_tag);
		first_time = 0;
    }
    snprintf(fname, MAX_NUM_FILENAME_CHARS, "rawdir/mcp_%02d-%02d-%02d_%02d:%02d_%2x/%s/%u_%s",
             start_time.tm_mday, start_time.tm_mon + 1 , start_time.tm_year + 1900,
             start_time.tm_hour, start_time.tm_min, extra_tag, type, index, type);
//    blast_info("Will store next %s frame to %s", type, fname);
}

void store_data_1hz(void)
{
    static channel_t *mcp_1hz_framenum_addr = NULL;
    uint32_t mcp_1hz_framenum = 0;
	char type_1hz[12] = "1hz";
	int temp_fd = -1;
	uint16_t bytes_written = 0;

    char file_name[MAX_NUM_FILENAME_CHARS];
    if (mcp_1hz_framenum_addr == NULL) {
        mcp_1hz_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
    }

    mcp_1hz_framenum = GET_INT32(mcp_1hz_framenum_addr);

    get_write_file_name(file_name, type_1hz, mcp_1hz_framenum);

    if (frame_size[RATE_1HZ]) {
        temp_fd = file_open(file_name, "w+");
	    if (temp_fd >= 0) {
	        // blast_info("Opened file %s for writing.", file_name);
            bytes_written = file_write(temp_fd, channel_data[RATE_1HZ], frame_size[RATE_1HZ]);
		    if (bytes_written < frame_size[RATE_1HZ]) {
                blast_err("%s frame size is %u bytes but we were only able to write %u bytes",
                            type_1hz, (uint16_t) frame_size[RATE_1HZ], bytes_written);
		    }
            file_close(temp_fd);
        } else {
	        blast_err("Failed to open file %s for writing.", file_name);
        }
    }
}

void store_data_5hz(void)
{
    static channel_t *mcp_5hz_framenum_addr = NULL;
    uint32_t mcp_5hz_framenum = 0;
	char type_5hz[12] = "5hz";
	int temp_fd = -1;
	uint16_t bytes_written = 0;

    char file_name[MAX_NUM_FILENAME_CHARS];
    if (mcp_5hz_framenum_addr == NULL) {
        mcp_5hz_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
    }

    mcp_5hz_framenum = GET_INT32(mcp_5hz_framenum_addr);

    get_write_file_name(file_name, type_5hz, mcp_5hz_framenum);

    if (frame_size[RATE_5HZ]) {
        temp_fd = file_open(file_name, "w+");
	    if (temp_fd >= 0) {
	        // blast_info("Opened file %s for writing.", file_name);
            bytes_written = file_write(temp_fd, channel_data[RATE_5HZ], frame_size[RATE_5HZ]);
		    if (bytes_written < frame_size[RATE_5HZ]) {
                blast_err("%s frame size is %u bytes but we were only able to write %u bytes",
                            type_5hz, (uint16_t) frame_size[RATE_5HZ], bytes_written);
		    }
            file_close(temp_fd);
        } else {
	        blast_err("Failed to open file %s for writing.", file_name);
        }
    }
}

void store_data_100hz(void)
{
    static channel_t *mcp_100hz_framenum_addr = NULL;
    uint32_t mcp_100hz_framenum = 0;
	char type_100hz[12] = "100hz";
	int temp_fd = -1;
	uint16_t bytes_written = 0;

    char file_name[MAX_NUM_FILENAME_CHARS];
    if (mcp_100hz_framenum_addr == NULL) {
        mcp_100hz_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
    }

    mcp_100hz_framenum = GET_INT32(mcp_100hz_framenum_addr);

    get_write_file_name(file_name, type_100hz, mcp_100hz_framenum);

    if (frame_size[RATE_100HZ]) {
        temp_fd = file_open(file_name, "w+");
	    if (temp_fd >= 0) {
	        // blast_info("Opened file %s for writing.", file_name);
            bytes_written = file_write(temp_fd, channel_data[RATE_100HZ], frame_size[RATE_100HZ]);
		    if (bytes_written < frame_size[RATE_100HZ]) {
                blast_err("%s frame size is %u bytes but we were only able to write %u bytes",
                            type_100hz, (uint16_t) frame_size[RATE_100HZ], bytes_written);
		    }
            file_close(temp_fd);
        } else {
	        blast_err("Failed to open file %s for writing.", file_name);
        }
    }
}

void store_data_200hz(void)
{
    static channel_t *mcp_200hz_framenum_addr = NULL;
    uint32_t mcp_200hz_framenum = 0;
	char type_200hz[12] = "200hz";
	int temp_fd = -1;
	uint16_t bytes_written = 0;

    char file_name[MAX_NUM_FILENAME_CHARS];
    if (mcp_200hz_framenum_addr == NULL) {
        mcp_200hz_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
    }

    mcp_200hz_framenum = GET_INT32(mcp_200hz_framenum_addr);

    get_write_file_name(file_name, type_200hz, mcp_200hz_framenum);

    if (frame_size[RATE_200HZ]) {
        temp_fd = file_open(file_name, "w+");
	    if (temp_fd >= 0) {
	        // blast_info("Opened file %s for writing.", file_name);
            bytes_written = file_write(temp_fd, channel_data[RATE_200HZ], frame_size[RATE_200HZ]);
		    if (bytes_written < frame_size[RATE_200HZ]) {
                blast_err("%s frame size is %u bytes but we were only able to write %u bytes",
                            type_200hz, (uint16_t) frame_size[RATE_200HZ], bytes_written);
		    }
            file_close(temp_fd);
        } else {
	        blast_err("Failed to open file %s for writing.", file_name);
        }
    }
}
