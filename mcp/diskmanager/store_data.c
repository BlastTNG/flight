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
#include <derived.h>
#include <mputs.h>
#include <command_struct.h>
#include <diskmanager_tng.h>
#include <mcp.h>
#include <channel_macros.h>
#include <store_data.h>

static store_file_info_t storage_info_1hz = {0};
static store_file_info_t storage_info_5hz = {0};
static store_file_info_t storage_info_100hz = {0};
static store_file_info_t storage_info_200hz = {0};

int store_disks_ready() {
	static bool disk_init = false;
    if (!disk_init) {
        if (check_disk_init()) {
            blast_info("check_disk_init passed!");
            disk_init = true;
        }
    }
    if (disk_init) {
        return(1);
    } else {
        return(0);
    }
}

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

int store_data_header(fileentry_t **m_fp, channel_header_t *m_channels_pkg, char *m_type) {
    size_t pkg_size = sizeof(channel_header_t) + m_channels_pkg->length * sizeof(struct channel_packed);
    size_t bytes_written = 0;
    if (*m_fp) {
        bytes_written = file_write(*m_fp, (void*) m_channels_pkg, pkg_size);
		if (bytes_written < pkg_size) {
            blast_err("%s package header is %u bytes but we were only able to write %u bytes",
                        m_type, (uint16_t) pkg_size, (uint16_t) bytes_written);
		} else {
		    // We wrote the frame successfully.
		}
		return(bytes_written);
	} else {
	    blast_err("File pointer is invalid.  Returning -1");
	    return(-1);
	}
    return(0);
}

// Generic data storage routine that can be used for any rate or roach data;
void store_rate_data(store_file_info_t *m_storage) {
    // Checks the s_ready flag in diskmanager.
    uint16_t bytes_written = 0;
    if (!store_disks_ready() && !(m_storage->have_warned)) {
        blast_info("store_disks_ready is returning false!");
        m_storage->have_warned = true;
        return;
    }
    m_storage->mcp_framenum = GET_INT32(m_storage->mcp_framenum_addr);
    if (frame_size[m_storage->rate]) {
        if (!(m_storage->fp)) {
            get_write_file_name(m_storage->file_name, m_storage->type, m_storage->mcp_framenum);
		    blast_info("Opening %s", m_storage->file_name);
            m_storage->fp = file_open(m_storage->file_name, "w+");
            m_storage->header_written = false;
        }
	    if (m_storage->fp) {
	        // Have we already written enough data to the file so that a new one should be opened?
	        if ((m_storage->frames_stored) >= STORE_DATA_FRAMES_PER_FILE * m_storage->nrate) {
    	        blast_info("Closing %s", m_storage->file_name);
                file_close(m_storage->fp);
                m_storage->header_written = false;
                get_write_file_name(m_storage->file_name, m_storage->type, m_storage->mcp_framenum);
		        blast_info("Opening %s", m_storage->file_name);
                m_storage->fp = file_open(m_storage->file_name, "w+");
                (m_storage->frames_stored) = 0;
            }
            // Write the header to the beginning of the file;
	    	if (!m_storage->header_written) {
	    	    bytes_written = store_data_header(&m_storage->fp, m_storage->channels_pkg, m_storage->type);
	    	    if (bytes_written > 0) {
	    	        m_storage->header_written = true;
	    	    }
	    	}
            // Write the data to the file.
        	bytes_written = file_write(m_storage->fp, channel_data[m_storage->rate], frame_size[m_storage->rate]);
		    if (bytes_written < frame_size[m_storage->rate]) {
                if (m_storage->have_warned) {
                    blast_err("%s frame size is %u bytes but we were only able to write %u bytes",
                                m_storage->type, (uint16_t) frame_size[m_storage->rate], bytes_written);
                    m_storage->have_warned = true;
                }
		    } else {
		        // We wrote the frame successfully.
                (m_storage->frames_stored)++;
                m_storage->have_warned = false;
		    }
        } else {
	        if (m_storage->have_warned) {
	            blast_err("Failed to open file %s for writing.", m_storage->file_name);
	            m_storage->have_warned = true;
	        }
        }
    }
}

void store_data_1hz(void)
{
	if (!storage_info_1hz.init) {
	    storage_info_1hz.fp = NULL;
	    snprintf(storage_info_1hz.type, sizeof(storage_info_1hz.type), "1hz");
	    storage_info_1hz.mcp_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
	    storage_info_1hz.channels_pkg = channels_create_rate_map(channel_list, RATE_1HZ);
	    storage_info_1hz.rate = RATE_1HZ;
	    storage_info_1hz.nrate = 1;
	    storage_info_1hz.init = true;
	}
	store_rate_data(&storage_info_1hz);
}

void store_data_5hz(void)
{
	if (!storage_info_5hz.init) {
	    storage_info_5hz.fp = NULL;
	    snprintf(storage_info_5hz.type, sizeof(storage_info_5hz.type), "5hz");
	    storage_info_5hz.mcp_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
	    storage_info_5hz.channels_pkg = channels_create_rate_map(channel_list, RATE_5HZ);
	    storage_info_5hz.rate = RATE_5HZ;
	    storage_info_5hz.nrate = 5;
	    storage_info_5hz.init = true;
	}
	store_rate_data(&storage_info_5hz);
}

void store_data_100hz(void)
{
	if (!storage_info_100hz.init) {
	    storage_info_100hz.fp = NULL;
	    snprintf(storage_info_100hz.type, sizeof(storage_info_100hz.type), "100hz");
	    storage_info_100hz.mcp_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
	    storage_info_100hz.channels_pkg = channels_create_rate_map(channel_list, RATE_100HZ);
	    storage_info_100hz.rate = RATE_100HZ;
	    storage_info_100hz.nrate = 100;
	    storage_info_100hz.init = true;
	}
	store_rate_data(&storage_info_100hz);
}

void store_data_200hz(void)
{
	if (!storage_info_200hz.init) {
	    storage_info_200hz.fp = NULL;
	    snprintf(storage_info_200hz.type, sizeof(storage_info_200hz.type), "200hz");
	    storage_info_200hz.mcp_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
	    storage_info_200hz.channels_pkg = channels_create_rate_map(channel_list, RATE_200HZ);
	    storage_info_200hz.rate = RATE_200HZ;
	    storage_info_200hz.nrate = 200;
	    storage_info_200hz.init = true;
	}
	store_rate_data(&storage_info_200hz);
}
