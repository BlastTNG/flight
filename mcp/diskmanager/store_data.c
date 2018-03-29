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
#include <roach.h>
#include <diskmanager_tng.h>
#include <mcp.h>
#include <crc.h>
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

void get_write_file_name(char* fname, char* chlist, char* type, uint32_t index)
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
    snprintf(chlist, MAX_NUM_FILENAME_CHARS, "rawdir/mcp_%02d-%02d-%02d_%02d:%02d_%2x/channel_list.dat",
    		 start_time.tm_mday, start_time.tm_mon + 1 , start_time.tm_year + 1900,
             start_time.tm_hour, start_time.tm_min, extra_tag);
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
    static bool first_time = 1;
    channel_header_t *channels_pkg = NULL;
    fileentry_t *fp_chlist;
    if (!store_disks_ready() && !(m_storage->have_warned)) {
        blast_info("store_disks_ready is returning false!");
        m_storage->have_warned = true;
        return;
    }

    m_storage->mcp_framenum = GET_INT32(m_storage->mcp_framenum_addr);
    // Once we have initialized write out the entire frame structure so that defricher can reconstruct the dirfile.
    if (first_time) {
        if (!(channels_pkg = channels_create_map(channel_list))) {
        	blast_err("Exiting framing routine because we cannot get the channel list");
        	return;
    	}
    	get_write_file_name(m_storage->file_name, m_storage->chlist_name, m_storage->type, m_storage->mcp_framenum);
    	fp_chlist = file_open(m_storage->chlist_name, "w+");
    	bytes_written = file_write(fp_chlist, (void*) channels_pkg, sizeof(channels_pkg));
    	file_close(fp_chlist);
    	first_time = 0;
    }
    if (frame_size[m_storage->rate]) {
        if (!(m_storage->fp)) {
            get_write_file_name(m_storage->file_name, m_storage->chlist_name, m_storage->type, m_storage->mcp_framenum);
		    blast_info("Opening %s", m_storage->file_name);
            m_storage->fp = file_open(m_storage->file_name, "w+");
            m_storage->header_written = false;
            m_storage->crc = BLAST_MAGIC32;
        }
	    if (m_storage->fp) {
	        // Have we already written enough data to the file so that a new one should be opened?
	        if ((m_storage->frames_stored) >= STORE_DATA_FRAMES_PER_FILE * m_storage->nrate) {
	        	bytes_written = file_write(m_storage->fp, (void*) &(m_storage->crc), sizeof(uint32_t));
    	        blast_info("Closing %s", m_storage->file_name);
                file_close(m_storage->fp);
                m_storage->header_written = false;
                m_storage->crc = BLAST_MAGIC32;
                get_write_file_name(m_storage->file_name, m_storage->chlist_name,
                                    m_storage->type, m_storage->mcp_framenum);
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
		        // We wrote the frame successfully.  Update the crc.
                (m_storage->frames_stored)++;
                m_storage->have_warned = false;
                m_storage->crc = crc32(m_storage->crc, channel_data[m_storage->rate], frame_size[m_storage->rate]);
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

// Write each udp packet to the harddisk.
void store_roach_udp_packet(data_udp_packet_t *m_packet, roach_handle_data_t *m_roach_udp,
                            uint16_t packet_err)
{
	int temp_fd = -1;
	uint16_t bytes_written = 0;
    roach_packet_header_out_t packet_header_out;
    char type_roach[7];
    char file_name[MAX_NUM_FILENAME_CHARS];
    char channel_list_name[MAX_NUM_FILENAME_CHARS];
    size_t header_size, packet_size;

    bytes_written = snprintf(type_roach, sizeof(type_roach), "roach%i", m_roach_udp->index + 1);
    if (bytes_written < (sizeof(type_roach)-1)) {
        blast_err("Could not print roach type string!  bytes_written = %u, sizeof(type_roach) = %i, type_roach = %s",
                   bytes_written, (int) sizeof(type_roach), type_roach);
        return;
    }
    get_write_file_name(file_name, channel_list_name, type_roach, m_roach_udp->roach_packet_count);

    header_size = sizeof(packet_header_out);
    packet_size = sizeof(*m_packet);
    // Write header information for the packet.
    packet_header_out.packet_err_code = packet_err;
    packet_header_out.write_time = time(NULL); // Time before we call write to harddrive.
    packet_header_out.packet_crc = crc32(BLAST_MAGIC32, m_packet, packet_size); // CRC of the packet
    packet_header_out.which = m_roach_udp->which;
    packet_header_out.want_reset = m_roach_udp->want_reset;
    packet_header_out.port = m_roach_udp->port;
    packet_header_out.roach_packet_count = m_roach_udp->roach_packet_count;

    temp_fd = file_open(file_name, "w+");
	if (temp_fd >= 0) {
	    // blast_info("Opened file %s for writing.", file_name);
        bytes_written = file_write(temp_fd, (char*) (&packet_header_out), header_size);
        if (bytes_written < header_size) {
            blast_err("%s packet header size is %u bytes but we were only able to write %u bytes",
                      type_roach, (uint16_t) header_size, bytes_written);
		}
	    if (packet_size) {
            bytes_written = file_write(temp_fd, (char*) m_packet, packet_size);
            if (bytes_written < packet_size) {
                blast_err("%s packet header size is %u bytes but we were only able to write %u bytes",
                         type_roach, (uint16_t) packet_size, bytes_written);
            }
		}
        file_close(temp_fd);
    } else {
	    blast_err("Failed to open file %s for writing.", file_name);
    }
}
