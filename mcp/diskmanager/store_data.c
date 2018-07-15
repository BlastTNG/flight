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

#include <linklist.h>
#include <linklist_compress.h>
#include <linklist_writer.h>

typedef struct {
    bool    header_written;
    bool    have_warned;
    bool    init;
    uint32_t    crc;
    uint32_t mcp_framenum;
    uint32_t frames_stored;
    E_RATE rate;
    uint16_t nrate;
    char file_name[MAX_NUM_FILENAME_CHARS];
    char chlist_name[MAX_NUM_FILENAME_CHARS];
    char type[12];
    fileentry_t *fp;
    channel_t *mcp_framenum_addr;
    channel_header_t *channels_pkg;
} store_file_info_t;

static store_file_info_t storage_info_1hz = {0};
static store_file_info_t storage_info_5hz = {0};
static store_file_info_t storage_info_100hz = {0};
static store_file_info_t storage_info_200hz = {0};
static store_file_info_t storage_info_hk = {0};
// =======
// #define MAX_NUM_FILENAME_CHARS 72
typedef struct {
    fileentry_t *fp;
    uint32_t pkts_written_ct;
    char type[7];
    char file_name[MAX_NUM_FILENAME_CHARS];
} roach_udp_write_info_t;

roach_udp_write_info_t roach_udp_write_info[NUM_ROACHES];
// >>>>>>> origin/master

// housekeeping linklist
#define LL_TMP_NAME "/tmp/TMP"
linklist_t * ll_hk = NULL;
extern unsigned int ll_rawfile_default_fpf;

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
// =======
// // Handles the file_open, file_write, and file_close calls.
// static int store_data(fileentry_t **m_fp, char *m_file, char *m_type, uint16_t m_rate,
//                 int32_t m_frame_number, uint32_t *m_counter, uint16_t m_freq)
// {
//     uint16_t bytes_written = 0;
//     if ((*m_counter) >= STORE_DATA_FRAMES_PER_FILE * m_freq) {
//     	blast_info("Closing %s", m_file);
//         file_close(*m_fp);
//         get_write_file_name(m_file, m_type, m_frame_number);
// 		blast_info("Opening %s", m_file);
//         *m_fp = file_open(m_file, "w+");
//         (*m_counter) = 0;
//     }
// >>>>>>> origin/master
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

unsigned int copy_file_to_diskmanager(char * fileout, char * filein) {
    fileentry_t *fp_chlist = file_open(filein, "w+");
    unsigned int bytes_written = 0;
    unsigned int bytes_read = 0;

    if (fp_chlist == NULL) {
        return 0;
    }

    FILE * fp = fopen(filein, "r");
    if (fp == NULL) {
        file_close(fp_chlist);
        return 0;
    }

    uint8_t buffer[1024] = {0};

    while (!feof(fp)) {
        bytes_read = fread(buffer, 1, sizeof(buffer), fp);
        bytes_written += file_write(fp_chlist, (void *) buffer, bytes_read);
        memset(buffer, 0, sizeof(buffer));
    }

    fclose(fp);
    file_close(fp_chlist);

    return bytes_written;
}

void make_hk_name(char * filename) {
    // get the date string for file saving
    time_t now = time(0);
    char datestring[80] = {0};
    struct tm tm_t = {0};
    localtime_r(&now, &tm_t);
    strftime(datestring, sizeof(datestring)-1, "%Y-%m-%d-%H-%M-%S", &tm_t);
    char tempname[80] = {0};
    snprintf(tempname, sizeof(tempname), "master_%s", datestring);
    snprintf(filename, MAX_NUM_FILENAME_CHARS, "%s/%s/%s", archive_dir, tempname, tempname);
}

void store_data_hk(uint8_t * sf_buffer) {
    uint16_t bytes_written = 0;
    char fileout_name[MAX_NUM_FILENAME_CHARS] = {0};

    static int file_index = 0;
    static uint8_t * comp_buffer = NULL;
    static bool first_time = 1;
    if (!store_disks_ready() && !(storage_info_hk.have_warned)) {
        blast_info("store_disks_ready is not ready.");
        storage_info_hk.have_warned = true;
        return;
    }

    if (first_time) {
        // initialize the store_file_info struct
	      storage_info_hk.fp = NULL;
	      snprintf(storage_info_hk.type, sizeof(storage_info_hk.type), "linklist");
	      storage_info_hk.mcp_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
	      storage_info_hk.channels_pkg = NULL;
	      storage_info_hk.rate = RATE_1HZ;
	      storage_info_hk.nrate = 1;
	      storage_info_hk.init = true;
        storage_info_hk.frames_stored = 0;
        storage_info_hk.fp = NULL;

        // allocate buffer
        comp_buffer = calloc(1, ll_hk->blk_size);

        // open and write the temporary superframe, linklist, and calspecs format files
        snprintf(archive_dir, sizeof(archive_dir), "rawdir");
        ll_rawfile_default_fpf = 900;

        linklist_rawfile_t * ll_rawfile = open_linklist_rawfile(LL_TMP_NAME, ll_hk);
        channels_write_calspecs(LL_TMP_NAME CALSPECS_FORMAT_EXT, derived_list);
        close_and_free_linklist_rawfile(ll_rawfile);

        // copy the format files to the diskmanager
        make_hk_name(storage_info_hk.file_name);
        snprintf(fileout_name, MAX_NUM_FILENAME_CHARS, "%s" LINKLIST_FORMAT_EXT, storage_info_hk.file_name);
        bytes_written = copy_file_to_diskmanager(fileout_name, LL_TMP_NAME LINKLIST_FORMAT_EXT);

        snprintf(fileout_name, MAX_NUM_FILENAME_CHARS, "%s" SUPERFRAME_FORMAT_EXT, storage_info_hk.file_name);
        bytes_written = copy_file_to_diskmanager(fileout_name, LL_TMP_NAME SUPERFRAME_FORMAT_EXT);
        snprintf(fileout_name, MAX_NUM_FILENAME_CHARS, "%s" CALSPECS_FORMAT_EXT, storage_info_hk.file_name);
        bytes_written = copy_file_to_diskmanager(fileout_name, LL_TMP_NAME CALSPECS_FORMAT_EXT);

        first_time = 0;
    }

    // close the file once enough frames are written
    if (storage_info_hk.frames_stored >= ll_rawfile_default_fpf) {
	      bytes_written = file_write(storage_info_hk.fp, (void*) &(storage_info_hk.crc), sizeof(uint32_t));
        file_close(storage_info_hk.fp);
        storage_info_hk.fp = NULL;
    }

    // open the file if it hasn't been opened or was closed
    if (!storage_info_hk.fp) {
        snprintf(fileout_name, MAX_NUM_FILENAME_CHARS, "%s" LINKLIST_EXT ".%.2u",
                                storage_info_hk.file_name, file_index);
        storage_info_hk.frames_stored = 0;
        storage_info_hk.crc = BLAST_MAGIC32;
        storage_info_hk.fp = file_open(fileout_name, "w+");
        file_index++;
    }

    // write to the fie if opened
    if (storage_info_hk.fp) {
        // compress the linklist
        compress_linklist(comp_buffer, ll_hk, sf_buffer);
        bytes_written = file_write(storage_info_hk.fp, (void *) comp_buffer, ll_hk->blk_size);
		    if (bytes_written < ll_hk->blk_size) {
            if (storage_info_hk.have_warned) {
                blast_err("Compressed size is %u bytes but we were only able to write %u bytes",
                            ll_hk->blk_size, bytes_written);
                storage_info_hk.have_warned = true;
            }
		    } else {
		        // We wrote the frame successfully.
            (storage_info_hk.frames_stored)++;
            storage_info_hk.have_warned = false;
            storage_info_hk.crc = crc32(storage_info_hk.crc, comp_buffer, ll_hk->blk_size);
		    }
		} else {
		  	if (storage_info_hk.have_warned) {
			  		blast_err("Failed to open file %s for writing.", storage_info_hk.file_name);
				  	storage_info_hk.have_warned = true;
		  	}
		}
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
// =======
// 	    if (temp_fp) {
//            store_data(&temp_fp, file_name, type_1hz, RATE_1HZ,
//                                    mcp_1hz_framenum, &frames_stored_to_1hz, 1);
// >>>>>>> origin/master
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
    static int first_call = 1;
	roach_udp_write_info_t* m_roach_write;
	uint16_t bytes_written = 0;
    char channel_list_name[MAX_NUM_FILENAME_CHARS];
    size_t header_size, packet_size;
    roach_packet_header_out_t packet_header_out;

	if (m_roach_udp->first_packet) {
	    blast_info("Called store_roach_udp_packet for the first packet of roach%u", m_roach_udp->which);
	}

    if (first_call) { // Initialize the roach_udp_write_info structure.
        blast_info("Initializing roach_udp_write_info structure.");
        for (int i = 0; i < NUM_ROACHES; i++) {
            roach_udp_write_info[i].fp = NULL;
            roach_udp_write_info[i].pkts_written_ct = 0;
            bytes_written = snprintf(roach_udp_write_info[i].type,
                                     sizeof(roach_udp_write_info[i].type),
                                     "roach%i", i + 1);
            blast_info("Set type for roach%d to %s", i, roach_udp_write_info[i].type);
            if (bytes_written < (sizeof(roach_udp_write_info[i].type)-1)) {
                   blast_err("Could not print roach type string!  bytes_written = %u, sizeof(type) = %i, type = %s",
                   bytes_written, (int) sizeof(roach_udp_write_info[i].type), roach_udp_write_info[i].type);
                   return;
            }
        }
        first_call = 0;
    }
//    get_write_file_name(file_name, channel_list_name, type_roach, m_roach_udp->roach_packet_count);

    if (!store_disks_ready()) return;

     m_roach_write = (roach_udp_write_info_t*) &(roach_udp_write_info[m_roach_udp->i_which]);
// >>>>>>> origin/master

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

    if ((m_roach_write->pkts_written_ct >= STORE_DATA_FRAMES_PER_FILE * 488) || !(m_roach_write->fp)) {
        if (m_roach_write->fp) {
    	    blast_info("Closing %s for roach%u", m_roach_write->file_name, m_roach_udp->which);
            file_close(m_roach_write->fp);
        }
        get_write_file_name(m_roach_write->file_name, channel_list_name,
                            m_roach_write->type, m_roach_udp->roach_packet_count);
		blast_info("Opening %s for roach%u", m_roach_write->file_name, m_roach_udp->which);
        m_roach_write->fp = file_open(m_roach_write->file_name, "w+");
        m_roach_write->pkts_written_ct = 0;
    }
    if (m_roach_write->fp) {
	    // blast_info("writing to %s", m_file);
        bytes_written = file_write(m_roach_write->fp, (char*) (&packet_header_out), header_size);
		if (bytes_written < header_size) {
            blast_err("%s header size is %u bytes but we were only able to write %u bytes",
                        m_roach_write->type, (uint16_t) header_size, bytes_written);
		}
        bytes_written = file_write(m_roach_write->fp, (char*) m_packet, packet_size);
		if (bytes_written < packet_size) {
            blast_err("%s packet size is %u bytes but we were only able to write %u bytes",
                        m_roach_write->type, (uint16_t) packet_size, bytes_written);
		} else {
		    // We wrote the frame successfully.
            (m_roach_write->pkts_written_ct)++;
		}
    }
}
