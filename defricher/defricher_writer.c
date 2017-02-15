/* 
 * dirfile_writer.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 3, 2015 by Seth Hillbrand
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <getdata.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <glib.h>
#include <bzlib.h>

#include <channels_tng.h>
#include <channel_macros.h>
#include <derived.h>
#include <blast.h>

#include "defricher.h"
#include "defricher_utils.h"
#include "defricher_data.h"

extern struct ri_struct ri;

channel_t *channels = NULL;
channel_t *new_channels = NULL;
derived_tng_t *derived_channels = NULL;

static GAsyncQueue *packet_queue = NULL;
typedef union {
    gpointer ptr;
    struct {
        uint8_t rate;
        uint16_t _dummy;
    };
} queue_data_t;

static int dirfile_offset = -1;
static int dirfile_create_new = 0;
static int dirfile_update_derived = 0;
static bool dirfile_ready = false;
static int dirfile_frames_written = 0;

static void defricher_add_derived(DIRFILE *m_file, derived_tng_t *m_derived)
{
    char lincom2_fields[2][FIELD_LEN];
    const char *lincom2_ptrs[2] = { lincom2_fields[0], lincom2_fields[1] };
    double lincom2_m[2];
    double lincom2_b[2];
    const char *lincom_name;

    for (derived_tng_t *derived = m_derived; derived && derived->type != DERIVED_EOC_MARKER; derived++) {
        switch (derived->type) {
            case 'w':
            case 'b':
                gd_add_bit(m_file, derived->bitword.field, derived->bitword.source, derived->bitword.offset, derived->bitword.length, 0);
                break;
            case 't':
                gd_add_linterp(m_file, derived->linterp.field, derived->linterp.source, derived->linterp.lut, 0);
                break;
            case 'c':
                lincom_name = derived->lincom.source;
                gd_add_lincom(m_file, derived->lincom.field, 1, &lincom_name, &derived->lincom.m_c2e, &derived->lincom.b_e2e, 0);
                break;
            case '2':
                strcpy(lincom2_fields[0], derived->lincom2.source);
                strcpy(lincom2_fields[1], derived->lincom2.source2);
                lincom2_b[0] = derived->lincom2.b_e2e;
                lincom2_b[1] = derived->lincom2.b2_e2e;
                lincom2_m[0] = derived->lincom2.m_c2e;
                lincom2_m[1] = derived->lincom2.m2_c2e;

                gd_add_lincom(m_file, derived->lincom2.field, 2, lincom2_ptrs, lincom2_m, lincom2_b, 0);
                break;
            case '#':
                break;
            case 'u':
                ///TODO: Remove this break after getdata library updated to avoid segfault here
                break;
                if (derived->units.units[0])
                    gd_madd_string(m_file, derived->units.source, "units", derived->units.units);
                if (derived->units.quantity[0])
                    gd_madd_string(m_file, derived->units.source, "quantity", derived->units.quantity);
                break;
            case 'p':
                gd_add_phase(m_file, derived->phase.field, derived->phase.source, derived->phase.shift, 0);
                break;
            case 'r':
                gd_add_recip(m_file, derived->recip.field, derived->recip.source, derived->recip.dividend, 0);
                break;
            case '*':
                gd_add_multiply(m_file, derived->math.field, derived->math.source, derived->math.source2, 0);
                break;
            case '/':
                gd_add_divide(m_file, derived->math.field, derived->math.source, derived->math.source2, 0);
                break;
            case 'x':
                gd_add_mplex(m_file, derived->mplex.field, derived->mplex.source, derived->mplex.index, derived->mplex.value, derived->mplex.max, 0);
                break;
            default:
                defricher_warn("Unknown type %c", derived->type);
                break;
        }
    }
    gd_metaflush(m_file);
}

//TODO: Fix get_new_dirfilename to take options into account
static char *defricher_get_new_dirfilename(void)
{
    char *filename = NULL;
    static time_t start_time = 0;
    static uint32_t chunk = 0;

    if (!start_time) start_time = time(NULL);
    if (asprintf(&filename, "%s/%lu_%03X.dirfile", rc.dest_dir, start_time, chunk++) < 0)
    {
        defricher_fatal("Could not allocate memory for filename!");
        filename=NULL;
    }

    return filename;
}
static void defricher_update_current_link(const char *m_name)
{
    unlink(rc.symlink_name);

    defricher_mkdir_file(rc.symlink_name,1);
    if (symlink(m_name, rc.symlink_name))
        defricher_strerr("Could not create symlink from %s to %s", m_name, rc.symlink_name);
}

static void defricher_free_channels_list (channel_t *m_channel_list)
{
    if (!m_channel_list) return;

    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        defricher_cache_node_t *node = (defricher_cache_node_t*)channel->var;
        if (    node &&
                node->magic == BLAST_MAGIC32)
        {
            if (rc.bzip_output && node->output.bzfp) {
            	BZ2_bzclose(node->output.bzfp);
            	node->output.bzfp = NULL;
            }
            if (!rc.bzip_output && node->output.fp) {
                if (fclose(node->output.fp))
                    defricher_strerr("Could not close %s", node->output.name?node->output.name:"UNK");
                node->output.fp = NULL;
            }
            free(node);
            channel->var = NULL;
        }

    }
}


static int defricher_update_cache_fp(DIRFILE *m_dirfile, channel_t *m_channel_list)
{
    char *filename;
    char error_str[2048];

    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        defricher_cache_node_t *outfile_node = channel->var;
        if (outfile_node && outfile_node->magic == BLAST_MAGIC32) {
            filename = gd_raw_filename(m_dirfile, outfile_node->output.name);
            if (!filename) {
                gd_error_string(m_dirfile, error_str, 2048);
                defricher_err("Could not get filename for %s: %s", outfile_node->output.name, error_str);
                return -1;
            }

            if (rc.bzip_output)
            	outfile_node->output.bzfp = BZ2_bzopen(filename, "w+");
            else
            	outfile_node->output.fp = fopen(filename, "w+");

            if ((rc.bzip_output && !outfile_node->output.bzfp) ||
            	(!rc.bzip_output && !outfile_node->output.fp)) {
                defricher_err("Could not open %s", filename);
                free(filename);
                switch(errno) {
                    case ELOOP:
                        defricher_err("Too many loops in the directory %s.  Please remove the directory and try again.", outfile_node->output.name);
                        break;
                    case EPERM:
                    case EACCES:
                        defricher_err("Could not create %s.  Please check that you have permission to write to the parent directory.", outfile_node->output.name);
                        break;
                    case EMFILE:
                        defricher_err("Too many open files.  Typically this happens when your system limit is too low.");
                        defricher_err("\tOn Linux and Mac, you can check the number of open files you allow by typing 'ulimit -n' in a terminal.");
                        defricher_err("\tAs a general rule of thumb, you should allocate at least 65536.");
                        defricher_err("\tTo do this on Linux, you can put `* soft nofiles 65536` and `* hard nofiles 65536` in /etc/security/limits.conf");
                        defricher_err("\tTo do this on Mac, you can put `limit maxfiles 65536` in /etc/launchd.conf");
                        defricher_err("\tOn both systems, you can then reboot for the changes to take effect");
                        break;
                    case ENOSPC:
                        defricher_err("No more space on device.  Please clear additional storage space and try again.");
                        break;
                    default:
                        defricher_strerr("Unhandled error while opened files.");
                        break;
                }
                return -1;
            }
            free(filename);
            outfile_node->output.offset = 0;
        }
    }
    return 0;
}
static inline gd_type_t defricher_get_gd_type(const E_TYPE m_type)
{
    gd_type_t type = GD_UNKNOWN;
    switch (m_type) {
        case TYPE_INT8:
            type = GD_INT8;
            break;
        case TYPE_UINT8:
            type = GD_UINT8;
            break;
        case TYPE_INT16:
            type = GD_INT16;
            break;
        case TYPE_UINT16:
            type = GD_UINT16;
            break;
        case TYPE_INT32:
            type = GD_INT32;
            break;
        case TYPE_UINT32:
            type = GD_UINT32;
            break;
        case TYPE_INT64:
            type = GD_INT64;
            break;
        case TYPE_UINT64:
            type = GD_UINT64;
            break;
        case TYPE_FLOAT:
            type = GD_FLOAT32;
            break;
        case TYPE_DOUBLE:
            type = GD_FLOAT64;
            break;
        default:
            defricher_err( "Unknown type %d", m_type);
    }
    return type;
}

static inline int defricher_get_rate(const E_RATE m_rate)
{
    E_RATE rate = -1;
    switch (m_rate) {
        case RATE_1HZ:
            rate = 1;
            break;
        case RATE_5HZ:
            rate = 5;
            break;
        case RATE_100HZ:
            rate = 100;
            break;
        case RATE_200HZ:
            rate = 200;
            break;
        case RATE_244HZ:
            rate = 244;
            break;
        default:
            defricher_err( "Unknown rate %d", m_rate);
    }
    return rate;
}

static void defricher_transfer_channel_list(channel_t *m_channel_list)
{
    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        gd_type_t type;
        defricher_cache_node_t *node = balloc(fatal, sizeof(defricher_cache_node_t));
        node->magic = BLAST_MAGIC32;
        node->raw_data = channel->var;
        node->output.name = channel->field;
        node->output.desc = NULL;
        node->output.lastval = 0;
        node->output.offset = 0;

        type = defricher_get_gd_type(channel->type);
        node->output.element_size = GD_SIZE(type);
        node->output.is_float = type & GD_IEEE754;
        node->output.is_signed = type & GD_SIGNED;

        node->output.rate = defricher_get_rate(channel->rate);
        channel->var = node;
    }
}

static DIRFILE *defricher_init_new_dirfile(const char *m_name, channel_t *m_channel_list)
{
    DIRFILE *new_file;
    gd_type_t type;
    char tmp_str[512];
    double m,b;

    defricher_info("Intializing %s", m_name);
    if (defricher_mkdir_file(m_name, true) < 0)
    {
        defricher_strerr("Could not make directory for %s", m_name);
        return false;
    }

    new_file = gd_open(m_name, GD_CREAT|GD_RDWR|GD_PRETTY_PRINT|
    							(rc.bzip_output?GD_BZIP2_ENCODED:0)|GD_VERBOSE);
    if (gd_error(new_file) == GD_E_OK)
    {
        m = 1e-5;
        b = 0.0;

        gd_alter_endianness(new_file, GD_BIG_ENDIAN, 0, 0);
        
        for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
            
            defricher_cache_node_t *node = (defricher_cache_node_t*)channel->var;

            type = defricher_get_gd_type(channel->type);
            
            if (gd_add_raw(new_file, node->output.name, type, node->output.rate, 0))
            {
                gd_error_string(new_file, tmp_str, 512);
                defricher_err( "Could not add %s: %s", node->output.name, tmp_str);
            }
            else
            {
                snprintf(tmp_str, 128, "%s", node->output.name);
                m = channel->m_c2e;
                b = channel->b_e2e;

                /// By default we set the converted field to upper case
                for (int i = 0; tmp_str[i]; i++) tmp_str[i] = toupper(tmp_str[i]);
                /// If our scale/offset are unity/zero respectively, tell defile to use the easier zero-phase
                if (fabs(m - 1.0) <= DBL_EPSILON && fabs(b - 0.0) <= DBL_EPSILON)
                    gd_add_phase(new_file, tmp_str, node->output.name, 0, 0);
                else
                    gd_add_lincom(new_file, tmp_str, 1, (const char **)&(node->output.name), &m, &b, 0);
            }
        }

        gd_reference(new_file, "mcp_200hz_framecount");
        gd_alter_frameoffset(new_file, dirfile_offset, GD_ALL_FRAGMENTS, 0);
        gd_metaflush(new_file);
    }
    else
    {
        gd_error_string(new_file, tmp_str, 512);
        defricher_fatal("Could not open %s as DIRFILE: %s", m_name, tmp_str);
        gd_close(new_file);
        return NULL;
    }

    return new_file;
}

void defricher_queue_packet(uint16_t m_rate)
{
    queue_data_t new_pkt = {._dummy = (1<<15)};//Set this bit to avoid glib assertions

    if (dirfile_offset < 0) {
        if (m_rate == RATE_1HZ) {
            channel_t *frame_offset = channels_find_by_name("mcp_1hz_framecount");
            if (frame_offset) {
                defricher_cache_node_t *outfile_node = frame_offset->var;
                dirfile_offset = be32toh(*outfile_node->_32bit_data);
                defricher_info("Setting offset to %d", dirfile_offset);
            }
            else {
                defricher_err("Missing \"mcp_1hz_framecount\" channel.  Please report this!");
                dirfile_offset = 0;
            }
        } else {
            /**
             * We are looking for the 1Hz packet to mark the start of a new frame.  Until we receive
             * it, we discard the extra sub-frames.
             */
            return;
        }
    }
    new_pkt.rate = m_rate;

    g_async_queue_push(packet_queue, new_pkt.ptr);
}


static int defricher_write_packet(uint16_t m_rate)
{
    static int have_warned = 1;

    if (ri.writer_done) {
        defricher_info("Not writing frame due to shutdown request");
        return -1;
    }

    if (!dirfile_ready) {
        if (!have_warned) defricher_info("Discarding frame due to DIRFILE not being ready for writing");
        have_warned = 1;
        return -1;
    }

    if (m_rate == RATE_200HZ) ri.wrote ++;
    have_warned = 0;
    for (channel_t *channel = channels; channel->field[0]; channel++) {
        if (channel->rate != m_rate) continue;
        defricher_cache_node_t *outfile_node = channel->var;
        if (outfile_node && outfile_node->magic == BLAST_MAGIC32 &&
        		((rc.bzip_output && outfile_node->output.bzfp) ||
				(!rc.bzip_output && outfile_node->output.fp))) {
        	if ((rc.bzip_output && BZ2_bzwrite(outfile_node->output.bzfp, outfile_node->raw_data,
        			outfile_node->output.element_size) != outfile_node->output.element_size) ||
        		(!rc.bzip_output && fwrite(outfile_node->raw_data, outfile_node->output.element_size,
            		1, outfile_node->output.fp) != 1)) {
                defricher_err( "Could not write to %s", outfile_node->output.name);
                continue;
            }
        }
    }

    dirfile_frames_written++;
    return 0;
}

static void *defricher_write_loop(void *m_arg)
{
    DIRFILE *dirfile = NULL;
    char *dirfile_name = NULL;

    bprintf(info, "Starting Defricher Write task\n");

    while (!ri.writer_done)
    {
        if (ri.new_channels) {
            ri.channels_ready = false;
            ri.new_channels = false;
            if (channels_initialize(new_channels) < 0) {
                defricher_err("Could not initialize channels");
                free(new_channels);
                new_channels = NULL;
            } else {
                defricher_free_channels_list(channels);
                if (channels) free(channels);
                channels = new_channels;
                new_channels = NULL;
                defricher_transfer_channel_list(channels);
                ri.channels_ready = true;
                dirfile_create_new = 1;
                dirfile_offset = -1;
                dirfile_ready = false;
            }

        }
        if (dirfile_create_new && dirfile_offset >= 0) {
            dirfile_name = rc.output_dirfile;
            rc.output_dirfile = defricher_get_new_dirfilename();
            BLAST_SAFE_FREE(dirfile_name);

            if (dirfile) gd_close(dirfile);
            dirfile = NULL;

            if (!(dirfile = defricher_init_new_dirfile(rc.output_dirfile, channels))) {
                defricher_err( "Not creating new DIRFILE %s", rc.output_dirfile);
                sleep(1);
                continue;
            }
            if (defricher_update_cache_fp(dirfile, channels) < 0) {
                defricher_err("Could not open all files for writing.  "
                		"Please report this error to Seth (seth.hillbrand@gmail.com)");
                ri.writer_done = true;
            } else {
                dirfile_create_new = 0;
                dirfile_ready = true;
                ri.symlink_updated = false;
                dirfile_frames_written = 0;
            }
        }

        /// We wait until a frame is written to the dirfile before updating the link (for KST)
        if (dirfile_frames_written > 400 && !ri.symlink_updated) {
            defricher_update_current_link(rc.output_dirfile);
            ri.symlink_updated = true;
        }

        if (dirfile_ready && dirfile_update_derived) {
            defricher_add_derived(dirfile, derived_channels);
            dirfile_update_derived = 0;
        }

        while(g_async_queue_length(packet_queue)) {
            queue_data_t pkt;

            if ((pkt.ptr = g_async_queue_pop(packet_queue))) {
                defricher_write_packet(pkt.rate);
                usleep(1);
            }

        }
        usleep(100);
    }

    if (dirfile) gd_close(dirfile);
    defricher_free_channels_list(channels);
    if (channels) free(channels);
    if (derived_channels) free(derived_channels);
    if (packet_queue) g_async_queue_unref(packet_queue);

    return NULL;
}

void defricher_request_new_dirfile(void)
{
    dirfile_create_new = 1;
}

void defricher_request_updated_derived(void)
{
    dirfile_update_derived = 1;
}

/**
 * Initializes the Defricher writing variables and loop
 * @return
 */
pthread_t defricher_writer_init(void)
{
    size_t stack_size;
    pthread_attr_t attr;
    pthread_t write_thread;

    pthread_attr_init(&attr);
    pthread_attr_getstacksize(&attr, &stack_size);

    /**
     * GetData requires a rather large stack, on top of our own usage, so we set a
     * minimum of 2MB here to guard against smashing
     */
    if (stack_size < (1<<21)) {
        stack_size = (1<<21);
        pthread_attr_setstacksize(&attr, stack_size);
    }

    packet_queue = g_async_queue_new();
    if (!pthread_create(&write_thread, &attr, &defricher_write_loop, NULL))
        return write_thread;
    return 0;
}
