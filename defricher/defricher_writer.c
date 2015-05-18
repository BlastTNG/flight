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

#include <channels_tng.h>
#include <channel_macros.h>
#include <derived.h>
#include <blast.h>

#include "defricher.h"
#include "defricher_utils.h"
#include "defricher_data.h"

extern struct ri_struct ri;
pthread_t write_thread;

channel_t *channels = NULL;
channel_t *new_channels = NULL;
derived_tng_t *derived_channels = NULL;

static int dirfile_create_new = 0;
static int dirfile_update_derived = 0;
static int dirfile_ready = 0;
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
    static char *filename = NULL;
    static time_t start_time = 0;
    static uint32_t chunk = 0;

    if (!start_time) start_time = time(NULL);
    BLAST_SAFE_FREE(filename);
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
            if (node->output.fp) {
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
    const char *filename;
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
            outfile_node->output.fp = fopen(filename, "w+");

            if (!outfile_node->output.fp) {
                defricher_err("Could not open %s", filename);
                return -1;
            }
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
    char error_str[2048];
    double m,b;
    char *upper_field;

    defricher_info("Intializing %s", m_name);
    if (defricher_mkdir_file(m_name, true) < 0)
    {
        defricher_strerr("Could not make directory for %s", m_name);
        return false;
    }
    defricher_info("2");
    new_file = gd_open(m_name, GD_CREAT|GD_RDWR|GD_PRETTY_PRINT|GD_VERBOSE);
    if (gd_error(new_file) == GD_E_OK)
    {
        m = 1e-5;
        b = 0.0;

        gd_alter_endianness(new_file, GD_BIG_ENDIAN, 0, 0);
        
        defricher_info("3");
        for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
            
            defricher_info("4.%d", (channel - m_channel_list));
            defricher_cache_node_t *node = (defricher_cache_node_t*)channel->var;

            type = defricher_get_gd_type(channel->type);
            
            defricher_info("5.%d, %p, %s, %d, %d", (channel - m_channel_list), new_file, node->output.name, type, node->output.rate);
            if (gd_add_raw(new_file, node->output.name, type, node->output.rate, 0))
            {
                defricher_info("7.%d", (channel - m_channel_list));
                gd_error_string(new_file, error_str, 2048);
                defricher_err( "Could not add %s: %s", node->output.name, error_str);
            }
            else
            {
                defricher_info("6.%d", (channel - m_channel_list));
                blast_tmp_sprintf(upper_field, "%s", node->output.name);
                m = channel->m_c2e;
                b = channel->b_e2e;

                /// By default we set the converted field to upper case
                for (int i = 0; upper_field[i]; i++) upper_field[i] = toupper(upper_field[i]);
                /// If our scale/offset are unity/zero respectively, tell defile to use the easier zero-phase
                if (fabs(m - 1.0) <= DBL_EPSILON && fabs(b - 0.0) <= DBL_EPSILON)
                    gd_add_phase(new_file, upper_field, node->output.name, 0, 0);
                else
                    gd_add_lincom(new_file, upper_field, 1, (const char **)&(node->output.name), &m, &b, 0);
            }
        }

        gd_reference(new_file, "mcp_200hz_framecount");
        gd_metaflush(new_file);
    }
    else
    {
        gd_error_string(new_file, error_str, 2048);
        defricher_fatal("Could not open %s as DIRFILE: %s", m_name, error_str);
        gd_close(new_file);
        return NULL;
    }
    defricher_info("5");

    return new_file;
}

//TODO: Add FIFO buffering
int defricher_write_packet(channel_t *m_channel_list, E_SRC m_source, E_RATE m_rate)
{
    static int have_warned = 1;

    if (ri.writer_done) {
        defricher_info("Not writing frame due to shutdown request");
        return -1;
    }

    if (!ri.dirfile_ready) {
        if (!have_warned) defricher_info("Discarding frame due to DIRFILE not being ready for writing");
        have_warned = 1;
        return -1;
    }

    ri.wrote ++;
    have_warned = 0;
    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        if (channel->source != m_source || channel->rate != m_rate) continue;
        defricher_cache_node_t *outfile_node = channel->var;
        if (outfile_node && outfile_node->magic == BLAST_MAGIC32 ) {
            if (fwrite(outfile_node->raw_data, outfile_node->output.element_size, 1, outfile_node->output.fp) != 1) {
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
            if (channels_initialize(new_channels) < 0) {
                defricher_err("Could not initialize channels");
                ri.new_channels = false;
                free(new_channels);
                new_channels = NULL;
            } else {
                defricher_free_channels_list(channels);
                ri.new_channels = false;
                free(channels);
                channels = new_channels;
                new_channels = NULL;
                defricher_transfer_channel_list(channels);
                ri.channels_ready = true;
                dirfile_create_new = 1;
            }

        }
        if (dirfile_create_new) {
            dirfile_ready = 0;
            dirfile_name = rc.output_dirfile;
            rc.output_dirfile = defricher_get_new_dirfilename();
            BLAST_SAFE_FREE(dirfile_name);
            if (!(dirfile = defricher_init_new_dirfile(rc.output_dirfile, channels))) {
                defricher_err( "Not creating new DIRFILE %s", rc.output_dirfile);
                sleep(1);
                continue;
            }
            defricher_update_cache_fp(dirfile, channels);
            dirfile_create_new = 0;
            ri.dirfile_ready = true;
            ri.symlink_updated = false;
            dirfile_frames_written = 0;
        }

        /// We wait until a frame is written to the dirfile before updating the link (for KST)
        if (dirfile_frames_written > 400 && !ri.symlink_updated) {
            defricher_update_current_link(rc.output_dirfile);
            ri.symlink_updated = true;
        }

        if (ri.dirfile_ready && dirfile_update_derived) {
            defricher_add_derived(dirfile, derived_channels);
            dirfile_update_derived = 0;
        }
        usleep(100);
    }

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
int defricher_writer_init(void)
{

    pthread_create(&write_thread, NULL, &defricher_write_loop, NULL);

    return 0;
}
