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


#include <stdio.h>
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

#include <channels_tng.h>
#include <channel_macros.h>
#include <derived.h>
#include <blast.h>

#include "defricher.h"
#include "defricher_utils.h"
#include "defricher_data.h"

//extern union DerivedUnion DerivedChannels[];
extern channel_t *channels;
extern int frame_stop;
extern struct ri_struct ri;
pthread_t write_thread;

static int dirfile_create_new = 0;
static int dirfile_ready = 0;

//static void defricher_add_derived(DIRFILE *m_file)
//{
//    char lincom2_fields[2][FIELD_LEN];
//    const char *lincom2_ptrs[2] = {lincom2_fields[0], lincom2_fields[1]};
//    double lincom2_m[2];
//    double lincom2_b[2];
//    const char *lincom_name;
//    for (int i = 0; i < ccDerived; ++i)
//    {
//        switch(DerivedChannels[i].bitfield.type)
//        {
//            case 'b':
//                for (int j = 0; DerivedChannels[i].bitfield.field[j][0]; j++)
//                    gd_add_bit(m_file, DerivedChannels[i].bitfield.field[j],
//                            DerivedChannels[i].bitfield.source, j, 1, 0);
//                break;
//            case 'w':
//                gd_add_bit(m_file, DerivedChannels[i].bitword.field,
//                        DerivedChannels[i].bitword.source,
//                        DerivedChannels[i].bitword.offset,
//                        DerivedChannels[i].bitword.length, 0);
//                break;
//            case 't':
//                gd_add_linterp(m_file, DerivedChannels[i].linterp.field, DerivedChannels[i].linterp.source,
//                        DerivedChannels[i].linterp.lut, 0);
//                break;
//            case 'c':
//                lincom_name = DerivedChannels[i].lincom.source;
//                gd_add_lincom(m_file, DerivedChannels[i].lincom.field, 1,
//                        &lincom_name,
//                        &DerivedChannels[i].lincom.m_c2e,
//                        &DerivedChannels[i].lincom.b_e2e, 0);
//                break;
//            case '2':
//                strcpy(lincom2_fields[0], DerivedChannels[i].lincom2.source);
//                strcpy(lincom2_fields[1], DerivedChannels[i].lincom2.source2);
//                lincom2_b[0] = DerivedChannels[i].lincom2.b_e2e;
//                lincom2_b[1] = DerivedChannels[i].lincom2.b2_e2e;
//                lincom2_m[0] = DerivedChannels[i].lincom2.m_c2e;
//                lincom2_m[1] = DerivedChannels[i].lincom2.m2_c2e;
//
//                gd_add_lincom(m_file, DerivedChannels[i].lincom2.field, 2, lincom2_ptrs, lincom2_m, lincom2_b, 0);
//                break;
//            case '#':
//                break;
//            case 'u':
//                //Units
//                break;
//            case 'p':
//                gd_add_phase(m_file, DerivedChannels[i].phase.field,
//                        DerivedChannels[i].phase.source,
//                        DerivedChannels[i].phase.shift, 0 );
//                break;
//            case 'r':
//                gd_add_recip(m_file, DerivedChannels[i].recip.field, DerivedChannels[i].recip.source,
//                        DerivedChannels[i].recip.dividend, 0);
//                break;
//            case '*':
//                gd_add_multiply(m_file, DerivedChannels[i].math.field,
//                        DerivedChannels[i].math.source, DerivedChannels[i].math.source2, 0);
//                break;
//            case '/':
//                gd_add_divide(m_file, DerivedChannels[i].math.field,
//                        DerivedChannels[i].math.source, DerivedChannels[i].math.source2, 0);
//                break;
//            case 'x':
//                gd_add_mplex(m_file, DerivedChannels[i].mplex.field,
//                        DerivedChannels[i].mplex.source, DerivedChannels[i].mplex.index,
//                        DerivedChannels[i].mplex.value, DerivedChannels[i].mplex.max, 0);
//                break;
//            default:
//                break;
//        }
//    }
//}

static const char *defricher_get_new_dirfilename(void)
{
    static char *filename = NULL;
    static time_t start_time = 0;
    static uint32_t chunk = 0;

    if (!start_time) start_time = time(NULL);
    BLAST_SAFE_FREE(filename);
    if (asprintf(&filename, "/data/defricher/%lu_%03X.dirfile", start_time, chunk++) < 0)
    {
        defricher_fatal("Could not allocate memory for filename!");
        filename=NULL;
    }

    return filename;
}
static void defricher_update_current_link(const char *m_name)
{
    unlink("/data/etc/defricher.cur");

    if (symlink(m_name, "/data/etc/defricher.cur"))
        defricher_err("Could not create symlink");
}

static void defricher_file_close_all(channel_t *m_channel_list)
{

    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        defricher_cache_node_t *node = (defricher_cache_node_t*)channel->var;
        if (node && node->magic == BLAST_MAGIC32 && node->output.fp)
        {
            if (fclose(node->output.fp))
                defricher_err("Could not close %s", node->output.name?node->output.name:"UNK");
            node->output.fp = NULL;
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

static DIRFILE *defricher_init_new_dirfile(const char *m_name, channel_t *m_channel_list)
{
    DIRFILE *new_file;
    gd_type_t type;
    char error_str[2048];
    double m,b;
    char *upper_field;

    defricher_info("Intializing %s", m_name);
    defricher_file_close_all(m_channel_list);
    if (defricher_mkdir_file(m_name, true) < 0)
    {
        defricher_strerr("Could not make directory for %s", m_name);
        return false;
    }

    new_file = gd_open(m_name, GD_CREAT|GD_RDWR|GD_PRETTY_PRINT);
    if (gd_error(new_file) == GD_E_OK)
    {
        m = 1e-5;
        b = 0.0;

        for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {

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

            if (gd_add_raw(new_file, node->output.name, type, node->output.rate, 0))
            {
                gd_error_string(new_file, error_str, 2048);
                defricher_err( "Could not add %s: %s", node->output.name, error_str);
                bfree(info, node);
            }
            else
            {
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

                channel->var = node;
            }
        }

        gd_metaflush(new_file);
    }
    else
    {
        gd_error_string(new_file, error_str, 2048);
        defricher_fatal("Could not open %s as DIRFILE: %s", m_name, error_str);
        gd_close(new_file);
        return NULL;
    }

    return new_file;
}


//TODO: Add FIFO buffering
int defricher_write_packet(channel_t *m_channel_list, E_SRC m_source, E_RATE m_rate)
{

    if (frame_stop) {
        defricher_info("Not writing frame due to shutdown request");
        return -1;
    }

    if (!dirfile_ready) {
        defricher_info("Discarding frame due to DIRFILE not being ready for writing");
        return -1;
    }

    ri.wrote ++;
    for (channel_t *channel = m_channel_list; channel->field[0]; channel++) {
        if (channel->source != m_source || channel->rate != m_rate) continue;

        defricher_cache_node_t *outfile_node = channel->var;
        if (outfile_node && outfile_node->magic == BLAST_MAGIC32 ) {
            switch(outfile_node->output.element_size) {
                case 2:
                    *outfile_node->_16bit_data = be16toh(*outfile_node->_16bit_data);
                    break;
                case 4:
                    *outfile_node->_32bit_data = be32toh(*outfile_node->_32bit_data);
                    break;
                case 8:
                    *outfile_node->_64bit_data = be64toh(*outfile_node->_64bit_data);
                    break;
            }
            if (fwrite(outfile_node->raw_data, outfile_node->output.element_size, 1, outfile_node->output.fp) != 1) {
                defricher_err( "Could not write to %s", outfile_node->output.name);
                continue;
            }
        }
    }

    return 0;
}

static void *defricher_write_loop(void *m_arg)
{
    DIRFILE *dirfile = NULL;
    const char *dirfile_name = NULL;

    bprintf(info, "Starting Defricher Write task\n");

    while (!frame_stop)
    {
        if (dirfile_create_new) {
            dirfile_ready = 0;
            dirfile_name = defricher_get_new_dirfilename();
            if (!(dirfile = defricher_init_new_dirfile(dirfile_name, channels))) {
                defricher_err( "Not creating new DIRFILE %s", dirfile_name);
                sleep(1);
                continue;
            }
            defricher_update_cache_fp(dirfile, channels);
            defricher_update_current_link(gd_dirfilename(dirfile));
            dirfile_create_new = 0;
            dirfile_ready = 1;
        }

    }

    return NULL;
}

void defricher_request_new_dirfile(void)
{
    dirfile_create_new = 1;
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
