/* 
 * channels_v2.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
 * Created on: Aug 5, 2014 by seth
 */

/**
 * Code Overview:
 *
 * This file provides the interface to the Channels structure for BLAST-TNG.
 *
 * Channels lives in tx_struct_tng.c and list all known channels for the experiment.
 * On startup, the channels structure is read and separated into a number of distinct
 * frames based on rate.  Thus there is a 1Hz frame, a 5Hz frame, a 100Hz frame,
 * etc.  The frames are packed structures, ordered first by their source and then
 * alphabetically by their channel name and stored in big-endian byte-order.
 *
 * Each frame also has a small header that records the timestamp and computer id.
 *
 * On MCP, there are frame structures for each computer (UEI1, UEI2, FC1, FC2, SC1, SC2).
 * The data are populated by listening to the MQTT server.
 *
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"


static GHashTable *frame_table = NULL;
static int channel_count[SRC_END][RATE_END][TYPE_END] = {{{0}}};
extern channel_t channel_list[];

void *channel_data[SRC_END][RATE_END] = {{0}};
static void *channel_ptr[SRC_END][RATE_END] = {{0}};

static inline size_t channel_size(channel_t *m_channel)
{
    switch (m_channel->type) {
        case TYPE_INT8:
        case TYPE_UINT8:
            return 1;
        case TYPE_INT16:
        case TYPE_UINT16:
            return 2;
        case TYPE_INT32:
        case TYPE_UINT32:
        case TYPE_FLOAT:
            return 4;
        case TYPE_INT64:
        case TYPE_UINT64:
        case TYPE_DOUBLE:
            return 8;
        default:
            return 0;
    }
    return 0;
}
static guint channel_hash(gconstpointer m_data)
{
    const char *field_name = (const char*)m_data;

    return PMurHash32(CHANNELS_HASH_SEED, field_name, strnlen(field_name, FIELD_LEN));
}

static void channel_map_fields(gpointer m_key, gpointer m_channel, gpointer m_userdata)
{
    channel_t *channel = (channel_t*)m_channel;

    /// If channel is invalid, do not process
    if (!channel->field[0]) return;

    /**
     * channel_ptr maintains the current location in the array of the next free element.
     * We assign this location to the channel pointer, allowing us to reference the
     * individual location in the future based on a lookup in the hash table.
     */
    if (channel->source < SRC_END && channel->rate < RATE_END) {
        if (!channel_ptr[channel->source][channel->rate]) {
            bprintf(fatal, "Invalid Channel setup");
        }
        channel->var = channel_ptr[channel->source][channel->rate];
        channel_ptr[channel->source][channel->rate] += channel_size(channel);
    }
    else {
        bprintf(fatal, "Could not map %d and %d to source and rate!", channel->source, channel->rate);
    }
}

channel_t *channels_find_by_name(const char *m_name)
{
    guint name_hash = PMurHash32(CHANNELS_HASH_SEED, m_name, strnlen(m_name, FIELD_LEN));
    return (channel_t*)g_hash_table_lookup(frame_table, m_name);
}

int channels_initialize(const char *m_datafile)
{
    channel_t *channel;

    frame_table = g_hash_table_new(channel_hash, g_str_equal);

    if (frame_table == NULL) return 1;

    for (int i = 0; i < SRC_END; i++) {
        for (int j = 0; j < RATE_END; j++) {
            for (int k = 0; k < TYPE_END; k++) channel_count[i][j][k] = 0;
            if (channel_data[i][j]) {
                free(channel_data[i][j]);
                channel_data[i][j] = NULL;
            }
        }
    }

    /**
     * First Pass:  Add each entry in the channels array to a hash table for later lookup.
     * Then count each type of channel, separating by source, variable type and rate
     */
    for (channel = channel_list; channel->field[0]; channel++) {
        g_hash_table_insert(frame_table, channel->field, channel);
        if (channel->rate < RATE_END && channel->type < TYPE_END) {
            channel_count[channel->source][channel->rate][channel->type]++;
        }
        else {
            bprintf(fatal, "Could not map %d and %d to rate and type!", channel->rate, channel->type);
            return 1;
        }
    }

    /**
     * Second Pass: Allocate a set of packed arrays representing the data frames for each source/rate.
     * We also set channel_ptr, our placeholder for the next free element in the array, to the first entry in each frame.
     */
    for (int src = 0; src < SRC_END; src++) {
        for (int rate = 0; rate < RATE_END; rate++) {
            size_t frame_size = (channel_count[src][rate][TYPE_INT8]+channel_count[src][rate][TYPE_UINT8]) +
                    2 * (channel_count[src][rate][TYPE_INT16]+channel_count[src][rate][TYPE_UINT16]) +
                    4 * (channel_count[src][rate][TYPE_INT32]+channel_count[src][rate][TYPE_UINT32]+channel_count[src][rate][TYPE_FLOAT]) +
                    8 * (channel_count[src][rate][TYPE_INT64]+channel_count[src][rate][TYPE_UINT64]+channel_count[src][rate][TYPE_DOUBLE]);

            if (frame_size) {
                channel_data[src][rate] = malloc(frame_size);
            }
            else {
                channel_data[src][rate] = NULL;
            }
            channel_ptr[src][rate] = channel_data[src][rate];
        }
    }

    /**
     * Third Pass: Iterate over the hash table and assign the lookup pointers to their place in the frame.
     */
    g_hash_table_foreach(frame_table, channel_map_fields, NULL);

    bprintf(startup, "Successfully initialized Channels data structures");
    return 0;
}
