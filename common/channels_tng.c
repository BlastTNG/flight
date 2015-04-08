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
 * etc.  The frames are packed structures, ordered first by their source.
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
#include <stdio.h>
#include <string.h>
#include <glib.h>

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"

static GHashTable *frame_table = NULL;
static int channel_count[SRC_END][RATE_END][TYPE_END] = {{{0}}};

void *channel_data[SRC_END][RATE_END] = {{0}};
size_t frame_size[SRC_END][RATE_END] = {{0}};
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

    return PMurHash32(BLAST_MAGIC32, field_name, strnlen(field_name, FIELD_LEN));
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

channel_header_t *channels_create_map(channel_t *m_channel_list)
{
    channel_header_t *new_pkt = NULL;
    size_t channel_count;

    for (channel_count = 0; m_channel_list[channel_count].field[0]; channel_count++);
    channel_count++; // Add one extra channel to allow for the NULL terminating field

    new_pkt = balloc(err, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);

    if (!new_pkt) return NULL;

    new_pkt->magic = BLAST_MAGIC32;
    new_pkt->version = BLAST_TNG_CH_VERSION;
    new_pkt->length = channel_count;
    new_pkt->crc = 0;

    /**
     * Copy over the data values one at a time from the aligned to the packed structure
     */
    for (size_t i = 0; i < channel_count; i++) {
        memcpy(new_pkt->data[i].field, m_channel_list[i].field, FIELD_LEN);
        new_pkt->data[i].m_c2e = m_channel_list[i].m_c2e;
        new_pkt->data[i].b_e2e = m_channel_list[i].b_e2e;
        new_pkt->data[i].type = m_channel_list[i].type;
        new_pkt->data[i].rate = m_channel_list[i].rate;
        new_pkt->data[i].source = m_channel_list[i].source;
        memcpy(new_pkt->data[i].quantity, m_channel_list[i].quantity, UNITS_LEN);
        memcpy(new_pkt->data[i].units, m_channel_list[i].units, UNITS_LEN);
    }

    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);

    return new_pkt;
}

/**
 * Translates a stored channel map to the channel_list structure
 * @param m_map Pointer to the #channel_header_t structure storing our packet
 * @param m_len Length in bytes of the packet passed via m_map
 * @param m_channel_list Double pointer to where we will store the channel_list
 * @return -1 on failure, positive number of channels read otherwise
 */
int channels_read_map(channel_header_t *m_map, size_t m_len, channel_t **m_channel_list)
{
    uint32_t crcval = m_map->crc;

    if (m_map->version != BLAST_TNG_CH_VERSION) {
        bprintf(err, "Unknown channels version %d", m_map->version);
        return -1;
    }

    if (m_len < sizeof(channel_header_t)) {
        bprintf(err, "Invalid size %zu for channel packet", m_len);
        return -1;
    }

    if (m_len != sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed)) {
        bprintf(err, "Length of data packet %zu does not match header data %zu", m_len, sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed));
        return -1;
    }

    m_map->crc = 0;
    if (crcval != PMurHash32(BLAST_MAGIC32, m_map, m_len)) {
        bprintf(err, "CRC match failed!");
        return -1;
    }
    m_map->crc = crcval;

    *m_channel_list = balloc(err, sizeof(channel_t) * m_map->length);
    if (!(*m_channel_list)) return -1;


    /**
     * Copy over the data values one at a time from the packed to the aligned structure
     */
    for (size_t channel_count = 0; channel_count < m_map->length; channel_count++) {
        memcpy((*m_channel_list)[channel_count].field, m_map->data[channel_count].field, FIELD_LEN);
        (*m_channel_list)[channel_count].m_c2e = m_map->data[channel_count].m_c2e;
        (*m_channel_list)[channel_count].b_e2e = m_map->data[channel_count].b_e2e;
        (*m_channel_list)[channel_count].type = m_map->data[channel_count].type;
        (*m_channel_list)[channel_count].rate = m_map->data[channel_count].rate;
        (*m_channel_list)[channel_count].source = m_map->data[channel_count].source;
        memcpy((*m_channel_list)[channel_count].quantity, m_map->data[channel_count].quantity, UNITS_LEN);
        memcpy((*m_channel_list)[channel_count].units, m_map->data[channel_count].units, UNITS_LEN);
        (*m_channel_list)[channel_count].var = NULL;
    }

    return m_map->length;
}

channel_t *channels_find_by_name(const char *m_name)
{
    channel_t *retval = (channel_t*)g_hash_table_lookup(frame_table, m_name);

    if (!retval) bprintf(err,"Could not find %s!\n", m_name);
    return retval;
}

int channels_store_data(E_SRC m_src, E_RATE m_rate, const void *m_data, size_t m_len)
{
	if (m_len != frame_size[m_src][m_rate]) {
		bprintf(err, "Size mismatch storing data for %s:%s!\n", SRC_LOOKUP_TABLE[m_src].text, RATE_LOOKUP_TABLE[m_rate].text );
		return -1;
	}

	memcpy(channel_data[m_src][m_rate], m_data, m_len);
	return 0;
}

/**
 * Initialize the channels structure and associated hash tables.
 * @return 0 on success.  -1 otherwise
 */
int channels_initialize(const channel_t * const m_channel_list)
{
    const channel_t *channel;

    frame_table = g_hash_table_new(channel_hash, g_str_equal);

    if (frame_table == NULL) return -1;

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
    for (channel = m_channel_list; channel->field[0]; channel++) {
        g_hash_table_insert(frame_table, (gpointer)channel->field, (gpointer)channel);
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
            frame_size[src][rate] = (channel_count[src][rate][TYPE_INT8]+channel_count[src][rate][TYPE_UINT8]) +
                    2 * (channel_count[src][rate][TYPE_INT16]+channel_count[src][rate][TYPE_UINT16]) +
                    4 * (channel_count[src][rate][TYPE_INT32]+channel_count[src][rate][TYPE_UINT32]+channel_count[src][rate][TYPE_FLOAT]) +
                    8 * (channel_count[src][rate][TYPE_INT64]+channel_count[src][rate][TYPE_UINT64]+channel_count[src][rate][TYPE_DOUBLE]);

            if (frame_size[src][rate]) {
                channel_data[src][rate] = malloc(frame_size[src][rate]);
                bprintf(startup, "Allocating %zu bytes for %u channels at %s:%s", frame_size[src][rate],
                        (channel_count[src][rate][TYPE_INT8]+channel_count[src][rate][TYPE_UINT8]) +
                        (channel_count[src][rate][TYPE_INT16]+channel_count[src][rate][TYPE_UINT16]) +
                        (channel_count[src][rate][TYPE_INT32]+channel_count[src][rate][TYPE_UINT32]+channel_count[src][rate][TYPE_FLOAT]) +
                        (channel_count[src][rate][TYPE_INT64]+channel_count[src][rate][TYPE_UINT64]+channel_count[src][rate][TYPE_DOUBLE]),
                        SRC_LOOKUP_TABLE[src].text,
                        RATE_LOOKUP_TABLE[rate].text);
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
