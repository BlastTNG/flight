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
#include "derived.h"

static GHashTable *frame_table = NULL;
static int channel_count[RATE_END][TYPE_END] = {{0}};

void *channel_data[RATE_END] = {0};
size_t frame_size[RATE_END] = {0};
static void *channel_ptr[RATE_END] = {0};

static inline size_t channel_size(channel_t *m_channel)
{
    size_t retsize = 0;
    switch (m_channel->type) {
        case TYPE_INT8:
        case TYPE_UINT8:
            retsize = 1;
            break;
        case TYPE_INT16:
        case TYPE_UINT16:
            retsize = 2;
            break;
        case TYPE_INT32:
        case TYPE_UINT32:
        case TYPE_FLOAT:
            retsize = 4;
            break;
        case TYPE_INT64:
        case TYPE_UINT64:
        case TYPE_DOUBLE:
            retsize = 8;
            break;
        default:
            blast_fatal("Invalid Channel size!");
    }
    return retsize;
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
    if (channel->rate < RATE_END) {
        if (!channel_ptr[channel->rate]) {
            blast_fatal("Invalid Channel setup");
        }
        channel->var = channel_ptr[channel->rate];
        channel_ptr[channel->rate] += channel_size(channel);
    } else {
        blast_fatal("Could not map %d to rate!", channel->rate);
    }
}

/**
 * Takes an aligned channel list and re-packs it into a packed structure for sharing over
 * MQTT.
 * @param m_channel_list Pointer to the aligned channel list
 * @return Newly allocated channel_header_t structure or NULL on failure
 */
channel_header_t *channels_create_map(channel_t *m_channel_list)
{
    channel_header_t *new_pkt = NULL;
    size_t channel_count;

    for (channel_count = 0; m_channel_list[channel_count].field[0]; channel_count++)
        continue;
    channel_count++; // Add one extra channel to allow for the NULL terminating field

    new_pkt = balloc(err, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);
    if (!new_pkt) return NULL;
    memset(new_pkt, 0, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);

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
        memcpy(new_pkt->data[i].quantity, m_channel_list[i].quantity, UNITS_LEN);
        memcpy(new_pkt->data[i].units, m_channel_list[i].units, UNITS_LEN);
    }

    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(channel_header_t) +
                              sizeof(struct channel_packed) * channel_count);

    return new_pkt;
}


/**
 * Takes an aligned channel list and re-packs it into a packed structure for sharing over
 * MQTT.
 * @param m_channel_list Pointer to the aligned channel list
 * @return Newly allocated channel_header_t structure or NULL on failure
 */
derived_header_t *channels_create_derived_map(derived_tng_t *m_derived)
{
    derived_header_t *new_pkt = NULL;
    size_t channel_count;

    for (channel_count = 0; m_derived[channel_count].type != DERIVED_EOC_MARKER; channel_count++)
        continue;
    channel_count++; // Add one extra channel to allow for the NULL terminating field

    new_pkt = balloc(err, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);
    if (!new_pkt) return NULL;
    memset(new_pkt, 0, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);

    new_pkt->magic = BLAST_MAGIC32;
    new_pkt->version = BLAST_TNG_CH_VERSION | 0x20; // 0x20 marks the packet as a derived packet
    new_pkt->length = channel_count;
    new_pkt->crc = 0;

    /**
     * Copy over the data values.  Union structure is already packed.
     */
    memcpy(new_pkt->data, m_derived, channel_count * sizeof(derived_tng_t));


    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);

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
        blast_err("Unknown derived channels version %d", m_map->version);
        return -1;
    }

    if (m_len < sizeof(channel_header_t)) {
        blast_err("Invalid size %zu for channel packet", m_len);
        return -1;
    }

    if (m_len != sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed)) {
        blast_err(" m-Map has length %u", m_map->length);
        blast_err("struct channelpacked has size %zu", sizeof(struct channel_packed));
        blast_err("channel header has size %zu", sizeof(channel_header_t));
        blast_err("Length of data packet %zu does not match header data %zu",
                  m_len, sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed));
        return -1;
    }

    m_map->crc = 0;
    if (crcval != PMurHash32(BLAST_MAGIC32, m_map, m_len)) {
        blast_err("CRC match failed!");
        return -1;
    }
    m_map->crc = crcval;

    *m_channel_list = calloc(m_map->length, sizeof(channel_t));
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
        memcpy((*m_channel_list)[channel_count].quantity, m_map->data[channel_count].quantity, UNITS_LEN);
        memcpy((*m_channel_list)[channel_count].units, m_map->data[channel_count].units, UNITS_LEN);
        (*m_channel_list)[channel_count].var = NULL;
    }

    return m_map->length;
}

/**
 * Translates a stored derived packet to the derived structure
 * @param m_map Pointer to the #derived_header_t structure storing our packet
 * @param m_len Length in bytes of the packet passed via m_map
 * @param m_channel_list Double pointer to where we will store the channel_list
 * @return -1 on failure, positive number of channels read otherwise
 */
int channels_read_derived_map(derived_header_t *m_map, size_t m_len, derived_tng_t **m_channel_list)
{
    uint32_t crcval = m_map->crc;

    if (m_map->version != (BLAST_TNG_CH_VERSION | 0x20)) {  // 0x20 marks the packet as a derived packet
        blast_err("Unknown channels version %d", m_map->version);
        return -1;
    }

    if (m_len < sizeof(derived_header_t)) {
        blast_err("Invalid size %zu for derived packet", m_len);
        return -1;
    }

    if (m_len != sizeof(derived_header_t) + m_map->length * sizeof(derived_tng_t)) {
        blast_err("Length of data packet %zu does not match header data %zu",
                  m_len, sizeof(derived_header_t) + m_map->length * sizeof(derived_tng_t));
        return -1;
    }

    m_map->crc = 0;
    if (crcval != PMurHash32(BLAST_MAGIC32, m_map, m_len)) {
        blast_err("CRC match failed!");
        return -1;
    }
    m_map->crc = crcval;

    *m_channel_list = balloc(err, sizeof(derived_tng_t) * m_map->length);
    if (!(*m_channel_list)) return -1;


    memcpy(*m_channel_list, m_map->data, sizeof(derived_tng_t) * m_map->length);

    return m_map->length;
}

channel_t *channels_find_by_name(const char *m_name)
{
    channel_t *retval = (channel_t*)g_hash_table_lookup(frame_table, m_name);

    if (!retval) blast_err("Could not find %s!\n", m_name);
    return retval;
}

int channels_store_data(E_RATE m_rate, const void *m_data, size_t m_len)
{
	if (m_len != frame_size[m_rate]) {
		blast_err("Size mismatch storing data for %s! Got %zu bytes, expected %zu",
		        RATE_LOOKUP_TABLE[m_rate].text, m_len, frame_size[m_rate]);
		return -1;
	}

	memcpy(channel_data[m_rate], m_data, m_len);
	return 0;
}

/**
 * Initialize the channels structure and associated hash tables.
 * @return 0 on success.  -1 otherwise
 */
int channels_initialize(const channel_t * const m_channel_list)
{
    const channel_t *channel;

    if (frame_table) g_hash_table_destroy(frame_table);
    frame_table = g_hash_table_new(channel_hash, g_str_equal);

    if (frame_table == NULL) return -1;

    for (int j = 0; j < RATE_END; j++) {
        for (int k = 0; k < TYPE_END; k++) channel_count[j][k] = 0;
        if (channel_data[j]) {
            free(channel_data[j]);
            channel_data[j] = NULL;
        }
    }

    /**
     * First Pass:  Add each entry in the channels array to a hash table for later lookup.
     * Then count each type of channel, separating by source, variable type and rate
     */
    for (channel = m_channel_list; channel->field[0]; channel++) {
        g_hash_table_insert(frame_table, (gpointer)channel->field, (gpointer)channel);
        if (channel->rate < RATE_END && channel->type < TYPE_END) {
            channel_count[channel->rate][channel->type]++;
        } else {
            blast_fatal("Could not map %d and %d to rate and type!", channel->rate, channel->type);
            return 1;
        }
    }

    /**
     * Second Pass: Allocate a set of packed arrays representing the data frames for each source/rate.
     * We also set channel_ptr, our placeholder for the next free element in the array, to the first entry in each frame.
     */

    for (int rate = 0; rate < RATE_END; rate++) {
        frame_size[rate] = (channel_count[rate][TYPE_INT8]+channel_count[rate][TYPE_UINT8]) +
                       2 * (channel_count[rate][TYPE_INT16]+channel_count[rate][TYPE_UINT16]) +
                       4 * (channel_count[rate][TYPE_INT32]+channel_count[rate][TYPE_UINT32] +
                            channel_count[rate][TYPE_FLOAT]) +
                       8 * (channel_count[rate][TYPE_INT64]+channel_count[rate][TYPE_UINT64] +
                            channel_count[rate][TYPE_DOUBLE]);

        if (frame_size[rate]) {
            /**
             * Ensure that we can dereference the data without knowing its type by
             * taking at least 8 bytes
             */
            size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
            channel_data[rate] = calloc(1, allocated_size);
            blast_mem("Allocating %zu bytes for %u channels at %s", frame_size[rate],
                    (channel_count[rate][TYPE_INT8]+channel_count[rate][TYPE_UINT8]) +
                    (channel_count[rate][TYPE_INT16]+channel_count[rate][TYPE_UINT16]) +
                    (channel_count[rate][TYPE_INT32]+channel_count[rate][TYPE_UINT32] +
                            channel_count[rate][TYPE_FLOAT]) +
                    (channel_count[rate][TYPE_INT64]+channel_count[rate][TYPE_UINT64] +
                            channel_count[rate][TYPE_DOUBLE]),
                    RATE_LOOKUP_TABLE[rate].text);
        } else {
            channel_data[rate] = NULL;
        }
        channel_ptr[rate] = channel_data[rate];
    }

    /**
     * Third Pass: Iterate over the hash table and assign the lookup pointers to their place in the frame.
     */
    g_hash_table_foreach(frame_table, channel_map_fields, NULL);


    blast_startup("Successfully initialized Channels data structures");
    return 0;
}

