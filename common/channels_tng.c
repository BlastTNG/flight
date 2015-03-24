/* 
 * channels_v2.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
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
 * Created on: Aug 5, 2014 by Seth Hillbrand
 */

#include <stdio.h>
#include <stdint.h> // For uintX_t types
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <string.h>
#include <sys/time.h> // For finding unix_time and nano_time

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"

/**
* data_packet_t structures handle the data transfer from FC1 and FC2 to MCP.
* The MCP looks for a tag in the datastream (something like 0x12345678) to
* identify the start of a datapacket. Then it then looks at src to find out
* what kind of packet it's received. After reading through the data, it totals
* up everything in the packet and compares this to the checksum; if the values
* match, then we know the packet was transferred correctly.
*
* Note: This structure doesn't include the 4-byte tag indicating that a packet
* is beginning - that tag is always the same, so it doesn't need memory here.
*/
typedef struct {
    uint32_t src:4; // First 4 bits: Source (uint8_t isn't always defined, so use uint32.)
    uint32_t rate:4; // Next 4 bits: Rate
    uint32_t unix_time; // Seconds since midnight of January 1, 1970 (UTC)
    uint32_t nano_time; // Nanoseconds since last second
    uint32_t checksum; // Total of everything in the packet.
    uint32_t length; // We don't allocate a specific length for data[], so
    int data[0]; // length tells how much there is in data[] beyond the 0th element.
} data_packet_t;

/* The macros SRC_, RATE_, and TYPE_END are assigned by lookup.h and uei_framing.c
 * to be the size of the source, rate, and type lists - the code also recognizes
 * other strings after the prefix; e.g. RATE_1HZ evaluates to 0, RATE_5HZ == 1, etc.
 */
static GHashTable *frame_table = NULL;
static int channel_count[SRC_END][RATE_END][TYPE_END] = {{{0}}};
extern channel_t channel_list[];

void *channel_data[N_ROWS][SRC_END][RATE_END] = {{{0}}};
static void *channel_ptr[N_ROWS][SRC_END][RATE_END] = {{{0}}};

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
    // len takes the length of field_name, to a max of FIELD_LEN.
    size_t len = (strlen(field_name) < FIELD_LEN) ? strlen(field_name) : FIELD_LEN;
    return PMurHash32(CHANNELS_HASH_SEED, field_name, len);
}

static void channel_map_fields(gpointer m_key, gpointer m_channel, gpointer m_userdata)
{
    channel_t *channel[N_ROWS];
    for (int row = 0; row < N_ROWS; row ++){
        channel[row] = (channel_t*)m_channel;

        /// If channel is invalid, do not process
        if (!channel[row]->field[row]) return;

        /**
         * channel_ptr maintains the current location in the array of the next free element.
         * We assign this location to the channel pointer, allowing us to reference the
         * individual location in the future based on a lookup in the hash table.
         */
        if (channel[row]->source < SRC_END && channel[row]->rate < RATE_END) {
            if (!channel_ptr[row][channel[row]->source][channel[row]->rate]) {
                bprintf(fatal, "Invalid Channel setup");
            }
            channel[row]->var = channel_ptr[row][channel[row]->source][channel[row]->rate];
            channel_ptr[row][channel[row]->source][channel[row]->rate] += channel_size(channel[row]);
        }
        else {
            bprintf(fatal, "Could not map %d and %d to source and rate!", channel[row]->source, channel[row]->rate);
        }
    }
}

channel_t *channels_find_by_name(const char *m_name)
{
    // len takes the length of m_name, to a max of FIELD_LEN.
    size_t len = (strlen(m_name) < FIELD_LEN) ? strlen(m_name) : FIELD_LEN;
    guint name_hash = PMurHash32(CHANNELS_HASH_SEED, m_name, len);
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
            for (int k = 0; k < N_ROWS; k++) {
                if (channel_data[k][i][j]) {
                    free(channel_data[k][i][j]);
                    channel_data[k][i][j] = NULL;
                }
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

            for (int row = 0; row < N_ROWS; row++){
                if (frame_size) {
                    channel_data[row][src][rate] = malloc(frame_size);
                }
                else {
                    channel_data[row][src][rate] = NULL;
                }
                channel_ptr[row][src][rate] = channel_data[row][src][rate];
            }
        }
    }

    /**
     * Third Pass: Iterate over the hash table and assign the lookup pointers to their place in the frame.
     */
    for (int row = 0; row < N_ROWS; row++) {
        g_hash_table_foreach(frame_table, channel_map_fields, NULL);
    }
    FILE *fp = fopen("HashTbl.txt", "w");
    fprintf (fp, "");
    fclose (fp);

    bprintf(startup, "Successfully initialized Channels data structures");
    return 0;
}



