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
 * Created on: Aug 5, 2014 by Dr. Seth Hillbrand
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <string.h>
#include <time.h> // For finding unix time

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"

static int iread;  // These take a value from 0 to N_ROWS-1, showing which part of
static int iwrite; // the curcular data buffer is being read from and written to.
static int row; // This indicates which row is being looked at right now.

/**
* MWG: data_packet_t structures handle the data transfer from FC1 and FC2 to MCP.
* The MCP looks for a tag in the datastream (something like 0x12345678),
* and then looks at the src to find out what kind of packet it's received.
* After reading through the data, it totals up everything in the packet
* and compares this to the checksum; if the values match, the packet was
* transferred correctly.
*
* Note: Not included is the 4-byte tag indicating that a packet is beginning.
*/
typedef struct {
    uint32_t src:4; // First 4 bits: Source (uint8_t isn't always defined, so use uint32.)
    uint32_t rate:4; // Next 4 bits: Rate
    uint32_t unix_time; // Seconds since midnight of January 1, 1970 (UTC)
    uint32_t nano_time; // Nanoseconds since last second
    uint64_t time; // First 4 bytes: Unix Time.  Next 4 bytes: nanoseconds.
    uint32_t checksum; // Total of everything in the packet.
    uint32_t length; // We don't allocate a specific length for the data, so
    int data[0]; // length tells how much there is in data[].
} data_packet_t;

/* MWG: The macros SRC_ RATE_ and TYPE_END are assigned by lookup.h and uei_framing.c
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

    return PMurHash32(CHANNELS_HASH_SEED, field_name, strnlen(field_name, FIELD_LEN));
}

static void channel_map_fields(gpointer m_key, gpointer m_channel, gpointer m_userdata)
{
    channel_t *channel[N_ROWS];
    for (row == 0; row < N_ROWS; row ++){
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

            for (row = 0; row < N_ROWS; row++){
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
    for (row = 0; row < N_ROWS; row++) {
        g_hash_table_foreach(frame_table, channel_map_fields, NULL);
    }
    FILE *fp = fopen("HashTbl.txt", "w");
    fprintf (fp, "");
    fclose (fp);

    bprintf(startup, "Successfully initialized Channels data structures");
    return 0;
}


/**
 * Builds a packet's header, calculating and storing the checksum and length.
 * The data for the given source and rate is packed in underneath.
 *
 * The checksum is calculated by counting up everything in the packet,
 * including the TAG and header, but excluding the checksum itself, and
 * then storing the total value in this integer variable (sum).
 * Length refers to the length of the data, calculated below and poked into
 * Packet.length.
 *
 * @param source
 * @param rate
 * @return Packet
 */

data_packet_t BuildPacket(int src, int rate)
{
    data_packet_t Packet;
    /*
     */
    int sum = 0;
    int length = 0;

    // Packet.src == source;
    // Packet.rate == rate;
    // sum += Packet.src * 16 + Packet.rate; // Packs source & rate into 4 bytes

    Packet.unix_time = (int)time(NULL); // time in seconds
    Packet.nano_time = 0; // time in nanoseconds
    // MWG: Still working out nanosecond time
    // Packet.nano_time = tms.tv_nsec/1000; // nanoseconds
    sum += Packet.unix_time + Packet.nano_time;

    Packet.length = (channel_count[src][rate][TYPE_INT8]+channel_count[src][rate][TYPE_UINT8]) +
            2 * (channel_count[src][rate][TYPE_INT16]+channel_count[src][rate][TYPE_UINT16]) +
            4 * (channel_count[src][rate][TYPE_INT32]+channel_count[src][rate][TYPE_UINT32]+channel_count[src][rate][TYPE_FLOAT]) +
            8 * (channel_count[src][rate][TYPE_INT64]+channel_count[src][rate][TYPE_UINT64]+channel_count[src][rate][TYPE_DOUBLE]);

    for (int i = 0; i < Packet.length; i++){
        Packet.data[i] += *((int*)channel_data[iread][Packet.src][Packet.rate] + i);
        sum += Packet.data[i];
    }
    Packet.length = length;
    Packet.checksum = sum;
    return Packet;
}

/**
 * This function takes an existing packet and appends it to Packets.txt
 *
 * @param Packet
 */
void SaveData(data_packet_t Packet)
{
    FILE *fp = fopen("Packets.txt", "a");

    // Header
    fprintf (fp, "%s", TAG);
    fprintf (fp, "%d", Packet.src * 16 + Packet.rate); // packs both into 4 bytes
    fprintf (fp, "%d", Packet.time);
    fprintf (fp, "%d", Packet.checksum);
    fprintf (fp, "%d", Packet.length);

    // Channel Data
    for (int i = 0; i < Packet.length; i++){
        fprintf (fp, "%d", Packet.data[i]);
    }

    fclose (fp);
}

