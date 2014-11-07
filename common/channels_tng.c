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
#include <time.h> // MWG: For finding unix time

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"

int iread;  // MWG: These take a value from 0 to N_ROWS-1, showing which part of
int iwrite; // the curcular data buffer is being read from and written to.
int row; // MWG: This indicates which row is being looked at right now.

/**
* MWG: DataPacket structures handle the data transfer from FC1 and FC2 to MCP.
* The MCP looks for a tag in the datastream (something like 0x12345678),
* and then looks at the src to find out what kind of packet it's received.
* After reading through the data, it totals up everything in the packet
* and compares this to the checksum; if the values match, the packet was
* transferred correctly.
*
* Note: Not included is the 4-byte tag indicating that a packet is beginning.
*/
typedef struct {
    struct src {
        unsigned s:4; // First 4 bits: Source
        unsigned r:4; // Next 4 bits: Rate
    } src;
    unsigned long long time; // First 4 bytes: Unix Time.  Next 4 bytes: nanoseconds.
    unsigned int checksum; // Total of everything in the packet.
    // MWG: I'm allocating 4000 bytes here as discussed, but why 4000?
    int data[4000]; // All the data contained in the packet
}DataPacket;

/* MWG: The macros SRC_ RATE_ and TYPE_END are assigned by lookup.h and uei_framing.c
 * to be the size of the source, rate, and type lists - the code also recognizes
 * other strings after the prefix; e.g. RATE_1HZ evaluates to 0, RATE_5HZ == 1, etc.
 */
static GHashTable *frame_table = NULL;
static int channel_count[SRC_END][RATE_END][TYPE_END] = {{{0}}};
extern channel_t channel_list[];

void *channel_data[N_ROWS][SRC_END][RATE_END] = {{0}};
static void *channel_ptr[N_ROWS][SRC_END][RATE_END] = {{0}};

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
        if (!channel_ptr[row][channel->source][channel->rate]) {
            bprintf(fatal, "Invalid Channel setup");
        }
        channel->var = channel_ptr[row][channel->source][channel->rate];
        channel_ptr[row][channel->source][channel->rate] += channel_size(channel);
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

            if (frame_size) {
                channel_data[row][src][rate] = malloc(frame_size * N_ROWS);
            }
            else {
                channel_data[row][src][rate] = NULL;
            }
            // MWG: The for loop lets it compile, but I didn't mean to assign this to all rows...
            for (row = 0; row < N_ROWS; row++) {
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
 * Builds a packet's header, calculating and storing the checksum, and includes
 * the data underneath.
 *
 * @return Packet
 */
DataPacket BuildPacket()
{
    DataPacket Packet;
    /* MWG: The checksum is calculated by counting up everything in the packet,
     * including the TAG and header, but excluding the checksum itself, and
     * then storing the total value in this integer variable (sum).
     */
    int sum = 0;

    // Packet.src.s == source;
    // Packet.src.r == rate;
    // sum += Packet.src;

    int unixtime = (int)time(NULL); // time in seconds (for the first 4 bytes)

    // MWG: Still working out microsecond time
    //int microtime = tms.tv_nsec/1000; // microseconds
    Packet.time == unixtime * 256^4; // need to add + microtime to this line
    sum += Packet.time;

    /* MWG: this below isn't right - nothing to loop across, just one line.
     * (Where is the data?) But the idea is to read the row in the circular
     * data buffer which is currently being read, on the appropriate source
     * and rate.
     */
    Packet.data[0] += *((int*)channel_data[iread][Packet.src.s][Packet.src.r]);
    sum += Packet.data[0];

    return Packet;
}

/**
 * This function takes an existing packet and appends it to Packets.txt
 */
void SaveData(DataPacket Packet)
{
    FILE *fp = fopen("Packets.txt", "a");

    fprintf (fp, "%s", TAG);
    fprintf (fp, "%d", Packet.src.s * 8 + Packet.src.r);
    fprintf (fp, "%d", Packet.time);
    fprintf (fp, "%d", Packet.checksum);

    /* MWG: Finally, we need to actually print the data to the file.
     * Again, this is wrong - I don't understand where the data is, or
     * what to give for source and rate - I only know it must live in
     * row [iwrite],[source],[rate]. There should ultimately be a for
     * loop when I understand what to loop over.)
     */

    fclose (fp);
}




