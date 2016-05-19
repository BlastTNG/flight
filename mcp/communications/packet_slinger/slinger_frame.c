/**
 * @file slinger_frame.c
 *
 * @date Sep 14, 2010
 * @author seth
 * 
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdint.h>

#include <channels_tng.h>
#include <blast.h>

#include <slinger_frame.h>
#include <slinger_data.h>
#include <slinger_time.h>

static slinger_frame_data_t *slinger_frame_lookup = NULL;
slinger_time_t timestamp_a = {0};
slinger_time_t timestamp_b = {0};
static int times1_ch = 4;
static int times2_ch = 5;
static int times3_ch = 6;
static int times4_ch = 7;
static int times1b_ch = 4;
static int times2b_ch = 5;
static int times3b_ch = 6;
static int times4b_ch = 7;

static int can1_timestamp_index = 0;
static int can1_timeperiod_index = 0;
static int can1_timesource_index = 0;
static int can1_timestamp_ch = 0;
static int can1_timeperiod_ch = 0;
static int can1_timesource_ch = 0;

#define SLINGER_FRAME_LOOKUP(_row,_col) (slinger_frame_lookup[(_row)*DiskFrameWords+(_col)])

/**
 * Get the approximate rate of a NIOS channel
 * @param m_channel Name of channel to lookup
 * @return 0 on failure, E_RATE otherwise
 */
uint32_t slinger_get_channel_rate(const char *m_name)
{
	const channel_t channel = channels_find_by_name(m_name);
	if (!channel)
	{
		blast_err("Could not match a data field to '%s'", (char*)m_name);
		return 0;
	}
	return channel.rate;

}

/**
 * Adds a channel as a potential downlink source.  While adding, it attaches the cache node
 * pointer to the lookup table for easy reference
 * @param m_channel Pointer to the channel name
 * @param m_node Pointer to the allocated cache node
 * @return
 */
bool slinger_add_frame_channel(const char *m_channel, slinger_cache_node_t *m_node)
{
	const struct NiosStruct *nios = NULL;
	uint16_t channel = 0;
	uint16_t frame_index = 0;
	hash_t hash;
	int32_t size;

	nios = get_NIOS_address_nonfatal(m_channel);
	if (nios == NULL)
	{
		blast_err("Could not match a data field to '%s'", (char*)m_channel);
		return false;
	}

	hash = ebex_hash_key(m_channel, strlen(m_channel));
	channel = BiPhaseLookup[BI0_MAGIC(nios->bbcAddr)].channel;
	frame_index = BiPhaseLookup[BI0_MAGIC(nios->bbcAddr)].index;
	size = (bool)nios->wide ? 4 : 2;

    if (frame_index == NOT_MULTIPLEXED)
    {
        for (uint16_t temp_index = 0; temp_index < FAST_PER_SLOW; temp_index++)
        {
            if (SLINGER_FRAME_LOOKUP(temp_index,channel).hash || SLINGER_FRAME_LOOKUP(temp_index,channel).size)
                ebex_fatal("Collision between 0x%08x and 0x%08x, %s", SLINGER_FRAME_LOOKUP(temp_index,channel).hash, hash, m_channel);
            SLINGER_FRAME_LOOKUP(temp_index,channel).hash = hash;
            SLINGER_FRAME_LOOKUP(temp_index,channel).size = size;
            SLINGER_FRAME_LOOKUP(temp_index,channel).is_slow = false;
            SLINGER_FRAME_LOOKUP(temp_index,channel).node = m_node;
        }
    }
    else
    {
        if (SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).hash || SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).size)
            ebex_fatal("Collision between 0x%08x and 0x%08x, %s", SLINGER_FRAME_LOOKUP(frame_index,channel).hash, hash, m_channel);
        SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).hash = hash;
        SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).size = size;
        SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).is_slow = true;
        SLINGER_FRAME_LOOKUP(frame_index,channel + SLOW_OFFSET).node = m_node;
    }

    return true;
}

void initialize_slinger_frame_lookup(void)
{
	const struct NiosStruct *nios = NULL;

	slinger_frame_lookup = (slinger_frame_data_t*)balloc(fatal, FAST_PER_SLOW * DiskFrameWords * sizeof(slinger_frame_data_t));
	memset(slinger_frame_lookup, 0, FAST_PER_SLOW * DiskFrameWords * sizeof(slinger_frame_data_t));

}

/**
 * Process a single BLASTBus frame.  Each frame should be fully populated, thus this should only be called
 * after the slow frames have been filled.
 * @param m_frame Pointer to the frame structure
 */
void slinger_process_frame(uint16_t *m_frame)
{
	uint16_t frame_index = 0;
	static slinger_time_t slow_timestamp = {0};
	static uint_fast8_t slow_timestamp_received = 0;

	bool have_timestamp = false;
	int i = 0;
	uint32_t temp_val = 0;
	slinger_cache_node_t *node = NULL;
	int32_t  temp_size;
	/**
	 * framedelta is the offset in EBEX time of each successive fast frame relative to the
	 * starting fast frame.  Since the slow channels are multiplexed, we assign each to
	 * the EBEX time of the start of the frame.
	 */
	static const uint64_t framedelta[FAST_PER_SLOW] =
			{      0,   998,   1997,  2995,  3994,
				4992,  5990,   6989,  7987,  8986,
				9984,  10982, 11981, 12979, 13978,
				14976, 15974, 16973, 17971, 18970
			};


	/// If we haven't initialized Packet Slinger, do use this function
	if (!slinger_frame_lookup) return;

	frame_index = m_frame[3];
	if (frame_index >= FAST_PER_SLOW)
	{
		blast_err("Invalid frame number %" PRIu16, frame_index);
		return;
	}

	/// By default, we use ACS timestamps that come in the fast frame
	timestamp_a.time_ok = (m_frame[times4_ch] >> 2 )& 0b1111;
	timestamp_a.board_id = m_frame[times4_ch] & 0b11;
	timestamp_a.acs.intlow = m_frame[times1_ch] & 0x3fff;
	timestamp_a.acs.intmid = (m_frame[times2_ch] + (m_frame[times2_ch + 1] << 16)) & 0x3ffff;
	timestamp_a.period = m_frame[times3_ch] & 0x3fff;

	timestamp_b.time_ok = (m_frame[times4b_ch] >> 2 )& 0b1111;
	timestamp_b.board_id = m_frame[times4b_ch] & 0b11;
	timestamp_b.acs.intlow = m_frame[times1b_ch] & 0x3fff;
	timestamp_b.acs.intmid = (m_frame[times2b_ch] + (m_frame[times2b_ch + 1] << 16)) & 0x3ffff;
	timestamp_b.period = m_frame[times3b_ch] & 0x3fff;

	if (timestamp_a.time_ok)
	{
		slinger_time_sub(&timestamp_a, framedelta[frame_index], &slow_timestamp);
		have_timestamp = true;
	}
	else if (timestamp_b.time_ok)
	{
		slinger_time_sub(&timestamp_b, framedelta[frame_index], &slow_timestamp);
		timestamp_a.qword = timestamp_b.qword;
		have_timestamp = true;
	}

	slinger_time_update(&timestamp_a, &timestamp_b);

	if (!have_timestamp)
	{
		/**
		 * If we don't see the ACS timestamps, we can fall back to the CANBus timestamp
		 * message.  These only arrive in slow frames, so we store the most recent message in
		 * our slow timestamp and adjust the fast offset.  There is a strong possibility of the
		 * CANBus messages not updating in time (if the SYNC message is delayed), in which case,
		 * the slinger data may not align if we choose to start a new packet at the same framesamp
		 * as the delayed message.
		 */
		if (frame_index == can1_timesource_index)
		{
			slow_timestamp.board_id = m_frame[can1_timesource_ch] & 0b11;
			slow_timestamp_received |= 0b001;
		}
		if (frame_index == can1_timeperiod_index)
		{
			slow_timestamp.period = m_frame[can1_timeperiod_ch];
			slow_timestamp_received |= 0b010;
		}
		if (frame_index == can1_timestamp_index)
		{
			slow_timestamp.bolo.ticks = ((uint32_t)(m_frame[can1_timestamp_ch + 1]) << 16) + m_frame[can1_timestamp_ch];
			slow_timestamp_received |= 0b100;
		}

		/**
		 * It will take some time for the first CANBus time word to be recorded.  Don't send down frames that
		 * are not timestamped!
		 */
		if (slow_timestamp_received != 0b111) return;
		slinger_time_add(&slow_timestamp, framedelta[frame_index], &timestamp_a);
	}

	for (i = 0; i < DiskFrameWords; i++)
	{
		/** The table is initialized to 0 and only channels we care about have their sizes set (either 2 or 4) */
		if (!(temp_size = SLINGER_FRAME_LOOKUP(frame_index,i).size)) continue;
		node = SLINGER_FRAME_LOOKUP(frame_index,i).node;
		temp_val = m_frame[i];
		if (temp_size == 4)
		{
			temp_val += ((uint32_t)m_frame[++i] << 16);
		}
		if (SLINGER_FRAME_LOOKUP(frame_index, i).is_slow)
		{
			/// We ensure that slow frames are sent down with a timestamp that matches the first timestamp of the frame
			slinger_cache_stream_data_known(&slow_timestamp, node, true, &temp_val, temp_size);
		}
		else
			slinger_cache_stream_data_known(&timestamp_a, node, true, &temp_val, temp_size);
	}

}
