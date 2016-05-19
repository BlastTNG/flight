/**
 * @file slinger_bolo.c
 *
 * @date Feb 14, 2011
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
#include <stddef.h>
#include <netinet/in.h>
#include <byteswap.h>
#include <stdint.h>
#include <inttypes.h>

#include <channels.h>
#include <command_struct.h>

#include <blast.h>
#include <slinger_data.h>

typedef struct udps_descriptor_s 	udps_descriptor_t;
typedef struct udps_bd_16bit_s 	udps_bolo_t;
typedef struct udps_hwp_s udps_hwp_t;
typedef struct udps_ts_canbus_s 	udps_ts_t;
typedef slinger_cache_node_t* slinger_cache_p;

static slinger_cache_node_t *bolo_node[1<<8][1<<6] = {{NULL}};


/**
 * Receives and unpacks a UDP packet from the UDP monitor.  The packet is broken into individual kids and
 * forwarded to the slinger downlink.
 * @param pkt Pointer to the UDPS packet received by FCP
 */
void slinger_process_UDPpacket(struct udps_pkt_s *pkt, uint8_t m_board)
{
    udps_descriptor_t *descriptor = NULL;
    udps_bolo_t *bolo_descriptor = NULL;
    udps_hwp_t *hwp_descriptor = NULL;
    udps_ts_t *ts_descriptor = NULL;
    uint_fast8_t wire;
    uint_fast8_t channel;
    uint_fast8_t nsamp = 0;

    slinger_time_t timestamp;

    int16_t *sample_data = NULL;

    uint16_t type = 0;
    uint32_t desc_len;


    int i = 0;
    for (wire = 0; wire < 4; wire++) {
        for (channel = (wire * 17); channel < (((wire + 1) * 17) - 1); channel++) {
            if (bolo_node[m_board][i] && slinger_priority_downlink(
                    bolo_node[m_board][i]->output_stream->bundle->output_bundle->priority)) {
                slinger_cache_stream_data_known(&timestamp, bolo_node[m_board][i], false,
                                                &sample_data[channel], sizeof(uint16_t));
            }
            i++;
        }
    }
}

void slinger_add_bolo_channel(uint8_t m_board, uint8_t m_offset, slinger_cache_node_t *m_node)
{
    bolo_node[m_board][m_offset] = m_node;
}



