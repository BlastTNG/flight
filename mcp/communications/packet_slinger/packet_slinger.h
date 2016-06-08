/**
 * @file packet_slinger.h
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


#ifndef PACKET_SLINGER_H_
#define PACKET_SLINGER_H_

#include <stdint.h>

#include <lookup.h>
#include <blast_packet_format.h>

#define BLAST_SLINGER_MAGIC 0xEB
#define BLAST_SLINGER_VERSION 0x2

#define SLINGER_LOOPS_TO_CLEAR 100
#define SLINGER_PRIORITY_ADJUST_RATE 0.1

typedef enum
{
	slinger_link_highrate = 0,
	slinger_link_biphase = 1,
	slinger_link_openport = 2
} e_slinger_link;

/** In slinger_preferences.c */
bool slinger_xml_load_preferences(const char *m_name);

/** In packet_slinger.c */
bool initialize_packet_slinger();
bool slinger_start_link(e_slinger_link m_link);
bool slinger_stop_link(e_slinger_link m_link);
void *slinger_monitor(void *m_arg __attribute__((unused)));
void slinger_receive_ack_packet(int m_link, uint32_t m_packet_id);

/** In slinger_bolo.c */
void initialize_slinger_slow_packets(void);
void slinger_set_bitshift(uint32_t m_shift);
void slinger_set_active_boards(uint32_t m_boards);
void slinger_activate_board(uint32_t m_board);
void slinger_deactivate_board(uint32_t m_board);
void slinger_reduce_board_count(uint32_t m_count);

void slinger_process_frame(uint16_t *m_frame);
void slinger_process_UDPpacket(struct udps_pkt_s *pkt, uint8_t m_board);

void slinger_acknowledge_packet(blast_master_packet_t *m_header);

void slinger_link_activate(void);
void slinger_link_deactivate(void);
bool slinger_link_is_active(void);

void set_slinger_dl_rate(uint32_t m_bps);

#endif /* PACKET_SLINGER_H_ */
