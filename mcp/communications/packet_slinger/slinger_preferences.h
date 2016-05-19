/**
 * @file slinger_preferences.h
 *
 * @date Apr 5, 2011
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


#ifndef SLINGER_PREFERENCES_H_
#define SLINGER_PREFERENCES_H_

#include <lookup.h>

/**
 * These defines correspond to the elements in the packetslinger.dtd file.
 */
#define _SLINGER_XML_TAGS(x,_)	\
	_(x, bundle)				\
	_(x, periodic)				\
	_(x, bolo)					\
	_(x, channel)				\
	_(x, hwp)					\
	_(x, dfmux_slow)

BLAST_FUNCTION_LOOKUP_TABLE(SLINGER_XML_TAG, static);

/**
 * The BUFFER_OVERHEAD is extra space in case of multiple threads adding data to the buffer while
 * it is compressed.  This prevents (the currently impossible) overflow of memory writes
 */
#define SLINGER_DOWNLINK_BUFFER_OVERHEAD 64


/// In slinger_bolo.c
bool slinger_add_slow_packet_field(const char *m_field, slinger_cache_node_t *m_node);
void slinger_add_bolo_channel(uint8_t m_board, uint8_t m_offset, slinger_cache_node_t *m_node);

#endif /* SLINGER_PREFERENCES_H_ */
