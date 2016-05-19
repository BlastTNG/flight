/**
 * @file slinger_frame.h
 *
 * @date Feb 21, 2011
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


#ifndef SLINGER_FRAME_H_
#define SLINGER_FRAME_H_
#include <stdbool.h>
#include <stdint.h>

#include <ebex_sl.h>
#include <ebex_hash.h>
#include <slinger_data.h>

typedef struct slinger_frame_data
{
	hash_t 	hash;				//!< hash Gives the hashed channel name for lookup
	int32_t	size;				//!< size number of bytes in the field.  Wide fields = 4, Narrow = 2
	bool	is_slow;			//!< is_slow if TRUE, frame is a slow frame
	slinger_cache_node_t *node;	//!< node pointer to the cache node containing the cached data
} slinger_frame_data_t;

void initialize_slinger_frame_lookup(void);
bool slinger_add_frame_channel(const char *m_channel, slinger_cache_node_t *m_node);
uint32_t slinger_get_channel_rate(const char *m_channel);

#endif /* SLINGER_FRAME_H_ */
