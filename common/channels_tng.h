/* 
 * channels_v2.h: 
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

#ifndef CHANNELS_V2_H_
#define CHANNELS_V2_H_

#include <stdint.h>
#include <stdlib.h>

#include <channel_macros.h>
#include "lookup.h"

#define FIELD_LEN 32
#define UNITS_LEN 48
#define CHANNELS_HASH_SEED 0xEB90

#define _RATES(x,_)	\
	_(x, 1HZ)					\
	_(x, 5HZ)					\
	_(x, 100HZ)					\
	_(x, 200HZ)

BLAST_LOOKUP_TABLE(RATE, static);

#define _TYPES(x,_)	\
	_(x, UINT8)		\
	_(x, UINT16)	\
	_(x, UINT32)	\
	_(x, UINT64)	\
	_(x, INT8)		\
	_(x, INT16)		\
	_(x, INT32)		\
	_(x, INT64)		\
	_(x, FLOAT)		\
	_(x, DOUBLE)
BLAST_LOOKUP_TABLE(TYPE, static);

#define _SRCS(x,_)	\
	_(x, OF_UEI)	\
	_(x, IF_UEI)	\
	_(x, FC)		\
	_(x, SC)
BLAST_LOOKUP_TABLE(SRC, static);


#pragma pack(push,1)
struct channel {
	char field[FIELD_LEN]; 		/// name of channel for FileFormats and CalSpecs
	double m_c2e; 				/// Conversion from counts to enginering units is
	double b_e2e; 				///   e = c * m_c2e + b_e2e
	e_TYPE type;			/// Type of data stored
	e_RATE rate; 				/// Rate at which the channel is recorded
	e_SRC source;       /// Source of the data channel (who writes)
	char quantity[UNITS_LEN]; 	/// eg, "Temperature" or "Angular Velocity"
	char units[UNITS_LEN]; 		/// eg, "K" or "^o/s"
	void *var;                  /// Pointer to the variable in the current frame
};
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    uint64_t timestamp;
    uint8_t  source;
} frame_header_t;
#pragma pack(pop)

extern void *channel_data[SRC_END][RATE_END];
extern size_t frame_size[SRC_END][RATE_END];

int channels_initialize(const char *m_filename);
channel_t *channels_find_by_name(const char *m_name);
int channels_store_data(e_SRC m_src, e_RATE m_rate, const void *m_data, size_t m_len);

#endif /* CHANNELS_V2_H_ */
