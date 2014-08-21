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

#include <channel_macros.h>

#define FIELD_LEN 32
#define UNITS_LEN 48
#define CHANNELS_HASH_SEED 0xEB90

typedef enum frame_rate{
    RATE_1HZ = 0,
    RATE_5HZ,
    RATE_100HZ,
    RATE_200HZ,
    RATE_END
} e_rates;

typedef enum variable_type{
    TYPE_UINT8 = 0,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_UINT64,
    TYPE_INT8,
    TYPE_INT16,
    TYPE_INT32,
    TYPE_INT64,
    TYPE_FLOAT,
    TYPE_DOUBLE,
    TYPE_END
} e_vartypes;

typedef enum data_source{
    SRC_UEI1 = 0,
    SRC_UEI2,
    SRC_FC,
    SRC_SC,
    SRC_END
} e_datasources;

#pragma pack(push,1)
struct channel {
	char field[FIELD_LEN]; 		/// name of channel for FileFormats and CalSpecs
	double m_c2e; 				/// Conversion from counts to enginering units is
	double b_e2e; 				///   e = c * m_c2e + b_e2e
	e_vartypes type;			/// Type of data stored
	e_rates rate; 				/// Rate at which the channel is recorded
	e_datasources source;       /// Source of the data channel (who writes)
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


int channels_initialize(const char *m_filename);
channel_t *channels_find_by_name(const char *m_name);


#endif /* CHANNELS_V2_H_ */
