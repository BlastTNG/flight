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

#ifndef INCLUDE_CHANNELS_TNG_H
#define INCLUDE_CHANNELS_TNG_H

#include <stdint.h>
#include <stdlib.h>

#include "channel_macros.h"
#include "lookup.h"
#include "linklist.h"

#define FIELD_LEN 32
#define UNITS_LEN 48
#define BLAST_TNG_CH_VERSION 1

#define NUM_KIDS 1024
#define NUM_ROACHES 5
#define NUM_RTYPES 3
#define NUM_ROACH_TLM 15
#define ROACH_NAME_SARRAY "ROACH_NAMES"
#define FILE_LINKLIST "z_file_dl.ll"

#include "derived.h"

#define _RATES(x, _)	\
	_(x, 1HZ)					\
	_(x, 5HZ)					\
	_(x, 100HZ)					\
	_(x, 200HZ)                 \
	_(x, 244HZ)                 \
    _(x, 488HZ)

BLAST_LOOKUP_TABLE(RATE, static);

#define _TYPES(x, _)	\
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


struct channel {
    char field[FIELD_LEN];      /// name of channel for FileFormats and CalSpecs
    double m_c2e;               /// Conversion from counts to engineering units is
    double b_e2e;               ///   e = c * m_c2e + b_e2e
    E_TYPE type;                /// Type of data stored
    E_RATE rate;                /// Rate at which the channel is recorded
    char quantity[UNITS_LEN];   /// eg, "Temperature" or "Angular Velocity"
    char units[UNITS_LEN];      /// eg, "K" or "^o/s"
    void *var;                  /// Pointer to the variable in the current frame
} __attribute__((aligned));

/**
 * These next two structures are packed for network transmission and disk storage.  Do
 * not use them for working with structures
 */
#pragma pack(push, 1)
struct channel_packed {
    char field[FIELD_LEN];      /// name of channel for FileFormats and CalSpecs
    double m_c2e;               /// Conversion from counts to engineering units is
    double b_e2e;               ///   e = c * m_c2e + b_e2e
    int8_t type;                /// Type of data stored
    int8_t rate;                /// Rate at which the channel is recorded
    char quantity[UNITS_LEN];   /// eg, "Temperature" or "Angular Velocity"
    char units[UNITS_LEN];      /// eg, "K" or "^o/s"
};

typedef struct {
    uint32_t    magic;
    uint8_t     version;
    uint32_t    length;
    uint32_t    crc;
    struct channel_packed data[0];
} channel_header_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint64_t timestamp;
    uint8_t  source;
} frame_header_t;
#pragma pack(pop)

extern void *channel_data[RATE_END];
extern size_t frame_size[RATE_END];
extern channel_t channel_list[];
extern derived_tng_t derived_list[];
extern int channels_count;
extern superframe_t * superframe;

int channels_initialize(const channel_t * const m_channel_list);
channel_t *channels_find_by_name(const char *m_name);
int channels_store_data(E_RATE m_rate, const void *m_data, size_t m_len);
int channels_check_size_of_frame(E_RATE m_rate, size_t m_len);
int channels_read_map(channel_header_t *m_map, size_t m_len, channel_t **m_channel_list);
channel_header_t *channels_create_map(channel_t *m_channel_list);
channel_header_t *channels_create_rate_map(channel_t *m_channel_list, E_RATE m_rate);
size_t channel_size(channel_t *);
unsigned int add_frame_to_superframe(void * , E_RATE , void *, unsigned int *);
unsigned int extract_frame_from_superframe(void * , E_RATE , void *, unsigned int *);
unsigned int get_spf(unsigned int);

int channels_read_derived_map(derived_header_t *m_map, size_t m_len, derived_tng_t **m_channel_list);
derived_header_t *channels_create_derived_map(derived_tng_t *m_derived);
superframe_t * channels_generate_superframe(const channel_t * const m_channel_list);
void make_name_from_roach_index(unsigned int , char * name);
int get_roach_index(unsigned int , unsigned int , unsigned int);
void read_roach_index(unsigned int *, unsigned int *, unsigned int *, unsigned int);
linklist_t * generate_housekeeping_linklist(linklist_t * , char *);
void channels_write_calspecs(char *, derived_tng_t *);
double channel_data_to_double(uint8_t * data, uint8_t type);
int channel_double_to_data(uint8_t * data, double dub, uint8_t type);

#endif /* CHANNELS_V2_H_ */
