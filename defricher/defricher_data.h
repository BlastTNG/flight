/* 
 * defricher_data.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 3, 2015 by Seth Hillbrand
 */

#ifndef DEFRICHER_DATA_H_
#define DEFRICHER_DATA_H_

#include <bzlib.h>
#include <stdbool.h>
#include <time.h>

#include <lookup.h>
#include <blast_time.h>

#define FILE_HASH 0

#define _DEFRICHER_CACHE_TYPES(x,_) \
    _(x, fc)                        \
    _(x, roach)                     \
    _(x, uei)
BLAST_LOOKUP_TABLE(DEFRICHER_CACHE_TYPE, static);

typedef struct defricher_file_data
{
    char                    *name;          /**< name Stream name - filename in the DIRFILE */
    char                    *desc;          /**< desc Description of the channel */
    FILE                    *fp;            /**< fp File pointer to write in DIRFILE */
    BZFILE					*gzfp;			/**< bzfp GZip file pointer for raw data */
    off_t                   offset;         /**< offset Current offset (from 0) in the file */
    size_t                  element_size;   /**< element_size Size in bytes of each data value */
    bool                    is_signed;      /**< is_signed if true, the integer is signed */
    bool                    is_float;       /**< is_float if true, the stream is floating pointer */
    uint32_t                rate;           /**< rate Number of samples per frame (current 1s frames) of the file */
    uint64_t                lastval;        /**< lastval The last value written to disk (for filling) */
} defricher_output_file_data_t;


typedef struct defricher_periodic_data
{
    uint32_t                hash;                   /**< hash The hash (without mods) of the periodic data name */
    struct timespec         time;                   /**< time The starting index of the node.  This is EBEX time */
    uint32_t                value;                  /**< value The previous value of the node.  If the reading changes from this value, a packet is generated */
} __attribute__((packed)) defricher_periodic_data_t;

typedef struct defricher_stream_data
{
    uint32_t                hash;           /**< hash The hash (without mods) of the stream name */
    blast_time_t            time;           /**< time The starting time of the node. */
    uint16_t                produced;       /**< produced Number of bytes in #buffer ready for compression/downlink */
    uint8_t                 buffer[];       /**< buffer Holds cached data */
} __attribute__((packed)) defricher_stream_data_t;

typedef struct defricher_cache_node
{
    uint32_t                        magic;              /**< magic The BLAST MAGIC word allowing quick check of type */
    E_DEFRICHER_CACHE_TYPE          type;
    defricher_output_file_data_t    output;
    union
    {
        void                        *raw_data;
        uint64_t                    *_64bit_data;
        uint32_t                    *_32bit_data;
        uint16_t                    *_16bit_data;
        defricher_stream_data_t     *input_stream;
        defricher_periodic_data_t   *input_periodic;
    };
} defricher_cache_node_t;


#endif /* DEFRICHER_DATA_H_ */
