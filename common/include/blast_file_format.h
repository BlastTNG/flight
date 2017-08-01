/*
 * blast_file_format.h:
 *
 * This software is copyright (C) 2017 Seth Hillbrand
 *
 * This file is part of blast_comms, created for the BLASTPol Project.
 *
 * blast_comms is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * blast_comms is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blast_comms; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: May 25, 2017 by Seth Hillbrand
 */

#ifndef INCLUDE_BLAST_FILE_FORMAT_H_
#define INCLUDE_BLAST_FILE_FORMAT_H_

#include <stdint.h>
#include <lookup.h>

#define _BLAST_FILE_TYPES(x, _) \
    _(x, FRAME_1HZ)             \
    _(x, FRAME_5HZ)             \
    _(x, FRAME_20HZ)            \
    _(x, FRAME_100HZ)           \
    _(x, FRAME_200HZ)           \
    _(x, FRAME_244HZ)           \
    _(x, FRAME_FORMAT)          \
    _(x, KID)                   \

typedef struct blast_file_packet
{
    uint8_t             magic;              /**< magic = 0xEB for all packets */
    uint32_t            version:4;          /**< version Currently at 1 */
    uint32_t            type:4;             /**< type The type of data in this packet */
    uint32_t            index;              /**< index Monotonically incrementing num (all types) */
    uint32_t            time;               /**< time UNIX time */
    uint32_t            time_ns;            /**< time_ns Nanoseconds since last second */
    uint32_t            length;             /**< length Number of bytes in the packet */
} __attribute__((packed)) blast_file_packet_t; /** 18 bytes */

#define BLAST_FILE_PACKET_FULL_LENGTH(_ptr) (sizeof(blast_file_packet_t) + (_ptr)->length + 4)
#define BLAST_FILE_PACKET_PAYLOAD(_ptr) ((uint8_t*)(((blast_file_packet_t*)(_ptr))+1))
#define BLAST_FILE_PACKET_CRC(_ptr) (*(uint32_t*)(&BLAST_FILE_PACKET_PAYLOAD(_ptr)[(_ptr)->length]))

#endif /* INCLUDE_BLAST_FILE_FORMAT_H_ */
