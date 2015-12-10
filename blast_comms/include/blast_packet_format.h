/* 
 * blast_packet_format.h: 
 *
 * This software is copyright (C) 2013-2015 Seth Hillbrand
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
 * Created on: Apr 7, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_BLAST_PACKET_FORMAT_H
#define INCLUDE_BLAST_PACKET_FORMAT_H

#include <lookup.h>

#define _BLAST_PORT_DEFS(x, _)      \
    _(x, DATA)                      \
    _(x, COMMAND)                   \
    _(x, FILE)                      \


static const uint8_t sip_start_byte = 0x10;
static const uint8_t sip_end_byte = 0x03;

typedef struct blast_master_packet
{
    uint8_t             magic;              /**< magic = 0xEB for all packets */

    uint32_t            version:2;          /**< version Currently at 1 */
    uint32_t            qos:1;              /**< qos Quality of service flag.  1 = Response required */
    uint32_t            has_ack:1;          /**< has_ack The packet has an appended acknowledgment signature */
    uint32_t            multi_packet:1;     /**< multi_packet 0 = single packet in the payload; 1 = multiple packets*/
    uint32_t            type:3;             /**< type The type of data in this payload */
    uint16_t            length;             /**< length Number of bytes in the payload */
} __attribute__((packed)) blast_master_packet_t; /** 4 bytes */


typedef struct blast_seg_pkt_hdr
{
    blast_master_packet_t   header;
    uint8_t                 seg_id;     /**< seg_id sequential id to group the segments */
    uint8_t                 seg_num;    /**< seg_num ordering number of multi-segment data */
} __attribute__((packed)) blast_seg_pkt_hdr_t;

typedef struct
{
    uint8_t start_byte;
    uint8_t link_route;     /**< link_route los, tdrss or iridium */
    uint8_t link_address;   /**!< link_address either comm1 or comm2 */
    uint8_t length;
} __attribute__((packed)) sip_rts_packet_t;

typedef struct blast_data_pkt_header
{
    uint32_t                hash;           /**< hash Hash of the data contained in the packet */
}__attribute__((packed)) blast_data_pkt_header_t;

typedef struct blast_cmd_pkt_header
{
    uint32_t                cmd;            /**< cmd Hash of the command name */
    uint16_t                cmd_num;        /**< cmd_num A sequential number allowing distinct acknowledgment */
}__attribute__((packed)) blast_cmd_pkt_header_t;

typedef struct blast_file_pkt_header
{
    char                    filename[1];    /**< filename Pointer to the name of the file found in the packet */
}__attribute__((packed)) blast_file_pkt_header_t;

#define BLAST_MASTER_PACKET_FULL_LENGTH(_ptr) (sizeof(blast_master_packet_t) + (_ptr)->length + 4)
#define BLAST_MASTER_PACKET_PAYLOAD(_ptr) ((uint8_t*)(((blast_master_packet_t*)(_ptr))+1))
#define BLAST_MASTER_PACKET_CRC(_ptr) (*(uint32_t*)(&BLAST_MASTER_PACKET_PAYLOAD(_ptr)[(_ptr)->length]))
#define BLAST_MASTER_PACKET_ACK(_ptr) (*(uint32_t*)(&BLAST_MASTER_PACKET_PAYLOAD(_ptr)[(_ptr)->length + 4]))
#define BLAST_SEGMENT_PACKET_COUNT(_ptr) \
    (((_ptr)->header.length + 4 - (255 - sizeof(blast_seg_pkt_hdr_t) -1)) / (255 - sizeof(blast_seg_pkt_hdr_t)))
#define BLAST_SEGMENT_PACKET_PAYLOAD(_ptr) ((uint8_t*)(((blast_seg_pkt_hdr_t*)(_ptr))+1))


#endif /* INCLUDE_BLAST_PACKET_FORMAT_H_ */
