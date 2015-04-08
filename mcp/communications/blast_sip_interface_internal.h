/**
 * @file ebex_sip_interface_internal.h
 *
 * @date 2011-02-03
 * @author Seth Hillbrand
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
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

#ifndef BLAST_SIP_INTERFACE_INTERNAL_H_
#define BLAST_SIP_INTERFACE_INTERNAL_H_
#include <pthread.h>
#include <termios.h>
#include <endian.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>

#include <comms_netbuf.h>
#include <comms_serial.h>
#include <blast_packet_format.h>
#include <lookup.h>
#include <comms_common.h>

/**
 * We assume that the byte order received from the SIP (LSB first, MSB last) can be read directly into the
 * data values.  This will be true for little endian (e.g. x86, ARM and most other desktops) but not true
 * for big endian.  We don't anticipate running anything on the ground from a big endian system but best
 * to be careful.
 *
 * Note: Implementing big endian compatibility is straight-forward if you want to do it.  See swab(3).
 */
#if	__BYTE_ORDER == __BIG_ENDIAN
#	error This code will not work on a big endian system
#endif

#define SIP_PORT_1 "/dev/ttyS0"
#define SIP_PORT_2 "/dev/ttyS1"

#define SUN_JAN_6_1980 315982800L 	/**< Number of seconds from the start of the UNIX epoch to Jan 6, 1980 */
#define SEC_IN_WEEK  604800L		/**< Number of seconds in a week */

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

#define _SIP_RECV_MSGS(x,_)		\
	_(x, GPS_DATA)				\
	_(x, GPS_TIME)				\
	_(x, MKS_DATA)				\
	_(x, READY)					\
	_(x, CMD)

static ssize_t SIP_RECV_MSG_GPS_DATA_CALLBACK(const uint8_t* m_data, size_t m_len);
static ssize_t SIP_RECV_MSG_GPS_TIME_CALLBACK (const uint8_t* m_data, size_t m_len);
static ssize_t SIP_RECV_MSG_MKS_DATA_CALLBACK (const uint8_t* m_data, size_t m_len);
static ssize_t SIP_RECV_MSG_READY_CALLBACK (const uint8_t* m_data, size_t m_len);
static ssize_t SIP_RECV_MSG_CMD_CALLBACK (const uint8_t* m_data, size_t m_len);

static int blast_sip_process_data(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)));
static void blast_sip_handle_error (int m_code, void *m_priv);
static int blast_sip_handle_finished (const void *m_data, size_t m_len, void *m_priv);

BLAST_GENERIC_LOOKUP_TABLE(SIP_RECV_MSG, static,
			ssize_t (*process_msg) (const uint8_t *, size_t);,
			_BLAST_FUNCTION_STRUCT_LIST,
			 );
//@TODO:Evaluate where to put sip control (if needed)
//BLAST_GENERIC_LOOKUP_TABLE(sip_ctrl_command, static,
//			ssize_t (*process_ctrl) (ebex_link_ctrl_pkt_t *);,
//			_BLAST_FUNCTION_STRUCT_LIST,
//			 );

#define _SIP_CMD_PKT_TYPES(x,_)	\
	_(x,CTRL)					\
	_(x,CMD)					\
	_(x,FILE)
BLAST_LOOKUP_TABLE(SIP_CMD_PKT_TYPE, static);

#define _SIP_CMD_SOURCES(x,_)	\
	_(x,LOS)					\
	_(x,TDRSS)					\
	_(x,IRIDIUM)
BLAST_LOOKUP_TABLE(SIP_CMD_SOURCE, static);

#define _SIP_CTRL_REFS(x,_)     \
	_(x,LOS)                    \
	_(x,TDRSS_HGA)              \
	_(x,TDRSS)                  \
	_(x,IRIDIUM)                \
	_(x,SLOW)
BLAST_LOOKUP_TABLE(SIP_CTRL_REF, static);

typedef enum sip_send_msg
{
	sip_send_msg_gps_pos		= 0x50,
	sip_send_msg_gps_time,
	sip_send_msg_mks_alt,
	sip_send_msg_data
} e_sip_send_msg;

typedef enum sip_gps_sat_status
{
	sip_gps_sat_nominal	= 0,
	sip_gps_sat_no_time,
	sip_gps_sat_wait_for_almanac,
	sip_gps_sat_PDOP_too_high,
	sip_gps_sat_zero_sat,
	sip_gps_sat_one_sat,
	sip_gps_sat_two_sat,
	sip_gps_sat_three_sat,
	sip_gps_sat_zero_useable_sat,
	sip_gps_sat_one_usable_sat,
	sip_gps_sat_two_usable_sat,
	sip_gps_sat_three_usable_sat,
} e_sip_gps_sat_status;

typedef struct sip_std_hdr
{
	uint8_t		start_byte;
	uint8_t		id_byte;
} __attribute__((packed)) sip_std_hdr_t;

typedef struct sip_gps_pos
{
	sip_std_hdr_t			header;
	float					longitude;
	float					latitude;
	float					altitude;
	uint8_t					num_sats;
	e_sip_gps_sat_status	sat_status:8;
	uint8_t					end_byte;
} __attribute__((packed)) sip_gps_pos_t;

typedef struct sip_gps_time
{
	sip_std_hdr_t			header;
	float		week_sec;
	uint16_t	week_num;
	float		utc_offset;
	float		midnight_sec;
	uint8_t		end_byte;
} __attribute__((packed)) sip_gps_time_t;

typedef struct sip_mks_altitude
{
	sip_std_hdr_t			header;
	uint16_t	mks_hi;
	uint16_t	mks_med;
	uint16_t	mks_lo;
	uint8_t		end_byte;
} __attribute__((packed)) sip_mks_altitude_t;

typedef struct sip_science_cmd
{
	sip_std_hdr_t               header;
	uint8_t                     length;
	union
	{
		blast_master_packet_t   blast_header;
		uint8_t                 data[1];
	};
} __attribute__((packed)) sip_science_cmd_t;

static comms_serial_t *sip_comm1 = NULL;
static comms_serial_t *sip_comm2 = NULL;

#endif /* BLAST_SIP_INTERFACE_INTERNAL_H_ */
