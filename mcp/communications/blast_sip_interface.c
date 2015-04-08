/**
 * @file blast_sip_interface.c
 *
 * @date 2011-02-03
 * @author Seth Hillbrand
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011-2015 Seth Hillbrand
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

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include <pointing_struct.h>
#include <blast.h>
#include <comms_serial.h>
#include <blast_comms.h>
#include <crc.h>

#include <blast_sip_interface.h>
#include "blast_sip_interface_internal.h"



/**
 * Initialize the SIP monitoring/communication system.  Opens both COMM ports and connects them to the
 * BLAST comm port monitor thread
 * @return true on success, false on failure
 */
bool initialize_sip_interface(void)
{

	if (sip_comm1) comms_serial_free(sip_comm1);
	if (sip_comm2) comms_serial_free(sip_comm2);

	sip_comm1 = comms_serial_new(NULL);
	sip_comm2 = comms_serial_new(NULL);

	if (!sip_comm1 && !sip_comm2 )
	{
		blast_err("Could not allocate any serial port");
		return false;
	}

	if (sip_comm1)
	{
		sip_comm1->sock->callbacks = balloc(err, sizeof(netsock_callbacks_t));
		BLAST_ZERO_P(sip_comm1->sock->callbacks);
		sip_comm1->sock->callbacks->data = blast_sip_process_data;
		sip_comm1->sock->callbacks->error = blast_sip_handle_error;
		sip_comm1->sock->callbacks->finished = blast_sip_handle_finished;
		sip_comm1->sock->callbacks->priv = sip_comm1;

		comms_serial_setspeed(sip_comm1, B1200);
		if (comms_serial_connect(sip_comm1, SIP_PORT_1) != NETSOCK_OK)
		{
			bfree(err, sip_comm1->sock->callbacks);
			comms_serial_free(sip_comm1);
			sip_comm1 = NULL;
		}
		else
		{
		    //TODO:Add periodic function interface
//			ebex_periodic_register_function(sip_req_pos, 50, 5, sip_comm1, NULL);
//			ebex_periodic_register_function(sip_req_time, 50, 15, sip_comm1, NULL);
//			ebex_periodic_register_function(sip_req_alt, 50, 25, sip_comm1, NULL);
		}
	}
	if (sip_comm2)
	{
		sip_comm2->sock->callbacks = balloc(err, sizeof(netsock_callbacks_t));
		BLAST_ZERO_P(sip_comm2->sock->callbacks);
		sip_comm2->sock->callbacks->data = blast_sip_process_data;
		sip_comm2->sock->callbacks->error = blast_sip_handle_error;
		sip_comm2->sock->callbacks->finished = blast_sip_handle_finished;
		sip_comm2->sock->callbacks->priv = sip_comm2;

		comms_serial_setspeed(sip_comm2, B1200);
		if (comms_serial_connect(sip_comm2, SIP_PORT_2) != NETSOCK_OK)
		{
			bfree(err, sip_comm2->sock->callbacks);
			comms_serial_free(sip_comm2);
			sip_comm2 = NULL;
		}
		else
		{
//			ebex_periodic_register_function(sip_req_pos, 50, 5, sip_comm2, NULL);
//			ebex_periodic_register_function(sip_req_time, 50, 15, sip_comm2, NULL);
//			ebex_periodic_register_function(sip_req_alt, 50, 25, sip_comm2, NULL);
		}
	}

	if (!(blast_comms_add_port(sip_comm1) /** N.B. This is a binary OR, not a boolean.  Do not change unless fully refactoring! */
			| blast_comms_add_port(sip_comm2)))
	{
		blast_err("Could not add either comm port to our monitor");
		return false;
	}

	blast_startup("Initialized SIP interface");
	return true;
}

/**
 * Sends a formatted request packet to the SIP
 * @param m_serial Serial port over which to send the packet
 * @param m_req Command byte to transmit (valid values: REQ_POSITION, REQ_TIME, REQ_ALTITUDE)
 */
static inline void sip_send_request (comms_serial_t *m_serial, uint8_t m_req)
{
	uint8_t buffer[3];

	buffer[0] = sip_start_byte;
	buffer[1] = m_req;
	buffer[2] = sip_end_byte;

	if (m_serial)
		comms_serial_write(m_serial, buffer, 3);
	else
		blast_err("Attempted to write to NULL serial pointer");
}

/**
 * Wrapper function for periodic callback, requesting position
 * @param m_arg Pointer to requested serial port
 */
void sip_req_pos(void *m_arg)
{
	sip_send_request((comms_serial_t*)m_arg, REQ_POSITION);
}

/**
 * Wrapper function for periodic callback, requesting time from the SIP
 * @param m_arg Pointer to requested serial port
 */
void sip_req_time(void *m_arg)
{
	sip_send_request((comms_serial_t*)m_arg, REQ_TIME);
}

/**
 * Wrapper function for periodic callback, requesting altitude from the SIP
 * @param m_arg Pointer to requested serial port
 */
void sip_req_alt(void *m_arg)
{
	sip_send_request((comms_serial_t*)m_arg, REQ_ALTITUDE);
}

/**
 * Parses and stores the GPS data structure from the SIP
 * @param m_data Pointer to the data stream from the SIP
 * @param m_len Length in bytes of the available data
 * @return Number of bytes consumed
 */
static ssize_t SIP_RECV_MSG_GPS_DATA_CALLBACK (const uint8_t* m_data, size_t m_len)
{
	sip_gps_pos_t *gps_pos_pkt = NULL;

	if (m_len < sizeof(sip_gps_pos_t))
		return 0;

	gps_pos_pkt = (sip_gps_pos_t*)m_data;
	if (gps_pos_pkt->end_byte != sip_end_byte)
	{
		blast_info("Received corrupted GPS Data Packet");
		return 1;
	}

	SIPData.GPSpos.alt = gps_pos_pkt->altitude;
	SIPData.GPSpos.lat = gps_pos_pkt->latitude;
	SIPData.GPSpos.lon = gps_pos_pkt->longitude;
	SIPData.GPSstatus1 = gps_pos_pkt->num_sats;
	SIPData.GPSstatus2 = gps_pos_pkt->sat_status;

	blast_dbg("Received valid GPS Data Packet from SIP");
	return sizeof(sip_gps_pos_t);
}

/**
 * Parses and stores the GPS time structure from the SIP
 * @param m_data Pointer to the data stream from the SIP
 * @param m_len Length in bytes of the available data
 * @return Number of bytes consumed
 */
static ssize_t SIP_RECV_MSG_GPS_TIME_CALLBACK (const uint8_t* m_data, size_t m_len)
{
	sip_gps_time_t *gps_time_pkt = NULL;

	if (m_len < sizeof(sip_gps_time_t))
		return 0;

	gps_time_pkt = (sip_gps_time_t*)m_data;
	if (gps_time_pkt->end_byte != sip_end_byte)
	{
		blast_info("Received corrupted GPS Time Packet");
		return 1;
	}
	SIPData.GPStime.CPU = gps_time_pkt->midnight_sec;
	SIPData.GPStime.UTC = (unsigned long)(SUN_JAN_6_1980 +
			(SEC_IN_WEEK * gps_time_pkt->week_num) + gps_time_pkt->week_sec - gps_time_pkt->utc_offset);

	blast_dbg("Received valid GPS Time Packet %d (%d sec from midnight) from SIP", SIPData.GPStime.UTC, SIPData.GPStime.CPU);
	return sizeof(sip_gps_time_t);
}

/**
 * Parses and stores the MKS data structure from the SIP
 * @param m_data Pointer to the data stream from the SIP
 * @param m_len Length in bytes of the available data
 * @return Number of bytes consumed
 */
static ssize_t SIP_RECV_MSG_MKS_DATA_CALLBACK (const uint8_t* m_data, size_t m_len)
{
	sip_mks_altitude_t *mks_pkt = NULL;

	if (m_len < sizeof(sip_mks_altitude_t))
		return 0;

	mks_pkt = (sip_mks_altitude_t*)m_data;
	if (mks_pkt->end_byte != sip_end_byte)
	{
		blast_info("Received corrupted MKS packet");
		return 1;
	}

	blast_dbg("Received valid MKS Data Packet from SIP");

	SIPData.MKSalt.hi = mks_pkt->mks_hi;
	SIPData.MKSalt.med = mks_pkt->mks_med;
	SIPData.MKSalt.lo = mks_pkt->mks_lo;
	return sizeof(sip_mks_altitude_t);
}

/**
 * Handle the SIP asking for new data over the SLOW link (single 255-byte packet)
 * @param m_data Pointer to the data stream from the SIP
 * @param m_len Length in bytes of the available data
 * @return Number of bytes consumed (static 3 bytes)
 */
static ssize_t SIP_RECV_MSG_READY_CALLBACK (const uint8_t* m_data __attribute__((unused)), size_t m_len __attribute__((unused)))
{
    ///TODO: Add slow downlink data
	blast_dbg("Received request for slow downlink data");
	return 3;
}

/**
 * Handle the SIP passing along a command for us to process
 * @param m_data Pointer to the data stream (starting at the length byte)
 * @param m_len Length in bytes of the buffer holding the data stream
 * @return Number of bytes consumed
 */
static ssize_t SIP_RECV_MSG_CMD_CALLBACK (const uint8_t* m_data, size_t m_len)
{
	sip_science_cmd_t *cmd_pkt = (sip_science_cmd_t*)m_data;
	blast_master_packet_t *header;
	static comms_netbuf_t *buffer = NULL;
	ssize_t consumed = 0;

	blast_dbg("Received command packet with %u bytes over SIP connection", (unsigned) cmd_pkt->length);

	if (!buffer) buffer = comms_netbuf_new();
	/**
	 * m_len counts the packet + sip_header (2 bytes) + length byte + terminating byte
	 * cmd_pkt->length counts just the packet data
	 * Thus m_len should be 2 bytes longer than cmd_pkt->length for a completed packet
	 */
	if ((size_t)(cmd_pkt->length + 4) > m_len )
	{
		blast_dbg("Packet incomplete.  Waiting for more data");
		return 0;
	}

	/**
	 * Note that the sip_end_byte is the byte immediately following the last data byte.  So #cmd_pkt->header.length
	 * is the offset of the +1 byte
	 */
	if (cmd_pkt->data[cmd_pkt->length] != sip_end_byte)
	{
		blast_err("Did not find packet end byte at expected location!");
		return 1;
	}

	/**
	 * Place the packet data in our netbuffer.  We don't yet know whether we have full packets or not, so the full SIP
	 * packet content gets buffered, which we then use for determining whether we have the full BLAST packet (which may
	 * be larger).
	 *
	 * If we are still waiting for more data, consume the SIP packet out of the serial queue and await the next packet, which
	 * should contain further segments for the full BLAST packet
	 */
	comms_netbuf_add(buffer, cmd_pkt->data, cmd_pkt->length);
	consumed = cmd_pkt->length + 4; /// This represents the start byte, id byte, length byte and end byte (plus data length)

	header = (blast_master_packet_t*)comms_netbuf_get_head(buffer);
	while (header->magic != BLAST_MAGIC8
			|| header->version != 1
			|| header->length > 255)
	{
		if (!comms_netbuf_eat(buffer, 1))
		{
			blast_warn("Reached end of SIP command buffer without EBEX start byte");
			return consumed;
		}
		header = (blast_master_packet_t*)comms_netbuf_get_head(buffer);
	}
	if (comms_netbuf_remaining(buffer) < BLAST_MASTER_PACKET_FULL_LENGTH(header))
		return consumed;


	/**
	 * If the CRC doesn't match, we abort now and consume the first byte of the payload.  This allows us to re-try the
	 * buffered payload, looking for the next EBEX start byte.  However, note that this will not occur until we receive
	 * further command data over the SIP
	 */
	if (crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(header), header->length) !=
			BLAST_MASTER_PACKET_CRC(header))
	{
		blast_err("Mismatched CRC in uplink packet");
		comms_netbuf_eat(buffer, 1);
		return consumed;
	}


	//TODO: Add packet processing
//	comms_netbuf_eat(buffer, ebex_process_packet(header));

	return consumed;
}

/**
 * Handles the data inflow from the SIP.
 * @param m_data Pointer to the next byte in the SIP stream
 * @param m_len length of data queued
 * @param m_userdata Pointer to the SIP port connection
 * @return Number of bytes consumed by processing
 */
static int blast_sip_process_data(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
	uint8_t *sip_packet = (uint8_t*)m_data;
	size_t consumed = 0;
	comms_serial_t *port = (comms_serial_t*)m_userdata;
	sip_std_hdr_t *header = NULL;

	if (!m_len) return 0;

	while ((consumed < m_len) && (sip_packet[consumed] != sip_start_byte))
	{
		consumed++;
	}

	if (sip_packet[consumed] != sip_start_byte)
	{
		log_leave("Could not find start byte in %d input bytes", (int)m_len);
		return m_len;
	}

	if (m_len - consumed < 3)
	{
		log_leave("Insufficient bytes for meaningful input");
		return consumed;
	}

	header = (sip_std_hdr_t*) &sip_packet[consumed];
	if ((header->id_byte < 0x10)
			|| (header->id_byte >= SIP_RECV_MSG_END + 0x10))
	{
		blast_err("Invalid packet type: 0x%x from %s", header->id_byte, port->sock->host);
		consumed++;

	}
	else
	{
		consumed += SIP_RECV_MSG_LOOKUP_TABLE[header->id_byte & 0xf].process_msg((uint8_t*)header, m_len - consumed);
	}

	return consumed;

}

static void blast_sip_handle_error (int m_code, void *m_priv)
{
	comms_serial_t *port = (comms_serial_t*)m_priv;
	blast_err("Got error %d on SIP comm %s: %s", m_code, port->sock->host, strerror(m_code));
}

static int blast_sip_handle_finished (const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
	comms_serial_t *port = (comms_serial_t*)m_userdata;
	if (port && port->sock && port->sock->host)
		blast_err("Got closed socket on %s!  That shouldn't happen", port->sock->host);
	else
		blast_err("Got closed socket on unknown SIP port!");

	if (m_data) blast_sip_process_data(m_data, m_len, m_userdata);

	if (port->sock) BLAST_SAFE_FREE(port->sock->callbacks);
	comms_sock_free(port->sock);
	port->sock = NULL;

	return 0;
}
