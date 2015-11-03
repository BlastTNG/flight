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
#include <glib.h>

#include <pointing_struct.h>
#include <blast.h>
#include <comms_serial.h>
#include <blast_comms.h>
#include <crc.h>

#include <blast_sip_interface.h>
#include "blast_sip_interface_internal.h"


static gint sip_segment_compare (gconstpointer  a, gconstpointer  b) {
    blast_seg_pkt_hdr_t *seg_a = (blast_seg_pkt_hdr_t *)a;
    blast_seg_pkt_hdr_t *seg_b = (blast_seg_pkt_hdr_t *)b;

    if (seg_a->seg_id == seg_b->seg_id)
        return (seg_a->seg_num > seg_b->seg_num) - (seg_a->seg_num < seg_b->seg_num);
    else
        return (seg_a->seg_id > seg_b->seg_id) - (seg_a->seg_id < seg_b->seg_id);
}

/**
 * Removes all elements from the segment linked list with a given id
 * @param m_list Pinter to the Linked List of packet fragments
 * @param m_id Packet segment group id
 */
static inline void sip_segment_remove_id(GList *m_list, uint8_t m_id) {
    GList *iter = m_list;
    while (iter) {
        GList *next = iter->next;
        blast_seg_pkt_hdr_t *segment = (blast_seg_pkt_hdr_t*)(iter->data);
        if (segment->seg_id == m_id) {
            free(iter->data);
            m_list = g_list_delete_link(m_list, iter);
        }
        iter = next;
    }
}

/**
 * Iterates through the linked list of segments building a full packet if available
 * @param m_list Linked List of packet fragments
 * @return NULL if no complete packet found.  Pointer to a newly allocated, full BLAST
 *          packet if available.  Must be freed by calling function
 */
static blast_master_packet_t *sip_segment_get_complete_packet(GList *m_list)
{
    GList *iter = m_list;
    uint8_t id = 0;
    size_t pkt_count;
    uint8_t *p;
    blast_master_packet_t *retval = NULL;
    bool found_packet = false;

    if (!m_list)
        return NULL;

    while (1) {
        /**
         * This is our reset condition that finds new packets
         */
        if (id != ((blast_seg_pkt_hdr_t*) (iter->data))->seg_id) {
            BLAST_SAFE_FREE(retval);
            id = ((blast_seg_pkt_hdr_t*) (iter->data))->seg_id;
            pkt_count = BLAST_SEGMENT_PACKET_COUNT((blast_seg_pkt_hdr_t*) (iter->data));

            /**
             * If we might have enough data in the list, optimistically allocate the packet and begin filling it.
             * We allocate more memory than we will need, allowing us to avoid unnecessary logical gymnastics to get
             * the exact byte count.  Instead, since we expect a certain number of packets than can be no more than
             * 255 bytes, we simply allocate this amount as the extra bytes are trivial.
             */
            if (pkt_count < g_list_length(m_list))
                break;
            p = calloc(pkt_count, 255);
            retval = (blast_master_packet_t*) p;
            memcpy(p, iter->data, sizeof(blast_master_packet_t));
            p += sizeof(blast_master_packet_t);
        }

        memcpy(p, BLAST_SEGMENT_PACKET_PAYLOAD(iter->data), 255 - sizeof(blast_seg_pkt_hdr_t));
        p += (255 - sizeof(blast_seg_pkt_hdr_t));
        if (--pkt_count <= 0) {  /// This is our success condition
            found_packet = true;
            break;
        }

        iter = iter->next;
        /// This is our failure condition
        if (!iter)
            break;
    }

    /**
     * If we have successfully extracted a full packet, it is stored in *p and we remove all of its
     * elements from our list
     */
    if (found_packet)
        sip_segment_remove_id(m_list, id);
    else
        BLAST_SAFE_FREE(retval);

    return retval;
}

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
		sip_comm1->sock->callbacks.data = blast_sip_process_data;
		sip_comm1->sock->callbacks.error = blast_sip_handle_error;
		sip_comm1->sock->callbacks.finished = blast_sip_handle_finished;
		sip_comm1->sock->callbacks.priv = sip_comm1;

		comms_serial_setspeed(sip_comm1, B1200);
		if (comms_serial_connect(sip_comm1, SIP_PORT_1) != NETSOCK_OK)
		{
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
		sip_comm2->sock->callbacks.data = blast_sip_process_data;
		sip_comm2->sock->callbacks.error = blast_sip_handle_error;
		sip_comm2->sock->callbacks.finished = blast_sip_handle_finished;
		sip_comm2->sock->callbacks.priv = sip_comm2;

		comms_serial_setspeed(sip_comm2, B1200);
		if (comms_serial_connect(sip_comm2, SIP_PORT_2) != NETSOCK_OK)
		{
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

	if ((!blast_comms_add_port(sip_comm1))
			&& (!blast_comms_add_port(sip_comm2)))
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
	bool free_header = false;
	static GList *upload_llist = NULL;
	ssize_t consumed = 0;

	blast_dbg("Received command packet with %u bytes over SIP connection", (unsigned) cmd_pkt->length);

	if (!upload_llist) upload_llist = g_list_alloc();

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

	header = (blast_master_packet_t*)(cmd_pkt->data);
	if (header->magic != BLAST_MAGIC8
			|| header->version != 2)
	{
	    blast_warn("Received invalid packet over SIP.  Magic byte 0x%02X and version %d", header->magic, header->version);
	    /// Consume 1 byte here to revert to searching for the SIP start byte.
	    return 1;
	}

	/**
	 * If we have a multiple-packet chunk, add it to the linked list of chunks.
	 */

	if (header->multi_packet) {
	    blast_seg_pkt_hdr_t *segment = calloc(1,255);
	    memcpy(segment, header, cmd_pkt->length);
	    /// The only packet that _could_ be smaller than 255 in a segmented packet is the last one.
	    /// We place the smaller length in the cmd_pkt header to note this for rebuilding.
	    if (cmd_pkt->length < 255) segment->header.length = cmd_pkt->length;
	    upload_llist = g_list_insert_sorted(upload_llist, segment, sip_segment_compare);

	    /**
	     * Since we just received a new multi-packet, it is a good time to check for the full packet in our
	     * linked list.
	     */

	    if ((header = sip_segment_get_complete_packet(upload_llist)) != NULL) {
	        free_header = true;
	    }

	}

	/**
	 * If the CRC doesn't match, we abort now and consume the first byte of the payload.  This allows us to re-try the
	 * buffered payload, looking for the next BLAST start byte.  However, note that this will not occur until we receive
	 * further command data over the SIP
	 */
	if (crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(header), header->length) !=
			BLAST_MASTER_PACKET_CRC(header))
	{
		blast_err("Mismatched CRC in uplink packet 0x%08X", BLAST_MASTER_PACKET_CRC(header));
		if (free_header) free(header);
		return consumed;
	}


	//TODO: Add packet processing
//	netbuf_eat(buffer, ebex_process_packet(header));

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

	return 0;
}
