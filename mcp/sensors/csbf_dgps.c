/**
 * @file csbf_dgps.c
 *
 * @date Aug 11, 2012
 * @author seth
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


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include <blast.h>
#include <blast_comms.h>
#include <comms_common.h>
#include <comms_serial.h>
#include <mcp.h>
#include <tx.h>
#include <pointing_struct.h>


static int csbf_gps_process_data(const void *m_data, size_t m_len, void *m_userdata);
static void csbf_gps_handle_error (int m_code, void *m_priv);
static int csbf_gps_handle_finished (const void *m_data __attribute__((unused)), size_t m_len __attribute__((unused)), void *m_userdata);


#define CSBFGPSCOM "/dev/ttyCSBF"

comms_serial_t	*csbf_gps_comm = NULL;
struct DGPSAttStruct csbf_gps_att[3] = {{.az = 0.0}};
int csbf_dgpsatt_index = 0;

struct DGPSPosStruct csbf_gps_pos[3] = {{.lon = 0.0}};
int csbf_dgpspos_index = 0;

time_t csbf_gps_time;


void initialize_csbf_gps_monitor(void)
{

	if (csbf_gps_comm) comms_serial_free(csbf_gps_comm);
	csbf_gps_comm = comms_serial_new();

	csbf_gps_comm->sock->callbacks = balloc(err, sizeof(netsock_callbacks_t));
	BLAST_ZERO_P(csbf_gps_comm->sock->callbacks);
	csbf_gps_comm->sock->callbacks->data = csbf_gps_process_data;
	csbf_gps_comm->sock->callbacks->error = csbf_gps_handle_error;
	csbf_gps_comm->sock->callbacks->finished = csbf_gps_handle_finished;
	csbf_gps_comm->sock->callbacks->priv = NULL;
	comms_serial_setspeed(csbf_gps_comm, B19200);
	if (comms_serial_connect(csbf_gps_comm, CSBFGPSCOM) != NETSOCK_OK || !blast_comms_add_port(csbf_gps_comm))
	{
		blast_err("Failed to open CSBF GPS!");
		bfree(err, csbf_gps_comm->sock->callbacks);
		comms_serial_free(csbf_gps_comm);
		csbf_gps_comm = NULL;
	}
	else
	{
		blast_startup("Initialized csbf gps status monitor");
	}
}


static int csbf_gps_process_data(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
	const char *data = (const char*)m_data;
	char *tmpstring;
	size_t consumed = 0;
	size_t line_len = 0;
	uint8_t checksum = 0;
	struct tm ts;

	if (!m_len) return 0;

	while (data[consumed] != '$' && consumed < m_len) consumed++;

	if (consumed == m_len) return consumed;

	while (data[consumed + line_len] != '\r' && (consumed + line_len) < m_len) line_len++;

	if (consumed + line_len == m_len) return consumed;

	tmpstring = alloca(line_len);
	memcpy(tmpstring, &data[consumed], line_len);
	tmpstring[line_len - 1] = '\0';

	for (size_t i = 0; i < line_len && tmpstring[i] != '*'; i++)	checksum ^= tmpstring[i];

	switch (data[consumed + 1])
	{
		case 'P':
			/// Handle PASHR
			if (tmpstring[7] == 'P')
			{
				unsigned lat_deg;
				char lat_ns;
				unsigned lon_deg;
				char lon_ew;

				if (sscanf(tmpstring, "$PASHR,POS,"
						"%*d," /// Raw/Diff
						"%d," /// SV Count
						"%*f," /// UTC
						"%2u%lf,"  /// Latitude
						"%c,"
						"%3u%lf,"
						"%c,"
						"%lf" /// Altitude
						"%*d," /// ID
						"%lf," /// Heading
						"%lf,", /// Speed
						&csbf_gps_pos[csbf_dgpspos_index].n_sat,
						&lat_deg, &csbf_gps_pos[csbf_dgpspos_index].lat, &lat_ns,
						&lon_deg, &csbf_gps_pos[csbf_dgpspos_index].lon, &lon_ew,
						&csbf_gps_pos[csbf_dgpspos_index].alt,
						&csbf_gps_pos[csbf_dgpspos_index].direction,
						&csbf_gps_pos[csbf_dgpspos_index].speed) == 8)
				{
					csbf_gps_pos[csbf_dgpspos_index].lat = (double)lat_deg + csbf_gps_pos[csbf_dgpspos_index].lat / 60.0;
					if (lat_ns == 'S') csbf_gps_pos[csbf_dgpspos_index].lat *= -1.0;

					csbf_gps_pos[csbf_dgpspos_index].lon = (double)lon_deg + csbf_gps_pos[csbf_dgpspos_index].lon / 60.0;
					if (lon_ew == 'W') csbf_gps_pos[csbf_dgpspos_index].lon *= -1.0;

          if (csbf_gps_pos[csbf_dgpspos_index].n_sat > 3) {
            csbf_dgpspos_index = INC_INDEX(csbf_dgpspos_index);
          }
				}
			}
			else if (tmpstring[7] == 'S')
			{
				unsigned sat;
				unsigned antenna;
				if (sscanf(tmpstring, "$PASHR,SA4,"
						"%u," //Antenna
						"%u", //Number of Sat
						&antenna, &sat) == 2)
				{
					if (antenna > 0 && antenna < 5)
					{
						csbf_gps_att[csbf_dgpsatt_index].ant[antenna - 1] = sat;
					}
				}
			}
			break;
		case 'G':
			/// Handle GPPAT/GPZDA
			if (tmpstring[3] == 'P')
			{
				if (sscanf(tmpstring, "$GPPAT,"
						"%*f," //UTC hhmmss.ss
						"%*f,%*c," //Latitude ddmm.mmmmm N/S
						"%*f,%*c," //Longitude dddmm.mmmmm E/W
						"%*f," //Altitude
						"%lf," //Heading
						"%lf," //Pitch
						"%lf," //Roll
						"%lf," //MRMS
						"%lf," //BRMS
						"%d", //Reset Flag
						&csbf_gps_att[csbf_dgpsatt_index].az,
						&csbf_gps_att[csbf_dgpsatt_index].pitch,
						&csbf_gps_att[csbf_dgpsatt_index].roll,
						&csbf_gps_att[csbf_dgpsatt_index].mrms,
						&csbf_gps_att[csbf_dgpsatt_index].brms,
						&csbf_gps_att[csbf_dgpsatt_index].att_ok) == 6)
				{
          csbf_gps_att[csbf_dgpsatt_index].att_ok ^= 1;  // GPS flag 0 = good, but we're using opposite
					csbf_dgpsatt_index = INC_INDEX(csbf_dgpsatt_index);
				}


			}
			else if (data[consumed + 3] == 'Z')
			{
			      sscanf(tmpstring,"$GPZDA,"
			    		  "%2d%2d%2d.%*d,"
			    		  "%d,"
			    		  "%d,"
			    		  "%d,"
			    		  "%*d,"
			    		  "%*d,",
			    		  &(ts.tm_hour),&(ts.tm_min),
			    		  &(ts.tm_sec),&(ts.tm_mday),
			    		  &(ts.tm_mon),&(ts.tm_year));

			      ts.tm_year -= 1900;

			      ts.tm_isdst = 0;
			      ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

			      csbf_gps_time = mktime(&ts);
			}
	}

	return consumed + line_len;
}

static void csbf_gps_handle_error (int m_code, void *m_priv __attribute__((unused)))
{
	blast_err("Got error %d on CSBF GPS comm %s: %s", m_code, CSBFGPSCOM, strerror(m_code));
}

static int csbf_gps_handle_finished (const void *m_data __attribute__((unused)), size_t m_len __attribute__((unused)), void *m_userdata __attribute__((unused)))
{
	blast_err("Got closed socket on %s!  That shouldn't happen.  BIG ALL CAPS: REPORT THIS ERROR!!!!", CSBFGPSCOM);

	if (csbf_gps_comm && csbf_gps_comm->sock) BLAST_SAFE_FREE(csbf_gps_comm->sock->callbacks);
	comms_serial_free(csbf_gps_comm);
	csbf_gps_comm = NULL;

	return 0;
}
