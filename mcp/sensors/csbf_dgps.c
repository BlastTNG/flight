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


#include <phenom/serial.h>

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include <blast.h>
#include <comms_serial.h>
#include <mcp.h>
#include <tx.h>
#include <pointing_struct.h>

#define CSBFGPSCOM "/dev/ttyCSBF"

ph_serial_t	*csbf_gps_comm = NULL;
struct DGPSAttStruct csbf_gps_att[3] = {{.az = 0.0}};
int csbf_dgpsatt_index = 0;

struct DGPSPosStruct csbf_gps_pos[3] = {{.lon = 0.0}};
int csbf_dgpspos_index = 0;

time_t csbf_gps_time;



static bool csbf_gps_verify_checksum(const char *m_buf, size_t m_linelen)
{
    uint8_t checksum = 0;
    uint8_t recv_checksum = (uint8_t) strtol(&(m_buf[m_linelen - 3]), (char **)NULL, 16);

    for (size_t i = 0; i < m_linelen && m_buf[i] != '*'; i++) checksum ^= m_buf[i];
    if (recv_checksum != checksum) {
        blast_info("Received invalid checksum from CSBF GPS Data!");
        return false;
    }
    return true;
}
static void process_pashr_pos(const char *m_data) {

    unsigned lat_deg;
    char lat_ns;
    unsigned lon_deg;
    char lon_ew;

    if (sscanf(m_data,
            "$PASHR,POS,"
                    "%*d," /// Raw/Diff
                    "%d,"/// SV Count
                    "%*f,"/// UTC
                    "%2u%lf,"/// Latitude
                    "%c,"
                    "%3u%lf,"
                    "%c,"
                    "%lf,"/// Altitude
                    "%*d,"/// ID
                    "%lf,"/// Heading
                    "%lf,",/// Speed
            &csbf_gps_pos[csbf_dgpspos_index].n_sat, &lat_deg, &csbf_gps_pos[csbf_dgpspos_index].lat, &lat_ns, &lon_deg,
            &csbf_gps_pos[csbf_dgpspos_index].lon, &lon_ew, &csbf_gps_pos[csbf_dgpspos_index].alt,
            &csbf_gps_pos[csbf_dgpspos_index].direction, &csbf_gps_pos[csbf_dgpspos_index].speed) == 8) {
        csbf_gps_pos[csbf_dgpspos_index].lat = (double) lat_deg + csbf_gps_pos[csbf_dgpspos_index].lat / 60.0;
        if (lat_ns == 'S') csbf_gps_pos[csbf_dgpspos_index].lat *= -1.0;

        csbf_gps_pos[csbf_dgpspos_index].lon = (double) lon_deg + csbf_gps_pos[csbf_dgpspos_index].lon / 60.0;
        if (lon_ew == 'W') csbf_gps_pos[csbf_dgpspos_index].lon *= -1.0;

        if (csbf_gps_pos[csbf_dgpspos_index].n_sat > 3) {
            csbf_dgpspos_index = INC_INDEX(csbf_dgpspos_index);
        }
    }
}

static void process_pashr_sa4(const char *m_data) {
    unsigned sat;
    unsigned antenna;
    if (sscanf(m_data, "$PASHR,SA4,"
            "%u," //Antenna
            "%u",//Number of Sat
    &antenna, &sat) == 2) {
        if (antenna > 0 && antenna < 5) {
            csbf_gps_att[csbf_dgpsatt_index].ant[antenna - 1] = sat;
        }
    }
}

static void process_gppat(const char *m_data) {
    if (sscanf(m_data,
            "$GPPAT,"
                    "%*f," //UTC hhmmss.ss
                    "%*f,%*c,"//Latitude ddmm.mmmmm N/S
                    "%*f,%*c,"//Longitude dddmm.mmmmm E/W
                    "%*f,"//Altitude
                    "%lf,"//Heading
                    "%lf,"//Pitch
                    "%lf,"//Roll
                    "%lf,"//MRMS
                    "%lf,"//BRMS
                    "%d",//Reset Flag
            &csbf_gps_att[csbf_dgpsatt_index].az, &csbf_gps_att[csbf_dgpsatt_index].pitch,
            &csbf_gps_att[csbf_dgpsatt_index].roll, &csbf_gps_att[csbf_dgpsatt_index].mrms,
            &csbf_gps_att[csbf_dgpsatt_index].brms, &csbf_gps_att[csbf_dgpsatt_index].att_ok) == 6) {
        csbf_gps_att[csbf_dgpsatt_index].att_ok ^= 1;  // GPS flag 0 = good, but we're using opposite
        csbf_dgpsatt_index = INC_INDEX(csbf_dgpsatt_index);
    }
}

static void process_gpzda(const char *m_data)
{
    struct tm ts;
    sscanf(m_data, "$GPZDA,"
            "%2d%2d%2d.%*d,"
            "%d,"
            "%d,"
            "%d,"
            "%*d,"
            "%*d,",
            &(ts.tm_hour),
            &(ts.tm_min),
            &(ts.tm_sec),
            &(ts.tm_mday),
            &(ts.tm_mon),
            &(ts.tm_year));

    ts.tm_year -= 1900;

    ts.tm_isdst = 0;
    ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

    csbf_gps_time = mktime(&ts);
}

static void csbf_gps_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);
    size_t line_len = 0;

    typedef struct
    {
        void (*proc)(const char*);
        char str[16];
    } nmea_handler_t;

    nmea_handler_t handlers[] = { { process_pashr_pos, "$PASHR,POS" },
                                  { process_pashr_sa4, "$PASHR,SA4" },
                                  { process_gppat, "$GPPAT," },
                                  { process_gpzda, "$GPZDA," },
                                  { NULL, "" } };
    char *bufp;
    ph_buf_t *buf;

    if (!(buf = ph_serial_read_record(serial, "\r", 1))) return;

    bufp = (char*) ph_buf_mem(buf);
    line_len = ph_buf_len(buf);

    /// Replace the terminating '\r' with a NULL for C-string functions.
    bufp[line_len - 1] = 0;
    while (*bufp != '$') {
        /// This is a partial string case where we missed the starting '$'
        if (!--line_len) return;
        bufp++;
    }

    if (!csbf_gps_verify_checksum(bufp, line_len)) return;

    for (nmea_handler_t *handler = handlers; handler->proc; handler++) {
        if (!strncmp(bufp, handler->str, strlen(handler->str))) handler->proc(bufp);
    }
    ph_buf_delref(buf);
}

void initialize_csbf_gps_monitor(void)
{

    if (csbf_gps_comm) ph_serial_free(csbf_gps_comm);
    csbf_gps_comm = ph_serial_open(CSBFGPSCOM, NULL, NULL);

    csbf_gps_comm->callback = csbf_gps_process_data;

    ph_serial_setspeed(csbf_gps_comm, B19200);
    ph_serial_enable(csbf_gps_comm, true);

    blast_startup("Initialized csbf gps status monitor");

}
