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
#include <gps.h>

#define CSBFGPSCOM "/dev/ttyCSBFGPS"

ph_serial_t *csbf_gps_comm = NULL;
struct DGPSAttStruct CSBFGPSAz = {.az = 0.0, .att_ok = 0};

struct GPSInfoStruct CSBFGPSData = {.longitude = 0.0};

// TODO(laura): We don't actually do anything with the time read out from the CSBF GPS,
// should we write it to the frame?
time_t csbf_gps_time;

static bool csbf_gps_verify_checksum(const char *m_buf, size_t m_linelen)
{
    uint8_t checksum = 0;
    uint8_t recv_checksum = (uint8_t) strtol(&(m_buf[m_linelen - 3]), (char **)NULL, 16);

    // Start from index 1 because the first character ($) should not be included in the checksum.s
    for (size_t i = 1; i < m_linelen && m_buf[i] != '*'; i++) {
        checksum ^= m_buf[i];
    }
    if (recv_checksum != checksum) {
        blast_info("Received invalid checksum from CSBF GPS Data!");
        return false;
    }
    return true;
}

// static void process_gppat(const char *m_data) {
//     if (sscanf(m_data,
//             "$GPPAT,"
//                     "%*f,"      // UTC hhmmss.ss
//                     "%*f,%*c,"  // Latitude ddmm.mmmmm N/S
//                     "%*f,%*c,"  // Longitude dddmm.mmmmm E/W
//                     "%*f,"      // Altitude
//                     "%lf,"      // Heading
//                     "%lf,"      // Pitch
//                     "%lf,"      // Roll
//                     "%lf,"      // MRMS
//                     "%lf,"      // BRMS
//                     "%d",       // Reset Flag
//             &csbf_gps_att[csbf_dgpsatt_index].az, &csbf_gps_att[csbf_dgpsatt_index].pitch,
//             &csbf_gps_att[csbf_dgpsatt_index].roll, &csbf_gps_att[csbf_dgpsatt_index].mrms,
//             &csbf_gps_att[csbf_dgpsatt_index].brms, &csbf_gps_att[csbf_dgpsatt_index].att_ok) == 6) {
//         csbf_gps_att[csbf_dgpsatt_index].att_ok ^= 1;  // GPS flag 0 = good, but we're using opposite
//         csbf_dgpsatt_index = INC_INDEX(csbf_dgpsatt_index);
//     }
// }

static void process_gngga(const char *m_data) {
    char lat_ns;
    char lon_ew;
    double lat, lat_mm;
    double lon, lon_mm;
    float age_gps;
    if (sscanf(m_data,
            "$GNGGA,"
                    "%*f,"      // UTC hhmmss.ss
                    "%2d%lf,%c,"  // Latitude ddmm.mmmmm N/S
                    "%3d%lf,%c,"  // Longitude dddmm.mmmmm E/W
                    "%d,"      // GPS quality: 0 -> no fix, 1 -> gps fix, 2 differential fix
                    "%d,"      // Number of satellites
                    "%*f,"      // Horizontal Dilution of precision
                    "%lf,M,"       // Altitude
                    "%*f,M,"      // Geoidal separation
                    "%f,"       // Age in seconds of GPS data
                    "%*d,",      // Differential reference station ID
            &lat, &lat_mm, &lat_ns,
            &lon, &lon_mm, &lon_ew,
            &(CSBFGPSData.quality), &(CSBFGPSData.num_sat),
            &(CSBFGPSData.altitude), &age_gps) == 10) {
            CSBFGPSData.latitude = lat + lat_mm*GPS_MINS_TO_DEG;
            CSBFGPSData.longitude = lon + lon_mm*GPS_MINS_TO_DEG;
            if (lat_ns == 'S') CSBFGPSData.latitude *= -1.0;
            if (lon_ew == 'W') CSBFGPSData.longitude *= -1.0;
        blast_info("Read GNGGA: lat = %lf, lon = %lf, qual = %d, num_sat = %d, alt = %lf, age =%f",
                  CSBFGPSData.latitude, CSBFGPSData.longitude, CSBFGPSData.quality,
                  CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
        CSBFGPSData.isnew = 1;
    } else {
        blast_info("Read error: %s", m_data);
        blast_info("Read GNGGA: lat = %d%lf%c, lon = %d%lf%c, qual = %d, num_sat = %d, alt = %lf, age =%f",
                  lat, lat_mm, lat_ns, lon, lon_mm, lon_ew, CSBFGPSData.quality,
                  CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
    }
}

static void process_gnhdt(const char *m_data)
{
    // Sometimes we don't get heading information.  mcp needs to be able to handle both cases.
    size_t bytes_read = strnlen(m_data, 20);
    if (bytes_read < 14) {
        // blast_info("Not enough characters.  We didn't get heading info.");
    } else {
        sscanf(m_data, "$GNZDA,"
            "%lf," // Heading (deg) x.x
            "%*c,", // True
            &CSBFGPSAz.az);
            blast_info("Read heading = %lf", CSBFGPSAz.az);
    }
}


static void process_gnzda(const char *m_data)
{
//    blast_info("Starting process_gnzda...");
    struct tm ts;
    sscanf(m_data, "$GNZDA,"
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
    blast_info("Read GPSZA: hr = %2d, min = %2d, sec = %2d, mday = %d, mon = %d, year =%d",
                  ts.tm_hour, ts.tm_min, ts.tm_sec,
                  ts.tm_mday, ts.tm_mon, ts.tm_year);

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

    nmea_handler_t handlers[] = { { process_gngga, "$GNGGA," }, // fix info
                                  { process_gnhdt, "$GNHDT," }, // date and time
                                  { process_gnzda, "$GNZDA," }, // heading
                                  { NULL, "" } };

    char *bufp;
    ph_buf_t *buf;

    if (!(buf = ph_serial_read_record(serial, "\r", 1))) return;

    bufp = (char*) ph_buf_mem(buf);
    line_len = ph_buf_len(buf);

    /// Replace the terminating '\r' with a NULL for C-string functions.
    bufp[line_len - 1] = 0;
//    blast_info("%s, line_len = %u", bufp, line_len);

    while (*bufp != '$') {
        /// This is a partial string case where we missed the starting '$'
        if (!--line_len) return;
        bufp++;
    }

    if (!csbf_gps_verify_checksum(bufp, line_len)) return;
    for (nmea_handler_t *handler = handlers; handler->proc; handler++) {
//        blast_info("buffer: %s, handler: %s (len: %d)", bufp, handler->str, strlen(handler->str)-1);
        if (!strncmp(bufp, handler->str, strlen(handler->str)-1)) handler->proc(bufp);
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
