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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include <blast.h>
#include <comms_serial.h>
#include <mcp.h>
#include <tx.h>
#include <pointing_struct.h>
#include <gps.h>

#define CSBFGPSCOM "/dev/ttyCSBFGPS"

void nameThread(const char*);
ph_serial_t *csbf_gps_comm = NULL;
struct DGPSAttStruct CSBFGPSAz = {.az = 0.0, .att_ok = 0};

struct GPSInfoStruct CSBFGPSData = {.longitude = 0.0};

// TODO(laura): We don't actually do anything with the time read out from the CSBF GPS,
// should we write it to the frame?
time_t csbf_gps_time;

int csbf_setserial(const char *input_tty, int verbosity)
{
  int fd;
  struct termios term;

  if (verbosity > 0) blast_info("Connecting to sip port %s...", input_tty);

  if ((fd = open(input_tty, O_RDWR)) < 0) {
    if (verbosity > 0) blast_err("Unable to open serial port");
    return -1;
  }
  if (tcgetattr(fd, &term)) {
    if (verbosity > 0) blast_err("Unable to get serial device attributes");
    return -1;
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200)) {          /*  <======= SET THE SPEED HERE */
    if (verbosity > 0) blast_err("Error setting serial output speed");
    if (fd >= 0) close(fd);
    return -1;
  }

  if (cfsetispeed(&term, B19200)) {         /*  <======= SET THE SPEED HERE */
    if (verbosity > 0) blast_err("Error setting serial input speed");
    if (fd >= 0) close(fd);
    return -1;
  }

  if (tcsetattr(fd, TCSANOW, &term)) {
    if (verbosity > 0) blast_err("Unable to set serial attributes");
    if (fd >= 0) close(fd);
    return -1;
  }
  return fd;
}

static bool csbf_gps_verify_checksum(const char *m_buf, size_t m_linelen)
{
    uint8_t checksum = 0;
    uint8_t recv_checksum = (uint8_t) strtol(&(m_buf[m_linelen - 2]), (char **)NULL, 16);

    // Start from index 1 because the first character ($) should not be included in the checksum.s
    for (size_t i = 1; i < m_linelen && m_buf[i] != '*'; i++) {
        checksum ^= m_buf[i];
    }
    if (recv_checksum != checksum) {
        blast_info("Received invalid checksum from CSBF GPS Data! Expecting = %2x, we received %2x",
                   checksum, recv_checksum);
        return false;
    }
    return true;
}

static void process_gngga(const char *m_data) {
    char lat_ns;
    char lon_ew;
    int lat, lon;
    double lat_mm;
    double lon_mm;
    float age_gps;
    static int first_time = 1;
    static int have_warned = 0;
//    blast_info("Starting process_gngga");
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
            &(CSBFGPSData.altitude), &age_gps) >= 9) {
            CSBFGPSData.latitude = (double)lat + lat_mm*GPS_MINS_TO_DEG;
            CSBFGPSData.longitude = (double)lon + lon_mm*GPS_MINS_TO_DEG;
            if (lat_ns == 'S') CSBFGPSData.latitude *= -1.0;
            if (lon_ew == 'W') CSBFGPSData.longitude *= -1.0;
        CSBFGPSData.isnew = 1;
        have_warned = 0;
         if (first_time) {
             blast_info("Recieved first GNGGA packet:");
             blast_info("Read GNGGA: lat = %lf, lon = %lf, qual = %d, num_sat = %d, alt = %lf, age =%f",
                   CSBFGPSData.latitude, CSBFGPSData.longitude, CSBFGPSData.quality,
                   CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
                   first_time = 0;
        }
    } else {
        if (!have_warned) {
            blast_info("Read error: %s", m_data);
            blast_info("Read GNGGA: lat = %d%lf%c, lon = %d%lf%c, qual = %d, num_sat = %d, alt = %lf, age =%f",
                      lat, lat_mm, lat_ns, lon, lon_mm, lon_ew, CSBFGPSData.quality,
                      CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
            have_warned = 1;
        }
    }
}

static void process_gpgga(const char *m_data) {
    char lat_ns;
    char lon_ew;
    int lat, lon;
    double lat_mm;
    double lon_mm;
    float age_gps;
    static int first_time = 1;
    static int have_warned = 0;
//    blast_info("Starting process_gpgga");
    if (sscanf(m_data,
            "$GPGGA,"
                    "%*f,"      // UTC hhmmss.ss
                    "%2d%lf,%c,"  // Latitude ddmm.mmmmm N/S
                    "%3d%lf,%c,"  // Longitude dddmm.mmmmm E/W
                    "%d,"      // GPS quality: 0 -> no fix, 1 -> gps fix, 2 differential fix
                    "%d,"      // Number of satellites
                    "%*f,"      // Horizontal Dilution of precision
                    "%lf,M,"       // Altitude
                    "%*f,M,"      // Geoidal separation
                    "%f,",       // Age in seconds of GPS data
            &lat, &lat_mm, &lat_ns,
            &lon, &lon_mm, &lon_ew,
            &(CSBFGPSData.quality), &(CSBFGPSData.num_sat),
            &(CSBFGPSData.altitude), &age_gps) >= 9) {
            CSBFGPSData.latitude = (double)lat + lat_mm*GPS_MINS_TO_DEG;
            CSBFGPSData.longitude = (double)lon + lon_mm*GPS_MINS_TO_DEG;
            if (lat_ns == 'S') CSBFGPSData.latitude *= -1.0;
            if (lon_ew == 'W') CSBFGPSData.longitude *= -1.0;
        CSBFGPSData.isnew = 1;
        have_warned = 0;
         if (first_time) {
             blast_info("Recieved first GPGGA packet:");
             blast_info("Read GPGGA: lat = %lf, lon = %lf, qual = %d, num_sat = %d, alt = %lf, age =%f",
                   CSBFGPSData.latitude, CSBFGPSData.longitude, CSBFGPSData.quality,
                   CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
                   first_time = 0;
        }
    } else {
        if (!have_warned) {
            blast_info("Read error: %s", m_data);
            blast_info("Read GPGGA: lat = %d%lf%c, lon = %d%lf%c, qual = %d, num_sat = %d, alt = %lf, age =%f",
                      lat, lat_mm, lat_ns, lon, lon_mm, lon_ew, CSBFGPSData.quality,
                      CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
            have_warned = 1;
        }
    }
}

static void process_gnhdt(const char *m_data)
{
    // Sometimes we don't get heading information.  mcp needs to be able to handle both cases.
    static int first_time = 1;
    static int have_warned = 0;
    double az_read = 0.0;
    size_t bytes_read = strnlen(m_data, 20);
    if (bytes_read < 14) {
        if (!have_warned) {
            blast_info("Not enough characters for GNHDT.  We didn't get heading info.");
            have_warned = 1;
            CSBFGPSAz.att_ok = 0;
        }
    } else {
        sscanf(m_data, "$GNHDT,"
            "%lf," // Heading (deg) x.x
            "%*c,", // True
            &az_read);
        if ((az_read <= 0.0) || (az_read >= 360.0)) {
             // this almost certainly means we didn't read anything
            CSBFGPSAz.att_ok = 0;
        } else {
            CSBFGPSAz.att_ok = 1;
            CSBFGPSAz.az = az_read;
        }
        if (first_time) {
            blast_info("Read GNHDT heading = %lf", az_read);
            first_time = 0;
        }
    }
}


static void process_gnzda(const char *m_data)
{
//    blast_info("Starting process_gnzda...");
    static int first_time = 1;
    static int have_warned = 0;
    struct tm ts;
    // blast_info("Starting process_gnzda");
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
    if (first_time) {
        blast_info("Read GNDZA: hr = %2d, min = %2d, sec = %2d, mday = %d, mon = %d, year =%d",
                   ts.tm_hour, ts.tm_min, ts.tm_sec,
                   ts.tm_mday, ts.tm_mon, ts.tm_year);
        first_time = 0;
    }

    csbf_gps_time = mktime(&ts);
}

void * DGPSMonitor(void * arg)
{
    char tname[6];
    int get_serial_fd = 1;
    int tty_fd;
    int timer = 0;
    unsigned char buf;
    char indata[128];
    uint16_t i_char = 0;
    static int has_warned = 0;
    static int first_time = 1;
    e_dgps_read_status readstage = DGPS_WAIT_FOR_START;
    typedef struct
    {
        void (*proc)(const char*);
        char str[16];
    } nmea_handler_t;

    nmea_handler_t handlers[] = { { process_gpgga, "$GPGGA," }, // fix info
                                  { process_gnhdt, "$GNHDT," }, // date and time
                                  { process_gnzda, "$GNZDA," }, // heading
                                  { NULL, "" } };
    snprintf(tname, sizeof(tname), "DGPS");
    nameThread(tname);
    blast_startup("Starting DGPSMonitor thread.");
    for (;;) {
        // usleep(10000); /* sleep for 10ms */
        // wait for a valid file descriptor
        while (get_serial_fd) {
            if ((tty_fd = csbf_setserial(CSBFGPSCOM, !has_warned)) >= 0) {
                break;
            }
            has_warned = 1;
            sleep(5);
        }
        has_warned = 0;
        get_serial_fd = 0;
        /* Loop until data come in */
        while (read(tty_fd, &buf, 1) <= 0) {
            usleep(10000); /* sleep for 1ms */
            timer++;
        }
        if (get_serial_fd) break;
        if (buf == '$') {
            readstage = DGPS_READING_PKT;
            i_char = 0;
        }
        if (readstage == DGPS_WAIT_FOR_START) continue; // still haven't found start byte
        if (i_char >= 128) {
            blast_err("Read from DGPS a packet longer than the buffer. %s", indata);
            readstage = DGPS_WAIT_FOR_START;
            continue;
        }
        indata[i_char] = buf;
        if (buf == '\r') {
            indata[i_char] = '\0'; // Terminate with '\0' instead of '\r'
            if (first_time) {
                blast_info("Finished reading packet %s", indata);
            }
            if (!csbf_gps_verify_checksum(indata, i_char)) {
                blast_err("checksum failed");
            } else {
                for (nmea_handler_t *handler = handlers; handler->proc; handler++) {
                    if (!strncmp(indata, handler->str, (int) strlen(handler->str)-1)) handler->proc(indata);
                }
            }
            if (first_time) {
                blast_info("Finished calling the handlers");
                first_time = 0;
            }
        }
        i_char++;
    }
}
