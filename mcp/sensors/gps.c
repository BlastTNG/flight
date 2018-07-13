/**
 * @file gps.c
 *
 * @date Mar 29, 2018
 * @author javier
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <endian.h>
#include <errno.h>

#include "mputs.h"
#include "gps.h"

struct GPSInfoStruct GPSData = {0};

void * GPSMonitor(void * arg) {
  FILE * fp = NULL;
  struct GPSInfoStruct * gps_info = (struct GPSInfoStruct *) arg;

  nameThread("GPS");

  blast_info("Started GPS thread");

  while (!fp) {
    fp = fopen("/data/etc/gps.log", "r");
    sleep(1);
  }

  blast_info("Opened GPS log file");

  char * line = NULL;
  size_t len = 0;

  // read to the end of the file
  while (getline(&line, &len, fp) != -1) usleep(1000);

  while (1) {
    if (getline(&line, &len, fp) != -1) {
      int i = 0;
      while (line[i] == ' ') i++;
      if (strncmp(line+i, "<trkpt", 6) == 0) {
        while (gps_info->reading) usleep(100);
        sscanf(line+i, "<trkpt lat=\"%lf\" lon=\"%lf\"%*s", &gps_info->latitude, &gps_info->longitude);
        // printf("Read GPS location lat=%f and lat=%f\n", gps_info->latitude, gps_info->longitude);
        gps_info->isnew = 1;
      }
    }
    usleep(500000);
  }
}
