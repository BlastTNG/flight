/* 
 * computer_sensors.c:
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 8, 2015 by Seth Hillbrand
 */


#include <stdint.h>
#include <stdbool.h>
#include <sys/statvfs.h>
#include <sensors/sensors.h>
#include <sensors/error.h>

#include <blast.h>
#include <command_struct.h>
#include <tx.h>
#include <channels_tng.h>
#include "computer_sensors.h"

#define CH_TEMP_CPU0 1
#define CH_TEMP_CPU1 2

#define CH_VOLT_12V  0
#define CH_VOLT_5V   1
#define CH_VOLT_BATT 2
#define CH_VOLT_CURRENT 5

#define CHIP_IMANAGER 0
#define CHIP_CPU 1

computer_sensors_t computer_sensors;

static const sensors_chip_name *chip[2] = {NULL};

void blast_store_cpu_health(void)
{
    int sensor_err;

    if (chip[CHIP_CPU]) {
        if ((sensor_err = sensors_get_value(chip[CHIP_CPU], CH_TEMP_CPU0, &computer_sensors.core0_temp))) {
            blast_err("Could not get temp CPU0 %s", sensors_strerror(sensor_err));
        }
        if ((sensor_err = sensors_get_value(chip[CHIP_CPU], CH_TEMP_CPU1, &computer_sensors.core1_temp))) {
            blast_err("Could not get temp CPU1 %s", sensors_strerror(sensor_err));
        }
    }
    if (chip[CHIP_IMANAGER]) {
        if ((sensor_err = sensors_get_value(chip[CHIP_IMANAGER], CH_VOLT_12V, &computer_sensors.volt_12V))) {
            blast_err("Could not get 12V %s", sensors_strerror(sensor_err));
        }
        if ((sensor_err = sensors_get_value(chip[CHIP_IMANAGER], CH_VOLT_5V, &computer_sensors.volt_5V))) {
            blast_err("Could not get 5V %s", sensors_strerror(sensor_err));
        }
        if ((sensor_err = sensors_get_value(chip[CHIP_IMANAGER], CH_VOLT_BATT, &computer_sensors.volt_battery))) {
            blast_err("Could not get battery voltage %s", sensors_strerror(sensor_err));
        }
        if ((sensor_err = sensors_get_value(chip[CHIP_IMANAGER], CH_VOLT_CURRENT, &computer_sensors.curr_input))) {
            blast_err("Could not get current %s", sensors_strerror(sensor_err));
        }
    }
}

void blast_store_disk_space(void)
{
    struct statvfs vfsbuf;

    if (statvfs("/data", &vfsbuf)) {
      berror(warning, "Cannot stat filesystem");
    } else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }
}

void initialize_CPU_sensors(void)
{
    int sensor_err;
    int nr = 0;
    const sensors_chip_name *new_chip;

    if ((sensor_err = sensors_init(NULL))) {
        blast_err("Could not initialize CPU sensors system");
        return;
    }

    new_chip = sensors_get_detected_chips(NULL, &nr);
    while (new_chip && nr < 3) {
        new_chip = sensors_get_detected_chips(NULL, &nr);
        if (new_chip) {
            if (!strncmp(new_chip->prefix, "coretemp", 8)) {
                chip[CHIP_CPU] = new_chip;
            } else if (!strncmp(new_chip->prefix, "imanager", 8)) {
                chip[CHIP_IMANAGER] = new_chip;
            }
        }
    }

    if (!chip[CHIP_CPU] && !chip[CHIP_IMANAGER]) {
        blast_err("Could not get sensors chips!");
        sensors_cleanup();
        return;
    }
}
