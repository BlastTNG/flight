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

#define CH_TEMP_MB1 0
#define CH_TEMP_MB2 2
#define CH_TEMP_CPU 0

double sensors_temp_mb1 = 0.0;
double sensors_temp_mb2 = 0.0;
double sensors_temp_cpu = 0.0;

static const sensors_chip_name *chip[2] = {NULL};

void blast_store_cpu_health(void)
{
    int err;
    if (!chip[0]) return;

    if((err = sensors_get_value(chip[1], CH_TEMP_CPU, &sensors_temp_cpu)))
    {
        blast_err("Could not get temp CPU %s", sensors_strerror(err));
    }

    if((err = sensors_get_value(chip[0], CH_TEMP_MB1, &sensors_temp_mb1)))
    {
        blast_err("Could not get temp MB1 %s", sensors_strerror(err));
    }

    if((err = sensors_get_value(chip[0], CH_TEMP_MB2, &sensors_temp_mb2)))
    {
        blast_err("Could not get temp MB2 %s", sensors_strerror(err));
    }
    CommandData.temp1 = sensors_temp_cpu * 10.0;
    CommandData.temp2 = sensors_temp_mb1 * 10.0;
    CommandData.temp3 = sensors_temp_mb2 * 10.0;
}

void blast_store_disk_space(void)
{
    struct statvfs vfsbuf;

    if (statvfs("/data", &vfsbuf))
      berror(warning, "Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }
}

void initialize_CPU_sensors(void)
{
    int err;
    int nr = 0;

    if ((err = sensors_init(NULL)))
    {
        blast_err("Could not initialize CPU sensors system");
        return;
    }

    chip[0] = sensors_get_detected_chips(NULL, &nr);
    chip[1] = sensors_get_detected_chips(NULL, &nr);
    if (!chip[0])
    {
        blast_err("Could not get sensors chip!");
        sensors_cleanup();
        return;
    }
}
