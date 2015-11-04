/* 
 * computer_sensors.h: 
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

#ifndef INCLUDE_COMPUTER_SENSORS_H_
#define INCLUDE_COMPUTER_SENSORS_H_

typedef struct {
    double core0_temp;
    double core1_temp;
    double volt_12V;
    double volt_5V;
    double volt_battery;
    double curr_input;
    double disk_free;
} computer_sensors_t;

extern computer_sensors_t computer_sensors;

void initialize_CPU_sensors(void);
void blast_store_cpu_health(void);
void blast_store_disk_space(void);

#endif /* INCLUDE_COMPUTER_SENSORS_H_ */
