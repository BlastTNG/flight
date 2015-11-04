/**
 * @file data_sharing.h
 *
 * @date Dec 25, 2012
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 * Revisions copyright (C) 2015 Seth Hillbrand
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

#ifndef DATA_SHARING_H_
#define DATA_SHARING_H_

#include <stdint.h>
typedef struct
{
    double      t_cpu0;
    double      t_cpu1;
    double      v_12;
    double      v_5;
    double      v_bat;
    double      i_flc;
    time_t      time;
    uint16_t    df;
    time_t      timeout;
    uint16_t    last_command;
    uint16_t    command_count;
} __attribute__((packed)) data_sharing_t;

void initialize_data_sharing(void);
void data_sharing_send_data(const data_sharing_t*);
void data_sharing_get_data(data_sharing_t*);
void data_sharing_request_commanddata(void);
void data_sharing_send_commanddata(void);

#endif /* DATA_SHARING_H_ */
