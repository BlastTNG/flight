/*
 * roach.h
 *
 * This software is copyright (C) 2013-2014 University of Pennsylvania
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
 *  Created on: Apr 26, 2016
 *      Author: seth
 */

#ifndef INCLUDE_ROACH_H_
#define INCLUDE_ROACH_H_

#include <stddef.h>
#include <stdint.h>

typedef struct roach_state roach_state_t;

const char *roach_get_name(roach_state_t *m_roach);
int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
                            size_t m_len, uint32_t m_offset, int m_timeout);
int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size, int ms_timeout);
int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset);
int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename);


void write_roach_channels_244hz(void);
void shutdown_roaches(void);

#endif /* INCLUDE_ROACH_H_ */
