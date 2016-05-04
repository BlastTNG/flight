/*
 * uei.h:
 *
 * This software is copyright
 *  (C) 2013-2015 University of Pennsylvania
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
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: Nov 25, 2015 by vagrant
 */

#ifndef INCLUDE_UEI_H_
#define INCLUDE_UEI_H_

void uei_100hz_loop(void);
void uei_1hz_loop(void);

void *uei_hwmon_loop(void *m_arg);

int uei_508_read_bytes(int m_port, char *m_buf, size_t m_bytes);
int uei_508_read_record(int m_port, char *m_buf, size_t m_buflen, const char *m_delim, uint32_t m_delimlen);
int uei_508_write(int m_port, const char *m_buf, uint32_t m_len);
void *uei_508_loop(void *m_arg);

void *uei_dmap_update_loop(void *m_arg);

int initialize_uei_of_channels(void);

#endif /* INCLUDE_UEI_H_ */
