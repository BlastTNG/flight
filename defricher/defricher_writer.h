/* 
 * defricher_writer.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 6, 2015 by Seth Hillbrand
 */

#ifndef DEFRICHER_WRITER_H_
#define DEFRICHER_WRITER_H_

#include <stdint.h>
#include <glib.h>

extern channel_t *channels;
extern channel_t *new_channels;
extern derived_tng_t *derived_channels;

pthread_t defricher_writer_init(void);
void defricher_queue_packet(uint16_t m_rate);
void defricher_request_new_dirfile(void);
void defricher_request_updated_derived(void);
bool ready_to_read(uint16_t m_rate);

#endif /* DEFRICHER_WRITER_H_ */
