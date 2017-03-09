/* 
 * framing.h: 
 *
 * This software is copyright (C) 2013-2015 Seth Hillbrand
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
 * Created on: Mar 31, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_FRAMING_H_
#define INCLUDE_FRAMING_H_

#include "command_struct.h"
#include "channel_macros.h"
#include "derived.h"

int framing_init(channel_t *channel_list, derived_tng_t *m_derived);
void framing_shutdown(void);

int32_t get_200hz_framenum(void);
int32_t get_100hz_framenum(void);
int32_t get_5hz_framenum(void);
int32_t get_1hz_framenum(void);

void framing_publish_244hz(void);
void framing_publish_200hz(void);
void framing_publish_100hz(void);
void framing_publish_5hz(void);
void framing_publish_1hz(void);

void framing_publish_command_data(struct CommandDataStruct *m_commanddata);

#endif /* INCLUDE_FRAMING_H_ */
