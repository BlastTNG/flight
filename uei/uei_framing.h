/* 
 * uei_framing.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
 * Created on: Aug 12, 2014 by seth
 */

#ifndef UEI_FRAMING_H_
#define UEI_FRAMING_H_

#include <channels_tng.h>

typedef struct
{
    channel_t   *channel;
    char        name[FIELD_LEN];
    int         channel_num;
    union {
        int     gain;
    };
} uei_channel_map_t;

int uei_framing_init(void);
void uei_framing_deinit(void);
void uei_framing_routine(void *m_arg);
void uei_store_analog32_data(uei_channel_map_t *m_map, uint32_t *m_data);
void uei_store_analog16_data(uei_channel_map_t *m_map, uint16_t *m_data);

#endif /* UEI_FRAMING_H_ */
