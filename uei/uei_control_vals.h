/* 
 * uei_control_vals.h: 
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
 * Created on: Aug 13, 2014 by seth
 */

#ifndef UEI_CONTROL_VALS_H_
#define UEI_CONTROL_VALS_H_

#include <channels_tng.h>

typedef struct {
    char name[32];
    e_TYPE type;
    void *data;
} varentry_t;

int uei_vals_initialize(void);

#endif /* UEI_CONTROL_VALS_H_ */
