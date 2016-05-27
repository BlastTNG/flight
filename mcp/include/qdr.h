/*
 * qdr.h
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

#ifndef INCLUDE_QDR_H_
#define INCLUDE_QDR_H_

#include <stdbool.h>
#include <stdint.h>

#include "roach.h"

bool qdr_cal2(roach_state_t *m_roach, int m_whichqdr);
void qdr_reset(roach_state_t *m_roach, int m_whichqdr);
uint32_t qdr_delay_clk_get(roach_state_t *m_roach, int m_whichqdr);

#endif /* INCLUDE_QDR_H_ */
