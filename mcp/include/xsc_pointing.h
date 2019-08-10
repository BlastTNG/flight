/**
 * @file xsc_pointing.h
 *
 * @date Nov 5, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLAST project
 *
 * This software is copyright (C) 2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef INCLUDE_XSC_POINTING_H_
#define INCLUDE_XSC_POINTING_H_

#include <stdint.h>
#include "pointing_struct.h"

void xsc_control_triggers(void);
void xsc_control_heaters(void);
int32_t xsc_get_loop_counter(void);
xsc_last_trigger_state_t *xsc_get_trigger_data(int);
void xsc_trigger(int m_which, int m_value);

#endif /* INCLUDE_XSC_POINTING_H_ */
