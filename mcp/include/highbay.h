/**
 * @file highbay.h
 *
 * @date Oct 14, 2017
 * @author Ian
 *
 * @brief This file is part of MCP, created for the BLAST-TNG project
 *
 * This software is copyright (C) 2017 University of Pennsylvania
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


#ifndef INCLUDE_HIGHBAY_H_
#define INCLUDE_HIGHBAY_H_
void monitor_flow(int on);
void highbay(int);
void mapper_command(int mux1, int mux2, int polarity, float voltage);
#endif /* HIGHBAY_H_ */

