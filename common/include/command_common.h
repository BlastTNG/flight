/* mcp: the BLAST master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
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
 *
 */
#ifndef INCLUDE_COMMAND_COMMON_H
#define INCLUDE_COMMAND_COMMON_H

#include "command_list.h"

int SIndex(enum singleCommand command);
const char* SName(enum singleCommand command);
int MIndex(enum multiCommand command);
const char* MName(enum multiCommand command);
const char* CommandName(int is_multi, int command);

#endif
