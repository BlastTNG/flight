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
#include "command_common.h"
#include "command_list.h"

static const char *UnknownCommand = "Unknown Command";

const char* SName(enum singleCommand command)
{
  int i = SIndex(command);
  return (i == -1) ? UnknownCommand : scommands[i].name;
}

const char* MName(enum multiCommand command)
{
  int i = MIndex(command);
  return (i == -1) ? UnknownCommand : mcommands[i].name;
}

const char* CommandName(int is_multi, int command)
{
  return (is_multi) ? MName(command) : SName(command);
}
