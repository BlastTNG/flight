/* mcp: the Spider master control program
 *
 * This software is copyright (C) 2002-2013 D. V. Wiebe and others
 *
 * This file is part of pcm.
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
 */
#include "netcmd.h"

#ifndef MCP_SCHED_H
#define MCP_SCHED_H

/* This structure is now also used by the MCE command passthrough.  Be careful
 * when modifying it.
 */
struct ScheduleEvent {
  int t;
  int is_multi;
  int command;
  int done; /* MCEserv only; not used by scheduler */
  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];
};

struct ScheduleType {
  int n_sched;
  time_t t0;
  struct ScheduleEvent* event;
};

#endif
