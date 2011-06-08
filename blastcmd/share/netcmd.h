/* narsil: groundstation BLAST command software
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NETCMD_H
#define NETCMD_H
#include "command_list.h"

#define SOCK_PORT 41414

#define CMD_NONE 0
#define CMD_ERRR 1
#define CMD_USER 2
#define CMD_PING 3
#define CMD_LURK 4
#define CMD_LIST 5
#define CMD_CONN 6
#define CMD_BCMD 7
#define CMD_LIMT 8
#define CMD_SENT 9

extern unsigned short client_n_scommands;
extern unsigned short client_n_mcommands;
extern struct scom *client_scommands;
extern struct mcom *client_mcommands;
extern char client_command_list_serial[1024];

int  NetCmdConnect(const char*, int, int);
void NetCmdDrop(void);
void NetCmdSend(const char*);
int  NetCmdReceive(int);
int  NetCmdSendAndReceive(const char*, int);
int  NetCmdGetCmdList(void);
int  NetCmdTakeConn(int);
const char* NetCmdBanner(void);
int  NetCmdPing(void);
#endif
