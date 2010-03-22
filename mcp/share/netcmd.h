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

extern unsigned short client_n_scommands;
extern unsigned short client_n_mcommands;
extern struct scom *client_scommands;
extern struct mcom *client_mcommands;
extern char client_command_list_serial[1024];

const char* NetCmdBanner(void);
void NetCmdConnect(const char*, int, int);
void NetCmdDrop(void);
int  NetCmdGetAck(int*, int);
int  NetCmdGetCmdList(void);
void NetCmdSend(const char*);
int  NetCmdSubmitCommand(char, char, int, char *[], int);
int  NetCmdTakeConn(void);
void NetCmdUpdateConn(void);
#endif
