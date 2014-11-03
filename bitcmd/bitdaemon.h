/* daemon: groundstation BIT command daemon
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is part of bitcmd.
 * 
 * bitcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * bitcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef DAEMON_H
#define DAEMON_H

void Daemonise(int, int);
void WriteLogFile(int, char *[], unsigned int);

#define ERR_MESSAGE_LEN 1024
#define TARGET_PORT 21212

extern char err_message[ERR_MESSAGE_LEN];
extern char host[1024];
extern char target[1024];

extern unsigned short N_SCOMMANDS;
extern unsigned short N_MCOMMANDS;
extern struct scom *scommands;
extern struct mcom *mcommands;
extern int N_GROUPS;
extern char **GroupNames;
extern char command_list_serial[1024];
#endif
