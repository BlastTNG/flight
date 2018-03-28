/* daemon: groundstation BLAST command daemon
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
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

#define LINK_DEFAULT      0x01    /* Default link is TDRSS */
#define LINK_DEFAULT_CHAR 'T'     /* Default link is TDRSS */

#define ROUTING_DEFAULT      0x09 /* Default routing is COM1 */
#define ROUTING_DEFAULT_CHAR '1'  /* Default routing is COM1 */

int  bc_setserial(void);
void Daemonise(int, int, int, char *[2]);
void SendMcommand(int, int, int, int, char *[], int, unsigned int *);
void SendScommand(int, int, int, int, unsigned int*);
void WriteLogFile(int, const char *[], int);

extern int link_disabled[256][2];

#define ERR_MESSAGE_LEN 1024
extern char err_message[ERR_MESSAGE_LEN];
#endif
