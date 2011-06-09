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

/*
 * Definition of command structures (formerly in command_list.h)
 */
#define MAX_N_PARAMS	12	    /* narsil REALLY likes this to be even */
#define CMD_STRING_LEN	32	    /* max length of command string */
#define SIZE_NAME	80	    /* max length for command name, */
#define SIZE_ABOUT	80	    /*	... description, */
#define SIZE_PARNAME	80	    /*	... and paramenter name */
#define CONFIRM         0x80000000  /* group bit if command needs confirm */

#pragma pack(4) //32-bit and 64-bit sytems disagree on packing
struct scom {
  int command;	    //really enum singleCommand where appropriate
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
};
struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};
struct mcom {
  int command;	    //really enum multiCommand where appropriate
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
  char numparams;
  struct par params[MAX_N_PARAMS];
};
#pragma pack()   //return to default packing

/*
 * Client (blastcmd, narsil) interface to NetCmd protocol
 * Server interface located in blastcmd's daemon.c
 *
 */
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

extern int client_n_groups;
extern char **client_group_names;

int  NetCmdConnect(const char*, int, int);
void NetCmdDrop(void);
void NetCmdSend(const char*);
int  NetCmdReceive(int);
int  NetCmdSendAndReceive(const char*, int);
int  NetCmdGetCmdList(void);
int  NetCmdGetGroupNames(void);
int  NetCmdTakeConn(int);
const char* NetCmdBanner(void);
int  NetCmdPing(void);
#endif
