/* quenya.h: Quenya Protocol definitions
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * quendi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * quendi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with quendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef QUENYA_H
#define QUENYA_H

/* Responses */
#define QUENYA_RESPONSE_LISTENING      123
#define QUENYA_RESPONSE_SENDING_DATA   151
#define QUENYA_RESPONSE_SENDING_SPEC   150
#define QUENYA_RESPONSE_SERVICE_READY  220
#define QUENYA_RESPONSE_GOODBYE        221
#define QUENYA_RESPONSE_PORT_OPENED    222
#define QUENYA_RESPONSE_ACCESS_GRANTED 230
#define QUENYA_RESPONSE_OK             250
#define QUENYA_RESPONSE_DATA_STAGED    251
#define QUENYA_RESPONSE_TRANS_COMPLETE 252
#define QUENYA_RESPONSE_OPEN_ERROR     420
#define QUENYA_RESPONSE_NO_CUR_DATA    450
#define QUENYA_RESPONSE_SYNTAX_ERROR   500
#define QUENYA_RESPONSE_PARAM_ERROR    501
#define QUENYA_RESPONSE_CMD_NOT_IMPL   502
#define QUENYA_RESPONSE_PORT_NOT_OPEN  520
#define QUENYA_RESPONSE_TIMEOUT        521
#define QUENYA_RESPONSE_NOT_IDENTIFIED 530
#define QUENYA_RESPONSE_NO_ACCESS      531
#define QUENYA_RESPONSE_NO_DATA_STAGED 550
#define QUENYA_RESPONSE_PORT_CLOSE_ERR 620

/* Commands */
#define QUENYA_COMMAND_DATA          1000
#define QUENYA_COMMAND_SPEC          1001
#define QUENYA_COMMAND_SYNC          1002
#define QUENYA_COMMAND_IDEN          1003
#define QUENYA_COMMAND_OPEN          1004
#define QUENYA_COMMAND_ASYN          1005
#define QUENYA_COMMAND_NOOP          1006
#define QUENYA_COMMAND_CLOS          1007
#define QUENYA_COMMAND_QUIT          1008
#define QUENYA_COMMAND_QNOW          1009

#endif
