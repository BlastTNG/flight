/* quenya.h: Quenya Protocol definitions
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this file; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef QUENYA_H
#define QUENYA_H

/* Responses */
#define QUENYA_RESPONSE_LISTENING      123
#define QUENYA_RESPONSE_SENDING_SPEC   152
#define QUENYA_RESPONSE_SENDING_DATA   153
#define QUENYA_RESPONSE_OK             200
#define QUENYA_RESPONSE_DATA_STAGED    211
#define QUENYA_RESPONSE_FRAME_SIZE     212
#define QUENYA_RESPONSE_STAGED_NEXT    213
#define QUENYA_RESPONSE_SERVICE_READY  220
#define QUENYA_RESPONSE_GOODBYE        221
#define QUENYA_RESPONSE_PORT_OPENED    223
#define QUENYA_RESPONSE_ACCESS_GRANTED 230
#define QUENYA_RESPONSE_TRANS_COMPLETE 250
#define QUENYA_RESPONSE_BLOCK_CRC      316
#define QUENYA_RESPONSE_OPEN_ERROR     423
#define QUENYA_RESPONSE_NO_CUR_DATA    451
#define QUENYA_RESPONSE_SYNTAX_ERROR   500
#define QUENYA_RESPONSE_PARAM_ERROR    501
#define QUENYA_RESPONSE_CMD_NOT_IMPL   502
#define QUENYA_RESPONSE_PORT_INACTIVE  503
#define QUENYA_RESPONSE_PORT_NOT_OPEN  523
#define QUENYA_RESPONSE_PORT_ACTIVE    524
#define QUENYA_RESPONSE_NOT_IDENTIFIED 530
#define QUENYA_RESPONSE_NO_ACCESS      531
#define QUENYA_RESPONSE_NO_DATA_STAGED 551
#define QUENYA_RESPONSE_TIMEOUT        621
#define QUENYA_RESPONSE_PORT_CLOSE_ERR 623

/* Commands */
#define QUENYA_COMMAND_DATA          1000
#define QUENYA_COMMAND_SPEC          1001
#define QUENYA_COMMAND_SYNC          1002
#define QUENYA_COMMAND_RTBK          1003
#define QUENYA_COMMAND_IDEN          1004
#define QUENYA_COMMAND_OPEN          1005
#define QUENYA_COMMAND_ASYN          1006
#define QUENYA_COMMAND_NOOP          1007
#define QUENYA_COMMAND_ABOR          1008
#define QUENYA_COMMAND_CLOS          1009
#define QUENYA_COMMAND_QUIT          1010
#define QUENYA_COMMAND_CONT          1011
#define QUENYA_COMMAND_QNOW          1012
#define QUENYA_COMMAND_RDVS          1013
#define QUENYA_COMMAND_SIZE          1014

#endif
