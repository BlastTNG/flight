/* quendi.h: an implementation of the Quendi Protocol
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

#ifndef QUENDI_H
#define QUENDI_H

/* Responses */
#define QUENDR_LISTENING      123
#define QUENDR_SERVICE_READY  220
#define QUENDR_GOODBYE        221
#define QUENDR_PORT_OPENED    222
#define QUENDR_ACCESS_GRANTED 230
#define QUENDR_OK             250
#define QUENDR_OPEN_ERROR     420
#define QUENDR_SYNTAX_ERROR   500
#define QUENDR_PARAM_ERROR    501
#define QUENDR_CMD_NOT_IMPL   502
#define QUENDR_PORT_NOT_OPEN  520
#define QUENDR_NOT_IDENTIFIED 530
#define QUENDR_NO_ACCESS      531

/* Commands */
#define QUENDC_SYNC          1000
#define QUENDC_HERE          1001
#define QUENDC_FORM          1002
#define QUENDC_OPEN          1003
#define QUENDC_ASYN          1004
#define QUENDC_NOOP          1005
#define QUENDC_QUIT          1006
#define QUENDC_QNOW          1007

/* Server Constants */
#define QUENDI_COMMAND_LENGTH  1024
#define QUENDI_RESPONSE_LENGTH 1024

int   quendi_access_ok(int sock, int level);
int   quendi_cmdnum(char* buffer);
int   quendi_get_dp_connect(int csock);
int   quendi_dp_open(int sock);
int   quendi_get_next_param(char* buffer, int* nparams, char** params);
char* quendi_make_response(char* buffer, int response_num, const char* message);
int   quendi_parse(char* buffer, int* nparams, char** params);
int   quendi_respond(int sock, int response_num, const char* message);
void  quendi_server_init(const char* server_version, const char* server_name,
    const char* server_host);
void  quendi_server_shutdown(void);
int   quendi_server_start(int sock, const char* server_version,
    const char* server_name, const char* server_host);

#endif
