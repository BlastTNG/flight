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
#define QUENDR_SENDING_SPEC   150
#define QUENDR_SERVICE_READY  220
#define QUENDR_GOODBYE        221
#define QUENDR_PORT_OPENED    222
#define QUENDR_ACCESS_GRANTED 230
#define QUENDR_OK             250
#define QUENDR_DATA_STAGED    251
#define QUENDR_TRANS_COMPLETE 252
#define QUENDR_OPEN_ERROR     420
#define QUENDR_NO_CUR_DATA    450
#define QUENDR_SYNTAX_ERROR   500
#define QUENDR_PARAM_ERROR    501
#define QUENDR_CMD_NOT_IMPL   502
#define QUENDR_PORT_NOT_OPEN  520
#define QUENDR_NOT_IDENTIFIED 530
#define QUENDR_NO_ACCESS      531
#define QUENDR_NO_DATA_STAGED 550

/* Commands */
#define QUENDC_SPEC          1000
#define QUENDC_SYNC          1001
#define QUENDC_IDEN          1002
#define QUENDC_OPEN          1003
#define QUENDC_ASYN          1004
#define QUENDC_NOOP          1005
#define QUENDC_QUIT          1006
#define QUENDC_QNOW          1007

/* Server Constants */
#define QUENDI_COMMAND_LENGTH  1024
#define QUENDI_RESPONSE_LENGTH 1024

struct quendi_data {
  const char* server_version;
  const char* server_name;
  const char* server_host;
  int access_level;
  char* directory;
  int csock;
};

int quendi_access_ok(
    int
    );

int quendi_cmdnum(
    char*
    );

int quendi_dp_connect(
    void
    );

int quendi_get_cmd(
    char*
    );

int quendi_get_next_param(
    char*,
    int*,
    char**
    );

char* quendi_make_response(
    char*,
    int,
    const char*
    );

int quendi_parse(
    char*,
    int*,
    char**
    );

int quendi_respond(
    int,
    const char*
    );

void quendi_send_spec(
    int,
    const char*,
    unsigned long
    );

void quendi_server_init(
    struct quendi_data*
    );

void quendi_server_shutdown(
    void
    );

int quendi_stage_data(
    const char*,
    unsigned long,
    int
    );

#endif
