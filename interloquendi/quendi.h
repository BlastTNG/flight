/* quendi.h: an implementation of the Quenya Protocol
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

/* Protocol definitions */
#include "quenya.h"

/* Server Constants */
#define QUENDI_COMMAND_LENGTH  1024
#define QUENDI_RESPONSE_LENGTH 1024

struct quendi_server_data_t {
  const char* server_version;
  const char* server_name;
  const char* server_host;
  int access_level;
  char* directory;
  int csock;
};

struct quendi_data_port_t {
  int sock;
  int staged;
  int persist;
  int port_active;
  unsigned frame_size;
  unsigned long pos;
  char name[PATH_MAX];
};

int quendi_access_ok(
    int
    );

void quendi_add_data_port(
    const struct quendi_data_port_t*
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

void quendi_send_data(
    int,
    const char*,
    unsigned long,
    unsigned,
    int,
    int
    );

void quendi_send_spec(
    int,
    const char*,
    unsigned long
    );

void quendi_server_init(
    const struct quendi_server_data_t*
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
