/**
 * @file blast_comms.h
 *
 * @date 2011-02-08
 * @author Seth Hillbrand
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011-2015 Seth Hillbrand
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_BLAST_COMMS_H_
#define INCLUDE_BLAST_COMMS_H_
#include <stddef.h>
#include <stdbool.h>

#include <comms_common.h>
#include <comms_netsock.h>
#include <comms_serial.h>

#define BLAST_CMD_SERVER_PORT 41414


bool initialize_blast_comms(void);
bool blast_comms_add_socket(comms_socket_t *m_sock);
bool blast_comms_add_port(comms_serial_t *m_tty);
bool blast_comms_init_cmd_server(void);


#endif /* BLAST_COMMS_H_ */

