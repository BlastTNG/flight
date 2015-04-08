/**
 * @file blast_comms_internal.h
 *
 * @date 2011-02-08
 * @author Seth Hillbrand
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
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

#ifndef BLAST_COMMS_INTERNAL_H_
#define BLAST_COMMS_INTERNAL_H_
#include <pthread.h>

#include <comms_serial.h>
#include <lookup.h>

typedef struct blast_comms_list
{
	struct blast_comms_list 	*next;
	comms_serial_t			*tty;
} blast_comms_list_t;

static comms_net_async_ctx_t *comms_ctx = NULL;
static pthread_t comms_thread = (pthread_t)-1;
static blast_comms_list_t *comms_ports = NULL;
static pthread_t network_monitor_thread = (pthread_t) -1;


static void *blast_comms_monitor(void *m_arg);
static void blast_comms_cleanup(void *m_arg);

static void blast_comms_net_new_connection(int m_status, int m_errorcode, void *m_userdata);
static int blast_comms_net_process_data(const void *m_data, size_t m_len, void *m_userdata );
static int blast_comms_net_cleanup(const void *m_data, size_t m_len, void *m_userdata );
static void blast_comms_net_error(int m_code, void *m_userdata);
static void *blast_comms_network_monitor(void *m_arg __attribute__((unused)));


//#define _ebex_uplink_types _ebex_port_defs
//#define _BLAST_UPLINK_FN_DEF(_prefix,_ref) void *_prefix ## _ ## _ref ## _callback(size_t, void *);
//BLAST_GENERIC_LOOKUP_TABLE(ebex_uplink_type, static,
//			void *(*process_fn) (size_t, void*);,
//			_BLAST_FUNCTION_STRUCT_LIST,
//			_ebex_uplink_types(ebex_uplink_type, _BLAST_UPLINK_FN_DEF )
//			 );
#endif /* BLAST_COMMS_INTERNAL_H_ */
