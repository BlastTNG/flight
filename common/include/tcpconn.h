/* -----------------------------------------------------------------------
 * -------------------------- TCP Communications -------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is TCP network setup and configuration for communications with
 * ground station links. Contained are the functions used to transfer
 * frame files over the network along with the format files and
 * linklists required to parse the received data. The main programs that
 * use this code are mole and bittlm.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: August 29, 2017
 *
 * Copyright 2017 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */
#ifndef TCPCONN_H_
#define TCPCONN_H_

#define SERVER_LL_REQ 0xEE000001
#define SERVER_LL_LIST_REQ 0xEE000002
#define SERVER_ARCHIVE_REQ 0xEE000003
#define SERVER_SERIAL_REQ 0xEE000004
#define SERVER_SET_LL_NAME_REQ 0xEE000005
#define SERVER_ARCHIVE_LIST_REQ 0xEE000006

#define TCPCONN_LOOP 0x01
#define TCPCONN_FILE_RAW 0x02
#define TCPCONN_NOLOOP 0x04

#ifdef __cplusplus
extern "C" {
#endif

struct TCPCONN
{
  char ip[128];
  int fd;
	int flag;
	uint16_t theday, themonth, theyear;
	uint32_t serial;
};


int connect_tcp(struct TCPCONN * );
int close_connection(struct TCPCONN *);
unsigned int initialize_connection(struct TCPCONN * , uint32_t );
int request_server_list(struct TCPCONN * , char [][64]);
int request_server_archive_list(struct TCPCONN * , char [][64]);
int retrieve_data(struct TCPCONN * , uint64_t , unsigned int , uint8_t *);
void set_server_linklist_name(struct TCPCONN * , char *);
uint32_t request_server_file(struct TCPCONN * , char * , unsigned int );
int send_client_file(struct TCPCONN * , char * , uint32_t );
void send_client_error(struct TCPCONN *);

#ifdef __cplusplus
}
#endif

#endif /* TCPCONN_H_ */

