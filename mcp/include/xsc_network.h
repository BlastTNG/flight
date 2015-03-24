#ifndef XSC_NETWORK_H
#define XSC_NETWORK_H

#include <xsc_protocol.h>
#include <stdbool.h>

#define xsc_num_buffers 5
#define xsc_server_data xsc_mserver_data[xsc_server_read_index[which]]
#define xsc_client_data xsc_mclient_data[xsc_client_write_index[which]]

extern XSCServerData xsc_mserver_data[xsc_num_buffers][2];
extern XSCClientData xsc_mclient_data[xsc_num_buffers][2];
extern XSCChannelInfos xsc_channel_infos;
extern int xsc_server_read_index[2];
extern int xsc_server_write_index[2];
extern int xsc_client_read_index[2];
extern int xsc_client_write_index[2];

void clear_server_data(int which);
void clear_client_data(int which);
void xsc_networking_init(int which);
bool xsc_connect(int* socketd, int which);
void xsc_network_loop(int which);

#endif
