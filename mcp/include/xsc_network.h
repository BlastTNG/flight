#ifndef XSC_NETWORK_H
#define XSC_NETWORK_H

#include <xsc_protocol.h>
#include <stdbool.h>

#define XSC_SERVER_DATA(_which) xsc_mserver_data[_which][GETREADINDEX(xsc_server_index[_which])]

extern XSCServerData xsc_mserver_data[2][3];
extern int xsc_server_index[2];

void xsc_networking_init(int which);
void xsc_write_data(int which);

#endif
