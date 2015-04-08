#include "xsc_network.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>

#include <conversions.h>
#include "blast.h"
#include "mcp.h"
#include "angles.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define XSC_PROTOCOL_VERSION 3

XSCServerData xsc_mserver_data[xsc_num_buffers][2];
XSCClientData xsc_mclient_data[xsc_num_buffers][2];

int xsc_server_read_index[2] = {0, 0};
int xsc_server_write_index[2] = {0, 0};
int xsc_client_read_index[2] = {0, 0};
int xsc_client_write_index[2] = {0, 0};

static char addresses[2][16] = {"192.168.1.6", "192.168.1.7"};
static int port = 2017;

extern short int InCharge;
extern short int InChargeSet;

void xsc_clear_all_client_data(int which)
{
    for (unsigned int i=0; i<xsc_num_buffers; i++) {
        xsc_clear_client_data(&xsc_mclient_data[i][which]);
    }
}

void xsc_clear_all_server_data(int which)
{
    for (unsigned int i=0; i<xsc_num_buffers; i++) {
        xsc_clear_server_data(&xsc_mserver_data[i][which]);
    }
}

void xsc_networking_init(int which)
{
    xsc_clear_all_client_data(which);
    xsc_clear_all_server_data(which);
    for (unsigned int i=0; i<xsc_num_buffers; i++) {    
        xsc_init_server_data(&xsc_mserver_data[i][which], &xsc_channel_infos);    
    }
}

void xsc_set_client_data(int which)
{
    xsc_client_data[which].horizontal.valid = false;
}

bool xsc_connect(int* socketd, int which)
{
    struct hostent *host;
    struct sockaddr_in server_addr;
    int result;
    int flags;
    //bprintf(loglevel_info, "Trying to connect to %s\n", addresses[which]);

    if (which == 1) {
        usleep(3000000);
    }

    host = gethostbyname(addresses[which]);

    *socketd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (*socketd == -1) {
        return false;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero), 8);

    result = connect(*socketd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
    if (result == -1) {
    	close(*socketd);
    	*socketd = -1;
        return false;
    }
    flags = fcntl(*socketd, F_GETFL, 0);
    fcntl(*socketd, F_SETFL, flags | O_NONBLOCK);

    return true;
}

static void xsc_fill_client_data_pointing_locations(int which)
{
    int pointing_read_index = GETREADINDEX(point_index);
    xsc_client_data[which].solver.filters.hor_location_limit_az = from_degrees(PointingData[pointing_read_index].estimated_xsc_az_deg[which]);
    xsc_client_data[which].solver.filters.hor_location_limit_el = from_degrees(PointingData[pointing_read_index].estimated_xsc_el_deg[which]);
    xsc_client_data[which].solver.filters.eq_location_limit_ra = from_hours(PointingData[pointing_read_index].estimated_xsc_ra_hours[which]);
    xsc_client_data[which].solver.filters.eq_location_limit_dec = from_degrees(PointingData[pointing_read_index].estimated_xsc_dec_deg[which]);
}

static void xsc_fill_client_data(int which)
{
    int pointing_read_index = GETREADINDEX(point_index);

    xsc_client_data[which] = CommandData.XSC[which].net;

    xsc_client_data[which].horizontal.valid = true;
    xsc_client_data[which].horizontal.lat = (double) ((PointingData[pointing_read_index].lat) * M_PI/180.0);
    xsc_client_data[which].horizontal.lst = ((double) PointingData[pointing_read_index].lst) * 2.0*M_PI/(24.0*3600.0);
    xsc_fill_client_data_pointing_locations(which);
    xsc_client_data[which].in_charge = (InChargeSet && (InCharge != 0));

    int age_of_end_of_trigger_cs = xsc_pointing_state[which].last_trigger.age_of_end_of_trigger_cs;
    int post_trigger_counter_fcp_share_delay_cs = CommandData.XSC[which].trigger.post_trigger_counter_fcp_share_delay_cs;
    if (age_of_end_of_trigger_cs > post_trigger_counter_fcp_share_delay_cs) {
        xsc_client_data[which].counter_fcp = xsc_pointing_state[which].counter_fcp;
    } else {
        xsc_client_data[which].counter_fcp = xsc_pointing_state[which].last_counter_fcp;
    }
    // note: everything that must be valid, and doesn't come from CommandData.XSC.net, must be set here or it will be overwritten by CommandData.XSC.net

    xsc_client_data[which].xsc_protocol_version = XSC_PROTOCOL_VERSION;
}

void xsc_network_loop(int which)
{

    int socketd, bytes_received, bytes_sent, select_result = 0; 
    fd_set fdr, fdw;
    bool connected = false;
    int next_write_index, next_read_index = 0;
    static int write_timer = 0;

    xsc_server_read_index[which] = 0;
    xsc_server_write_index[which] = (xsc_server_read_index[which]+1)%5;
    xsc_client_read_index[which] = 0;
    xsc_client_write_index[which] = (xsc_client_read_index[which]+1)%5;

    while (true) {

        if (!connected) {
            xsc_clear_all_server_data(which);
            connected = xsc_connect(&socketd, which);
            if (!connected) {
                usleep(8000000);
            }
        }

        if (connected) {
            bprintf(info, "Connected to %s", addresses[which]);
            write_timer = 0;
            while(1)
            {
                write_timer++;

                FD_ZERO(&fdr);
                FD_ZERO(&fdw);
                FD_SET(socketd, &fdr);
                FD_SET(socketd, &fdw);
                select_result = select(socketd + 1, &fdr, &fdw, NULL, NULL);
                if (select_result == -1) {
                    continue;
                }
                if (FD_ISSET(socketd, &fdr)) {
                    bytes_received = recv(socketd, &xsc_mserver_data[xsc_server_write_index[which]][which],
                        sizeof(xsc_mserver_data[xsc_server_write_index[which]][which]),0);
                    if (bytes_received == sizeof(xsc_mserver_data[xsc_server_write_index[which]][which])) {
                        if(xsc_mserver_data[xsc_server_write_index[which]][which].xsc_protocol_version == XSC_PROTOCOL_VERSION) {
                            next_read_index = xsc_server_write_index[which];
                            xsc_server_write_index[which] = (xsc_server_write_index[which]+1)%5;
                            xsc_server_read_index[which] = next_read_index;
                            //xsc_print_received(which);
                        } else {
                            bprintf(info, "received packet with incorrect xsc_protocol_version: %u\n",
                                xsc_mserver_data[xsc_server_write_index[which]][which].xsc_protocol_version);
                        }
                    }
                    else {
                        bprintf(info, "Connection to %s dropped after failed receive (%i bytes recieved instead of %zu)",
                            addresses[which], bytes_received, sizeof(xsc_mserver_data[xsc_server_write_index[which]][which]));
                    	close (socketd);
                    	socketd = -1;
                        connected = false;
                        break;
                    }
                }
                if (FD_ISSET(socketd, &fdw)) {
                    if (write_timer > 50) {
                        xsc_fill_client_data(which);
                        next_write_index = (xsc_client_write_index[which]+1)%5;
                        next_read_index = xsc_client_write_index[which];
                        memcpy(&xsc_mclient_data[next_write_index][which],
                               &xsc_mclient_data[xsc_client_write_index[which]][which],
                               sizeof(xsc_mclient_data[next_write_index][which]));
                        xsc_client_write_index[which] = next_write_index;
                        xsc_client_read_index[which] = next_read_index;
                        bytes_sent = send(socketd, &xsc_mclient_data[xsc_client_read_index[which]][which],
                            sizeof(xsc_mclient_data[xsc_client_read_index[which]][which]), 0); 
                        if (bytes_sent == sizeof(xsc_mclient_data[xsc_client_read_index[which]][which])) {
                            write_timer = 0;
                            xsc_set_client_data(which);
                        }
                        else {
                            bprintf(info, "Connection to %s dropped after failed send (%i bytes sent instead of %zu)",
                                addresses[which], bytes_sent, sizeof(xsc_mserver_data[xsc_server_write_index[which]][which]));
                            connected = false;
                            close (socketd);
                            socketd = -1;
                            break;
                        }

                    }
                }

                usleep(10000);

            }   

        }
        usleep(1000000);
    }
}


void XSCNetworkLoop(int which)
{
    xsc_network_loop(which);
}



