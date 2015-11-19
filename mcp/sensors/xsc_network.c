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
#include "blast_comms.h"
#include "mcp.h"
#include "angles.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define XSC_PROTOCOL_VERSION 3

XSCServerData xsc_mserver_data[2][3];

int xsc_server_index[2] = {0, 0};

static char addresses[2][16] = {"192.168.1.7", "192.168.1.8"};
static int port = 2017;
static comms_socket_t *xsc_sock[2] = {NULL, NULL};
extern short int InCharge;

/**
 * Handles the incoming XSC data packet
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata unused
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int xsc_process_packet(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
    comms_socket_t *sock = (comms_socket_t*) m_userdata;
    int which = (intptr_t) sock->priv_data;
    XSCServerData *data = (XSCServerData*) m_data;
    int array_index = xsc_server_index[which];

    if (data->xsc_protocol_version == XSC_PROTOCOL_VERSION) {
        memcpy(&xsc_mserver_data[which][array_index], m_data, sizeof(XSCServerData));
        xsc_server_index[which] = INC_INDEX(xsc_server_index[which]);
    }
    else {
        blast_info("received packet with incorrect xsc_protocol_version: %u\n",
                data->xsc_protocol_version);
    }

    return sizeof(XSCServerData);
}

/**
 * Handles the network command hangup.
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata socket pointer
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int xsc_net_cleanup(const void *m_data, size_t m_len, void *m_userdata )
{
    comms_socket_t *sock = (comms_socket_t*)m_userdata;
    blast_warn("Received hangup from XSC Connection %s", sock->host?sock->host:"(UNK)");

    return 0;
}

/**
 * Currently unused function to process errors received on the Network socket
 * @param m_code
 * @param m_userdata
 */
static void xsc_net_error(int m_code, void *m_userdata)
{
    comms_socket_t *sock = (comms_socket_t*)m_userdata;
    blast_warn("Receive Error %d from XSC Connection %s", m_code, sock->host?sock->host:"(UNK)");
    return;
}

/**
 * Currently unused function to process errors received on the Network socket
 * @param m_code
 * @param m_userdata
 */
static void xsc_net_connected(int m_code, int m_errcode, void *m_priv)
{
    comms_socket_t *sock = (comms_socket_t*)m_priv;
    if (m_code == NETSOCK_OK) {
        blast_info("XSC Network successfully connected to %s", sock->host?sock->host:"(Unknown SC)");
    } else {
        blast_warn("Received Error %d from XSC connecting to %s", m_errcode, sock->host?sock->host:"(UNK)");
    }
    return;
}

void xsc_networking_init(int which)
{
    for (unsigned int i=0; i<3; i++) {
        xsc_clear_server_data(&xsc_mserver_data[which][i]);
        xsc_init_server_data(&xsc_mserver_data[which][i]);
    }

    xsc_sock[which] = comms_sock_new();

    xsc_sock[which]->priv_data = (void*)((intptr_t)which);

    comms_sock_connect(xsc_sock[which], addresses[which], port);

    xsc_sock[which]->callbacks.priv = xsc_sock[which];
    xsc_sock[which]->callbacks.connected = xsc_net_connected;
    xsc_sock[which]->callbacks.data = xsc_process_packet;
    xsc_sock[which]->callbacks.finished = xsc_net_cleanup;
    xsc_sock[which]->callbacks.error = xsc_net_error;

    blast_comms_add_socket(xsc_sock[which]);

}

void xsc_write_data(int which)
{
    XSCClientData xsc_client_data;
    int pointing_read_index = GETREADINDEX(point_index);

    memcpy(&xsc_client_data, &CommandData.XSC[which].net, sizeof(XSCClientData));

    xsc_client_data.horizontal.valid = true;
    xsc_client_data.horizontal.lat = from_degrees(PointingData[pointing_read_index].lat);
    xsc_client_data.horizontal.lst = from_seconds(PointingData[pointing_read_index].lst);
    xsc_client_data.solver.filters.hor_location_limit_az = from_degrees(PointingData[pointing_read_index].estimated_xsc_az_deg[which]);
    xsc_client_data.solver.filters.hor_location_limit_el = from_degrees(PointingData[pointing_read_index].estimated_xsc_el_deg[which]);
    xsc_client_data.solver.filters.eq_location_limit_ra = from_hours(PointingData[pointing_read_index].estimated_xsc_ra_hours[which]);
    xsc_client_data.solver.filters.eq_location_limit_dec = from_degrees(PointingData[pointing_read_index].estimated_xsc_dec_deg[which]);
    ///TODO: Adjust fcp<->stars in charge flag
    xsc_client_data.in_charge = 1;

    if (xsc_pointing_state[which].last_trigger.age_of_end_of_trigger_cs >
            CommandData.XSC[which].trigger.post_trigger_counter_fcp_share_delay_cs) {
        xsc_client_data.counter_fcp = xsc_pointing_state[which].counter_mcp;
    } else {
        xsc_client_data.counter_fcp = xsc_pointing_state[which].last_counter_fcp;
    }
    // note: everything that must be valid, and doesn't come from CommandData.XSC.net, must be set here or it will be overwritten by CommandData.XSC.net

    xsc_client_data.xsc_protocol_version = XSC_PROTOCOL_VERSION;

    switch (xsc_sock[which]->state) {
        case NETSOCK_STATE_CONNECTED:
            comms_sock_write(xsc_sock[which], &xsc_client_data, sizeof(xsc_client_data));
            break;
        case NETSOCK_STATE_CLOSED:
            comms_sock_reconnect(xsc_sock[which]);
            break;
        case NETSOCK_STATE_EOS:
        case NETSOCK_STATE_ERROR:
            comms_sock_close(xsc_sock[which]);
            comms_sock_reconnect(xsc_sock[which]);
            break;
        default:
            break;
    }

}


