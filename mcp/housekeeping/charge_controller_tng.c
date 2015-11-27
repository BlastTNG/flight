/*
 * charge_controller_tng.c:
 *
 * This software is copyright
 *  (C) 2013-2015 University of Pennsylvania
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: Nov 25, 2015 by vagrant
 */

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

typedef struct {
  int which;
  bool connected;
  bool have_warned_version;
  uint32_t backoff_sec;
  struct timeval timeout;
  ph_job_t connect_job;
  ph_sock_t *sock;
} cc_state_t;

typedef struct {
	int id;
	uint8_t address;
	char type;
	int size;
	char name[32];
} cc_value_t;

static char *cc_hosts[2] = {"192.168.1.25", "192.168.1.26"};
static const uint16_t port = 80;
static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;
static cc_value_t[] = {
		{1, 38, 'V', 1, "Battery Voltage"},
		{1, 51, 'V', 1,"Target Voltage"},
		{1, 39, 'A', 1,"Charge Current"},
		{1, 27, 'V', 1,"Array Voltage"},
		{1, 29, 'A', 1,"Array Current"},
		{1, 58, 'W', 1,"Output Power"},
		{1, 61, 'V', 1,"Sweep Vmp"},
		{1, 62, 'V', 1,"Sweep Voc"},
		{1, 60, 'W', 1,"Sweep Pmax"},
		{1, 52, 'h', 2,"Amp Hours"},
		{1, 56, 'k', 1,"Kilowatt Hours"},
		{}
};
static char request_str[] = "GET MBCSV.cgi?ID=%d&F=4&AHI=%d&ALO=%d&RHI=%d&RLO=%d"

void ScaledValueDisplayClass(mbid, addr, factor, numWords) {
    struct this this, document, elements;
	this.MBID = mbid;
    this.MBaddress = addr;
    this.ScaleFactor = factor;
    this.updateLVText = function() {

	GetScaledValue(this.MBID, this.MBaddress, this.ScaleFactor, numWords) + " " + this.ScaleFactor;

}

int GetScaledValue(mbid, mbaddr, units, numWords) {
#define var int
	var D = 0;
	struct Factors Factors;
	D = MBJSReadModbusInts(MBP, mbid, mbaddr, numWords);
	if (E > 1) {
		var C = D.split("#");
		D = (parseInt(C[0]) * 65536) + parseInt(C[1]);
	} else {
		D <<= 16;
		D >>= 16;
	}
	if (units == "V") {
		return ((D * Factors.VScale) / 32768 / 10);
	} else {
		if (units == "A") {
			return ((D * Factors.IScale) / 32768 / 10);
		} else {
			if (units == "W") {
				return ((D * Factors.IScale * Factors.VScale) / 131072 / 100);
			} else {
				if (units == "Ah") {
					return (D * 0.1);
				} else {
					if (units == "kWh") {
						return (D);
					} else {
						return (D);
					}
				}
			}
		}
	}
}
int MBJSReadModbusInts(MBP, mbid, mbAddr, numWords) {
    var F = MBJSReadCSV(MBP, mbid, mbAddr, numWords);
    var D = F.split(",");
    var A = D[2];
    var I = 3;
    var G = "";
    var H;
    while (I < parseInt(A) + 2) {
        H = (parseInt(D[I++]) * 256);
        H += parseInt(D[I++]);
        if (I < parseInt(A) + 2) {
            G += H.toString() + "#"
        } else {
            G += H.toString()
        }
    }
    return G
}
function MBJSReadCSV(MBP, mbid, mbAddr, numWords) {
    return ajaxget(mbid, mbAddr, numWords, 4)
}
function ajaxget(M, H, numWords, G) {
    var L = 0;
    var I = new ajaxRequest();
    var E = "";
    var A = encodeURIComponent(M);
    var C = encodeURIComponent(G);
    var N = encodeURIComponent(parseInt(H) >> 8);
    var B = encodeURIComponent(parseInt(H) & 255);
    var J = encodeURIComponent(parseInt(numWords) >> 8);
    var K = encodeURIComponent(parseInt(numWords) & 255);
    I.open("GET", "MBCSV.cgi?ID=" + A + "&F=" + 4 + "&AHI=" + N + "&ALO=" + B + "&RHI=" + J + "&RLO=" + K);
        E = I.responseText
    }
    return E
}
//var BTemp = new TempDisplayClass(MBID,37,"fDBT","Battery");
//var HSTemp = new TempDisplayClass(MBID,35,"fDHST","Heat Sink");
//
//var Factors = new ScaleFactors();
//var ChState = ["Start", "Night Check", "Disconnect", "Night", "Fault", "MPPT", "Absorption", "Float", "Equalize"];
//var AlarmsDisplay = new BitFieldTextDisplayClass(MBID,46,2,1,"fDAlarms","lblAlarm","Alarms","lblvalAlarm",AlarmsArray);
//var FaultsDisplay = new BitFieldTextDisplayClass(MBID,44,1,0,"fDAlarms","lblError","Faults","lblvalError",FaultsArray);
//var ChStDisp = new EnumTextDisplayClass(MBID,50,1,"fDChSt","lblSt","Charge State","lblvalSt",ChState);
//var TStamp = new TStampClass("flastU","valLastU");

static void connected_cc(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    cc_state_t *state = (cc_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", cc_hosts[state->which], port, gai_strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`", cc_hosts[state->which], port, m_errcode, strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;
    }

    blast_info("Connected to charge controller %d at %s", state->which, cc_hosts[state->which]);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    state->backoff_sec = min_backoff_sec;
    m_sock->callback = xsc_process_packet;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);

}

static void connect_cc(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    cc_state_t *state = (cc_state_t*)m_data;

    ph_sock_resolve_and_connect(cc_hosts[state->which], port,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM,
        connected, m_data);
}

void charge_controller_init(int m_which)
{

	cc_state_t *state;

	state = calloc(sizeof(cc_state_t), 1);
	state->which = m_which;
	state->timeout.tv_sec = 1;
	ph_job_init(&state->connect_job);
	state->connect_job.callback = connect_cc;
	state->connect_job.data = state;
}
