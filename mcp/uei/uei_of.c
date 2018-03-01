/*
 * uei_of.c:
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
 * Created on: Nov 15, 2015 by seth
 */
#include <time.h>
#include <PDNA.h>

#include "phenom/sysutil.h"
#include "phenom/thread.h"
#include "phenom/buffer.h"

#include "channels_tng.h"
#include "command_struct.h"
#include "blast_time.h"
#include "mcp.h"

static double frequency = 200.0;
static int hd_of = 0;       // Base UEI handle
static int hd_dmap = 0;     // DMAP UEI handle
static int hd_508 = 0;      // Serial card UEI handle

static int dmapid = 0;
static int vmapid_508 = 0;

#define CFG_508(OPER, MODE, BAUD, WIDTH, STOP, PARITY, ERROR) \
    (OPER << DQ_SL501_OPER_SH | MODE << DQ_SL501_MODE_SH | BAUD << DQ_SL501_BAUD_SH | \
    WIDTH << DQ_SL501_WIDTH_SH | STOP  << DQ_SL501_STOP_SH | PARITY << DQ_SL501_PARITY_SH | \
    ERROR << DQ_SL501_ERROR_SH)

#define CFG_485(_BAUD) CFG_508(DQ_SL501_OPER_NORM, DQ_SL501_MODE_485H, _BAUD, \
    DQ_SL501_WIDTH_8, DQ_SL501_STOP_1, DQ_SL501_PARITY_NONE, 0)
#define CFG_232(_BAUD) CFG_508(DQ_SL501_OPER_NORM, DQ_SL501_MODE_232, _BAUD, \
    DQ_SL501_WIDTH_8, DQ_SL501_STOP_1, DQ_SL501_PARITY_NONE, 0)

static channel_t *uei_of_channels[6][48] = {{NULL}};
static uint32_t num_of_channels[6] = {0};
static double raw_dmap_input[6][48] = {{0}};

static double diag_data[28] = {0.0};
static uint32_t raw_diag_data[28] = {0};

static ph_bufq_t *SL508_write_buffer[8] = {NULL};
static ph_bufq_t *SL508_read_buffer[8] = {NULL};

#define CPU_LAYER 5

void *uei_hwmon_loop(void *m_arg)
{
    struct timespec next;
    uint64_t periodns = NSEC_PER_SEC;
    int ret;
    uint32_t diag_channels[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

    ph_thread_set_name("UEI_HW");
    blast_startup("Starting UEI HWMon loop");

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!shutdown_mcp) {
        ret = DqAdvDnxpRead(hd_of, CPU_LAYER, sizeof(diag_channels) / sizeof(uint32_t), diag_channels,
                            raw_diag_data, diag_data);
        if (ret < 0) {
            blast_err("Could not read CPU Diagnostics: %s", DqTranslateError(ret));
        }
        timespec_add_ns(&next, periodns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    return NULL;
}

int uei_508_read_bytes(int m_port, char *m_buf, size_t m_bytes)
{
    ph_buf_t *tmp_buf;

    if ((tmp_buf = ph_bufq_consume_bytes(SL508_read_buffer[m_port], m_bytes))) {
        memcpy(m_buf, ph_buf_mem(tmp_buf), m_bytes);
        ph_buf_delref(tmp_buf);
        return m_bytes;
    } else {
        return 0;
    }
}

int uei_508_read_record(int m_port, char *m_buf, size_t m_buflen, const char *m_delim, uint32_t m_delimlen)
{
    ph_buf_t *tmp_buf;

    if ((tmp_buf = ph_bufq_consume_record(SL508_read_buffer[m_port], m_delim, m_delimlen))) {
        size_t len = min((size_t)ph_buf_len(tmp_buf), m_buflen);
        if (len) memcpy(m_buf, ph_buf_mem(tmp_buf), len);
        ph_buf_delref(tmp_buf);
        return len;
    } else {
        return 0;
    }
}

int uei_508_write(int m_port, const char *m_buf, uint32_t m_len)
{
    uint64_t written_bytes = 0;
    if (m_port >= 8 || m_port < 0) {
        blast_err("Invalid port %d", m_port);
        return -1;
    }
    ph_bufq_append(SL508_write_buffer[m_port], m_buf, m_len, &written_bytes);
    return written_bytes;
}

void *uei_508_loop(void *m_arg)
{
    struct timespec next;
    uint64_t periodns = ((double)(NSEC_PER_SEC) / 10);
    int ret;
    int channel_list_508[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint32_t channel_cfg_508[8] = {
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600),
            CFG_485(DQ_SL501_BAUD_9600)
    };
    int channel_flags_508[8] = { 0 };

    ph_library_init();
    ph_thread_set_name("UEI508");
    blast_startup("Starting UEI 508 loop");

    // set channel configuration
    for (int i = 0; i < 8; i++) {
        if ((ret = DqAdv501SetChannelCfg(hd_508, 4, i, channel_cfg_508[i])) < 0) {
           blast_err("Error in DqAdv501SetChannelCfg()");
        }
    }
    ret = DqRtVmapAddChannel(hd_508, vmapid_508, 4, DQ_SS0IN, channel_list_508, channel_flags_508, 8);
    ret = DqRtVmapAddChannel(hd_508, vmapid_508, 4, DQ_SS0OUT, channel_list_508, channel_flags_508, 8);
    ret = DqRtVmapStart(hd_508, vmapid_508);

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!shutdown_mcp) {
        for (int i = 0; i < 8; i++) {
            size_t num_bytes;
            ph_buf_t *outbuf = NULL;
            if ((num_bytes = ph_bufq_len(SL508_write_buffer[i]))) {
                outbuf = ph_bufq_consume_bytes(SL508_write_buffer[i], num_bytes);
                DqRtVmapWriteOutput(hd_508, vmapid_508, 4, i, num_bytes, ph_buf_mem(outbuf));
                ph_buf_delref(outbuf);
            }
            DqRtVmapRequestInput(hd_508, vmapid_508, 4, i, 128);
        }
        // Write output data to each TX port FIFO and Read each RX port FIFO
        ret = DqRtVmapRefresh(hd_508, vmapid_508, 0);

        // Read data received during the last refresh
        for (int i = 0; i < 8; i++) {
            uint8_t read_buffer[128];
            int read_len;
            DqRtVmapReadInput(hd_508, vmapid_508, 4, i, sizeof(read_buffer), &read_len, read_buffer);
            if (read_len > 0) {
                ph_bufq_append(SL508_read_buffer[i], read_buffer, read_len, NULL);
            }
        }

        timespec_add_ns(&next, periodns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    DqRtVmapStop(hd_508, vmapid_508);
    DqRtVmapClose(hd_508, vmapid_508);
    return NULL;
}

void *uei_dmap_update_loop(void *m_arg) {
	int ret;
    uint32_t chentry;
	struct timespec next;
	uint64_t periodns = ((double)(NSEC_PER_SEC) / frequency);

    ph_thread_set_name("OF_DMP");
    blast_startup("Starting UEI OF DMap loop");

    /**
     * Add all channels except those read out through the diagnostic interface
     */
    for (channel_t *ch = channel_list; ch->field[0]; ch++) {
        /**
         * Only select out frame UEI channels and ignore any listing board numbers
         * larger than number of slots available on UEI
         */
        if ((ch->source != SRC_OF_UEI) || ch->board > 5) continue;

        if (ch->board == 2) {
            chentry = ch->chan | DQ_LNCL_GAIN(0) | DQ_LNCL_DIFF;
        } else {
            chentry = ch->chan | DQ_LNCL_GAIN(0);
        }
        uei_of_channels[ch->board][num_of_channels[ch->board]++] = ch;
        DqRtDmapAddChannel(hd_dmap, dmapid, ch->board, DQ_SS0IN, &chentry, 1);
    }

    /**
     * Add the Digital output channel to the DMAP.  WARNING! This is hard-coded
     * and will need to be changed if the physical order of the UEI boards
     * change!
     */
    DqRtDmapAddChannel(hd_dmap, dmapid, 3, DQ_SS0OUT, 0, 0);

    // Start the layers
    DqRtDmapStart(hd_dmap, dmapid);

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!shutdown_mcp) {
        DqRtDmapWriteRawData32(hd_dmap, dmapid, 3, &(CommandData.uei_command.uei_of_dio_432_out), 1);

		ret = DqRtDmapRefresh(hd_dmap, dmapid);
		if (ret < 0) {
			blast_err("DqRtDmapRefresh: error %d", ret);
		}

		for (int i = 0; i < 6; i++) {
		    if (num_of_channels[i]) {
                if ((ret = DqRtDmapReadScaledData(hd_dmap, dmapid, i, raw_dmap_input[i], num_of_channels[i])) < 0) {
                    blast_err("Could not read scaled data from DMAP");
                    continue;
                }
		    }
		}

		timespec_add_ns(&next, periodns);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

    DqRtDmapStop(hd_dmap, dmapid);
    DqRtDmapClose(hd_dmap, dmapid);
	DqCloseIOM(hd_dmap);

	return NULL;
}

static void uei_of_store_hk(void)
{
	static channel_t *uei_of_2_5V_channel = NULL;
	static channel_t *uei_of_3_3V_channel = NULL;
	static channel_t *uei_of_24V_channel = NULL;
	static channel_t *uei_of_Vin_channel = NULL;
	static channel_t *uei_of_1_5V_channel = NULL;
	static channel_t *uei_of_1_2V_channel = NULL;
	static channel_t *uei_of_i_in_channel = NULL;
	static channel_t *uei_of_temp1_channel = NULL;
	static channel_t *uei_of_temp2_channel = NULL;

	if (!uei_of_2_5V_channel) {
		uei_of_2_5V_channel = channels_find_by_name("uei_of_2_5V");
		uei_of_3_3V_channel = channels_find_by_name("uei_of_3_3V");
		uei_of_24V_channel = channels_find_by_name("uei_of_24V");
		uei_of_Vin_channel = channels_find_by_name("uei_of_Vin");
		uei_of_1_5V_channel = channels_find_by_name("uei_of_1_5V");
		uei_of_1_2V_channel = channels_find_by_name("uei_of_1_2V");
		uei_of_i_in_channel = channels_find_by_name("uei_of_i_in");
		uei_of_temp1_channel = channels_find_by_name("uei_of_temp1");
		uei_of_temp2_channel = channels_find_by_name("uei_of_temp2");
	}

	SET_UINT32(uei_of_2_5V_channel, raw_diag_data[DQ_LDIAG_ADC_V_2_5]);
	SET_UINT32(uei_of_3_3V_channel, raw_diag_data[DQ_LDIAG_ADC_V_3_3]);
	SET_UINT32(uei_of_24V_channel, raw_diag_data[DQ_LDIAG_ADC_V_24]);
	SET_UINT32(uei_of_Vin_channel, raw_diag_data[DQ_LDIAG_ADC_V_IN]);
	SET_UINT32(uei_of_1_5V_channel, raw_diag_data[DQ_LDIAG_ADC_V_1_5]);
	SET_UINT32(uei_of_1_2V_channel, raw_diag_data[DQ_LDIAG_ADC_V_1_2]);
	SET_UINT32(uei_of_i_in_channel, raw_diag_data[DQ_L2_ADC_I_IN]);
	SET_UINT32(uei_of_temp1_channel, raw_diag_data[DQ_LDIAG_ADC_TEMP1]);
	SET_UINT32(uei_of_temp2_channel, raw_diag_data[DQ_LDIAG_ADC_TEMP1]);
}

void uei_1hz_loop(void)
{
    uei_of_store_hk();
}

void uei_100hz_loop(void)
{
    for (int i = 0; i < 6; i++) {
        for (int ch = 0; ch < num_of_channels[i]; ch++) {
            if (!uei_of_channels[i][ch])
                blast_dbg("no channel here!");
            else
                SET_SCALED_VALUE(uei_of_channels[i][ch], raw_dmap_input[i][ch]);
        }
    }
}

int initialize_uei_of_channels(void)
{
    int ret;
    DQRDCFG *DQRdCfg = NULL;

    for (int i = 0; i < 8; i++) {
        SL508_write_buffer[i] = ph_bufq_new(0);
        SL508_read_buffer[i] = ph_bufq_new(0);
    }

    ret = DqInitDAQLib();
    if (ret < 0) {
        blast_err("Error %d in DqInitDAQLib", ret);
        goto uei_init_err;
    }

    ret = DqOpenIOM("192.168.1.10", DQ_UDP_DAQ_PORT, 1000, &hd_of, &DQRdCfg);
    if (ret < 0) {
        blast_err("Error %d in DqOpenIOM", ret);
        goto uei_init_err;
    }

    if ((ret = DqAddIOMPort(hd_of, &hd_dmap, DQ_UDP_DAQ_PORT, 1000)) < 0) {
        blast_err("Error %d adding DMAP port", ret);
        goto uei_init_err;
    }

    if ((ret = DqAddIOMPort(hd_of, &hd_508, DQ_UDP_DAQ_PORT, 1000)) < 0) {
        blast_err("Error %d adding 508 port", ret);
        goto uei_init_err;
    }

    if ((ret = DqRtDmapInit(hd_of, &dmapid, frequency)) < 0) {
        blast_err("Could not initialize DMAP: %s", DqTranslateError(ret));
    }

//    hd_508 = hd_of;
//    if ((ret = DqRtVmapInit(hd_508, &vmapid_508, 0.1)) < 0) {
//        blast_err("Could not initialize VMAP: %s", DqTranslateError(ret));
//    }

    return 0;
uei_init_err:
    if (hd_dmap)DqCloseIOM(hd_dmap);
    hd_dmap = 0;
    if (hd_508)DqCloseIOM(hd_508);
    hd_508 = 0;
    if (hd_of)DqCloseIOM(hd_of);
    hd_of = 0;
    DqCleanUpDAQLib();
    return -1;
}
