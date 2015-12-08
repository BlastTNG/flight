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
#include "channels_tng.h"
#include "blast_time.h"
#include "mcp.h"

static channel_t *uei_of_channels[6][48] = {{NULL}};
static uint32_t num_of_channels[6] = {0};

static double frequency = 100.0;
static int hd_of = 0;
static int dmapid_of = 0;
static double diag_data[14] = {0.0};
static uint32_t raw_diag_data[14] = {0};
#define CPU_LAYER 5

void *uei_loop(void *m_arg) {
	int ret;
	uint32_t diag_channels[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

	struct timespec next;
	uint64_t periodns = ((double)(NSEC_PER_SEC) / frequency);
	int countdown = (int)frequency;

    clock_gettime(CLOCK_MONOTONIC, &next);

    blast_startup("Starting UEI loop");

    while (!shutdown_mcp) {
		ret = DqRtDmapRefresh(hd_of, dmapid_of);
		if (ret < 0)
		{
			blast_err("DqRtDmapRefresh: error %d", ret);
		}

		if (--countdown <= 0) {
			countdown = (int)frequency;

			ret = DqAdvDnxpRead(hd_of, CPU_LAYER, sizeof(diag_channels) / sizeof(uint32_t),
					diag_channels, raw_diag_data, diag_data);
			if (ret < 0)
			{
				blast_err("Could not read CPU Diagnostics: %s", DqTranslateError(ret));
			}
		}
		timespec_add_ns(&next, periodns);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

    DqRtDmapStop(hd_of, dmapid_of);
    DqRtDmapClose(hd_of, dmapid_of);
	DqCloseIOM(hd_of);
    DqCleanUpDAQLib();

	return NULL;
}

int initialize_uei_of_channels (void)
{
	int ret;
	static  DQRDCFG *DQRdCfg = NULL;
	uint32_t chentry;

    ret = DqInitDAQLib();
    if (ret < 0) {
        blast_err("Error %d in DqInitDAQLib", ret);
        return -1;
    }
    ret = DqOpenIOM("192.168.1.10", DQ_UDP_DAQ_PORT, 1000, &hd_of, &DQRdCfg);
    if (ret < 0) {
        blast_err("Error %d in DqOpenIOM", ret);
        return -1;
    }

	if ((ret = DqRtDmapInit(hd_of, &dmapid_of, 2 * frequency)) < 0) {
		blast_err("Could not initialize DMAP: %d", ret);
		DqCloseIOM(hd_of);
		return -1;
	}

	/**
	 * Add all channels except those read out through the diagnostic interface
	 */
	for (channel_t *ch = channel_list; ch->field[0]; ch++) {
		if ((ch->source != SRC_OF_UEI) || ch->board == 14) continue;
		chentry = ch->chan;
		blast_dbg("Adding %s to %d:%d", ch->field, ch->board, ch->chan);
		uei_of_channels[ch->board][num_of_channels[ch->board]++] = ch;
		DqRtDmapAddChannel(hd_of, dmapid_of, ch->board, DQ_SS0IN, &chentry, 1);
	}

    // Start the layers
    DqRtDmapStart(hd_of, dmapid_of);

    return 0;
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
	int ret;
	double data[48];
	for (int i = 0; i < 6; i++) {
		if (num_of_channels[i]) {
			if ((ret = DqRtDmapReadScaledData(hd_of, dmapid_of, i, data, num_of_channels[i])) < 0) {
				blast_err("Could not read scaled data from DMAP");
				continue;
			}
			for (int ch = 0; ch < num_of_channels[i]; ch++){
				if (!uei_of_channels[i][ch]) blast_dbg("no channel here!");
				else SET_SCALED_VALUE(uei_of_channels[i][ch], data[ch]);
			}
		}
	}
	uei_of_store_hk();
}
