/* tx.c */

/* NB: As of 7 Sep 2003 this file has been split up into four pieces:
 *
 * auxiliary.c: Auxiliary controls: Lock Motor, Pumps, Electronics Heat, ISC
 * das.c:       DAS, Bias and Cryo controls
 * motors.c:    Motor commanding and Scan modes
 * tx.c:        Pointing data writeback, ADC sync, and standard Tx frame control
 * 
 * -dvw */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "bbc.h"

#include "tx_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"

extern short int SamIAm;
short int InCharge;

extern unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

extern short int write_ISC_pointing;  /* isc.c */
extern struct ISCStatusStruct SentState;  /* isc.c */

double round(double x);

/* in auxiliary.c */
void ControlAuxMotors(unsigned int *TxFrame,  unsigned short *RxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);
void ControlGyroHeat(unsigned int *TxFrame,  unsigned short *RxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);

/* in das.c */
void BiasControl (unsigned int* TxFrame,  unsigned short* RxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);
void CryoControl (unsigned int* Txfrmae,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);
void PhaseControl(unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);
void SetReadBits(unsigned int* TxFrame);

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex, unsigned int *TxFrame, unsigned short *RxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]);

int frame_num;

double getlst(time_t t, double lon);

void DoSched();

/************************************************************************/
/*                                                                      */
/*  WriteAux: write aux data, like cpu time, temperature, fan speed     */
/*                                                                      */
/************************************************************************/
void WriteAux(unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_fan = -1, j_fan = -1;
  static int i_time = -1, j_time = -1;
  static int i_samiam = -1, j_samiam = -1;
  static int i_df = -1, j_df = -1;
  static int aliceFileCh, aliceFileInd;
  static int timeoutCh, timeoutInd;
  static int cpuTemp1Ch, cpuTemp1Ind;
  static int cpuTemp2Ch, cpuTemp2Ind;
  static int cpuTemp3Ch, cpuTemp3Ind;
  static int incharge = -1;
  time_t t;
  int i_point;

  if (i_fan == -1) {
    SlowChIndex("cpu_fan", &i_fan, &j_fan);
    SlowChIndex("cpu_temp1", &cpuTemp1Ch, &cpuTemp1Ind);
    SlowChIndex("cpu_temp2", &cpuTemp2Ch, &cpuTemp2Ind);
    SlowChIndex("cpu_temp3", &cpuTemp3Ch, &cpuTemp3Ind);
    SlowChIndex("cpu_time", &i_time, &j_time);
    SlowChIndex("sam_i_am", &i_samiam, &j_samiam);
    SlowChIndex("disk_free", &i_df, &j_df);
    SlowChIndex("alice_file", &aliceFileCh, &aliceFileInd);
    SlowChIndex("timeout", &timeoutCh, &timeoutInd);
  }

  t = time(NULL);

  InCharge = !(SamIAm ^ slow_data[i_samiam][j_samiam]);
  if (InCharge != incharge && InCharge)
    mputs(MCP_INFO, "I have gained control.\n");
  else if (InCharge != incharge)
    mputs(MCP_INFO, "I have lost control.\n");

  incharge = InCharge;

  WriteSlow(i_time, j_time, t >> 16);
  WriteSlow(i_time + 1, j_time, t);

  WriteSlow(i_fan, j_fan, CommandData.fan);
  WriteSlow(cpuTemp1Ch, cpuTemp1Ind, CommandData.temp1);
  WriteSlow(cpuTemp2Ch, cpuTemp2Ind, CommandData.temp2);
  WriteSlow(cpuTemp3Ch, cpuTemp3Ind, CommandData.temp3);

  WriteSlow(i_samiam, j_samiam, SamIAm);
  WriteSlow(i_df, j_df, CommandData.df);

  i_point = GETREADINDEX(point_index);

  t = PointingData[i_point].t;

  WriteSlow(aliceFileCh, aliceFileInd, CommandData.alice_file);
  WriteSlow(timeoutCh, timeoutInd, CommandData.pointing_mode.t - t);
}

/*****************************************************************/
/* SyncADC: check to see if any boards need to be synced and     */
/* send the sync bit if they do.  Only one board can be synced   */
/* in each superframe.                                           */
/*****************************************************************/
int ADC_sync_timeout = 0;
void SyncADC (int TxIndex, unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int syncCh = -1, syncInd, nextInd;
  static int statusInd[17];
  static int statusCh[17];
  char buffer[9];

  int k, l;

  if (ADC_sync_timeout >= 600)
    return;

  ADC_sync_timeout++;
  
  /******** Obtain correct indexes the first time here ***********/
  if (syncCh == -1) {
    SlowChIndex("sync", &syncCh, &syncInd);
    nextInd = (syncInd + 1) % FAST_PER_SLOW;
    for (k = 0; k < 17; ++k) {
      sprintf(buffer, "status%02i", k);
      SlowChIndex(buffer, &statusCh[k], &statusInd[k]);
    }
  }

  /* are we currently syncing? */
  if (slowTxFields[syncCh][syncInd] & BBC_ADC_SYNC) {
    /* if yes, turn off sync bit if we sent the sync last frame */
    if (TxIndex == nextInd) {
      slowTxFields[syncCh][syncInd] = BBC_WRITE | BBC_NODE(17) | BBC_CH(56);
    }
  } else {
    /* if not, check to see if we need to sync a board */
    for (k = 0; k < 17; ++k) {
      /* read board status */
      if (slow_data[statusCh[k]][statusInd[k]] == 0x0001) {
        /* board needs to be synced */
        mprintf(MCP_INFO, "ADC Sync board %i\n", k);
        l = (k == 0) ? 21 : k;
        slowTxFields[syncCh][syncInd] =
          BBC_WRITE | BBC_NODE(l) | BBC_CH(56) |
          BBC_ADC_SYNC | 0xa5a3;
        k = 17;  /* ABORT! ABORT! */
      }
    }
  }
}

/************************************************************************/
/*                                                                      */
/*    Store derived acs and pointing data in frame                      */
/*                                                                      */
/************************************************************************/
void StoreData(int index, unsigned int* TxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {

  static int firsttime = 1;
  
  static int i_SS_XCCD;
  static int i_SS_PRIN, j_SS_PRIN;
  static int i_SIP_LAT, i_SIP_LON, i_SIP_ALT, i_SIP_TIME;
  static int j_SIP_LAT, j_SIP_LON, j_SIP_ALT, j_SIP_TIME;

  /** pointing mode indexes **/
  static int i_MODE, j_MODE;
  static int i_X, j_X, i_Y, j_Y;
  static int i_VAZ, j_VAZ, i_DEL, j_DEL;
  static int i_W, j_W, i_H, j_H;
  
  static int i_SVETO, j_SVETO;

  /** derived pointing data */
  static int mcpFrameCh;
  static int gy1OffCh, gy1OffInd;
  static int gy2OffCh, gy2OffInd;
  static int gy3OffCh, gy3OffInd;
  static int gyRollAmpCh, gyRollAmpInd;
  static int i_RA, j_RA;
  static int i_DEC, j_DEC;  
  static int i_LAT, j_LAT;
  static int i_LON, j_LON;
  static int timeCh, timeInd;
  static int i_LST, j_LST;
  static int magAzCh, magAzInd;
  static int i_MAG_MODEL, j_MAG_MODEL;
  static int magSigmaCh, magSigmaInd;
  static int dgpsAzCh, dgpsAzInd;
  static int dgpsPitchCh, dgpsPitchInd;
  static int dgpsRollCh, dgpsRollInd;
  static int dgpsSigmaCh, dgpsSigmaInd;
  static int ssAzCh, ssAzInd;
  static int ssSigmaCh, ssSigmaInd;
  static int i_AZ_SUN, j_AZ_SUN;
  static int iscAzCh, iscAzInd;
  static int iscElCh, iscElInd;
  static int iscSigmaCh, iscSigmaInd;
  static int encElCh, encElInd;
  static int encSigmaCh, encSigmaInd;
  static int clinElCh, clinElInd;
  static int clinSigmaCh, clinSigmaInd;

  /** dgps fields **/
  static int i_dgps_time, j_dgps_time;
  static int i_dgps_lat, j_dgps_lat;  
  static int i_dgps_lon, j_dgps_lon;  
  static int i_dgps_alt, j_dgps_alt;  
  static int i_dgps_speed, j_dgps_speed;  
  static int i_dgps_dir, j_dgps_dir;  
  static int i_dgps_climb, j_dgps_climb;
  static int i_dgps_att_ok, j_dgps_att_ok;
  static int i_dgps_att_index, j_dgps_att_index;
  static int i_dgps_pos_index, j_dgps_pos_index;
  static int i_dgps_n_sat, j_dgps_n_sat;

  /** isc fields **/
  static int blob0_xCh, blob0_xInd;
  static int blob1_xCh, blob1_xInd;
  static int blob2_xCh, blob2_xInd;
  static int blob0_yCh, blob0_yInd;
  static int blob1_yCh, blob1_yInd;
  static int blob2_yCh, blob2_yInd;
  static int blob0_fluxCh, blob0_fluxInd;
  static int blob1_fluxCh, blob1_fluxInd;
  static int blob2_fluxCh, blob2_fluxInd;
  static int blob0_snCh, blob0_snInd;
  static int blob1_snCh, blob1_snInd;
  static int blob2_snCh, blob2_snInd;
  static int isc_errorCh, isc_errorInd;
  static int isc_mapMeanCh, isc_mapMeanInd;
  static int isc_framenumCh, isc_framenumInd;
  static int isc_rd_sigmaCh, isc_rd_sigmaInd;
  static int isc_raCh, isc_raInd;
  static int isc_decCh, isc_decInd;
  static int isc_afocusCh, isc_afocusInd;
  static int isc_mcpnumCh, isc_mcpnumInd;
  static int isc_brraCh, isc_brraInd;
  static int isc_brdecCh, isc_brdecInd;
  static int isc_apertCh, isc_apertInd;
  static int isc_cenboxCh, isc_cenboxInd;
  static int isc_apboxCh, isc_apboxInd;
  static int isc_mdistCh, isc_mdistInd;
  static int isc_nblobsCh, isc_nblobsInd;
  static int isc_focusCh, isc_focusInd;
  static int isc_threshCh, isc_threshInd;
  static int isc_gridCh, isc_gridInd;
  static int isc_stateCh, isc_stateInd;
  static int isc_maxblobsCh, isc_maxblobsInd;
  static int isc_maglimitCh, isc_maglimitInd;
  static int isc_nradCh, isc_nradInd;
  static int isc_lradCh, isc_lradInd;
  static int isc_tolCh, isc_tolInd;
  static int isc_mtolCh, isc_mtolInd;
  static int isc_qtolCh, isc_qtolInd;
  static int isc_rtolCh, isc_rtolInd;
  static int isc_fpulseCh, isc_fpulseInd;
  static int isc_spulseCh, isc_spulseInd;
  static int isc_x_offCh, isc_x_offInd;
  static int isc_y_offCh, isc_y_offInd;
  static int isc_hold_iCh, isc_hold_iInd;
  static int isc_save_periodCh, isc_save_periodInd;

  static int blob_index = 0;

  time_t t;

  static int i_az = -1, i_el = -1;
  int i_ss;
  int i_point;
  int i_dgps;
  int sensor_veto;
  int i_isc = GETREADINDEX(iscdata_index);

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;	
    FastChIndex("az", &i_az);
    FastChIndex("el", &i_el);
    FastChIndex("ss_x_ccd", &i_SS_XCCD);
    FastChIndex("mcp_frame", &mcpFrameCh);

    SlowChIndex("ss_prin", &i_SS_PRIN, &j_SS_PRIN);
    SlowChIndex("sip_lat", &i_SIP_LAT, &j_SIP_LAT);
    SlowChIndex("sip_lon", &i_SIP_LON, &j_SIP_LON);
    SlowChIndex("sip_alt", &i_SIP_ALT, &j_SIP_ALT);
    SlowChIndex("sip_time", &i_SIP_TIME, &j_SIP_TIME);

    SlowChIndex("gy1_offset", &gy1OffCh, &gy1OffInd);
    SlowChIndex("gy2_offset", &gy2OffCh, &gy2OffInd);
    SlowChIndex("gy3_offset", &gy3OffCh, &gy3OffInd);
    SlowChIndex("gy_roll_amp", &gyRollAmpCh, &gyRollAmpInd);
    SlowChIndex("ra", &i_RA, &j_RA);
    SlowChIndex("dec", &i_DEC, &j_DEC);
    SlowChIndex("lat", &i_LAT, &j_LAT);
    SlowChIndex("lon", &i_LON, &j_LON);
    SlowChIndex("time", &timeCh, &timeInd);
    SlowChIndex("lst", &i_LST, &j_LST);
    SlowChIndex("mag_az", &magAzCh, &magAzInd);
    SlowChIndex("mag_model", &i_MAG_MODEL, &j_MAG_MODEL);
    SlowChIndex("mag_sigma", &magSigmaCh, &magSigmaInd);
    SlowChIndex("dgps_az", &dgpsAzCh, &dgpsAzInd);
    SlowChIndex("dgps_pitch", &dgpsPitchCh, &dgpsPitchInd);
    SlowChIndex("dgps_roll", &dgpsRollCh, &dgpsRollInd);
    SlowChIndex("dgps_sigma", &dgpsSigmaCh, &dgpsSigmaInd);
    SlowChIndex("ss_az", &ssAzCh, &ssAzInd);
    SlowChIndex("ss_sigma", &ssSigmaCh, &ssSigmaInd);
    SlowChIndex("sun_az", &i_AZ_SUN, &j_AZ_SUN);
    SlowChIndex("isc_az", &iscAzCh, &iscAzInd);
    SlowChIndex("isc_el", &iscElCh, &iscElInd);
    SlowChIndex("isc_sigma", &iscSigmaCh, &iscSigmaInd);
    SlowChIndex("enc_el", &encElCh, &encElInd);
    SlowChIndex("enc_sigma", &encSigmaCh, &encSigmaInd);
    SlowChIndex("clin_el", &clinElCh, &clinElInd);
    SlowChIndex("clin_sigma", &clinSigmaCh, &clinSigmaInd);

    SlowChIndex("p_mode", &i_MODE, &j_MODE);
    SlowChIndex("p_x_deg", &i_X, &j_X);
    SlowChIndex("p_y", &i_Y, &j_Y);
    SlowChIndex("p_vaz", &i_VAZ, &j_VAZ);
    SlowChIndex("p_del", &i_DEL, &j_DEL);
    SlowChIndex("p_w", &i_W, &j_W);
    SlowChIndex("p_h", &i_H, &j_H);

    SlowChIndex("sensor_veto", &i_SVETO, &j_SVETO);

    SlowChIndex("dgps_time", &i_dgps_time, &j_dgps_time);
    SlowChIndex("dgps_lat", &i_dgps_lat, &j_dgps_lat);
    SlowChIndex("dgps_lon", &i_dgps_lon, &j_dgps_lon);
    SlowChIndex("dgps_alt", &i_dgps_alt, &j_dgps_alt);
    SlowChIndex("dgps_speed", &i_dgps_speed, &j_dgps_speed);
    SlowChIndex("dgps_dir", &i_dgps_dir, &j_dgps_dir);
    SlowChIndex("dgps_climb", &i_dgps_climb, &j_dgps_climb);
    SlowChIndex("dgps_n_sat", &i_dgps_n_sat, &j_dgps_n_sat);
    SlowChIndex("dgps_pos_index", &i_dgps_pos_index, &j_dgps_pos_index);
    SlowChIndex("dgps_att_ok", &i_dgps_att_ok, &j_dgps_att_ok);
    SlowChIndex("dgps_att_index", &i_dgps_att_index, &j_dgps_att_index);

    SlowChIndex("blob0_x", &blob0_xCh, &blob0_xInd);
    SlowChIndex("blob1_x", &blob1_xCh, &blob1_xInd);
    SlowChIndex("blob2_x", &blob2_xCh, &blob2_xInd);
    SlowChIndex("blob0_y", &blob0_yCh, &blob0_yInd);
    SlowChIndex("blob1_y", &blob1_yCh, &blob1_yInd);
    SlowChIndex("blob2_y", &blob2_yCh, &blob2_yInd);
    SlowChIndex("blob0_flux", &blob0_fluxCh, &blob0_fluxInd);
    SlowChIndex("blob1_flux", &blob1_fluxCh, &blob1_fluxInd);
    SlowChIndex("blob2_flux", &blob2_fluxCh, &blob2_fluxInd);
    SlowChIndex("blob0_sn", &blob0_snCh, &blob0_snInd);
    SlowChIndex("blob1_sn", &blob1_snCh, &blob1_snInd);
    SlowChIndex("blob2_sn", &blob2_snCh, &blob2_snInd);
    SlowChIndex("isc_error", &isc_errorCh, &isc_errorInd);
    SlowChIndex("isc_mapmean", &isc_mapMeanCh, &isc_mapMeanInd);
    SlowChIndex("isc_rd_sigma", &isc_rd_sigmaCh, &isc_rd_sigmaInd);
    SlowChIndex("isc_framenum", &isc_framenumCh, &isc_framenumInd);
    SlowChIndex("isc_ra", &isc_raCh, &isc_raInd);
    SlowChIndex("isc_dec", &isc_decCh, &isc_decInd);
    SlowChIndex("isc_nblobs", &isc_nblobsCh, &isc_nblobsInd);
    SlowChIndex("isc_afocus", &isc_afocusCh, &isc_afocusInd);
    SlowChIndex("isc_mcpnum", &isc_mcpnumCh, &isc_mcpnumInd);

    SlowChIndex("isc_state", &isc_stateCh, &isc_stateInd);
    SlowChIndex("isc_focus", &isc_focusCh, &isc_focusInd);
    SlowChIndex("isc_brra", &isc_brraCh, &isc_brraInd);
    SlowChIndex("isc_brdec", &isc_brdecCh, &isc_brdecInd);
    SlowChIndex("isc_apert", &isc_apertCh, &isc_apertInd);
    SlowChIndex("isc_thresh", &isc_threshCh, &isc_threshInd);
    SlowChIndex("isc_grid", &isc_gridCh, &isc_gridInd);
    SlowChIndex("isc_cenbox", &isc_cenboxCh, &isc_cenboxInd);
    SlowChIndex("isc_apbox", &isc_apboxCh, &isc_apboxInd);
    SlowChIndex("isc_mdist", &isc_mdistCh, &isc_mdistInd);
    SlowChIndex("isc_maxblobs", &isc_maxblobsCh, &isc_maxblobsInd);
    SlowChIndex("isc_maglimit", &isc_maglimitCh, &isc_maglimitInd);
    SlowChIndex("isc_nrad", &isc_nradCh, &isc_nradInd);
    SlowChIndex("isc_lrad", &isc_lradCh, &isc_lradInd);
    SlowChIndex("isc_tol", &isc_tolCh, &isc_tolInd);
    SlowChIndex("isc_mtol", &isc_mtolCh, &isc_mtolInd);
    SlowChIndex("isc_qtol", &isc_qtolCh, &isc_qtolInd);
    SlowChIndex("isc_rtol", &isc_rtolCh, &isc_rtolInd);
    SlowChIndex("isc_fpulse", &isc_fpulseCh, &isc_fpulseInd);
    SlowChIndex("isc_spulse", &isc_spulseCh, &isc_spulseInd);
    SlowChIndex("isc_x_off", &isc_x_offCh, &isc_x_offInd);
    SlowChIndex("isc_y_off", &isc_y_offCh, &isc_y_offInd);
    SlowChIndex("isc_hold_i", &isc_hold_iCh, &isc_hold_iInd);
    SlowChIndex("isc_save_prd", &isc_save_periodCh, &isc_save_periodInd);
  }

  i_point = GETREADINDEX(point_index);
  i_ss = GETREADINDEX(ss_index);

  /********** Sun Sensor Data **********/
  WriteFast(i_SS_XCCD, SunSensorData[i_ss].raw_az);
  WriteSlow(i_SS_PRIN, j_SS_PRIN, SunSensorData[i_ss].prin);

  /********** SIP GPS Data **********/
  WriteSlow(i_SIP_LAT, j_SIP_LAT, (int)(SIPData.GPSpos.lat*DEG2I));
  WriteSlow(i_SIP_LON, j_SIP_LON, (int)(SIPData.GPSpos.lon*DEG2I));
  WriteSlow(i_SIP_ALT, j_SIP_ALT, (int)(SIPData.GPSpos.alt*0.25));
  t = SIPData.GPStime.UTC;
  WriteSlow(i_SIP_TIME, j_SIP_TIME, t >> 16);
  WriteSlow(i_SIP_TIME + 1, j_SIP_TIME, t);


  /************* processed pointing data *************/
  WriteFast(i_az, (unsigned int)(PointingData[i_point].az * 65536.0/360.0));
  WriteFast(i_el, (unsigned int)(PointingData[i_point].el * 65536.0/360.0));

  WriteSlow(i_RA, j_RA,
	    (unsigned int)(PointingData[i_point].ra * 65536.0/24.0));
  WriteSlow(i_DEC, j_DEC,
	    (unsigned int)(PointingData[i_point].dec * 65536.0/360.0));
	    
  WriteSlow(gy1OffCh, gy1OffInd,
      (signed int)(PointingData[i_point].gy1_offset * 32768.));
  WriteSlow(gy2OffCh, gy2OffInd,
      (signed int)(PointingData[i_point].gy2_offset * 32768.));
  WriteSlow(gy3OffCh, gy3OffInd,
      (signed int)(PointingData[i_point].gy3_offset * 32768.));
  WriteSlow(gyRollAmpCh, gyRollAmpInd,
      (unsigned int)(PointingData[i_point].gy_roll_amp * 65536.));

  WriteSlow(i_LAT, j_LAT, (unsigned int)(PointingData[i_point].lat * DEG2I));
  WriteSlow(i_LON, j_LON, (unsigned int)(PointingData[i_point].lon * DEG2I));

  WriteFast(mcpFrameCh, PointingData[i_point].mcp_frame);
  WriteSlow(timeCh, timeInd, PointingData[i_point].t >> 16);
  WriteSlow(timeCh + 1, timeInd, PointingData[i_point].t);
  t = PointingData[i_point].lst;
  WriteSlow(i_LST, j_LST, t >> 16);
  WriteSlow(i_LST + 1, j_LST, t);

  WriteSlow(magAzCh, magAzInd,
      (unsigned int)(PointingData[i_point].mag_az * 65536.0/360.0));
  WriteSlow(i_MAG_MODEL, j_MAG_MODEL,
      (unsigned int)(PointingData[i_point].mag_model * DEG2I));
  WriteSlow(magSigmaCh, magSigmaInd,
      (unsigned int)(PointingData[i_point].mag_sigma * DEG2I));
  WriteSlow(dgpsAzCh, dgpsAzInd,
      (unsigned int)(PointingData[i_point].dgps_az * DEG2I));
  WriteSlow(dgpsPitchCh, dgpsPitchInd,
      (unsigned int)(PointingData[i_point].dgps_pitch * DEG2I));
  WriteSlow(dgpsRollCh, dgpsRollInd,
      (unsigned int)(PointingData[i_point].dgps_roll * DEG2I));
  WriteSlow(dgpsSigmaCh, dgpsSigmaInd,
      (unsigned int)(PointingData[i_point].dgps_sigma * DEG2I));

  WriteSlow(ssAzCh, ssAzInd, (unsigned int)(PointingData[i_point].ss_az*DEG2I));
  WriteSlow(ssSigmaCh, ssSigmaInd,
      (unsigned int)(PointingData[i_point].ss_sigma * DEG2I));
  WriteSlow(i_AZ_SUN, j_AZ_SUN,
      (unsigned int)(PointingData[i_point].sun_az*DEG2I));

  WriteSlow(iscAzCh, iscAzInd,
      (unsigned int)(PointingData[i_point].isc_az * DEG2I));
  WriteSlow(iscElCh, iscElInd,
      (unsigned int)(PointingData[i_point].isc_el * DEG2I));
  WriteSlow(iscSigmaCh, iscSigmaInd,
      (unsigned int)(PointingData[i_point].isc_sigma * DEG2I));

  WriteSlow(encElCh, encElInd,
      (unsigned int)(PointingData[i_point].enc_el * DEG2I));
  WriteSlow(encSigmaCh, encSigmaInd,
      (unsigned int)(PointingData[i_point].enc_sigma * DEG2I));

  WriteSlow(clinElCh, clinElInd,
      (unsigned int)(PointingData[i_point].clin_el * DEG2I));
  WriteSlow(clinSigmaCh, clinSigmaInd,
      (unsigned int)(PointingData[i_point].clin_sigma * DEG2I));

  /************* Pointing mode fields *************/
  WriteSlow(i_MODE, j_MODE, (int)(CommandData.pointing_mode.mode));
  if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) ||
      (CommandData.pointing_mode.mode == P_AZ_SCAN)) {
    WriteSlow(i_X, j_X, (int)(CommandData.pointing_mode.X * DEG2I));
  } else {
    WriteSlow(i_X, j_X, (int)(CommandData.pointing_mode.X * H2I));
  }
  WriteSlow(i_Y, j_Y, (int)(CommandData.pointing_mode.Y * DEG2I));
  WriteSlow(i_VAZ, j_VAZ, (int)(CommandData.pointing_mode.vaz * VEL2I));
  WriteSlow(i_DEL, j_DEL, (int)(CommandData.pointing_mode.del * VEL2I));
  WriteSlow(i_W, j_W, (int)(CommandData.pointing_mode.w * DEG2I));
  WriteSlow(i_H, j_H, (int)(CommandData.pointing_mode.h * DEG2I));

  sensor_veto = (!CommandData.use_sun) | ((!CommandData.use_isc)<<1) |
    ((!CommandData.use_elenc)<<2) |
    ((!CommandData.use_mag)<<3) |
    ((!CommandData.use_gps)<<4) |
    ((!CommandData.use_elclin)<<5);

  if (PointingData[i_point].t >= CommandData.pointing_mode.t) {
    sensor_veto |= (1 << 6);
  }

  WriteSlow(i_SVETO, j_SVETO, sensor_veto);

  /************* dgps fields *************/
  t = DGPSTime;
  WriteSlow(i_dgps_time, j_dgps_time, t >> 16);
  WriteSlow(i_dgps_time + 1, j_dgps_time, t);

  /** Pos fields **/
  i_dgps = GETREADINDEX(dgpspos_index);
  WriteSlow(i_dgps_lat, j_dgps_lat, (int)(DGPSPos[i_dgps].lat * DEG2I));
  WriteSlow(i_dgps_lon, j_dgps_lon, (int)(DGPSPos[i_dgps].lon * DEG2I));
  WriteSlow(i_dgps_alt, j_dgps_alt, (int)(DGPSPos[i_dgps].alt));
  WriteSlow(i_dgps_speed, j_dgps_speed, (int)(DGPSPos[i_dgps].speed * DEG2I));
  WriteSlow(i_dgps_dir, j_dgps_dir, (int)(DGPSPos[i_dgps].direction * DEG2I));
  WriteSlow(i_dgps_climb, j_dgps_climb, (int)(DGPSPos[i_dgps].climb * DEG2I));
  WriteSlow(i_dgps_n_sat, j_dgps_n_sat, DGPSPos[i_dgps].n_sat);
  WriteSlow(i_dgps_pos_index, j_dgps_pos_index, i_dgps);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);
  WriteSlow(i_dgps_att_ok, j_dgps_att_ok, DGPSAtt[i_dgps].att_ok);
  WriteSlow(i_dgps_att_index, j_dgps_att_index, i_dgps);

  /** ISC Fields **/
  if (index == 0) {
    if (blob_index == 0) {
      i_isc = GETREADINDEX(iscdata_index); 
    }
  }

  /*** Blobs ***/
  WriteSlow(blob0_xCh, blob0_xInd,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 0] * 40.));
  WriteSlow(blob1_xCh, blob1_xInd,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 1] * 40.));
  WriteSlow(blob2_xCh, blob2_xInd,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 2] * 40.));

  WriteSlow(blob0_yCh, blob0_yInd,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 0] * 40.));
  WriteSlow(blob1_yCh, blob1_yInd,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 1] * 40.));
  WriteSlow(blob2_yCh, blob2_yInd,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 2] * 40.));

  WriteSlow(blob0_fluxCh, blob0_fluxInd,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 0] / 32.));
  WriteSlow(blob1_fluxCh, blob1_fluxInd,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 1] / 32.));
  WriteSlow(blob2_fluxCh, blob2_fluxInd,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 2] / 32.));

  WriteSlow(blob0_snCh, blob0_snInd,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 0] * 65.536));
  WriteSlow(blob1_snCh, blob1_snInd,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 1] * 65.536));
  WriteSlow(blob2_snCh, blob2_snInd,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 2] * 65.536));

  if (++blob_index >= 5)
    blob_index = 0;

  /*** Solution Info ***/
  WriteSlow(isc_framenumCh, isc_framenumInd,
      (unsigned int)ISCSolution[i_isc].framenum);
  WriteSlow(isc_raCh, isc_raInd,
      (unsigned int)(ISCSolution[i_isc].ra * RAD2LI) >> 16);
  WriteSlow(isc_raCh + 1, isc_raInd,
      (unsigned int)(ISCSolution[i_isc].ra * RAD2LI));
  WriteSlow(isc_decCh, isc_decInd,
      (unsigned int)((ISCSolution[i_isc].dec + M_PI / 2) * 2. * RAD2LI) >> 16);
  WriteSlow(isc_decCh + 1, isc_decInd,
      (unsigned int)((ISCSolution[i_isc].dec + M_PI / 2) * 2.* RAD2LI));
  WriteSlow(isc_nblobsCh, isc_nblobsInd,
      (unsigned int)ISCSolution[i_isc].n_blobs);
  if (ISCSolution[i_isc].sigma * RAD2ARCSEC > 65535) {
    WriteSlow(isc_rd_sigmaCh, isc_rd_sigmaInd, 65535);
  } else {
    WriteSlow(isc_rd_sigmaCh, isc_rd_sigmaInd,
        (unsigned int)(ISCSolution[i_isc].sigma * RAD2ARCSEC));
  }
  WriteSlow(isc_mcpnumCh, isc_mcpnumInd,
      (unsigned int)ISCSolution[i_isc].MCPFrameNum);
  WriteSlow(isc_afocusCh, isc_afocusInd,
      (unsigned int)ISCSolution[i_isc].autoFocusPosition);
  WriteSlow(isc_errorCh, isc_errorInd,
      (unsigned int)ISCSolution[i_isc].cameraerr);
  WriteSlow(isc_mapMeanCh, isc_mapMeanInd,
      (unsigned int)ISCSolution[i_isc].mapMean);

  /*** State Info ***/
  WriteSlow(isc_stateCh, isc_stateInd,
      (unsigned int)(SentState.pause * 2 + SentState.abort * 4 +
                     SentState.autofocus * 8 + SentState.brightStarMode * 16 +
                     SentState.shutdown * 32 + SentState.save));
  WriteSlow(isc_focusCh, isc_focusInd, (unsigned int)SentState.focus_pos);
  WriteSlow(isc_apertCh, isc_apertInd, (unsigned int)SentState.ap_pos);
  WriteSlow(isc_brraCh, isc_brraInd,
      (unsigned int)(SentState.brightRA * RAD2I));
  WriteSlow(isc_brdecCh, isc_brdecInd,
      (unsigned int)(SentState.brightDEC * RAD2I));
  WriteSlow(isc_threshCh, isc_threshInd,
      (unsigned int)(SentState.sn_threshold * 10.));
  WriteSlow(isc_gridCh, isc_gridInd, (unsigned int)SentState.grid);
  WriteSlow(isc_cenboxCh, isc_cenboxInd, (unsigned int)SentState.cenbox);
  WriteSlow(isc_apboxCh, isc_apboxInd, (unsigned int)SentState.apbox);
  WriteSlow(isc_mdistCh, isc_mdistInd, (unsigned int)SentState.mult_dist);
  WriteSlow(isc_maxblobsCh, isc_maxblobsInd,
      (unsigned int)SentState.maxBlobMatch);
  WriteSlow(isc_maglimitCh, isc_maglimitInd,
      (unsigned int)(SentState.mag_limit * 1000.));
  WriteSlow(isc_nradCh, isc_nradInd,
      (unsigned int)(SentState.norm_radius * RAD2I));
  WriteSlow(isc_lradCh, isc_lradInd,
      (unsigned int)(SentState.lost_radius * RAD2I));
  WriteSlow(isc_tolCh, isc_tolInd,
      (unsigned int)(SentState.tolerance * RAD2ARCSEC));
  WriteSlow(isc_mtolCh, isc_mtolInd,
      (unsigned int)(SentState.match_tol * 65535.));
  WriteSlow(isc_qtolCh, isc_qtolInd,
      (unsigned int)(SentState.quit_tol * 65535.));
  WriteSlow(isc_rtolCh, isc_rtolInd, (unsigned int)(SentState.rot_tol * RAD2I));
  WriteSlow(isc_x_offCh, isc_x_offInd, (unsigned int)(SentState.azBDA * RAD2I));
  WriteSlow(isc_y_offCh, isc_y_offInd, (unsigned int)(SentState.elBDA * RAD2I));
  WriteSlow(isc_hold_iCh, isc_hold_iInd,
      (unsigned int)(SentState.hold_current));
  WriteSlow(isc_fpulseCh, isc_fpulseInd,
      (unsigned int)(CommandData.ISC_fast_pulse_width));
  WriteSlow(isc_spulseCh, isc_spulseInd,
      (unsigned int)(CommandData.ISC_pulse_width));
  WriteSlow(isc_save_periodCh, isc_save_periodInd,
      (unsigned int)(CommandData.ISC_save_period));
}

/******************************************************************/
/*                                                                */
/* IsNewFrame: returns true if d is a begining of frame marker,   */
/*    unless this is the first beginning of frame.                */
/*                                                                */
/******************************************************************/
int IsNewFrame(unsigned int d) {
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | FILETYPE));
  if (is_bof && first_bof) {
    is_bof = 0; first_bof = 0;
  }

  return (is_bof);
}

void do_Tx_frame(int bbc_fp, unsigned int *TxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
    unsigned short *RxFrame, int reset) {
  static int firsttime = 1;
  static int index = 0;

  static int BBC_Fsync_Word = BBC_FSYNC;
  int i = 0, j;

  if (firsttime | reset) {
    TxFrame[0] = BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | FILETYPE; /* BOF */
    TxFrame[1] = BBC_WRITE | BBC_NODE(63) | BBC_CH(1); /* f_num lsb */
    TxFrame[2] = BBC_WRITE | BBC_NODE(63) | BBC_CH(2); /* f_num msb */
    TxFrame[3] = BBC_WRITE | BBC_NODE(63) | BBC_CH(3); /* index */

    for (j = 0; j < N_SLOW; j++) {
      for (i = 0; i < FAST_PER_SLOW; i++) {
        if (SlowChList[j][i].rw=='r') {
          slowTxFields[j][i] =
            BBC_READ |
            BBC_NODE(SlowChList[j][i].node) |
            BBC_CH(SlowChList[j][i].adr);
        } else {
          slowTxFields[j][i] =
            BBC_WRITE |
            BBC_NODE(SlowChList[j][i].node) |
            BBC_CH(SlowChList[j][i].adr);
        }
      }
    }

    for (i = 0; i < N_FASTCHLIST; i++) {
      if (FastChList[i].rw =='r') {
        TxFrame[i + FAST_OFFSET] =
          BBC_READ | BBC_NODE(FastChList[i].node) | BBC_CH(FastChList[i].adr);
      } else {
        TxFrame[i + FAST_OFFSET] =
          BBC_WRITE| BBC_NODE(FastChList[i].node) | BBC_CH(FastChList[i].adr);
      }
    }
    firsttime = 0;
  }

  /*** update frame num ***/
  WriteFast(1, frame_num);
  WriteFast(2, frame_num >> 16);
  frame_num++;

  /*** update mplex fields  ***/
  WriteFast(3, index);
  for (j = 0; j < N_SLOW; j++) {
    TxFrame[j + 4] = slowTxFields[j][index];
  }
  index++;
  if (index >= FAST_PER_SLOW)
    index = 0;

  /*** do Controls ***/
#ifndef BOLOTEST
  DoSched();
  UpdateAxesMode();
  StoreData(index, TxFrame, slowTxFields);
  ControlGyroHeat(TxFrame, RxFrame, slowTxFields);
  WriteMot(index, TxFrame, RxFrame, slowTxFields);
#endif
  BiasControl(TxFrame, RxFrame, slowTxFields);
  SyncADC(index, slowTxFields);

  /*** do slow Controls ***/
  if (index == 0) {
    WriteAux(slowTxFields);
    PhaseControl(slowTxFields);
  }
#ifndef BOLOTEST
  ControlAuxMotors(TxFrame, RxFrame, slowTxFields);
#endif
  CryoControl(TxFrame, slowTxFields);

  SetReadBits(TxFrame);

  /*** write FSync ***/
  write(bbc_fp, (void*)&BBC_Fsync_Word, sizeof(unsigned int));
  /*** Write Frame ***/
  write(bbc_fp, (void*)TxFrame, TX_FRAME_SIZE);
}
