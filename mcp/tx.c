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
#include "bbc_pci.h"

#include "tx_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"

extern short int SamIAm;
short int InCharge;

extern short int write_ISC_pointing;  /* isc.c */
extern struct ISCStatusStruct SentState;  /* isc.c */
extern int bbc_fp;

double round(double x);

/* in auxiliary.c */
void ControlAuxMotors(unsigned short *RxFrame);
void ControlGyroHeat(unsigned short *RxFrame);

/* in das.c */
void BiasControl(unsigned short* RxFrame);
void CryoControl();
void PhaseControl();

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex, unsigned short *RxFrame);

int frame_num;

double getlst(time_t t, double lon);

void DoSched();

/************************************************************************/
/*                                                                      */
/*  WriteAux: write aux data, like cpu time, temperature, fan speed     */
/*                                                                      */
/************************************************************************/
void WriteAux(void) {
  static struct NiosStruct* cpuFanAddr;
  static struct NiosStruct* cpuTimeAddr;
  static struct NiosStruct* diskFreeAddr;
  static struct NiosStruct* aliceFileAddr;
  static struct NiosStruct* timeoutAddr;
  static struct NiosStruct* cpuTemp1Addr;
  static struct NiosStruct* cpuTemp2Addr;
  static struct NiosStruct* cpuTemp3Addr;
  static struct NiosStruct* samIAmAddr;
  static struct BiPhaseStruct* samIAmReadAddr;
  static int incharge = -1;
  time_t t;
  int i_point;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    samIAmAddr = GetNiosAddr("sam_i_am");
    samIAmReadAddr = ExtractBiPhaseAddr(samIAmAddr);

    cpuFanAddr = GetNiosAddr("cpu_fan");
    cpuTemp1Addr = GetNiosAddr("cpu_temp1");
    cpuTemp2Addr = GetNiosAddr("cpu_temp2");
    cpuTemp3Addr = GetNiosAddr("cpu_temp3");
    cpuTimeAddr = GetNiosAddr("cpu_time");
    diskFreeAddr = GetNiosAddr("disk_free");
    aliceFileAddr = GetNiosAddr("alice_file");
    timeoutAddr = GetNiosAddr("timeout");
  }

  t = time(NULL);

  InCharge = !(SamIAm
      ^ slow_data[samIAmReadAddr->index][samIAmReadAddr->channel]);
  if (InCharge != incharge && InCharge)
    mputs(MCP_INFO, "I have gained control.\n");
  else if (InCharge != incharge)
    mputs(MCP_INFO, "I have lost control.\n");

  incharge = InCharge;

  WriteData(cpuTimeAddr, t >> 16);

  WriteData(cpuFanAddr, CommandData.fan);
  WriteData(cpuTemp1Addr, CommandData.temp1);
  WriteData(cpuTemp2Addr, CommandData.temp2);
  WriteData(cpuTemp3Addr, CommandData.temp3);

  WriteData(samIAmAddr, SamIAm);
  WriteData(diskFreeAddr, CommandData.df);

  i_point = GETREADINDEX(point_index);

  t = PointingData[i_point].t;

  WriteData(aliceFileAddr, CommandData.alice_file);
  WriteData(timeoutAddr, CommandData.pointing_mode.t - t);
}

/*****************************************************************/
/* SyncADC: check to see if any boards need to be synced and     */
/* send the sync bit if they do.  Only one board can be synced   */
/* in each superframe.                                           */
/*****************************************************************/
int ADC_sync_timeout = 0;
void SyncADC (int TxIndex) {
  static struct NiosStruct* syncAddr;
  static struct BiPhaseStruct* syncReadAddr;
  static struct BiPhaseStruct* statusAddr[17];
  static int doingSync = 0;
  unsigned int nextInd = 0;
  char buffer[9];

  int k, l;

  if (ADC_sync_timeout >= 600)
    return;

  ADC_sync_timeout++;
  
  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    syncAddr = GetNiosAddr("sync");
    syncReadAddr = ExtractBiPhaseAddr(syncAddr);

    nextInd = (syncReadAddr->index + 1) % FAST_PER_SLOW;
    for (k = 0; k < 17; ++k) {
      sprintf(buffer, "status%02i", k);
      statusAddr[k] = GetBiPhaseAddr(buffer);
    }
  }

  /* are we currently syncing? */
  if (doingSync) {
    /* if yes, turn off sync bit if we sent the sync last frame */
    if (TxIndex == nextInd) {
      RawNiosWrite(syncAddr->niosAddr, BBC_WRITE | BBC_NODE(17) | BBC_CH(56));
    }
  } else {
    /* if not, check to see if we need to sync a board */
    for (k = 0; k < 17; ++k) {
      /* read board status */
      if (slow_data[statusAddr[k]->index][statusAddr[k]->channel] == 0x0001) {
        /* board needs to be synced */
        doingSync = 1;
        mprintf(MCP_INFO, "ADC Sync board %i\n", k);
        l = (k == 0) ? 21 : k;
        RawNiosWrite(syncAddr->niosAddr, BBC_WRITE | BBC_NODE(l) | BBC_CH(56) |
          BBC_ADC_SYNC | 0xa5a3);
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
void StoreData(int index) {

  static int firsttime = 1;
  
  static struct NiosStruct* ssXCcdAddr;
  static struct NiosStruct* ssPrinAddr;
  static struct NiosStruct* sipLatAddr;
  static struct NiosStruct* sipLonAddr;
  static struct NiosStruct* sipAltAddr;
  static struct NiosStruct* sipTimeAddr;

  /** pointing mode indexes **/
  static struct NiosStruct* pModeAddr;
  static struct NiosStruct* pXDegAddr, *pYAddr;
  static struct NiosStruct* pVazAddr, *pDelAddr;
  static struct NiosStruct* pWAddr, *pHAddr;
  
  static struct NiosStruct* sensorVetoAddr;

  /** derived pointing data */
  static struct NiosStruct* mcpFrameAddr;
  static struct NiosStruct* gy1OffsetAddr;
  static struct NiosStruct* gy2OffsetAddr;
  static struct NiosStruct* gy3OffsetAddr;
  static struct NiosStruct* gyRollAmpAddr;
  static struct NiosStruct* azAddr;
  static struct NiosStruct* elAddr;
  static struct NiosStruct* raAddr;
  static struct NiosStruct* decAddr;
  static struct NiosStruct* latAddr;
  static struct NiosStruct* lonAddr;
  static struct NiosStruct* timeAddr;
  static struct NiosStruct* lstAddr;
  static struct NiosStruct* magAzAddr;
  static struct NiosStruct* magModelAddr;
  static struct NiosStruct* magSigmaAddr;
  static struct NiosStruct* dgpsAzAddr;
  static struct NiosStruct* dgpsPitchAddr;
  static struct NiosStruct* dgpsRollAddr;
  static struct NiosStruct* dgpsSigmaAddr;
  static struct NiosStruct* ssAzAddr;
  static struct NiosStruct* ssSigmaAddr;
  static struct NiosStruct* sunAzAddr;
  static struct NiosStruct* iscAzAddr;
  static struct NiosStruct* iscElAddr;
  static struct NiosStruct* iscSigmaAddr;
  static struct NiosStruct* encElAddr;
  static struct NiosStruct* encSigmaAddr;
  static struct NiosStruct* clinElAddr;
  static struct NiosStruct* clinSigmaAddr;

  /** dgps fields **/
  static struct NiosStruct* dgpsTimeAddr;
  static struct NiosStruct* dgpsLatAddr;  
  static struct NiosStruct* dgpsLonAddr;  
  static struct NiosStruct* dgpsAltAddr;  
  static struct NiosStruct* dgpsSpeedAddr;  
  static struct NiosStruct* dgpsDirAddr;  
  static struct NiosStruct* dgpsClimbAddr;
  static struct NiosStruct* dgpsAttOkAddr;
  static struct NiosStruct* dgpsAttIndexAddr;
  static struct NiosStruct* dgpsPosIndexAddr;
  static struct NiosStruct* dgpsNSatAddr;

  /** isc fields **/
  static struct NiosStruct* blob0XAddr;
  static struct NiosStruct* blob1XAddr;
  static struct NiosStruct* blob2XAddr;
  static struct NiosStruct* blob0YAddr;
  static struct NiosStruct* blob1YAddr;
  static struct NiosStruct* blob2YAddr;
  static struct NiosStruct* blob0FluxAddr;
  static struct NiosStruct* blob1FluxAddr;
  static struct NiosStruct* blob2FluxAddr;
  static struct NiosStruct* blob0SnAddr;
  static struct NiosStruct* blob1SnAddr;
  static struct NiosStruct* blob2SnAddr;
  static struct NiosStruct* iscErrorAddr;
  static struct NiosStruct* iscMapmeanAddr;
  static struct NiosStruct* iscFramenumAddr;
  static struct NiosStruct* iscRdSigmaAddr;
  static struct NiosStruct* iscRaAddr;
  static struct NiosStruct* iscDecAddr;
  static struct NiosStruct* iscAfocusAddr;
  static struct NiosStruct* iscMcpnumAddr;
  static struct NiosStruct* iscBrraAddr;
  static struct NiosStruct* iscBrdecAddr;
  static struct NiosStruct* iscApertAddr;
  static struct NiosStruct* iscCenboxAddr;
  static struct NiosStruct* iscApboxAddr;
  static struct NiosStruct* iscMdistAddr;
  static struct NiosStruct* iscNblobsAddr;
  static struct NiosStruct* iscFocusAddr;
  static struct NiosStruct* iscThreshAddr;
  static struct NiosStruct* iscGridAddr;
  static struct NiosStruct* iscStateAddr;
  static struct NiosStruct* iscMaxblobsAddr;
  static struct NiosStruct* iscMaglimitAddr;
  static struct NiosStruct* iscNradAddr;
  static struct NiosStruct* iscLradAddr;
  static struct NiosStruct* iscTolAddr;
  static struct NiosStruct* iscMtolAddr;
  static struct NiosStruct* iscQtolAddr;
  static struct NiosStruct* iscRtolAddr;
  static struct NiosStruct* iscFpulseAddr;
  static struct NiosStruct* iscSpulseAddr;
  static struct NiosStruct* iscXOffAddr;
  static struct NiosStruct* iscYOffAddr;
  static struct NiosStruct* iscHoldIAddr;
  static struct NiosStruct* iscSavePrdAddr;
  static struct NiosStruct* sipMksLoAddr;
  static struct NiosStruct* sipMksMedAddr;
  static struct NiosStruct* sipMksHiAddr;

  static int blob_index = 0;

  int i_ss;
  int i_point;
  int i_dgps;
  int sensor_veto;
  int i_isc = GETREADINDEX(iscdata_index);

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;	
    azAddr = GetNiosAddr("az");
    elAddr = GetNiosAddr("el");
    ssXCcdAddr = GetNiosAddr("ss_x_ccd");
    mcpFrameAddr = GetNiosAddr("mcp_frame");

    ssPrinAddr = GetNiosAddr("ss_prin");
    sipLatAddr = GetNiosAddr("sip_lat");
    sipLonAddr = GetNiosAddr("sip_lon");
    sipAltAddr = GetNiosAddr("sip_alt");
    sipTimeAddr = GetNiosAddr("sip_time");

    sipMksLoAddr = GetNiosAddr("sip_mks_lo");
    sipMksMedAddr = GetNiosAddr("sip_mks_med");
    sipMksHiAddr = GetNiosAddr("sip_mks_hi");

    gy1OffsetAddr = GetNiosAddr("gy1_offset");
    gy2OffsetAddr = GetNiosAddr("gy2_offset");
    gy3OffsetAddr = GetNiosAddr("gy3_offset");
    gyRollAmpAddr = GetNiosAddr("gy_roll_amp");
    raAddr = GetNiosAddr("ra");
    decAddr = GetNiosAddr("dec");
    latAddr = GetNiosAddr("lat");
    lonAddr = GetNiosAddr("lon");
    timeAddr = GetNiosAddr("time");
    lstAddr = GetNiosAddr("lst");
    magAzAddr = GetNiosAddr("mag_az");
    magModelAddr = GetNiosAddr("mag_model");
    magSigmaAddr = GetNiosAddr("mag_sigma");
    dgpsAzAddr = GetNiosAddr("dgps_az");
    dgpsPitchAddr = GetNiosAddr("dgps_pitch");
    dgpsRollAddr = GetNiosAddr("dgps_roll");
    dgpsSigmaAddr = GetNiosAddr("dgps_sigma");
    ssAzAddr = GetNiosAddr("ss_az");
    ssSigmaAddr = GetNiosAddr("ss_sigma");
    sunAzAddr = GetNiosAddr("sun_az");
    iscAzAddr = GetNiosAddr("isc_az");
    iscElAddr = GetNiosAddr("isc_el");
    iscSigmaAddr = GetNiosAddr("isc_sigma");
    encElAddr = GetNiosAddr("enc_el");
    encSigmaAddr = GetNiosAddr("enc_sigma");
    clinElAddr = GetNiosAddr("clin_el");
    clinSigmaAddr = GetNiosAddr("clin_sigma");

    pModeAddr = GetNiosAddr("p_mode");
    pXDegAddr = GetNiosAddr("p_x_deg");
    pYAddr = GetNiosAddr("p_y");
    pVazAddr = GetNiosAddr("p_vaz");
    pDelAddr = GetNiosAddr("p_del");
    pWAddr = GetNiosAddr("p_w");
    pHAddr = GetNiosAddr("p_h");

    sensorVetoAddr = GetNiosAddr("sensor_veto");

    dgpsTimeAddr = GetNiosAddr("dgps_time");
    dgpsLatAddr = GetNiosAddr("dgps_lat");
    dgpsLonAddr = GetNiosAddr("dgps_lon");
    dgpsAltAddr = GetNiosAddr("dgps_alt");
    dgpsSpeedAddr = GetNiosAddr("dgps_speed");
    dgpsDirAddr = GetNiosAddr("dgps_dir");
    dgpsClimbAddr = GetNiosAddr("dgps_climb");
    dgpsNSatAddr = GetNiosAddr("dgps_n_sat");
    dgpsPosIndexAddr = GetNiosAddr("dgps_pos_index");
    dgpsAttOkAddr = GetNiosAddr("dgps_att_ok");
    dgpsAttIndexAddr = GetNiosAddr("dgps_att_index");

    blob0XAddr = GetNiosAddr("blob0_x");
    blob1XAddr = GetNiosAddr("blob1_x");
    blob2XAddr = GetNiosAddr("blob2_x");
    blob0YAddr = GetNiosAddr("blob0_y");
    blob1YAddr = GetNiosAddr("blob1_y");
    blob2YAddr = GetNiosAddr("blob2_y");
    blob0FluxAddr = GetNiosAddr("blob0_flux");
    blob1FluxAddr = GetNiosAddr("blob1_flux");
    blob2FluxAddr = GetNiosAddr("blob2_flux");
    blob0SnAddr = GetNiosAddr("blob0_sn");
    blob1SnAddr = GetNiosAddr("blob1_sn");
    blob2SnAddr = GetNiosAddr("blob2_sn");
    iscErrorAddr = GetNiosAddr("isc_error");
    iscMapmeanAddr = GetNiosAddr("isc_mapmean");
    iscRdSigmaAddr = GetNiosAddr("isc_rd_sigma");
    iscFramenumAddr = GetNiosAddr("isc_framenum");
    iscRaAddr = GetNiosAddr("isc_ra");
    iscDecAddr = GetNiosAddr("isc_dec");
    iscNblobsAddr = GetNiosAddr("isc_nblobs");
    iscAfocusAddr = GetNiosAddr("isc_afocus");
    iscMcpnumAddr = GetNiosAddr("isc_mcpnum");

    iscStateAddr = GetNiosAddr("isc_state");
    iscFocusAddr = GetNiosAddr("isc_focus");
    iscBrraAddr = GetNiosAddr("isc_brra");
    iscBrdecAddr = GetNiosAddr("isc_brdec");
    iscApertAddr = GetNiosAddr("isc_apert");
    iscThreshAddr = GetNiosAddr("isc_thresh");
    iscGridAddr = GetNiosAddr("isc_grid");
    iscCenboxAddr = GetNiosAddr("isc_cenbox");
    iscApboxAddr = GetNiosAddr("isc_apbox");
    iscMdistAddr = GetNiosAddr("isc_mdist");
    iscMaxblobsAddr = GetNiosAddr("isc_maxblobs");
    iscMaglimitAddr = GetNiosAddr("isc_maglimit");
    iscNradAddr = GetNiosAddr("isc_nrad");
    iscLradAddr = GetNiosAddr("isc_lrad");
    iscTolAddr = GetNiosAddr("isc_tol");
    iscMtolAddr = GetNiosAddr("isc_mtol");
    iscQtolAddr = GetNiosAddr("isc_qtol");
    iscRtolAddr = GetNiosAddr("isc_rtol");
    iscFpulseAddr = GetNiosAddr("isc_fpulse");
    iscSpulseAddr = GetNiosAddr("isc_spulse");
    iscXOffAddr = GetNiosAddr("isc_x_off");
    iscYOffAddr = GetNiosAddr("isc_y_off");
    iscHoldIAddr = GetNiosAddr("isc_hold_i");
    iscSavePrdAddr = GetNiosAddr("isc_save_prd");
  }

  i_point = GETREADINDEX(point_index);
  i_ss = GETREADINDEX(ss_index);

  /********** Sun Sensor Data **********/
  WriteData(ssXCcdAddr, SunSensorData[i_ss].raw_az);
  WriteData(ssPrinAddr, SunSensorData[i_ss].prin);

  /********** SIP GPS Data **********/
  WriteData(sipLatAddr, (int)(SIPData.GPSpos.lat*DEG2I));
  WriteData(sipLonAddr, (int)(SIPData.GPSpos.lon*DEG2I));
  WriteData(sipAltAddr, (int)(SIPData.GPSpos.alt*0.25));
  WriteData(sipTimeAddr, SIPData.GPStime.UTC);

  /********** SIP MKS Altitude ************/
  WriteData(sipMksLoAddr, (int)(SIPData.MKSalt.lo * 0.25));
  WriteData(sipMksMedAddr, (int)(SIPData.MKSalt.med * 0.25));
  WriteData(sipMksHiAddr, (int)(SIPData.MKSalt.hi * 0.25));

  /************* processed pointing data *************/
  WriteData(azAddr, (unsigned int)(PointingData[i_point].az * 65536.0/360.0));
  WriteData(elAddr, (unsigned int)(PointingData[i_point].el * 65536.0/360.0));

  WriteData(raAddr, (unsigned int)(PointingData[i_point].ra * 65536.0/24.0));
  WriteData(decAddr, (unsigned int)(PointingData[i_point].dec * 65536.0/360.0));
	    
  WriteData(gy1OffsetAddr,
      (signed int)(PointingData[i_point].gy1_offset * 32768.));
  WriteData(gy2OffsetAddr,
      (signed int)(PointingData[i_point].gy2_offset * 32768.));
  WriteData(gy3OffsetAddr,
      (signed int)(PointingData[i_point].gy3_offset * 32768.));
  WriteData(gyRollAmpAddr,
      (unsigned int)(PointingData[i_point].gy_roll_amp * 65536.));

  WriteData(latAddr, (unsigned int)(PointingData[i_point].lat * DEG2I));
  WriteData(lonAddr, (unsigned int)(PointingData[i_point].lon * DEG2I));

  WriteData(mcpFrameAddr, PointingData[i_point].mcp_frame);
  WriteData(timeAddr, PointingData[i_point].t);
  WriteData(lstAddr, PointingData[i_point].lst);

  WriteData(magAzAddr,
      (unsigned int)(PointingData[i_point].mag_az * 65536.0/360.0));
  WriteData(magModelAddr,
      (unsigned int)(PointingData[i_point].mag_model * DEG2I));
  WriteData(magSigmaAddr,
      (unsigned int)(PointingData[i_point].mag_sigma * DEG2I));
  WriteData(dgpsAzAddr,
      (unsigned int)(PointingData[i_point].dgps_az * DEG2I));
  WriteData(dgpsPitchAddr,
      (unsigned int)(PointingData[i_point].dgps_pitch * DEG2I));
  WriteData(dgpsRollAddr,
      (unsigned int)(PointingData[i_point].dgps_roll * DEG2I));
  WriteData(dgpsSigmaAddr,
      (unsigned int)(PointingData[i_point].dgps_sigma * DEG2I));

  WriteData(ssAzAddr, (unsigned int)(PointingData[i_point].ss_az*DEG2I));
  WriteData(ssSigmaAddr,
      (unsigned int)(PointingData[i_point].ss_sigma * DEG2I));
  WriteData(sunAzAddr, (unsigned int)(PointingData[i_point].sun_az*DEG2I));

  WriteData(iscAzAddr,
      (unsigned int)(PointingData[i_point].isc_az * DEG2I));
  WriteData(iscElAddr,
      (unsigned int)(PointingData[i_point].isc_el * DEG2I));
  WriteData(iscSigmaAddr,
      (unsigned int)(PointingData[i_point].isc_sigma * DEG2I));

  WriteData(encElAddr,
      (unsigned int)(PointingData[i_point].enc_el * DEG2I));
  WriteData(encSigmaAddr,
      (unsigned int)(PointingData[i_point].enc_sigma * DEG2I));

  WriteData(clinElAddr,
      (unsigned int)(PointingData[i_point].clin_el * DEG2I));
  WriteData(clinSigmaAddr,
      (unsigned int)(PointingData[i_point].clin_sigma * DEG2I));

  /************* Pointing mode fields *************/
  WriteData(pModeAddr, (int)(CommandData.pointing_mode.mode));
  if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) ||
      (CommandData.pointing_mode.mode == P_AZ_SCAN)) {
    WriteData(pXDegAddr, (int)(CommandData.pointing_mode.X * DEG2I));
  } else {
    WriteData(pXDegAddr, (int)(CommandData.pointing_mode.X * H2I));
  }
  WriteData(pYAddr, (int)(CommandData.pointing_mode.Y * DEG2I));
  WriteData(pVazAddr, (int)(CommandData.pointing_mode.vaz * VEL2I));
  WriteData(pDelAddr, (int)(CommandData.pointing_mode.del * VEL2I));
  WriteData(pWAddr, (int)(CommandData.pointing_mode.w * DEG2I));
  WriteData(pHAddr, (int)(CommandData.pointing_mode.h * DEG2I));

  sensor_veto = (!CommandData.use_sun) | ((!CommandData.use_isc)<<1) |
    ((!CommandData.use_elenc)<<2) |
    ((!CommandData.use_mag)<<3) |
    ((!CommandData.use_gps)<<4) |
    ((!CommandData.use_elclin)<<5);

  if (PointingData[i_point].t >= CommandData.pointing_mode.t) {
    sensor_veto |= (1 << 6);
  }

  WriteData(sensorVetoAddr, sensor_veto);

  /************* dgps fields *************/
  WriteData(dgpsTimeAddr, DGPSTime);

  /** Pos fields **/
  i_dgps = GETREADINDEX(dgpspos_index);
  WriteData(dgpsLatAddr, (int)(DGPSPos[i_dgps].lat * DEG2I));
  WriteData(dgpsLonAddr, (int)(DGPSPos[i_dgps].lon * DEG2I));
  WriteData(dgpsAltAddr, (int)(DGPSPos[i_dgps].alt));
  WriteData(dgpsSpeedAddr, (int)(DGPSPos[i_dgps].speed * DEG2I));
  WriteData(dgpsDirAddr, (int)(DGPSPos[i_dgps].direction * DEG2I));
  WriteData(dgpsClimbAddr, (int)(DGPSPos[i_dgps].climb * DEG2I));
  WriteData(dgpsNSatAddr, DGPSPos[i_dgps].n_sat);
  WriteData(dgpsPosIndexAddr, i_dgps);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);
  WriteData(dgpsAttOkAddr, DGPSAtt[i_dgps].att_ok);
  WriteData(dgpsAttIndexAddr, i_dgps);

  /** ISC Fields **/
  if (index == 0)
    if (blob_index == 0)
      i_isc = GETREADINDEX(iscdata_index); 

  /*** Blobs ***/
  WriteData(blob0XAddr,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 0] * 40.));
  WriteData(blob1XAddr,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 1] * 40.));
  WriteData(blob2XAddr,
      (int)(ISCSolution[i_isc].blob_x[blob_index * 3 + 2] * 40.));

  WriteData(blob0YAddr,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 0] * 40.));
  WriteData(blob1YAddr,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 1] * 40.));
  WriteData(blob2YAddr,
      (int)(ISCSolution[i_isc].blob_y[blob_index * 3 + 2] * 40.));

  WriteData(blob0FluxAddr,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 0] / 32.));
  WriteData(blob1FluxAddr,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 1] / 32.));
  WriteData(blob2FluxAddr,
      (int)(ISCSolution[i_isc].blob_flux[blob_index * 3 + 2] / 32.));

  WriteData(blob0SnAddr,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 0] * 65.536));
  WriteData(blob1SnAddr,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 1] * 65.536));
  WriteData(blob2SnAddr,
      (int)(ISCSolution[i_isc].blob_sn[blob_index * 3 + 2] * 65.536));

  if (++blob_index >= 5)
    blob_index = 0;

  /*** Solution Info ***/
  WriteData(iscFramenumAddr, (unsigned int)ISCSolution[i_isc].framenum);
  WriteData(iscRaAddr, (unsigned int)(ISCSolution[i_isc].ra * RAD2LI));
  WriteData(iscDecAddr,
      (unsigned int)((ISCSolution[i_isc].dec + M_PI / 2) * 2. * RAD2LI));
  WriteData(iscNblobsAddr, (unsigned int)ISCSolution[i_isc].n_blobs);
  if (ISCSolution[i_isc].sigma * RAD2ARCSEC > 65535) {
    WriteData(iscRdSigmaAddr, 65535);
  } else {
    WriteData(iscRdSigmaAddr,
        (unsigned int)(ISCSolution[i_isc].sigma * RAD2ARCSEC));
  }
  WriteData(iscMcpnumAddr, (unsigned int)ISCSolution[i_isc].MCPFrameNum);
  WriteData(iscAfocusAddr, (unsigned int)ISCSolution[i_isc].autoFocusPosition);
  WriteData(iscErrorAddr, (unsigned int)ISCSolution[i_isc].cameraerr);
  WriteData(iscMapmeanAddr, (unsigned int)ISCSolution[i_isc].mapMean);

  /*** State Info ***/
  WriteData(iscStateAddr,
      (unsigned int)(SentState.pause * 2 + SentState.abort * 4 +
                     SentState.autofocus * 8 + SentState.brightStarMode * 16 +
                     SentState.shutdown * 32 + SentState.save));
  WriteData(iscFocusAddr, (unsigned int)SentState.focus_pos);
  WriteData(iscApertAddr, (unsigned int)SentState.ap_pos);
  WriteData(iscBrraAddr, (unsigned int)(SentState.brightRA * RAD2I));
  WriteData(iscBrdecAddr, (unsigned int)(SentState.brightDEC * RAD2I));
  WriteData(iscThreshAddr, (unsigned int)(SentState.sn_threshold * 10.));
  WriteData(iscGridAddr, (unsigned int)SentState.grid);
  WriteData(iscCenboxAddr, (unsigned int)SentState.cenbox);
  WriteData(iscApboxAddr, (unsigned int)SentState.apbox);
  WriteData(iscMdistAddr, (unsigned int)SentState.mult_dist);
  WriteData(iscMaxblobsAddr, (unsigned int)SentState.maxBlobMatch);
  WriteData(iscMaglimitAddr, (unsigned int)(SentState.mag_limit * 1000.));
  WriteData(iscNradAddr, (unsigned int)(SentState.norm_radius * RAD2I));
  WriteData(iscLradAddr, (unsigned int)(SentState.lost_radius * RAD2I));
  WriteData(iscTolAddr, (unsigned int)(SentState.tolerance * RAD2ARCSEC));
  WriteData(iscMtolAddr, (unsigned int)(SentState.match_tol * 65535.));
  WriteData(iscQtolAddr, (unsigned int)(SentState.quit_tol * 65535.));
  WriteData(iscRtolAddr, (unsigned int)(SentState.rot_tol * RAD2I));
  WriteData(iscXOffAddr, (unsigned int)(SentState.azBDA * RAD2I));
  WriteData(iscYOffAddr, (unsigned int)(SentState.elBDA * RAD2I));
  WriteData(iscHoldIAddr, (unsigned int)(SentState.hold_current));
  WriteData(iscFpulseAddr, (unsigned int)(CommandData.ISC_fast_pulse_width));
  WriteData(iscSpulseAddr, (unsigned int)(CommandData.ISC_pulse_width));
  WriteData(iscSavePrdAddr, (unsigned int)(CommandData.ISC_save_period));
}

void InitTxFrame(void)
{
  int bus, m, i, j, niosAddr, m0addr;

  mprintf(MCP_INFO, "Writing Initial Tx Frame.\n");

  for (bus = 0; bus < 2; ++bus) {
    for (m = 0; m < FAST_PER_SLOW; ++m) {
      for (i = 0; i < TxFrameWords[bus]; ++i) {
        niosAddr = i + bus * BBCPCI_MAX_FRAME_SIZE + m * TxFrameWords[bus];
        m0addr = i + bus * BBCPCI_MAX_FRAME_SIZE;
        if (i == 0) {  /* framesync */
          if (bus)
            RawNiosWrite(niosAddr, BBC_FSYNC | BBC_WRITE | BBC_NODE(63)
                | BBC_CH(4) | 0xB008);
          else
            RawNiosWrite(niosAddr, BBC_FSYNC | BBC_WRITE | BBC_NODE(63)
                | BBC_CH(0) | 0xEB90);
        } else if (i == 1 && bus == 0) /* fastsamp lsb */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(1));
        else if (i == 2 && bus == 0) /* fastsamp msb */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(2));
        else if (i == 3 && bus == 0) /* multiplex index */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(3) | m);
        else
          for (j = 0; j < ccTotal; ++j)
            if (NiosLookup[j].niosAddr == niosAddr)
              RawNiosWrite(niosAddr, NiosLookup[j].bbcAddr);
            else if (niosAddr == niosAddr - 1 && NiosLookup[j].wide)
              RawNiosWrite(niosAddr, BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr));
            else if (NiosLookup[j].fast && NiosLookup[j].niosAddr == m0addr)
              RawNiosWrite(niosAddr, NiosLookup[j].bbcAddr);
            else if (NiosLookup[j].fast && NiosLookup[j].niosAddr
                == m0addr - 1 && NiosLookup[j].wide)
              RawNiosWrite(niosAddr, BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr));

        for (j = 0; j < 2 * FAST_PER_SLOW; ++j)
          if (NiosSpares[j] == niosAddr)
            RawNiosWrite(niosAddr, BBCSpares[j]);

      }
    }
  }

}

void RawNiosWrite(unsigned int addr, unsigned int data)
{
  unsigned int niosData[2];

  niosData[0] = addr;
  niosData[1] = data;
  write(bbc_fp, niosData, 2 * sizeof(unsigned int));
}

void WriteData(struct NiosStruct* addr, unsigned int data)
{
  int i;

  if (addr->fast)
    for (i = 0; i < FAST_PER_SLOW; ++i) {
      RawNiosWrite(addr->niosAddr + i * TxFrameSize[addr->bus],
          addr->bbcAddr | (data & 0xffff));
      if (addr->wide)
        RawNiosWrite(addr->niosAddr + 1 + i * TxFrameSize[addr->bus],
            BBC_NEXT_CHANNEL(addr->bbcAddr) | (data >> 16));
    }
  else {
    /* slow data */
    RawNiosWrite(addr->niosAddr, addr->bbcAddr | (data & 0xffff));
    if (addr->wide)
      RawNiosWrite(addr->niosAddr + 1,
          BBC_NEXT_CHANNEL(addr->bbcAddr) | (data >> 16));
  }
}

void UpdateBBCFrame(unsigned short *RxFrame) {
  static int index = 0;

  /*** do Controls ***/
#ifndef BOLOTEST
  DoSched();
  UpdateAxesMode();
  StoreData(index);
  ControlGyroHeat(RxFrame);
  WriteMot(index, RxFrame);
#endif
  BiasControl(RxFrame);
  SyncADC(index);

  /*** do slow Controls ***/
  if (index == 0) {
    WriteAux();
    PhaseControl();
  }
#ifndef BOLOTEST
  ControlAuxMotors(RxFrame);
#endif
  CryoControl();
}
