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

extern struct ISCStatusStruct ISCSentState[2];  /* isc.c */
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

struct NiosStruct* GetSCNiosAddr(char* field, int which)
{
  char buffer[FIELD_LEN];
  sprintf(buffer, "%s_%s", which ? "osc" : "isc", field);

  return GetNiosAddr(buffer);
}

void StoreStarCameraData(int index, int which)
{
  static int firsttime[2] = {1, 1};

  static int blob_index[2] = {0, 0};
  int i_isc = GETREADINDEX(iscdata_index[which]);

  /** isc fields **/
  static struct NiosStruct* Blob0XAddr[2];
  static struct NiosStruct* Blob1XAddr[2];
  static struct NiosStruct* Blob2XAddr[2];
  static struct NiosStruct* Blob0YAddr[2];
  static struct NiosStruct* Blob1YAddr[2];
  static struct NiosStruct* Blob2YAddr[2];
  static struct NiosStruct* Blob0FluxAddr[2];
  static struct NiosStruct* Blob1FluxAddr[2];
  static struct NiosStruct* Blob2FluxAddr[2];
  static struct NiosStruct* Blob0SnAddr[2];
  static struct NiosStruct* Blob1SnAddr[2];
  static struct NiosStruct* Blob2SnAddr[2];
  static struct NiosStruct* ErrorAddr[2];
  static struct NiosStruct* MapmeanAddr[2];
  static struct NiosStruct* FramenumAddr[2];
  static struct NiosStruct* RdSigmaAddr[2];
  static struct NiosStruct* RaAddr[2];
  static struct NiosStruct* DecAddr[2];
  static struct NiosStruct* AfocusAddr[2];
  static struct NiosStruct* McpnumAddr[2];
  static struct NiosStruct* ApertAddr[2];
  static struct NiosStruct* CenboxAddr[2];
  static struct NiosStruct* ApboxAddr[2];
  static struct NiosStruct* MdistAddr[2];
  static struct NiosStruct* NblobsAddr[2];
  static struct NiosStruct* FocusAddr[2];
  static struct NiosStruct* ThreshAddr[2];
  static struct NiosStruct* GridAddr[2];
  static struct NiosStruct* StateAddr[2];
  static struct NiosStruct* MaxblobsAddr[2];
  static struct NiosStruct* MaglimitAddr[2];
  static struct NiosStruct* NradAddr[2];
  static struct NiosStruct* LradAddr[2];
  static struct NiosStruct* TolAddr[2];
  static struct NiosStruct* MtolAddr[2];
  static struct NiosStruct* QtolAddr[2];
  static struct NiosStruct* RtolAddr[2];
  static struct NiosStruct* FpulseAddr[2];
  static struct NiosStruct* SpulseAddr[2];
  static struct NiosStruct* XOffAddr[2];
  static struct NiosStruct* YOffAddr[2];
  static struct NiosStruct* HoldIAddr[2];
  static struct NiosStruct* SavePrdAddr[2];

  if (firsttime[which]) {
    Blob0XAddr[which] = GetSCNiosAddr("blob0_x", which);
    Blob1XAddr[which] = GetSCNiosAddr("blob1_x", which);
    Blob2XAddr[which] = GetSCNiosAddr("blob2_x", which);
    Blob0YAddr[which] = GetSCNiosAddr("blob0_y", which);
    Blob1YAddr[which] = GetSCNiosAddr("blob1_y", which);
    Blob2YAddr[which] = GetSCNiosAddr("blob2_y", which);
    Blob0FluxAddr[which] = GetSCNiosAddr("blob0_flx", which);
    Blob1FluxAddr[which] = GetSCNiosAddr("blob1_flx", which);
    Blob2FluxAddr[which] = GetSCNiosAddr("blob2_flx", which);
    Blob0SnAddr[which] = GetSCNiosAddr("blob0_sn", which);
    Blob1SnAddr[which] = GetSCNiosAddr("blob1_sn", which);
    Blob2SnAddr[which] = GetSCNiosAddr("blob2_sn", which);
    ErrorAddr[which] = GetSCNiosAddr("error", which);
    MapmeanAddr[which] = GetSCNiosAddr("mapmean", which);
    RdSigmaAddr[which] = GetSCNiosAddr("rd_sigma", which);
    FramenumAddr[which] = GetSCNiosAddr("framenum", which);
    RaAddr[which] = GetSCNiosAddr("ra", which);
    DecAddr[which] = GetSCNiosAddr("dec", which);
    NblobsAddr[which] = GetSCNiosAddr("nblobs", which);
    AfocusAddr[which] = GetSCNiosAddr("afocus", which);
    McpnumAddr[which] = GetSCNiosAddr("mcpnum", which);

    StateAddr[which] = GetSCNiosAddr("state", which);
    FocusAddr[which] = GetSCNiosAddr("focus", which);
    ApertAddr[which] = GetSCNiosAddr("apert", which);
    ThreshAddr[which] = GetSCNiosAddr("thresh", which);
    GridAddr[which] = GetSCNiosAddr("grid", which);
    CenboxAddr[which] = GetSCNiosAddr("cenbox", which);
    ApboxAddr[which] = GetSCNiosAddr("apbox", which);
    MdistAddr[which] = GetSCNiosAddr("mdist", which);
    MaxblobsAddr[which] = GetSCNiosAddr("maxblobs", which);
    MaglimitAddr[which] = GetSCNiosAddr("maglimit", which);
    NradAddr[which] = GetSCNiosAddr("nrad", which);
    LradAddr[which] = GetSCNiosAddr("lrad", which);
    TolAddr[which] = GetSCNiosAddr("tol", which);
    MtolAddr[which] = GetSCNiosAddr("mtol", which);
    QtolAddr[which] = GetSCNiosAddr("qtol", which);
    RtolAddr[which] = GetSCNiosAddr("rtol", which);
    FpulseAddr[which] = GetSCNiosAddr("fpulse", which);
    SpulseAddr[which] = GetSCNiosAddr("spulse", which);
    XOffAddr[which] = GetSCNiosAddr("x_off", which);
    YOffAddr[which] = GetSCNiosAddr("y_off", which);
    HoldIAddr[which] = GetSCNiosAddr("hold_i", which);
    SavePrdAddr[which] = GetSCNiosAddr("save_prd", which);
  }

  /** ISC Fields **/
  if (index == 0)
    if (blob_index[which] == 0)
      i_isc = GETREADINDEX(iscdata_index[which]); 

  /*** Blobs ***/
  WriteData(Blob0XAddr[which],
      (int)(ISCSolution[which][i_isc].blob_x[blob_index[which] * 3 + 0] * 40.));
  WriteData(Blob1XAddr[which],
      (int)(ISCSolution[which][i_isc].blob_x[blob_index[which] * 3 + 1] * 40.));
  WriteData(Blob2XAddr[which],
      (int)(ISCSolution[which][i_isc].blob_x[blob_index[which] * 3 + 2] * 40.));

  WriteData(Blob0YAddr[which],
      (int)(ISCSolution[which][i_isc].blob_y[blob_index[which] * 3 + 0] * 40.));
  WriteData(Blob1YAddr[which],
      (int)(ISCSolution[which][i_isc].blob_y[blob_index[which] * 3 + 1] * 40.));
  WriteData(Blob2YAddr[which],
      (int)(ISCSolution[which][i_isc].blob_y[blob_index[which] * 3 + 2] * 40.));

  WriteData(Blob0FluxAddr[which],
      (int)(ISCSolution[which][i_isc].blob_flux[blob_index[which] * 3 + 0]
            / 32.));
  WriteData(Blob1FluxAddr[which],
      (int)(ISCSolution[which][i_isc].blob_flux[blob_index[which] * 3 + 1]
            / 32.));
  WriteData(Blob2FluxAddr[which],
      (int)(ISCSolution[which][i_isc].blob_flux[blob_index[which] * 3 + 2]
            / 32.));

  WriteData(Blob0SnAddr[which],
      (int)(ISCSolution[which][i_isc].blob_sn[blob_index[which] * 3 + 0]
            * 65.536));
  WriteData(Blob1SnAddr[which],
      (int)(ISCSolution[which][i_isc].blob_sn[blob_index[which] * 3 + 1]
            * 65.536));
  WriteData(Blob2SnAddr[which],
      (int)(ISCSolution[which][i_isc].blob_sn[blob_index[which] * 3 + 2]
            * 65.536));

  if (++blob_index[which] >= 5)
    blob_index[which] = 0;

  /*** Solution Info ***/
  WriteData(FramenumAddr[which],
      (unsigned int)ISCSolution[which][i_isc].framenum);
  WriteData(RaAddr[which],
      (unsigned int)(ISCSolution[which][i_isc].ra * RAD2LI));
  WriteData(DecAddr[which],
      (unsigned int)((ISCSolution[which][i_isc].dec + M_PI / 2) * 2. * RAD2LI));
  WriteData(NblobsAddr[which], (unsigned int)ISCSolution[which][i_isc].n_blobs);

  if (ISCSolution[which][i_isc].sigma * RAD2ARCSEC > 65535)
    WriteData(RdSigmaAddr[which], 65535);
  else 
    WriteData(RdSigmaAddr[which],
        (unsigned int)(ISCSolution[which][i_isc].sigma * RAD2ARCSEC));

  WriteData(McpnumAddr[which],
      (unsigned int)ISCSolution[which][i_isc].MCPFrameNum);
  WriteData(AfocusAddr[which],
      (unsigned int)ISCSolution[which][i_isc].autoFocusPosition);
  WriteData(ErrorAddr[which],
      (unsigned int)ISCSolution[which][i_isc].cameraerr);
  WriteData(MapmeanAddr[which],
      (unsigned int)ISCSolution[which][i_isc].mapMean);

  /*** State Info ***/
  WriteData(StateAddr[which], (unsigned int)(ISCSentState[which].pause * 2
        + ISCSentState[which].abort * 4 + ISCSentState[which].autofocus * 8
        + ISCSentState[which].shutdown * 32 + ISCSentState[which].save));
  WriteData(FocusAddr[which], (unsigned int)ISCSentState[which].focus_pos);
  WriteData(ApertAddr[which], (unsigned int)ISCSentState[which].ap_pos);
  WriteData(ThreshAddr[which], (unsigned int)(ISCSentState[which].sn_threshold
        * 10.));
  WriteData(GridAddr[which], (unsigned int)ISCSentState[which].grid);
  WriteData(CenboxAddr[which], (unsigned int)ISCSentState[which].cenbox);
  WriteData(ApboxAddr[which], (unsigned int)ISCSentState[which].apbox);
  WriteData(MdistAddr[which], (unsigned int)ISCSentState[which].mult_dist);
  WriteData(MaxblobsAddr[which],
      (unsigned int)ISCSentState[which].maxBlobMatch);
  WriteData(MaglimitAddr[which], (unsigned int)(ISCSentState[which].mag_limit
        * 1000.));
  WriteData(NradAddr[which], (unsigned int)(ISCSentState[which].norm_radius
        * RAD2I));
  WriteData(LradAddr[which], (unsigned int)(ISCSentState[which].lost_radius
        * RAD2I));
  WriteData(TolAddr[which], (unsigned int)(ISCSentState[which].tolerance
        * RAD2ARCSEC));
  WriteData(MtolAddr[which], (unsigned int)(ISCSentState[which].match_tol
        * 65535.));
  WriteData(QtolAddr[which], (unsigned int)(ISCSentState[which].quit_tol
        * 65535.));
  WriteData(RtolAddr[which], (unsigned int)(ISCSentState[which].rot_tol
        * RAD2I));
  WriteData(XOffAddr[which], (unsigned int)(ISCSentState[which].azBDA * RAD2I));
  WriteData(YOffAddr[which], (unsigned int)(ISCSentState[which].elBDA * RAD2I));
  WriteData(HoldIAddr[which], (unsigned int)(ISCSentState[which].hold_current));
  WriteData(FpulseAddr[which],
      (unsigned int)(CommandData.ISCControl[which].fast_pulse_width));
  WriteData(SpulseAddr[which],
      (unsigned int)(CommandData.ISCControl[which].pulse_width));
  WriteData(SavePrdAddr[which],
      (unsigned int)(CommandData.ISCControl[which].save_period));
}

/************************************************************************/
/*                                                                      */
/*    Store derived acs and pointing data in frame                      */
/*                                                                      */
/************************************************************************/
void StoreData(int index)
{
  static int firsttime = 1;
  
  static struct NiosStruct* ssXCcdAddr;
  static struct NiosStruct* ssPrinAddr;
  static struct NiosStruct* sipLatAddr;
  static struct NiosStruct* sipLonAddr;
  static struct NiosStruct* sipAltAddr;
  static struct NiosStruct* sipTimeAddr;
  static struct NiosStruct* sipMksLoAddr;
  static struct NiosStruct* sipMksMedAddr;
  static struct NiosStruct* sipMksHiAddr;

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

  int i_ss;
  int i_point;
  int i_dgps;
  int sensor_veto;

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

  StoreStarCameraData(index, 0); /* write ISC data */
  StoreStarCameraData(index, 1); /* write OSC data */
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
