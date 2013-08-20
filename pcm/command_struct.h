/* mcp: the Spider master control program
 *
 * This software is copyright (C) 2002-2013 University of Toronto
 *
 * This file is part of pcm.
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
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef COMMAND_STRUCT_H
#define COMMAND_STRUCT_H

#include <stdbool.h>  //bool type!
#include <stdint.h>
#include <time.h>
#include "mcp_sched.h"
#include "command_list.h"
#include "channels.h"

#define AXIS_VEL      0
#define AXIS_POSITION 1
#define AXIS_LOCK     2

/* az pointing modes */
#define P_AZEL_GOTO  1
#define P_AZ_SCAN    2
#define P_DRIFT      3
#define P_RADEC_GOTO 4
#define P_LOCK       5
#define P_SPIDER     6
#define P_SINE       7

/* el pointing modes */
#define P_EL_NONE    0
#define P_EL_GOTO    1
#define P_EL_RELMOVE 2

/* pivot _control_ modes */
#define P_PIV_VEL 0
#define P_PIV_TORQUE 1

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN 20
/* time (in slow frames) to suppress ADC card watchdog, to induce reset */
#define RESET_ADC_LEN 80

#define PREV_STATUS_FILE "/data/etc/spider/pcm.prev_status"

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
  unsigned short int PT; // position velocity gain
};

// used for pivot loop gains
struct PivGainStruct {
  /* gains for velocity mode */
  unsigned short int V_RW; // prop to RW velocity
  unsigned short int V_AZ; // prop to gondola az speed
  unsigned short int P_RW; // prop to RW position (integrated velocity)
  unsigned short int T_RW; // prop to gondola az accel
  unsigned short int V_REQ; // prop to gondoal az speed *request*
  /* gains for torque mode */
  unsigned short int PE; // prop to gondola az speed err
  unsigned short int PV; // prop to RW speed err
  double F; // current offset for stiction

  double SP; // RW velocity Set Point
};

#define ACTBUS_FM_SLEEP  0
#define ACTBUS_FM_SERVO  1
#define ACTBUS_FM_FOCUS  2
#define ACTBUS_FM_OFFSET 3
#define ACTBUS_FM_THERMO 4
#define ACTBUS_FM_NOW    5  //unused
#define ACTBUS_FM_DELTA  6
#define ACTBUS_FM_PANIC  7
#define ACTBUS_FM_DELFOC 8
#define ACTBUS_FM_TRIM   9

#define TC_MODE_ENABLED  0
#define TC_MODE_AUTOVETO 1
#define TC_MODE_VETOED   2

#define XYSTAGE_PANIC  0
#define XYSTAGE_GOTO   1
#define XYSTAGE_JUMP   2
#define XYSTAGE_SCAN   3
#define XYSTAGE_RASTER 4

#define HWPR_PANIC    0
#define HWPR_SLEEP    1
#define HWPR_GOTO     2
#define HWPR_JUMP     3
#define HWPR_STEP     4
#define HWPR_REPEAT   5
#define HWPR_GOTO_I   6
#define HWPR_GOTO_POT 7

// mode        X     Y    vaz   del    w    h
// LOCK              el
// AZEL_GOTO   az    el
// AZ_SCAN     az    el   vaz
// DRIFT                  vaz
// RADEC_GOTO  ra    dec
// VCAP*       ra    dec  vaz   vel    r
// CAP*        ra    dec  vaz   elstep r
// BOX*        ra    dec  vaz   elstep w    h
// SPIDER      ra    dec        elstep
// SINE        az    el                w

// *not used for Spider

struct PointingModeStruct {
  int nw; /* used for gy-offset veto during slews */
  int mode;
  int el_mode;
  double X;
  double Y;
  double vaz;
  double del;
  double d_el_p; // port distance for rel move
  double d_el_s; // starboard distance for rel move
  double v_el_p; // port speed for rel move
  double v_el_s; // starboard speed for rel move
  int el_rel_move; // 1 if new el rel move command sent
  double w;
  double h;
  time_t t;
  double ra[4]; // the RAs for radbox (ie, quad)
  double dec[4]; // the decs for radbox (ie, quad)
  // TODO: Write all those below out to frame
  int Nscans;  // number of half-scans per el step in SPIDER mode
  int Nsteps;  // total number of el steps in SPIDER mode
  int new_spider; // set to 1 if a new, distinct spider scan is commanded
  int new_sine;  // set to 1 if a new, distinct sine scan is commanded
  double overshoot_band; // width of turn around zone for Spider and Sine scans
  double el_step; // size of el microstep used by Spider scan.
                  // Is equal to del iff gondola is in Spider scan mode.
  unsigned short is_turn_around; // flag to indicate we're in a scan turn around
  double az_delay; // propagate az sol'n forward by this (frames)
  int piv_mode;
  //TODO: get rid of this before flight:
  int is_beam_map; // it's a beam map
};

struct latch_pulse {
  int set_count;
  int rst_count;
};

// for HK Theo heaters
// max power in Watts, TODO update command_list with these values
#define HK_MT_BOTTOM_PMAX   9.0
#define HK_SFT_LINES_PMAX   12.0
#define HK_CAPILLARY_PMAX   4.5
#define HK_VCS2_HX_PMAX     3.6
#define HK_VCS1_HX_PMAX     6.3
#define HK_MT_LINES_PMAX    9.0
#define HK_SFT_BOTTOM_PMAX  4.5

struct PWMStruct {
  bool state;                 // heater state (0=off, 1=on)
  int duration;               // seconds requested (-1=infinity)
  time_t start_time;          // start time of pulse mode
  unsigned short duty_target; // duty cycle target (8-bit resolution)
  unsigned short duty_avg;    // running average duty cycle
};

// for HK RTD's
struct RTDStruct {
  double phase;
  double ampl;
  bool do_phase_step;
  double phase_start;
  double phase_end;
  double phase_step;
  int phase_dt;
  time_t phase_time;
};

struct SCCommandData {
  //camera and lens configuration
  short int paused;  //1=image capture paused, 0=image capture running
  int expInt;        //exposure interval (ms) (0=triggered)
  int expTime;       //exposure duration (ms)
  int focusRes;      //steps to divide lens range into for focus
  int focusRng;      //inverse fraction of total focal range to go through for autofocus
  int moveTol;       //precision (ticks) for lens moves
  double platescale; //arcsec per pixel

  //image processing configuration
  int maxBlobs;      //max number of blobs to find
  int grid;          //search grid cell size (pix)
  double threshold;  //# sigma threshold for star finding
  int minBlobDist;   //min dist (pix) between blobs
};

struct TableStruct {
  struct GainStruct tableGain; //PID
  double vel;
  double pos;
  int mode;  //0=track, 1=move to pos, 2=relative move
};

/* sync box paramater type */
enum SyncParam {
  sync_rl,
  sync_fr,
  sync_nr,
  sync_none
};

/* MCE Power operations */
enum mce_pow_op {
  mce_pow_nop = 0, mce_pow_on, mce_pow_off, mce_pow_cyc, mce_pow_wait
};

struct CommandDataStruct {
  unsigned short command_count;
  unsigned short last_command;

  struct {
    unsigned short dac_out[5];
    unsigned char setLevel[5];
  } Temporary;

  unsigned short int timeout;
  unsigned short int slot_sched; // what slot to use
  unsigned short int upslot_sched; // slot being uplinked
  unsigned int parts_sched; // bitfield up pulinked parts
  unsigned short int uplink_sched; // use uplink sched
  unsigned short int sucks;
  unsigned short int lat_range;
  unsigned short int at_float;
  unsigned int pilot_bw;
  unsigned int tdrss_bw;
  unsigned int iridium_bw;
  unsigned int channelset_oth;
  unsigned int n_arrays_stats_per_superframe;
  unsigned short mce_param_index;

  //struct GainStruct ele_gain;

  struct {
    double com;
    double pulse_port;
    double pulse_starboard;
    int manual_pulses;
  } ele_gain;


  struct {
    enum SyncParam write_param;
    int cmd;
    int param_value;
    int rl_value;
    int nr_value;
    int fr_value;
  } sync_box;


  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  struct SCCommandData StarCam[3];
  int    rsc_wait;
  int    pyramid;
  double bsc_delay;
  struct TableStruct table;

  struct {
    struct latch_pulse sc_tx;
    struct latch_pulse das;
    struct latch_pulse rsc;
    struct latch_pulse bsc;
    struct latch_pulse gps;
    struct latch_pulse rw;
    struct latch_pulse piv;
    struct latch_pulse elmot;
    struct latch_pulse bi0;
    struct latch_pulse table;
    struct latch_pulse charge;
    struct latch_pulse ifcharge;
    struct latch_pulse mcc1;
    struct latch_pulse mcc2;
    struct latch_pulse mcc3;
    struct latch_pulse mcc4;
    struct latch_pulse mcc5;
    struct latch_pulse mcc6;
    struct latch_pulse sync;
    int pv_data2_145_off;
    int pv_data2_236_off;
    int pv_data3_136_off;
    int pv_data3_245_off;
    int gybox_off;
    int gyro_off[6];
    int gyro_off_auto[6];
    int hub232_off;
    int lock_off;
    unsigned char adc_reset[16];
    int elmot_auto; // automatically power el motors on/off before/after a move
    int elmot_is_on; // 1 if el drive on, 0 if not
  } power;

  struct {
    enum mce_pow_op mce_op[3];
    int mce_mpcveto[3];
    struct latch_pulse mce[3];
    struct latch_pulse hwp;
    struct latch_pulse sftv;
    int hk_preamp_off;
  } ifpower;
  
  unsigned short sft_pump;

  unsigned short disable_az;
  unsigned short disable_el;
  unsigned short force_el;

  unsigned short reset_rw;
  unsigned short reset_piv;
  unsigned short reset_elev;
  unsigned short restore_piv;
  unsigned short restore_rw;  // JAS--added this in, not sure if we need it yet

  unsigned short verbose_rw;
  unsigned short verbose_piv;
  int az_autogyro;
  int el_autogyro;
  double offset_ofpch_gy;
  double offset_ofroll_gy;
  double offset_ofyaw_gy;
  unsigned int gymask;

  double t_set[N_HEATERS];

  unsigned char use_elenc1;
  unsigned char use_elenc2;
  unsigned char use_pss;
  unsigned char use_mag;
  unsigned char use_gps;

  double dgps_cov_limit;
  double dgps_ants_limit;

  unsigned short fast_offset_gy;
  unsigned int slew_veto;

  double az_accel;
  double az_accel_max; // max gondola accel in az

  double null_az_trim;
  double mag_az_trim;
  double dgps_az_trim;
  double pss_az_trim;

  double twist_default;
  double twist_limit;

  double cal_xmax_mag;
  double cal_xmin_mag;
  double cal_ymax_mag;
  double cal_ymin_mag;

  double cal_off_pss1;
  double cal_off_pss2;
  double cal_off_pss3;
  double cal_off_pss4;
  double cal_off_pss5;
  double cal_off_pss6;

  double cal_d_pss1;
  double cal_d_pss2;
  double cal_d_pss3;
  double cal_d_pss4;
  double cal_d_pss5;
  double cal_d_pss6;

  double cal_imin_pss;

  struct {
    bool pump_heat;
    bool heat_switch;
    int fphi_heat;
    bool ssa_heat;
    bool htr1_heat;
    bool htr2_heat;
    bool htr3_heat;
    double fplo_heat;
    double ring_heat;

    bool cernox_full_bias;

    bool pump_servo_on;
    double pump_servo_low;
    double pump_servo_high;

    struct RTDStruct cernox;
    struct RTDStruct ntd;

    struct {
      bool enabled;           // autocycle enabled
      bool force;             // force a cycle

      // "normal" fridge cycle parameters
      double t_fp_warm;       // fp temp above which to start a cycle
      int hsw_timeout;        // time for heat switch transition (s)
      double t_pump_hi;       // upper limit of pump servo
      double t_pump_lo;       // lower limit of pump servo
      double t_fp_cool;       // fp temp below which to stop pump heat
      int pump_timeout;       // time for pump heating (s)
      int settle_time;        // wait after heating pump (s)
      double t_fp_cold;       // temperature below which to call cycle done
      int cool_timeout;       // timeout on waiting for cold (s)
    } cycle;
  } hk[6];    //one per insert

  struct {
    double p_sft_boil;        // power level for boiling off the sft
    double p_cap_boil;        // power level for boiling off the capillaries
    double t_empty_sft;       // sft temp above which it's deemed empty
    int boil_timeout;         // timeout on sft boiling (s)
    double t_fp_hi;           // upper fp temperature limit during boil/bake
    double t_fp_lo;           // lower fp temperature limit during boil/bake
    double t_sft_bake_hi;     // upper sft temperature limit during bake
    double t_sft_bake_lo;     // lower sft temperature limit during bake
    double t_cap_bake_hi;     // upper capillary temperature limit during bake
    double t_cap_bake_lo;     // lower capillary temperature limit during bake
    double t_mt_cool;         // min tank temp below which bake is finished
    int bake_timeout;         // time after which to terminate bake (s)
    int settle_time;          // time after burp before fridge cycle (s)
  } burp_cycle;

  struct PWMStruct hk_theo_heat[8];

  short hk_last;
  double hk_vheat_last;
  unsigned short hk_bias_freq;

  struct {
    enum {sft_do_nothing = 0, sft_do_open, sft_do_close} goal_atm, goal_pump;
    enum {sft_unknown = 0, sft_closed, sft_open, sft_closing,
      sft_opening} state_atm, state_pump;
  } sftv;

  struct {
    enum {lock_do_nothing = 0, lock_insert, lock_el_wait_insert,
      lock_retract} goal;
    enum {lock_unknown = 0, lock_closed, lock_open, lock_closing,
      lock_opening} state_p, state_s;
    int pin_is_in;
  } lock;

  struct {
    /* move parameters */
    double vel;    //velocity in dps, shared by all motors
    double move_i; //hold current in A, shared by all motors

    /* command parameters */
    double delta[6]; //degrees
    enum {hwp_m_sleep, hwp_m_panic, hwp_m_halt, hwp_m_rel_move,
      hwp_m_step} mode[6];
    int force_repoll;
    double phase;

    /* arbitrary command */
    int cindex;
    int caddr[3];
    char command[3][CMD_STRING_LEN];
    unsigned short int bias_mask;
  } hwp;

  struct {
    int x1, y1, x2, y2, step, xvel, yvel, is_new, mode;
    int force_repoll;
  } xystage;

  /* sensors output: read in mcp:SensorReader() */
  unsigned short temp1, temp2, temp3;
  size_t df;

  unsigned short plover;

  unsigned short bi0FifoSize;
  unsigned short bbcFifoSize;

  unsigned int bbcIntFrameRate;         //internal frame rate (ADC samples)
  unsigned int bbcExtFrameRate;         //external frame rate (sync frames)
  unsigned int bbcExtFrameMeas;         //measured external rate (32MHz)
  unsigned char bbcIsExt;               //is the bbc in external mode
  unsigned char bbcAutoExt;             //should pcm auto-switch ext/int mode

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
  double lat;
  double lon;

  unsigned short questionable_behaviour;

  uint16_t mce_blob_num; /* blob serial number */

  int bset_num; /* current bolo set number */

  /* MCE data mode 32->16 bit conversion stuff */
  int data_mode;
  char data_mode_bits[13][2][2];
  int data_mode_bits_serial;

  /* commands to be relayed to the MCE computers */
  int mcecmd_index;
  struct ScheduleEvent mcecmd[3];

  /* mcc watchdog enable */
  unsigned short mcc_wdog;
  int mccs_off; /* mccs which are off */

  /* squid vetoing */
  uint8_t squidveto; /* commandable veto */
  uint8_t thermveto; /* veto for thermal reasons */
  uint8_t mce_power; /* MCE power banks are off (will also veto MCEs) */
  int thermveto_veto; /* veto the vetoing */

  /* bolo stats control */
  double bolo_filt_freq;
  double bolo_filt_bw;
  int bolo_filt_len;
};

void InitCommandData();
double LockPosition(double);
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;

#endif   //COMMAND_STRUCT_H

