#include "compressstruct.h"
#include "compressconst.h"

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];

/* Defined in compressstruct.h - included here for reference.
struct fieldStreamStruct {
  char name[256];
  int gain; // x_down = x/gain
  int samples_per_frame; // samples per ~1Hz frame
  int doAverage; // boxcar average before decimation
  int doDifferentiate; // send down derivative, not value
  int bits; // number of bits in stream: 4, 8, 16, 32
  int spikeMode; // SLOW: skip samples  SPIKE: report enlarged data
};
*/
// el_dest_mc, i_el
struct fieldStreamStruct streamList[N_OTH_SETS][MAX_OTH_STREAM_FIELDS] = {
  { // field set 0
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},
  {"framenum",1,1,NOAVG,DX,8,SLOW},
  {"bi0_fifo_size", 1, 5, NOAVG, NODX, 16, SLOW},
  {"bbc_fifo_size", 1, 5, NOAVG, NODX, 16, SLOW},
  {"mce000", 1, 5, NOAVG, NODX, 16, SLOW},
  {"mce001", 1, 5, NOAVG, NODX, 16, SLOW},
  {"timeout_b",1,1,NOAVG,DX,8,SLOW},
  {"timeout_i",1,1,NOAVG,DX,8,SLOW},
  {"ofpch_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ofroll_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"az", 33140, 5, AVG, DX, 8, SLOW},
  {"el", 33140, 5, AVG, DX, 8, SLOW},
  {"ra", 16570, 1, NOAVG, DX, 8, SLOW},
  {"dec", 16570, 1, NOAVG, DX, 8, SLOW},
  {"el_raw_1_enc", 5, 1, AVG, DX, 8, SLOW},
  {"el_raw_2_enc", 5, 1, AVG, DX, 8, SLOW},
  {"az_pss", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_mag", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss1", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss2", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss3", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss4", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss5", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss6", 5, 1, NOAVG, DX, 8, SLOW},
  {"step_1_el", 3, 5, NOAVG, NODX, 8, SLOW},
  {"step_2_el", 3, 5, NOAVG, NODX, 8, SLOW},
  {"dac_rw", 256, 5, NOAVG, NODX, 8, SLOW},
  {"dac_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_el", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 5, NOAVG, NODX, 8, SLOW},
  {"term_i_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"term_p_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"term_p_t_rw_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"term_p_v_req_az_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"term_p_v_rw_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"res_piv", 1, 1, NOAVG, DX, 8, SLOW},
  {"vel_ser_rw", 1, 5, NOAVG, DX, 8, SLOW},
  {"pulse_sc", 1, 100, NOAVG, NODX, 8, SLOW},

  {"ofpch_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofpch_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  
END_OF_STREAM
  },
  
  {  // field set 1
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},

  {"ofpch_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofpch_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  
END_OF_STREAM
  }
};

char *frameList[] = {
#include "framelist.inc"
  ""
};

