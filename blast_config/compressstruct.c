#include "compressstruct.h"

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
struct fieldStreamStruct streamList[] = {
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},
  {"framenum",1,1,NOAVG,DX,8,SLOW},
  {"count_n_cmd",1,1,NOAVG,DX,8,SLOW},  
  {"count_s_cmd",1,1,NOAVG,DX,8,SLOW},  
  {"ifel_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifroll_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"az", 33140, 5, AVG, DX, 8, SLOW},
  {"az_isc", 1, 5, NOAVG, DX, 8, SLOW},
  {"az_osc", 1, 5, NOAVG, DX, 8, SLOW},
  {"el", 33140, 5, AVG, DX, 8, SLOW},
  {"el_isc", 1, 5, NOAVG, DX, 8, SLOW},
  {"el_osc", 1, 5, NOAVG, DX, 8, SLOW},
  {"sigma_isc", 1, 5, NOAVG, NODX, 16, SLOW},
  {"sigma_osc", 1, 5, NOAVG, NODX, 16, SLOW},
  {"ra", 16570, 5, NOAVG, DX, 8, SLOW},
  {"dec", 16570, 5, NOAVG, DX, 8, SLOW},
  {"enc_hwpr", 1, 1, AVG, DX, 16, SLOW},
  {"el_enc", 5, 1, AVG, DX, 8, SLOW},
  {"el_raw_enc", 5, 1, AVG, DX, 8, SLOW},
  {"az_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_mag", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_mag", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_pss", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss", 5, 1, NOAVG, DX, 8, SLOW},
  {"res_piv", 1, 1, NOAVG, DX, 8, SLOW},
  {"vel_rw", 1, 5, NOAVG, DX, 8, SLOW},
  {"i_ser_el", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 5, NOAVG, NODX, 8, SLOW},
  {"n29c10", 12, 25, AVG, DX, 8, SLOW}, //
  {"n31c09", 12, 25, AVG, DX, 8, SLOW}, //
  {"n27c03", 12, 50, AVG, DX, 8, SLOW}, //
  {"n27c07", 12, 50, AVG, DX, 8, SLOW}, //
  {"n25c00", 12, 50, AVG, DX, 8, SLOW}, //
  {"n23c12", 12, 50, AVG, DX, 8, SLOW}, //
  {"n23c10", 12, 50, AVG, DX, 8, SLOW}, //
  {"n23c13", 12, 50, AVG, DX, 8, SLOW}, //
  {"n21c13", 12, 50, AVG, DX, 8, SLOW}, //
  
  {"i_term_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_term_el", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_term_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_err_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_rw_term_piv", 100, 5, NOAVG, NODX, 8, SLOW},
  
  {"dac_el", 256, 5, NOAVG, NODX, 8, SLOW},
  {"dac_rw", 256, 5, NOAVG, NODX, 8, SLOW},
  {"dac_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"ra_isc", 16570, 5, NOAVG, NODX, 16, SLOW},
  {"dec_isc", 16570, 5, NOAVG, NODX, 16, SLOW},
  {"mcpnum_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"framenum_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"ra_osc", 16570, 5, NOAVG, NODX, 16, SLOW},
  {"dec_osc", 16570, 5, NOAVG, NODX, 16, SLOW},
  {"mcpnum_osc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"framenum_osc", 1, 2, NOAVG, NODX, 16, SLOW},
//   {"pulse_sc", 1, 100, NOAVG, NODX, 8, SLOW},

  {"ifel_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ifroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ifel_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ifroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"el_raw_if_clin", 1, 5, AVG, DX, 8, SLOW},
  {"el_clin", 1, 5, AVG, DX, 8, SLOW},
  {"frict_term_piv", 100, 5, NOAVG, NODX, 8, SLOW},
  {"age_isc", 50, 1, NOAVG, NODX, 8, SLOW},
  {"age_osc", 50, 1, NOAVG, NODX, 8, SLOW},
  
  {"n29c00", 12, 25, AVG, DX, 8, SLOW},
  {"n29c01", 12, 25, AVG, DX, 8, SLOW},
  {"n29c02", 12, 25, AVG, DX, 8, SLOW},
  {"n29c03", 12, 25, AVG, DX, 8, SLOW},
  {"n29c04", 12, 25, AVG, DX, 8, SLOW},
  //{"n29c05", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n29c06", 12, 25, AVG, DX, 8, SLOW},
  {"n29c07", 12, 25, AVG, DX, 8, SLOW},
  {"n29c08", 12, 25, AVG, DX, 8, SLOW},
  {"n29c09", 12, 25, AVG, DX, 8, SLOW},
  {"n29c12", 12, 25, AVG, DX, 8, SLOW},
  {"n29c13", 12, 25, AVG, DX, 8, SLOW},
  {"n29c14", 12, 25, AVG, DX, 8, SLOW},
  {"n29c15", 12, 25, AVG, DX, 8, SLOW},
  {"n29c16", 12, 25, AVG, DX, 8, SLOW},
  {"n29c17", 12, 25, AVG, DX, 8, SLOW},
  {"n29c18", 12, 25, AVG, DX, 8, SLOW},
  {"n29c19", 12, 25, AVG, DX, 8, SLOW},
  {"n29c20", 12, 25, AVG, DX, 8, SLOW},
  {"n29c21", 12, 25, AVG, DX, 8, SLOW},
  //{"n29c22", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n29c23", 12, 25, AVG, DX, 8, SLOW},
  {"n31c01", 12, 25, AVG, DX, 8, SLOW},
  {"n31c02", 12, 25, AVG, DX, 8, SLOW},
  {"n31c03", 12, 25, AVG, DX, 8, SLOW},
  {"n31c04", 12, 25, AVG, DX, 8, SLOW},
  {"n31c05", 12, 25, AVG, DX, 8, SLOW},
  {"n31c06", 12, 25, AVG, DX, 8, SLOW},
  {"n31c07", 12, 25, AVG, DX, 8, SLOW},
  {"n31c08", 12, 25, AVG, DX, 8, SLOW},
  {"n31c10", 12, 25, AVG, DX, 8, SLOW},
  {"n31c11", 12, 25, AVG, DX, 8, SLOW},
  {"n31c13", 12, 25, AVG, DX, 8, SLOW},
  {"n31c14", 12, 25, AVG, DX, 8, SLOW},
  {"n31c15", 12, 25, AVG, DX, 8, SLOW},
  {"n31c16", 12, 25, AVG, DX, 8, SLOW},
  {"n31c17", 12, 25, AVG, DX, 8, SLOW},
  {"n31c18", 12, 25, AVG, DX, 8, SLOW},
  {"n31c19", 12, 25, AVG, DX, 8, SLOW},
  {"n31c20", 12, 25, AVG, DX, 8, SLOW},
  {"n31c21", 12, 25, AVG, DX, 8, SLOW},
  {"n31c23", 12, 25, AVG, DX, 8, SLOW},
  {"n25c01", 12, 25, AVG, DX, 8, SLOW},
  {"n25c02", 12, 25, AVG, DX, 8, SLOW},
  {"n25c03", 12, 25, AVG, DX, 8, SLOW},
  {"n25c04", 12, 25, AVG, DX, 8, SLOW},
  {"n25c05", 12, 25, AVG, DX, 8, SLOW},
  {"n25c06", 12, 25, AVG, DX, 8, SLOW},
  {"n25c07", 12, 25, AVG, DX, 8, SLOW},
  {"n25c08", 12, 25, AVG, DX, 8, SLOW},
  {"n25c09", 12, 25, AVG, DX, 8, SLOW},
  {"n25c10", 12, 25, AVG, DX, 8, SLOW},
  {"n25c11", 12, 25, AVG, DX, 8, SLOW},
  //{"n25c12", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n25c13", 12, 25, AVG, DX, 8, SLOW},
  {"n25c14", 12, 25, AVG, DX, 8, SLOW},
  {"n25c15", 12, 25, AVG, DX, 8, SLOW},
  {"n25c16", 12, 25, AVG, DX, 8, SLOW},
  {"n25c17", 12, 25, AVG, DX, 8, SLOW},
  {"n25c18", 12, 25, AVG, DX, 8, SLOW},
  {"n25c19", 12, 25, AVG, DX, 8, SLOW},
  {"n25c20", 12, 25, AVG, DX, 8, SLOW},
  //{"n25c22", 12, 25, AVG, DX, 8, SLOW}, // dead
  //{"n26c01", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n26c03", 12, 25, AVG, DX, 8, SLOW},
  {"n26c04", 12, 25, AVG, DX, 8, SLOW},
  {"n26c05", 12, 25, AVG, DX, 8, SLOW},
  {"n26c06", 12, 25, AVG, DX, 8, SLOW},
  //{"n26c07", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n26c08", 12, 25, AVG, DX, 8, SLOW},
  {"n26c09", 12, 25, AVG, DX, 8, SLOW},
  {"n26c10", 12, 25, AVG, DX, 8, SLOW},
  //{"n26c11", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n26c12", 12, 25, AVG, DX, 8, SLOW},
  {"n26c13", 12, 25, AVG, DX, 8, SLOW},
  {"n26c14", 12, 25, AVG, DX, 8, SLOW},
  {"n26c15", 12, 25, AVG, DX, 8, SLOW},
  {"n26c16", 12, 25, AVG, DX, 8, SLOW},
  {"n26c17", 12, 25, AVG, DX, 8, SLOW},
  {"n26c18", 12, 25, AVG, DX, 8, SLOW},
  {"n26c19", 12, 25, AVG, DX, 8, SLOW},
  {"n26c20", 12, 25, AVG, DX, 8, SLOW},
  {"n26c21", 12, 25, AVG, DX, 8, SLOW},
  {"n26c22", 12, 25, AVG, DX, 8, SLOW},
  {"n26c23", 12, 25, AVG, DX, 8, SLOW},
  {"n27c00", 12, 25, AVG, DX, 8, SLOW},
  {"n27c01", 12, 25, AVG, DX, 8, SLOW},
  {"n27c02", 12, 25, AVG, DX, 8, SLOW},
  {"n27c04", 12, 25, AVG, DX, 8, SLOW},
  {"n27c05", 12, 25, AVG, DX, 8, SLOW},
  {"n27c06", 12, 25, AVG, DX, 8, SLOW},
  {"n27c08", 12, 25, AVG, DX, 8, SLOW},
  {"n27c09", 12, 25 , AVG, DX, 8, SLOW},
  {"n27c10", 12, 25, AVG, DX, 8, SLOW},
  {"n27c11", 12, 25, AVG, DX, 8, SLOW},
  {"n27c12", 12, 25, AVG, DX, 8, SLOW},
  {"n27c13", 12, 25, AVG, DX, 8, SLOW},
  {"n27c14", 12, 25, AVG, DX, 8, SLOW},
  {"n27c15", 12, 25, AVG, DX, 8, SLOW},
  {"n27c16", 12, 25, AVG, DX, 8, SLOW},
  {"n27c17", 12, 25, AVG, DX, 8, SLOW},
  {"n27c18", 12, 25, AVG, DX, 8, SLOW},
  {"n27c19", 12, 25, AVG, DX, 8, SLOW},
  {"n27c20", 12, 25, AVG, DX, 8, SLOW},
  {"n27c21", 12, 25, AVG, DX, 8, SLOW},
  //{"n27c23", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n30c00", 12, 25, AVG, DX, 8, SLOW},
  {"n30c02", 12, 25, AVG, DX, 8, SLOW},
  {"n30c03", 12, 25, AVG, DX, 8, SLOW},
  {"n30c04", 12, 25, AVG, DX, 8, SLOW},
  {"n30c05", 12, 25, AVG, DX, 8, SLOW},
  {"n30c06", 12, 25, AVG, DX, 8, SLOW},
  {"n30c07", 12, 25, AVG, DX, 8, SLOW},
  {"n30c08", 12, 25, AVG, DX, 8, SLOW},
  {"n30c09", 12, 25, AVG, DX, 8, SLOW},
  {"n30c10", 12, 25, AVG, DX, 8, SLOW},
  {"n30c11", 12, 25, AVG, DX, 8, SLOW},
  {"n30c12", 12, 25, AVG, DX, 8, SLOW},
  {"n30c13", 12, 25, AVG, DX, 8, SLOW},
   //{"n30c14", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n30c15", 12, 25, AVG, DX, 8, SLOW},
  {"n30c16", 12, 25, AVG, DX, 8, SLOW},
  {"n30c17", 12, 25, AVG, DX, 8, SLOW},
  {"n30c18", 12, 25, AVG, DX, 8, SLOW},
  {"n30c19", 12, 25, AVG, DX, 8, SLOW},
  {"n30c20", 12, 25, AVG, DX, 8, SLOW},
  {"n30c21", 12, 25, AVG, DX, 8, SLOW},
  {"n30c22", 12, 25, AVG, DX, 8, SLOW},
  //{"n30c23", 12, 25, AVG, DX, 8, SLOW}, // dead
  {"n17c00", 12, 50, AVG, DX, 8, SLOW},
  {"n17c01", 12, 50, AVG, DX, 8, SLOW},
  {"n17c02", 12, 50, AVG, DX, 8, SLOW},
  {"n17c03", 12, 50, AVG, DX, 8, SLOW},
  {"n17c04", 12, 50, AVG, DX, 8, SLOW},
  {"n17c05", 12, 50, AVG, DX, 8, SLOW},
  {"n17c06", 12, 50, AVG, DX, 8, SLOW},
  {"n17c07", 12, 50, AVG, DX, 8, SLOW},
  {"n17c08", 12, 50, AVG, DX, 8, SLOW},
  {"n17c09", 12, 50, AVG, DX, 8, SLOW},
  {"n17c10", 12, 50, AVG, DX, 8, SLOW},
  {"n17c11", 12, 50, AVG, DX, 8, SLOW},
  {"n17c12", 12, 50, AVG, DX, 8, SLOW},
  {"n17c13", 12, 50, AVG, DX, 8, SLOW},
  {"n17c14", 12, 50, AVG, DX, 8, SLOW},
  {"n17c15", 12, 50, AVG, DX, 8, SLOW},
  {"n17c16", 12, 50, AVG, DX, 8, SLOW},
  {"n17c17", 12, 50, AVG, DX, 8, SLOW},
  {"n17c18", 12, 50, AVG, DX, 8, SLOW},
  {"n17c19", 12, 50, AVG, DX, 8, SLOW},
  {"n17c20", 12, 50, AVG, DX, 8, SLOW},
  {"n17c22", 12, 50, AVG, DX, 8, SLOW},
  {"n17c23", 12, 50, AVG, DX, 8, SLOW},
  {"n18c00", 12, 50, AVG, DX, 8, SLOW},
  {"n18c01", 12, 50, AVG, DX, 8, SLOW},
  {"n18c03", 12, 50, AVG, DX, 8, SLOW},
  {"n18c04", 12, 50, AVG, DX, 8, SLOW},
  {"n18c05", 12, 50, AVG, DX, 8, SLOW},
  {"n18c06", 12, 50, AVG, DX, 8, SLOW},
  {"n18c07", 12, 50, AVG, DX, 8, SLOW},
  {"n18c08", 12, 50, AVG, DX, 8, SLOW},
  {"n18c09", 12, 50, AVG, DX, 8, SLOW},
  {"n18c10", 12, 50, AVG, DX, 8, SLOW},
  {"n18c11", 12, 50, AVG, DX, 8, SLOW},
  {"n18c12", 12, 50, AVG, DX, 8, SLOW},
  {"n18c13", 12, 50, AVG, DX, 8, SLOW},
  {"n18c14", 12, 50, AVG, DX, 8, SLOW},
  {"n18c15", 12, 50, AVG, DX, 8, SLOW},
  {"n18c16", 12, 50, AVG, DX, 8, SLOW},
  {"n18c17", 12, 50, AVG, DX, 8, SLOW},
  {"n18c18", 12, 50, AVG, DX, 8, SLOW},
  {"n18c19", 12, 50, AVG, DX, 8, SLOW},
  {"n18c20", 12, 50, AVG, DX, 8, SLOW},
  {"n18c21", 12, 50, AVG, DX, 8, SLOW},
  {"n18c22", 12, 50, AVG, DX, 8, SLOW},
  {"n18c23", 12, 50, AVG, DX, 8, SLOW},
  {"n19c00", 12, 50, AVG, DX, 8, SLOW},
  {"n19c01", 12, 50, AVG, DX, 8, SLOW},
  {"n19c02", 12, 50, AVG, DX, 8, SLOW},
  {"n19c03", 12, 50, AVG, DX, 8, SLOW},
  {"n19c04", 12, 50, AVG, DX, 8, SLOW},
  {"n19c05", 12, 50, AVG, DX, 8, SLOW},
  {"n19c06", 12, 50, AVG, DX, 8, SLOW},
  {"n19c07", 12, 50, AVG, DX, 8, SLOW},
  {"n19c08", 12, 50, AVG, DX, 8, SLOW},
  {"n19c09", 12, 50, AVG, DX, 8, SLOW},
  //{"n19c10", 12, 50, AVG, DX, 8, SLOW}, // dead
  {"n19c11", 12, 50, AVG, DX, 8, SLOW},
  {"n19c12", 12, 50, AVG, DX, 8, SLOW},
  {"n19c13", 12, 50, AVG, DX, 8, SLOW},
  {"n19c14", 12, 50, AVG, DX, 8, SLOW},
  {"n19c15", 12, 50, AVG, DX, 8, SLOW},
  {"n19c16", 12, 50, AVG, DX, 8, SLOW},
  {"n19c17", 12, 50, AVG, DX, 8, SLOW},
  {"n19c18", 12, 50, AVG, DX, 8, SLOW},
  {"n19c19", 12, 50, AVG, DX, 8, SLOW},
  {"n19c20", 12, 50, AVG, DX, 8, SLOW},
  {"n19c21", 12, 50, AVG, DX, 8, SLOW},
  {"n19c23", 12, 50, AVG, DX, 8, SLOW},
  {"n21c00", 12, 50, AVG, DX, 8, SLOW},
  {"n21c01", 12, 50, AVG, DX, 8, SLOW},
  {"n21c02", 12, 50, AVG, DX, 8, SLOW},
  {"n21c03", 12, 50, AVG, DX, 8, SLOW},
  {"n21c04", 12, 50, AVG, DX, 8, SLOW},
  {"n21c05", 12, 50, AVG, DX, 8, SLOW},
  {"n21c06", 12, 50, AVG, DX, 8, SLOW},
  {"n21c07", 12, 50, AVG, DX, 8, SLOW},
  {"n21c08", 12, 50, AVG, DX, 8, SLOW},
  {"n21c09", 12, 50, AVG, DX, 8, SLOW},
  {"n21c10", 12, 50, AVG, DX, 8, SLOW},
  {"n21c11", 12, 50, AVG, DX, 8, SLOW},
  {"n21c12", 12, 50, AVG, DX, 8, SLOW},
  {"n21c14", 12, 50, AVG, DX, 8, SLOW},
  {"n21c15", 12, 50, AVG, DX, 8, SLOW},
  {"n21c16", 12, 50, AVG, DX, 8, SLOW},
  {"n21c17", 12, 50, AVG, DX, 8, SLOW},
  {"n21c18", 12, 50, AVG, DX, 8, SLOW},
  {"n21c19", 12, 50, AVG, DX, 8, SLOW},
  {"n21c20", 12, 50, AVG, DX, 8, SLOW},
  {"n21c21", 12, 50, AVG, DX, 8, SLOW},
  {"n21c22", 12, 50, AVG, DX, 8, SLOW},
  {"n21c23", 12, 50, AVG, DX, 8, SLOW},
  {"n22c01", 12, 50, AVG, DX, 8, SLOW},
  {"n22c03", 12, 50, AVG, DX, 8, SLOW},
  {"n22c04", 12, 50, AVG, DX, 8, SLOW},
  {"n22c05", 12, 50, AVG, DX, 8, SLOW},
  {"n22c06", 12, 50, AVG, DX, 8, SLOW},
  {"n22c07", 12, 50, AVG, DX, 8, SLOW},
  {"n22c08", 12, 50, AVG, DX, 8, SLOW},
  {"n22c09", 12, 50, AVG, DX, 8, SLOW},
  {"n22c10", 12, 50, AVG, DX, 8, SLOW},
  {"n22c11", 12, 50, AVG, DX, 8, SLOW},
  {"n22c12", 12, 50, AVG, DX, 8, SLOW},
  {"n22c13", 12, 50, AVG, DX, 8, SLOW},
  {"n22c14", 12, 50, AVG, DX, 8, SLOW},
  {"n22c15", 12, 50, AVG, DX, 8, SLOW},
  {"n22c16", 12, 50, AVG, DX, 8, SLOW},
  {"n22c17", 12, 50, AVG, DX, 8, SLOW},
  {"n22c18", 12, 50, AVG, DX, 8, SLOW},
  {"n22c19", 12, 50, AVG, DX, 8, SLOW},
  {"n22c20", 12, 50, AVG, DX, 8, SLOW},
  {"n22c21", 12, 50, AVG, DX, 8, SLOW},
  {"n22c22", 12, 50, AVG, DX, 8, SLOW},
  {"n22c23", 12, 50, AVG, DX, 8, SLOW},
  {"n23c00", 12, 50, AVG, DX, 8, SLOW},
  {"n23c01", 12, 50, AVG, DX, 8, SLOW},
  {"n23c02", 12, 50, AVG, DX, 8, SLOW},
  {"n23c03", 12, 50, AVG, DX, 8, SLOW},
  {"n23c04", 12, 50, AVG, DX, 8, SLOW},
  {"n23c05", 12, 50, AVG, DX, 8, SLOW},
  {"n23c06", 12, 50, AVG, DX, 8, SLOW},
  {"n23c07", 12, 50, AVG, DX, 8, SLOW},
  {"n23c08", 12, 50, AVG, DX, 8, SLOW},
  {"n23c09", 12, 50, AVG, DX, 8, SLOW},
  {"n23c11", 12, 50, AVG, DX, 8, SLOW},
  {"n23c14", 12, 50, AVG, DX, 8, SLOW},
  {"n23c15", 12, 50, AVG, DX, 8, SLOW},
  {"n23c16", 12, 50, AVG, DX, 8, SLOW},
  {"n23c17", 12, 50, AVG, DX, 8, SLOW},
  {"n23c18", 12, 50, AVG, DX, 8, SLOW},
  {"n23c19", 12, 50, AVG, DX, 8, SLOW},
  {"n23c20", 12, 50, AVG, DX, 8, SLOW},
  {"n23c21", 12, 50, AVG, DX, 8, SLOW},
  {"n23c22", 12, 50, AVG, DX, 8, SLOW},
  {"n23c23", 12, 50, AVG, DX, 8, SLOW},
  
  {"n29c11", 1, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n31c00", 1, 25, AVG, DX, 8, SLOW}, // resistor
  {"n31c12", 1, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n31c22", 1, 25, AVG, DX, 8, SLOW}, // dark
  {"n25c21", 1, 25, AVG, DX, 8, SLOW}, // dark
  {"n25c23", 1, 25, AVG, DX, 8, SLOW}, // resistor
  {"n26c00", 1, 25, AVG, DX, 8, SLOW}, // resistor
  {"n26c02", 1, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n27c22", 1, 25, AVG, DX, 8, SLOW}, // dark
  {"n30c01", 1, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n17c21", 1, 50, AVG, DX, 8, SLOW}, // dark
  {"n18c02", 1, 50, AVG, DX, 8, SLOW}, // thermistor
  {"n19c22", 1, 50, AVG, DX, 8, SLOW}, // dark
  {"n22c00", 1, 50, AVG, DX, 8, SLOW},// Resistor
  {"n22c02", 1, 50, AVG, DX, 8, SLOW}, // thermistor

END_OF_STREAM
};


char *frameList[] = {
"acc_act",
"accel_az",
"acc_hwpr",
"acc_lock",
"age_sf",
"alarm_hi_cc1",
"alarm_lo_cc1",
"alarm_hi_cc2",
"alarm_lo_cc2",
"alt",
"alt_dgps",
"alt_sip",
"ampl_250_bias",
"ampl_350_bias",
"ampl_500_bias",
"ampl_rox_bias",
"ampl_x_bias",
"apert_isc",
"apert_osc",
"att_ok_dgps",
"az_gy",
"az_sun",
"bbc_fifo_size",
"bi0_fifo_size",
"bits_bal",
"bits_vtx",
"blob00_f_isc",
"blob00_f_osc",
"blob00_f_sbsc",
"blob00_s_isc",
"blob00_s_osc",
"blob00_s_sbsc",
"blob00_x_isc",
"blob00_x_osc",
"blob00_x_sbsc",
"blob00_y_isc",
"blob00_y_osc",
"blob00_y_sbsc",
"blob01_f_isc",
"blob01_f_osc",
"blob01_f_sbsc",
"blob01_s_isc",
"blob01_s_osc",
"blob01_s_sbsc",
"blob01_x_isc",
"blob01_x_osc",
"blob01_x_sbsc",
"blob01_y_isc",
"blob01_y_osc",
"blob01_y_sbsc",
"blob02_f_isc",
"blob02_f_osc",
"blob02_f_sbsc",
"blob02_s_isc",
"blob02_s_osc",
"blob02_s_sbsc",
"blob02_x_isc",
"blob02_x_osc",
"blob02_x_sbsc",
"blob02_y_isc",
"blob02_y_osc",
"blob02_y_sbsc",
"blob_idx_isc",
"blob_idx_osc",
"blob_idx_sbsc",
"brdec_isc",
"brdec_osc",
"brra_isc",
"brra_osc",
"bus_reset_act",
"cal_xmax_mag",
"cal_xmin_mag",
"cal_ymax_mag",
"cal_ymin_mag",
"ccd_t_sbsc",
"chopper",
"climb_dgps",
"correction_sf",
"cos_el",
"cryostate",
"dac2_ampl",
"daz_p",
"dec_1_p",
"dec_2_p",
"dec_3_p",
"dec_4_p",
"declination_mag",
"dec_sbsc",
"delay_isc",
"delay_osc",
"delay_sbsc",
"del_p",
"dest_az_mc",
"dest_el_mc",
"df_n_flc",
"df_s_flc",
"dig21_das",
"dig43_das",
"dig65_das",
"dir_az_mc",
"dir_dgps",
"dir_el_mc",
"diskfree_isc",
"diskfree_osc",
"dith_el",
"dr_0_act",
"dr_1_act",
"dr_2_act",
"drive_err_cts_el",
"drive_err_cts_piv",
"drive_err_cts_rw",
"drive_info_el",
"drive_info_piv",
"drive_info_rw",
"el_lut_clin",
"el_sun",
"enc_0_act",
"enc_1_act",
"enc_2_act",
"enc_err_hwpr",
"enc_targ_hwpr",
"error_az",
"error_el",
"error_isc",
"error_osc",
"exp_int_sbsc",
"exposure_isc",
"exposure_osc",
"exp_time_sbsc",
"fault_cc1",
"fault_cc2",
"fault_el",
"fault_gy",
"fault_rw",
"fieldrot_isc",
"fieldrot_osc",
"flags_act",
"foc_off_isc",
"foc_off_osc",
"focpos_sbsc",
"foc_res_sbsc",
"focus_isc",
"focus_osc",
"focus_sf",
"force_sbsc",
"fpulse_isc",
"fpulse_osc",
"frame_sbsc",
"frict_off_piv",
"gain_bal",
"gain_isc",
"gain_osc",
"g_d_heat_gy",
"g_i_az",
"g_i_el",
"g_i_heat_gy",
"goal_0_act",
"goal_1_act",
"goal_2_act",
"goal_lock",
"goal_sf",
"g_p_az",
"g_p_el",
"g_pe_piv",
"g_p_heat_gy",
"g_prime_sf",
"g_pt_az",
"g_pt_el",
"g_pv_piv",
"grid_isc",
"grid_osc",
"grid_sbsc",
"g_second_sf",
"h_age_gy",
"he4_lev",
"he4_lev_old",
"heat_gy",
"h_hist_gy",
"h_p",
"hx_flag_isc",
"hx_flag_osc",
"i_300mk",
"i_acs",
"i_arr_cc1",
"i_batt_cc1",
"i_arr_cc2",
"i_batt_cc2",
"i_cal_lamp",
"i_charcoal",
"i_das",
"i_dith_el",
"i_el",
"i_flc",
"ifpm_hall",
"i_gy",
"i_hold_act",
"i_hold_hwpr",
"i_hold_isc",
"i_hold_lock",
"i_hold_osc",
"i_hs_char",
"i_hs_pot",
"i_jfet",
"i_move_act",
"i_move_hwpr",
"i_move_lock",
"i_piv",
"i_pos_hwpr",
"i_pos_rq_hwpr",
"i_rec",
"i_rw",
"i_sbsc",
"i_sc",
"i_step",
"i_tot",
"i_trans",
"jfet_set_off",
"jfet_set_on",
"last_n_cmd",
"last_s_cmd",
"lat",
"latch0",
"latch1",
"lat_dgps",
"lat_sip",
"led_cc1",
"led_cc2",
"level_off_bal",
"level_on_bal",
"level_target_bal",
"lon",
"lon_dgps",
"lon_sip",
"lrad_isc",
"lrad_osc",
"lst",
"lst_sched",
"lvdt_0_act",
"lvdt_1_act",
"lvdt_2_act",
"lvdt_63_act",
"lvdt_64_act",
"lvdt_65_act",
"lvdt_high_act",
"lvdt_low_act",
"lvdt_spread_act",
"maglimit_isc",
"maglimit_osc",
"mapmean_isc",
"mapmean_osc",
"mapmean_sbsc",
"mapsigma_sbsc",
"mask_gy",
"max_age_isc",
"max_age_osc",
"maxblob_sbsc",
"maxblobs_isc",
"maxblobs_osc",
"maxslew_isc",
"maxslew_osc",
"mcp_frame",
"mdist_isc",
"mdist_osc",
"mdist_sbsc",
"minblobs_isc",
"minblobs_osc",
"mks_hi_sip",
"mks_lo_sip",
"mks_med_sip",
"mode_act",
"mode_az_mc",
"mode_bal",
"mode_cal",
"mode_el_mc",
"mode_p",
"mode_sf",
"move_tol_sbsc",
"mtol_isc",
"mtol_osc",
"n13_phase",
"n17_phase",
"n18_phase",
"n19_phase",
"n21_phase",
"n22_phase",
"n23_phase",
"n25_phase",
"n26_phase",
"n27_phase",
"n29_phase",
"n30_phase",
"n31_phase",
"nblobs_isc",
"nblobs_osc",
"nblobs_sbsc",
"n_dith_p",
"next_i_dith_p",
"next_i_hwpr_p",
"nrad_isc",
"nrad_osc",
"n_sat_dgps",
"off_ifel_gy_isc",
"off_ifel_gy_osc",
"off_ifroll_gy_isc",
"off_ifroll_gy_osc",
"off_ifyaw_gy_isc",
"off_ifyaw_gy_osc",
"offset_0_act",
"offset_1_act",
"offset_2_act",
"offset_ifel_gy",
"offset_ifrollgps_gy",
"offset_ifroll_gy",
"offset_ifrollmag_gy",
"offset_ifrollpss_gy",
"offset_ifyawdgps_gy",
"offset_ifyaw_gy",
"offset_ifyawmag_gy",
"offset_ifyawpss_gy",
"offset_isc",
"offset_osc",
"offset_sf",
"out_shutter",
"overshoot_hwpr",
"parts_sched",
"pch_pyr_clin",
"period_cal",
"pin_in_lock",
"pitch_mag",
"pitch_raw_dgps",
"plover",
"pos_0_act",
"pos0_hwpr",
"pos_1_act",
"pos1_hwpr",
"pos_2_act",
"pos2_hwpr",
"pos3_hwpr",
"pos_focus_isc",
"pos_focus_osc",
"pos_hwpr",
"pos_lock",
"pos_shutter",
"pot_err_hwpr",
"pot_hwpr",
"pot_lock",
"pot_raw_hwpr",
"pot_ref_hwpr",
"pot_targ_hwpr",
"pref_tp_sf",
"pref_ts_sf",
"pressure1_isc",
"pressure1_osc",
"p_term_el",
"pulse_cal",
"qtol_isc",
"qtol_osc",
"ra_1_p",
"ra_2_p",
"ra_3_p",
"ra_4_p",
"ramp_ampl_bias",
"ramp_ena_bias",
"ra_sbsc",
"rate_atrim",
"rate_iridium",
"rate_tdrss",
"rd_sigma_isc",
"rd_sigma_osc",
"read_wait_hwpr",
"real_trig_isc",
"real_trig_osc",
"rel_move_hwpr",
"roll_pyr_clin",
"roll_raw_dgps",
"rtol_isc",
"rtol_osc",
"save_prd_isc",
"save_prd_osc",
"sec_sbsc",
"seized_act",
"set_rw",
"sigma_clin",
"sigma_dgps",
"sigma_enc",
"sigma_mag",
"sigma_pss",
"sin_el",
"slew_veto",
"snr_pss1",
"snr_pss2",
"snr_pss3",
"snr_pss4",
"speed_dgps",
"spread_sf",
"spulse_isc",
"spulse_osc",
"start_cycle",
"start_set_cycle",
"stat_1_el",
"stat_1_rw",
"stat_2_el",
"stat_2_rw",
"stat_control_hwpr",
"stat_dr_piv",
"state_cc1",
"state_cc2",
"state_cycle",
"state_isc",
"state_lock",
"state_osc",
"stat_s1_piv",
"status_actbus",
"status_eth",
"status_mcc",
"step_array_bias",
"step_ena_bias",
"step_ena_phase",
"step_end_bias",
"step_end_phase",
"step_n_bias",
"step_nsteps_phase",
"step_pul_len_bias",
"step_sf",
"steps_shutter",
"steps_slow_shutter",
"step_start_bias",
"step_start_phase",
"step_time_bias",
"step_time_phase",
"stop_cnt_hwpr",
"sveto_len",
"switch_gy",
"switch_misc",
"t_1_second",
"t_2_second",
"t_acs",
"t_box_bal",
"t_char_max_cycle",
"t_char_set_cycle",
"t_chip_flc",
"t_comp_isc",
"t_comp_osc",
"t_cpu_n_flc",
"t_cpu_s_flc",
"t_dcdc_acs",
"td_charcoal",
"t_dgps",
"td_hs_charcoal",
"td_hs_pot",
"td_jfet",
"td_lhe",
"td_lhe_filt",
"td_ln",
"td_ln_filt",
"td_vcs_filt",
"td_vcs_jfet",
"t_flange_isc",
"t_flange_osc",
"t_gy",
"t_heat_isc",
"t_heat_osc",
"t_hkdcdc_rec",
"thresh_atrim",
"thresh_isc",
"thresh_osc",
"thresh_sbsc",
"t_if_clin",
"timeout_n",
"timeout_s",
"time_atrim",
"time_char_cycle",
"time_dgps",
"time_n_flc",
"time_set_cycle",
"time_s_flc",
"time_sip",
"t_lens_isc",
"t_lens_osc",
"t_lock",
"t_mb_flc",
"t_mc_el",
"vt_mc_lock",
"t_mc_piv",
"t_mc_rw",
"t_mot_act",
"tol_act",
"tol_isc",
"tol_osc",
"t_padcdc_rec",
"t_pauram_rec",
"t_piv",
"t_pot_max_cycle",
"t_prime_sf",
"vt_pump_bal",
"t_push_plate",
"t_pyr_clin",
"tr_300mk_strap",
"tr_he3_fridge",
"tr_he4_pot",
"tr_horn_250",
"tr_horn_350",
"tr_horn_500",
"tr_hwpr",
//"trigger_isc",
//"trigger_osc",
"trig_l_sbsc",
"trig_s_sbsc",
"trig_type_isc",
"trig_type_osc",
"trim_clin",
"trim_dgps",
"trim_enc",
"trim_mag",
"trim_null",
"trim_pss",
"tr_m4",
"tr_m5",
"tr_optbox_filt",
"t_sbsc",
"t_second_sf",
"t_serial",
"t_set_gy",
"t_set_sbsc",
"t_start_cycle",
"t_wd_flc",
"upslot_sched",
"v1_1_pss",
"v1_2_pss",
"v1_3_pss",
"v1_4_pss",
"v2_1_pss",
"v2_2_pss",
"v2_3_pss",
"v2_4_pss",
"v3_1_pss",
"v3_2_pss",
"v3_3_pss",
"v3_4_pss",
"v4_1_pss",
"v4_2_pss",
"v4_3_pss",
"v4_4_pss",
"v_arr_cc1",
"v_batt_cc1",
"v_arr_cc2",
"v_batt_cc2",
"vel_act",
"vel_az_mc",
"vel_az_p",
"vel_calc_piv",
"vel_dps_az",
"vel_el_mc",
"vel_el_p",
"vel_hwpr",
"vel_lock",
"vel_req_az",
"vel_req_el",
"vel_ser_piv",
"verbose_el",
"verbose_piv",
"verbose_rw",
"veto_sensor",
"v_pump_bal",
"vt_1_bat",
"vt_1_prime",
"vt_2_bat",
"vt_2_prime",
"v_targ_cc1",
"v_targ_cc2",
"vt_array",
"vt_chin",
"vt_dac_box",
"vt_earth",
"vt_el",
"vt_hwpr_feed",
"vt_hwpr_mot",
"vt_if_bot_back",
"vt_if_bot_frnt",
"vt_if_top_back",
"vt_if_top_frnt",
"vt_mot_pump_val",
"vt_port_back",
"vt_port_das",
"vt_port_hexc",
"vt_port_pyr",
"vt_port_rec",
"vt_rw",
"vt_star_front",
"vt_stbd_das",
"vt_stbd_rec",
"vt_strut_bot",
"vt_strut_side",
"vt_sun",
"wait_sf",
"w_p",
"xel_if_clin",
"x_mag",
"x_off_isc",
"x_off_osc",
"x_p",
"y_mag",
"y_off_isc",
"y_off_osc",
"y_p",
"z_mag",
  ""
};
