#include <string.h>
#include "compressstruct.h"

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

struct fieldStreamStruct streamList[] = {
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},
  {"framenum",1,1,NOAVG,DX,8,SLOW},
  {"ifel_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifroll_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"az", 33140, 10, AVG, DX, 8, SLOW},
  {"az_isc", 1, 5, AVG, DX, 8, SLOW},
  {"az_osc", 1, 5, AVG, DX, 8, SLOW},
  {"el", 33140, 10, AVG, DX, 8, SLOW},
  {"el_isc", 1, 5, AVG, DX, 8, SLOW},
  {"el_osc", 1, 5, AVG, DX, 8, SLOW},
  {"sigma_isc", 1, 1, NOAVG, NODX, 16, SLOW},
  {"sigma_osc", 1, 1, NOAVG, NODX, 16, SLOW},
  {"age_isc", 1, 1, NOAVG, NODX, 8, SLOW},
  {"age_osc", 50, 1, NOAVG, NODX, 8, SLOW},
  {"ra", 1, 5, AVG, DX, 8, SLOW},
  {"dec", 1, 5, AVG, DX, 8, SLOW},
  {"pot_hwpr", 1, 1, AVG, DX, 16, SLOW},
  {"enc_hwpr", 1, 1, AVG, DX, 16, SLOW},
  
  {"el_enc", 1, 5, AVG, DX, 8, SLOW},
  {"el_raw_enc", 1, 5, AVG, DX, 8, SLOW},
  {"az_ss", 1, 5, AVG, DX, 8, SLOW},
  {"az_dgps", 1, 5, AVG, DX, 8, SLOW},
  {"az_mag", 1, 5, AVG, DX, 8, SLOW},
  {"az_raw_dgps", 1, 5, AVG, DX, 8, SLOW},
  {"az_rel_sun_ss", 1, 5, AVG, DX, 8, SLOW},
  {"az_pss1", 1, 5, AVG, DX, 8, SLOW},
  {"az_pss2", 1, 5, AVG, DX, 8, SLOW},
  {"ra_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"dac_el", 256, 5, AVG, NODX, 8, SLOW},
  {"dac_rw", 256, 5, AVG, NODX, 8, SLOW},
  {"dac_piv", 256, 5, AVG, NODX, 8, SLOW},
  {"i_ser_el", 256, 5, AVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 5, AVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 5, AVG, NODX, 8, SLOW},
  {"i_term_az", 256, 5, AVG, NODX, 8, SLOW},
  {"i_term_el", 256, 5, AVG, NODX, 8, SLOW},
  {"p_term_az", 256, 5, AVG, NODX, 8, SLOW},
  {"p_rw_term_piv", 100, 5, AVG, NODX, 8, SLOW},
  {"frict_term_piv", 100, 5, AVG, NODX, 8, SLOW},
  {"res_piv", 1, 5, AVG, DX, 8, SLOW},
  {"vel_rw", 1, 5, AVG, DX, 8, SLOW},

  {"n29c10", 5, 25, AVG, DX, 8, SLOW},
  {"n29c07", 5, 25, AVG, DX, 8, SLOW},
  {"n31c09", 5, 25, AVG, DX, 8, SLOW},
  {"n21c10", 5, 50, AVG, DX, 8, SLOW},
  {"n23c10", 5, 50, AVG, DX, 8, SLOW},
  {"n23c13", 5, 50, AVG, DX, 8, SLOW},
  {"n26c16", 5, 50, AVG, DX, 8, SLOW},
  {"n26c17", 5, 50, AVG, DX, 8, SLOW},
  {"n27c09", 5, 50, AVG, DX, 8, SLOW},

  {"ra_isc", 8, 2, NOAVG, NODX, 16, SLOW},
  {"dec_isc", 8, 2, NOAVG, NODX, 16, SLOW},
  {"fieldrot_isc", 1, 2, NOAVG, NODX, 8, SLOW},
  {"rd_sigma_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"mcpnum_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"framenum_isc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"ra_osc", 8, 2, NOAVG, NODX, 16, SLOW},
  {"dec_osc", 8, 2, NOAVG, NODX, 16, SLOW},
  {"fieldrot_osc", 1, 2, NOAVG, NODX, 8, SLOW},
  {"rd_sigma_osc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"mcpnum_osc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"framenum_osc", 1, 2, NOAVG, NODX, 16, SLOW},
  {"pulse_sc", 1, 100, NOAVG, NODX, 8, SLOW},

  {"ifel_1_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_1_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"ifroll_1_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"ifel_2_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_2_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"ifroll_2_gy", 32768, 10, AVG, NODX, 8, SLOW},
  {"el_raw_if_clin", 1, 5, AVG, DX, 8, SLOW},
  {"el_clin", 1, 5, AVG, DX, 8, SLOW},

  {"n29c00", 5, 25, AVG, DX, 8, SLOW},
  {"n29c01", 5, 25, AVG, DX, 8, SLOW},
  {"n29c02", 5, 25, AVG, DX, 8, SLOW},
  {"n29c03", 5, 25, AVG, DX, 8, SLOW},
  {"n29c04", 5, 25, AVG, DX, 8, SLOW},
  //{"n29c05", 5, 25, AVG, DX, 8, SLOW}, // dead
  {"n29c06", 5, 25, AVG, DX, 8, SLOW},
  {"n29c08", 5, 25, AVG, DX, 8, SLOW},
  {"n29c09", 5, 25, AVG, DX, 8, SLOW},
  //{"n29c11", 5, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n29c12", 5, 25, AVG, DX, 8, SLOW},
  {"n29c13", 5, 25, AVG, DX, 8, SLOW},
  {"n29c14", 5, 25, AVG, DX, 8, SLOW},
  {"n29c15", 5, 25, AVG, DX, 8, SLOW},
  {"n29c16", 5, 25, AVG, DX, 8, SLOW},
  {"n29c17", 5, 25, AVG, DX, 8, SLOW},
  {"n29c18", 5, 25, AVG, DX, 8, SLOW},
  {"n29c19", 5, 25, AVG, DX, 8, SLOW},
  {"n29c20", 5, 25, AVG, DX, 8, SLOW},
  {"n29c21", 5, 25, AVG, DX, 8, SLOW},
  //{"n29c22", 5, 25, AVG, DX, 8, SLOW}, // dead
  {"n29c23", 5, 25, AVG, DX, 8, SLOW},
  //{"n31c00", 5, 25, AVG, DX, 8, SLOW}, // resistor
  {"n31c01", 5, 25, AVG, DX, 8, SLOW},
  {"n31c02", 5, 25, AVG, DX, 8, SLOW},
  {"n31c03", 5, 25, AVG, DX, 8, SLOW},
  {"n31c04", 5, 25, AVG, DX, 8, SLOW},
  {"n31c05", 5, 25, AVG, DX, 8, SLOW},
  {"n31c06", 5, 25, AVG, DX, 8, SLOW},
  {"n31c07", 5, 25, AVG, DX, 8, SLOW},
  {"n31c08", 5, 25, AVG, DX, 8, SLOW},
  {"n31c10", 5, 25, AVG, DX, 8, SLOW},
  {"n31c11", 5, 25, AVG, DX, 8, SLOW},
  //{"n31c12", 5, 25, AVG, DX, 8, SLOW}, // thermistor
  {"n31c13", 5, 25, AVG, DX, 8, SLOW},
  {"n31c14", 5, 25, AVG, DX, 8, SLOW},
  {"n31c15", 5, 25, AVG, DX, 8, SLOW},
  {"n31c16", 5, 25, AVG, DX, 8, SLOW},
  {"n31c17", 5, 25, AVG, DX, 8, SLOW},
  {"n31c18", 5, 25, AVG, DX, 8, SLOW},
  {"n31c19", 5, 25, AVG, DX, 8, SLOW},
  {"n31c20", 5, 25, AVG, DX, 8, SLOW},
  {"n31c21", 5, 25, AVG, DX, 8, SLOW},
  //{"n31c22", 5, 25, AVG, DX, 8, SLOW}, // dark
  {"n31c23", 5, 25, AVG, DX, 8, SLOW},
  {"n25c00", 5, 50, AVG, DX, 8, SLOW},
  {"n25c01", 5, 50, AVG, DX, 8, SLOW},
  {"n25c02", 5, 50, AVG, DX, 8, SLOW},
  {"n25c03", 5, 50, AVG, DX, 8, SLOW},
  {"n25c04", 5, 50, AVG, DX, 8, SLOW},
  {"n25c05", 5, 50, AVG, DX, 8, SLOW},
  {"n25c06", 5, 50, AVG, DX, 8, SLOW},
  {"n25c07", 5, 50, AVG, DX, 8, SLOW},
  {"n25c08", 5, 50, AVG, DX, 8, SLOW},
  {"n25c09", 5, 50, AVG, DX, 8, SLOW},
  {"n25c10", 5, 50, AVG, DX, 8, SLOW},
  {"n25c11", 5, 50, AVG, DX, 8, SLOW},
  //{"n25c12", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n25c13", 5, 50, AVG, DX, 8, SLOW},
  {"n25c14", 5, 50, AVG, DX, 8, SLOW},
  {"n25c15", 5, 50, AVG, DX, 8, SLOW},
  {"n25c16", 5, 50, AVG, DX, 8, SLOW},
  {"n25c17", 5, 50, AVG, DX, 8, SLOW},
  {"n25c18", 5, 50, AVG, DX, 8, SLOW},
  {"n25c19", 5, 50, AVG, DX, 8, SLOW},
  {"n25c20", 5, 50, AVG, DX, 8, SLOW},
  //{"n25c21", 5, 50, AVG, DX, 8, SLOW}, // dark
  //{"n25c22", 5, 50, AVG, DX, 8, SLOW}, // dead
  //{"n25c23", 5, 50, AVG, DX, 8, SLOW}, // resistor
  //{"n26c00", 5, 50, AVG, DX, 8, SLOW}, // resistor
  //{"n26c01", 5, 50, AVG, DX, 8, SLOW}, // dead
  //{"n26c02", 5, 50, AVG, DX, 8, SLOW}, // thermistor
  {"n26c03", 5, 50, AVG, DX, 8, SLOW},
  {"n26c04", 5, 50, AVG, DX, 8, SLOW},
  {"n26c05", 5, 50, AVG, DX, 8, SLOW},
  {"n26c06", 5, 50, AVG, DX, 8, SLOW},
  //{"n26c07", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n26c08", 5, 50, AVG, DX, 8, SLOW},
  {"n26c09", 5, 50, AVG, DX, 8, SLOW},
  {"n26c10", 5, 50, AVG, DX, 8, SLOW},
  //{"n26c11", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n26c12", 5, 50, AVG, DX, 8, SLOW},
  {"n26c13", 5, 50, AVG, DX, 8, SLOW},
  {"n26c14", 5, 50, AVG, DX, 8, SLOW},
  {"n26c15", 5, 50, AVG, DX, 8, SLOW},
  {"n26c18", 5, 50, AVG, DX, 8, SLOW},
  {"n26c19", 5, 50, AVG, DX, 8, SLOW},
  {"n26c20", 5, 50, AVG, DX, 8, SLOW},
  {"n26c21", 5, 50, AVG, DX, 8, SLOW},
  {"n26c22", 5, 50, AVG, DX, 8, SLOW},
  {"n26c23", 5, 50, AVG, DX, 8, SLOW},
  {"n27c00", 5, 50, AVG, DX, 8, SLOW},
  {"n27c01", 5, 50, AVG, DX, 8, SLOW},
  {"n27c02", 5, 50, AVG, DX, 8, SLOW},
  {"n27c03", 5, 50, AVG, DX, 8, SLOW},
  {"n27c04", 5, 50, AVG, DX, 8, SLOW},
  {"n27c05", 5, 50, AVG, DX, 8, SLOW},
  {"n27c06", 5, 50, AVG, DX, 8, SLOW},
  {"n27c07", 5, 50, AVG, DX, 8, SLOW},
  {"n27c08", 5, 50, AVG, DX, 8, SLOW},
  {"n27c10", 5, 50, AVG, DX, 8, SLOW},
  {"n27c11", 5, 50, AVG, DX, 8, SLOW},
  {"n27c12", 5, 50, AVG, DX, 8, SLOW},
  {"n27c13", 5, 50, AVG, DX, 8, SLOW},
  {"n27c14", 5, 50, AVG, DX, 8, SLOW},
  {"n27c15", 5, 50, AVG, DX, 8, SLOW},
  {"n27c16", 5, 50, AVG, DX, 8, SLOW},
  {"n27c17", 5, 50, AVG, DX, 8, SLOW},
  {"n27c18", 5, 50, AVG, DX, 8, SLOW},
  {"n27c19", 5, 50, AVG, DX, 8, SLOW},
  {"n27c20", 5, 50, AVG, DX, 8, SLOW},
  {"n27c21", 5, 50, AVG, DX, 8, SLOW},
  //{"n27c22", 5, 50, AVG, DX, 8, SLOW}, // dark
  //{"n27c23", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n30c00", 5, 50, AVG, DX, 8, SLOW},
  //{"n30c01", 5, 50, AVG, DX, 8, SLOW}, // thermistor
  {"n30c02", 5, 50, AVG, DX, 8, SLOW},
  {"n30c03", 5, 50, AVG, DX, 8, SLOW},
  {"n30c04", 5, 50, AVG, DX, 8, SLOW},
  {"n30c05", 5, 50, AVG, DX, 8, SLOW},
  {"n30c06", 5, 50, AVG, DX, 8, SLOW},
  {"n30c07", 5, 50, AVG, DX, 8, SLOW},
  {"n30c08", 5, 50, AVG, DX, 8, SLOW},
  {"n30c09", 5, 50, AVG, DX, 8, SLOW},
  {"n30c10", 5, 50, AVG, DX, 8, SLOW},
  {"n30c11", 5, 50, AVG, DX, 8, SLOW},
  {"n30c12", 5, 50, AVG, DX, 8, SLOW},
  {"n30c13", 5, 50, AVG, DX, 8, SLOW},
   //{"n30c14", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n30c15", 5, 50, AVG, DX, 8, SLOW},
  {"n30c16", 5, 50, AVG, DX, 8, SLOW},
  {"n30c17", 5, 50, AVG, DX, 8, SLOW},
  {"n30c18", 5, 50, AVG, DX, 8, SLOW},
  {"n30c19", 5, 50, AVG, DX, 8, SLOW},
  {"n30c20", 5, 50, AVG, DX, 8, SLOW},
  {"n30c21", 5, 50, AVG, DX, 8, SLOW},
  {"n30c22", 5, 50, AVG, DX, 8, SLOW},
  //{"n30c23", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n17c00", 5, 50, AVG, DX, 8, SLOW},
  {"n17c01", 5, 50, AVG, DX, 8, SLOW},
  {"n17c02", 5, 50, AVG, DX, 8, SLOW},
  {"n17c03", 5, 50, AVG, DX, 8, SLOW},
  {"n17c04", 5, 50, AVG, DX, 8, SLOW},
  {"n17c05", 5, 50, AVG, DX, 8, SLOW},
  {"n17c06", 5, 50, AVG, DX, 8, SLOW},
  {"n17c07", 5, 50, AVG, DX, 8, SLOW},
  {"n17c08", 5, 50, AVG, DX, 8, SLOW},
  {"n17c09", 5, 50, AVG, DX, 8, SLOW},
  {"n17c10", 5, 50, AVG, DX, 8, SLOW},
  {"n17c11", 5, 50, AVG, DX, 8, SLOW},
  {"n17c12", 5, 50, AVG, DX, 8, SLOW},
  {"n17c13", 5, 50, AVG, DX, 8, SLOW},
  {"n17c14", 5, 50, AVG, DX, 8, SLOW},
  {"n17c15", 5, 50, AVG, DX, 8, SLOW},
  {"n17c16", 5, 50, AVG, DX, 8, SLOW},
  {"n17c17", 5, 50, AVG, DX, 8, SLOW},
  {"n17c18", 5, 50, AVG, DX, 8, SLOW},
  {"n17c19", 5, 50, AVG, DX, 8, SLOW},
  {"n17c20", 5, 50, AVG, DX, 8, SLOW},
  //{"n17c21", 5, 50, AVG, DX, 8, SLOW}, // dark
  {"n17c22", 5, 50, AVG, DX, 8, SLOW},
  {"n17c23", 5, 50, AVG, DX, 8, SLOW},
  {"n18c00", 5, 50, AVG, DX, 8, SLOW},
  {"n18c01", 5, 50, AVG, DX, 8, SLOW},
  //{"n18c02", 5, 50, AVG, DX, 8, SLOW}, // thermistor
  {"n18c03", 5, 50, AVG, DX, 8, SLOW},
  {"n18c04", 5, 50, AVG, DX, 8, SLOW},
  {"n18c05", 5, 50, AVG, DX, 8, SLOW},
  {"n18c06", 5, 50, AVG, DX, 8, SLOW},
  {"n18c07", 5, 50, AVG, DX, 8, SLOW},
  {"n18c08", 5, 50, AVG, DX, 8, SLOW},
  {"n18c09", 5, 50, AVG, DX, 8, SLOW},
  {"n18c10", 5, 50, AVG, DX, 8, SLOW},
  {"n18c11", 5, 50, AVG, DX, 8, SLOW},
  {"n18c12", 5, 50, AVG, DX, 8, SLOW},
  {"n18c13", 5, 50, AVG, DX, 8, SLOW},
  {"n18c14", 5, 50, AVG, DX, 8, SLOW},
  {"n18c15", 5, 50, AVG, DX, 8, SLOW},
  {"n18c16", 5, 50, AVG, DX, 8, SLOW},
  {"n18c17", 5, 50, AVG, DX, 8, SLOW},
  {"n18c18", 5, 50, AVG, DX, 8, SLOW},
  {"n18c19", 5, 50, AVG, DX, 8, SLOW},
  {"n18c20", 5, 50, AVG, DX, 8, SLOW},
  {"n18c21", 5, 50, AVG, DX, 8, SLOW},
  {"n18c22", 5, 50, AVG, DX, 8, SLOW},
  {"n18c23", 5, 50, AVG, DX, 8, SLOW},
  {"n19c00", 5, 50, AVG, DX, 8, SLOW},
  {"n19c01", 5, 50, AVG, DX, 8, SLOW},
  {"n19c02", 5, 50, AVG, DX, 8, SLOW},
  {"n19c03", 5, 50, AVG, DX, 8, SLOW},
  {"n19c04", 5, 50, AVG, DX, 8, SLOW},
  {"n19c05", 5, 50, AVG, DX, 8, SLOW},
  {"n19c06", 5, 50, AVG, DX, 8, SLOW},
  {"n19c07", 5, 50, AVG, DX, 8, SLOW},
  {"n19c08", 5, 50, AVG, DX, 8, SLOW},
  {"n19c09", 5, 50, AVG, DX, 8, SLOW},
  //{"n19c10", 5, 50, AVG, DX, 8, SLOW}, // dead
  {"n19c11", 5, 50, AVG, DX, 8, SLOW},
  {"n19c12", 5, 50, AVG, DX, 8, SLOW},
  {"n19c13", 5, 50, AVG, DX, 8, SLOW},
  {"n19c14", 5, 50, AVG, DX, 8, SLOW},
  {"n19c15", 5, 50, AVG, DX, 8, SLOW},
  {"n19c16", 5, 50, AVG, DX, 8, SLOW},
  {"n19c17", 5, 50, AVG, DX, 8, SLOW},
  {"n19c18", 5, 50, AVG, DX, 8, SLOW},
  {"n19c19", 5, 50, AVG, DX, 8, SLOW},
  {"n19c20", 5, 50, AVG, DX, 8, SLOW},
  {"n19c21", 5, 50, AVG, DX, 8, SLOW},
  //{"n19c22", 5, 50, AVG, DX, 8, SLOW}, // dark
  {"n19c23", 5, 50, AVG, DX, 8, SLOW},
  {"n21c00", 5, 50, AVG, DX, 8, SLOW},
  {"n21c01", 5, 50, AVG, DX, 8, SLOW},
  {"n21c02", 5, 50, AVG, DX, 8, SLOW},
  {"n21c03", 5, 50, AVG, DX, 8, SLOW},
  {"n21c04", 5, 50, AVG, DX, 8, SLOW},
  {"n21c05", 5, 50, AVG, DX, 8, SLOW},
  {"n21c06", 5, 50, AVG, DX, 8, SLOW},
  {"n21c07", 5, 50, AVG, DX, 8, SLOW},
  {"n21c08", 5, 50, AVG, DX, 8, SLOW},
  {"n21c09", 5, 50, AVG, DX, 8, SLOW},
  {"n21c11", 5, 50, AVG, DX, 8, SLOW},
  {"n21c12", 5, 50, AVG, DX, 8, SLOW},
  {"n21c13", 5, 50, AVG, DX, 8, SLOW},
  {"n21c14", 5, 50, AVG, DX, 8, SLOW},
  {"n21c15", 5, 50, AVG, DX, 8, SLOW},
  {"n21c16", 5, 50, AVG, DX, 8, SLOW},
  {"n21c17", 5, 50, AVG, DX, 8, SLOW},
  {"n21c18", 5, 50, AVG, DX, 8, SLOW},
  {"n21c19", 5, 50, AVG, DX, 8, SLOW},
  {"n21c20", 5, 50, AVG, DX, 8, SLOW},
  {"n21c21", 5, 50, AVG, DX, 8, SLOW},
  {"n21c22", 5, 50, AVG, DX, 8, SLOW},
  {"n21c23", 5, 50, AVG, DX, 8, SLOW},
  //{"n22c00", 5, 50, AVG, DX, 8, SLOW},// Resistor
  {"n22c01", 5, 50, AVG, DX, 8, SLOW},
  //{"n22c02", 5, 50, AVG, DX, 8, SLOW}, // thermistor
  {"n22c03", 5, 50, AVG, DX, 8, SLOW},
  {"n22c04", 5, 50, AVG, DX, 8, SLOW},
  {"n22c05", 5, 50, AVG, DX, 8, SLOW},
  {"n22c06", 5, 50, AVG, DX, 8, SLOW},
  {"n22c07", 5, 50, AVG, DX, 8, SLOW},
  {"n22c08", 5, 50, AVG, DX, 8, SLOW},
  {"n22c09", 5, 50, AVG, DX, 8, SLOW},
  {"n22c10", 5, 50, AVG, DX, 8, SLOW},
  {"n22c11", 5, 50, AVG, DX, 8, SLOW},
  {"n22c12", 5, 50, AVG, DX, 8, SLOW},
  {"n22c13", 5, 50, AVG, DX, 8, SLOW},
  {"n22c14", 5, 50, AVG, DX, 8, SLOW},
  {"n22c15", 5, 50, AVG, DX, 8, SLOW},
  {"n22c16", 5, 50, AVG, DX, 8, SLOW},
  {"n22c17", 5, 50, AVG, DX, 8, SLOW},
  {"n22c18", 5, 50, AVG, DX, 8, SLOW},
  {"n22c19", 5, 50, AVG, DX, 8, SLOW},
  {"n22c20", 5, 50, AVG, DX, 8, SLOW},
  {"n22c21", 5, 50, AVG, DX, 8, SLOW},
  {"n22c22", 5, 50, AVG, DX, 8, SLOW},
  {"n22c23", 5, 50, AVG, DX, 8, SLOW},
  {"n23c00", 5, 50, AVG, DX, 8, SLOW},
  {"n23c01", 5, 50, AVG, DX, 8, SLOW},
  {"n23c02", 5, 50, AVG, DX, 8, SLOW},
  {"n23c03", 5, 50, AVG, DX, 8, SLOW},
  {"n23c04", 5, 50, AVG, DX, 8, SLOW},
  {"n23c05", 5, 50, AVG, DX, 8, SLOW},
  {"n23c06", 5, 50, AVG, DX, 8, SLOW},
  {"n23c07", 5, 50, AVG, DX, 8, SLOW},
  {"n23c08", 5, 50, AVG, DX, 8, SLOW},
  {"n23c09", 5, 50, AVG, DX, 8, SLOW},
  {"n23c11", 5, 50, AVG, DX, 8, SLOW},
  {"n23c12", 5, 50, AVG, DX, 8, SLOW},
  {"n23c14", 5, 50, AVG, DX, 8, SLOW},
  {"n23c15", 5, 50, AVG, DX, 8, SLOW},
  {"n23c16", 5, 50, AVG, DX, 8, SLOW},
  {"n23c17", 5, 50, AVG, DX, 8, SLOW},
  {"n23c18", 5, 50, AVG, DX, 8, SLOW},
  {"n23c19", 5, 50, AVG, DX, 8, SLOW},
  {"n23c20", 5, 50, AVG, DX, 8, SLOW},
  {"n23c21", 5, 50, AVG, DX, 8, SLOW},
  {"n23c22", 5, 50, AVG, DX, 8, SLOW},
  {"n23c23", 5, 50, AVG, DX, 8, SLOW},
END_OF_STREAM
};

char *frameList[] = {
  "acc_act",
  "accel_az",
  "acc_hwpr",
  "acc_lock",
  "age_sf",
  "alarm_hi_cc",
  "alarm_lo_cc",
  "alt",
  "alt_dgps",
  "alt_sip",
  "ampl_250_bias",
  "ampl_350_bias",
  "ampl_500_bias",
  "ampl_rox_bias",
  "ampl_x_bias",
  "ant_e_dgps",
  "ant_n_dgps",
  "ants_lim_dgps",
  "ant_u_dgps",
  "apert_isc",
  "apert_osc",
  "att_ok_dgps",
  "az_cov_dgps",
  "az_gy",
  "az_raw_mag",
  "azraw_pss1",
  "azraw_pss2",
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
  "brdec_isc",
  "brdec_osc",
  "brra_isc",
  "brra_osc",
  "bus_reset_act",
  "ccd_t_sbsc",
  "chatter",
  "chopper",
  "climb_dgps",
  "correction_sf",
  "cos_el",
  "cov_lim_dgps",
  "cryostate",
  "cycle_start",
  "cycle_state",
  "dac2_ampl",
  "dec_1_p",
  "dec_2_p",
  "dec_3_p",
  "dec_4_p",
  "dec_isc",
  "declination_mag",
  "dec_osc",
  "del_p",
  "dest_az_mc",
  "dest_el_mc",
  "dig21_das",
  "dig43_das",
  "dig65_das",
  "dir_az_mc",
  "dir_dgps",
  "dir_el_mc",
  "disk_free",
  "diskfree_isc",
  "diskfree_osc",
  "dith_el",
  "dith_step_p",
  "drive_err_cts_el",
  "drive_err_cts_piv",
  "drive_err_cts_rw",
  "drive_info_el",
  "drive_info_piv",
  "drive_info_rw",
  "el_lut_clin",
  "elraw_pss1",
  "elraw_pss2",
  "el_sun",
  "enc_0_act",
  "enc_1_act",
  "enc_2_act",
  "enc_hwpr",
  "error_az",
  "error_el",
  "error_isc",
  "error_osc",
  "exp_int_sbsc",
  "exposure_isc",
  "exposure_osc",
  "exp_time_sbsc",
  "fault_cc",
  "fault_el",
  "fault_gy",
  "fault_rw",
  "fieldrot_isc",
  "fieldrot_osc",
  "flags_act",
  "foc_off_isc",
  "foc_off_osc",
  "foc_res_sbsc",
  "focus_isc",
  "focus_osc",
  "focus_sf",
  "force_sbsc",
  "fpulse_isc",
  "fpulse_osc",
  "framenum_isc",
  "framenum_osc",
  "frame_sbsc",
  "frict_off_piv",
  "frict_term_uf_piv",
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
  "i_arr_cc",
  "i_batt_cc",
  "i_cal_lamp",
  "i_charcoal",
  "i_das",
  "i_dgps",
  "i_el",
  "ifel_1_gy",
  "ifel_2_gy",
  "i_flc",
  "ifpm_hall",
  "ifroll_1_gy",
  "ifroll_2_gy",
  "ifyaw_1_gy",
  "ifyaw_2_gy",
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
  "i_sc",
  "i_step",
  "i_tot",
  "i_trans",
  "jfet_set_off",
  "jfet_set_on",
  "lat",
  "latch0",
  "latch1",
  "lat_dgps",
  "lat_sip",
  "led_cc",
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
  "mcpnum_isc",
  "mcpnum_osc",
  "mdist_isc",
  "mdist_osc",
  "mdist_sbsc",
  "minblobs_isc",
  "minblobs_osc",
  "mks_hi_sip",
  "mks_lo_sip",
  "mks_med_sip",
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
  "nrad_isc",
  "nrad_osc",
  "n_sat_dgps",
  "numblobs_sbsc",
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
  "offset_ifroll_gy",
  "offset_ifyaw_gy",
  "offset_isc",
  "offset_osc",
  "offset_sf",
  "overshoot_hwpr",
  "pch_pyr_clin",
  "period_cal",
  "p_err_term_piv",
  "phase_ss",
  "pin_in_lock",
  "pitch_cov_dgps",
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
  "pot_lock",
  "pot_raw_hwpr",
  "pot_ref_hwpr",
  "pref_tp_sf",
  "pref_ts_sf",
  "pressure1_isc",
  "pressure1_osc",
  "p_term_el",
  "pulse_cal",
  "pulse_sc",
  "qtol_isc",
  "qtol_osc",
  "ra_1_p",
  "ra_2_p",
  "ra_3_p",
  "ra_4_p",
  "ra_isc",
  "ramp_ampl_bias",
  "ramp_ena_bias",
  "ra_osc",
  "rate_iridium",
  "rate_tdrss",
  "raw_01_ss",
  "raw_02_ss",
  "raw_03_ss",
  "raw_04_ss",
  "raw_05_ss",
  "raw_06_ss",
  "raw_07_ss",
  "raw_08_ss",
  "raw_09_ss",
  "raw_10_ss",
  "raw_11_ss",
  "raw_12_ss",
  "rd_sigma_isc",
  "rd_sigma_osc",
  "read_wait_hwpr",
  "real_trig_isc",
  "real_trig_osc",
  "rel_move_hwpr",
  "roll_cov_dgps",
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
  "sigma_isc",
  "sigma_mag",
  "sigma_osc",
  "sigma_ss",
  "sin_el",
  "slew_veto",
  "snr_pss1",
  "snr_pss2",
  "snr_ss",
  "speed_dgps",
  "spread_sf",
  "spulse_isc",
  "spulse_osc",
  "stat_1_el",
  "stat_1_rw",
  "stat_2_el",
  "stat_2_rw",
  "stat_control_hwpr",
  "stat_dr_piv",
  "state_cc",
  "state_isc",
  "state_lock",
  "state_osc",
  "stat_s1_piv",
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
  "step_start_bias",
  "step_start_phase",
  "step_time_bias",
  "step_time_phase",
  "stop_cnt_hwpr",
  "sun_time_ss",
  "sveto_len",
  "switch_gy",
  "switch_misc",
  "t_1_bat",
  "t_1_prime",
  "t_1_second",
  "t_2_bat",
  "t_2_prime",
  "t_2_second",
  "t_acs",
  "t_array",
  "t_box_bal",
  "t_case_ss",
  "t_chin",
  "t_chip_flc",
  "t_comp_isc",
  "t_comp_osc",
  "t_cpu_flc",
  "t_cpu_ss",
  "t_dac_box",
  "t_das",
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
  "t_earth",
  "t_el",
  "t_flange_isc",
  "t_flange_osc",
  "t_gy",
  "t_hdd_ss",
  "t_heat_isc",
  "t_heat_osc",
  "t_hkdcdc_rec",
  "thresh_isc",
  "thresh_osc",
  "thresh_sbsc",
  "t_hs_cc",
  "t_hwpr_feed",
  "t_hwpr_mot",
  "t_if_bot_back",
  "t_if_bot_frnt",
  "t_if_clin",
  "t_if_top_back",
  "t_if_top_frnt",
  "time_dgps",
  "timeout",
  "time_sip",
  "t_lens_isc",
  "t_lens_osc",
  "t_lock",
  "t_mb_flc",
  "t_mc_el",
  "t_mc_lock",
  "t_mc_piv",
  "t_mc_rw",
  "t_mot_act",
  "t_mot_pump_val",
  "tol_isc",
  "tol_osc",
  "t_padcdc_rec",
  "t_pauram_rec",
  "t_piv",
  "t_port_back",
  "t_port_ss",
  "t_prime_sf",
  "t_pump_bal",
  "t_push_plate",
  "t_pyr_clin",
  "tr_300mk_strap",
  "t_rec",
  "tr_he3_fridge",
  "tr_he4_pot",
  "tr_horn_250",
  "tr_horn_350",
  "tr_horn_500",
  "tr_hwpr",
  "trigger_isc",
  "trigger_osc",
  "trig_l_sbsc",
  "trig_s_sbsc",
  "trig_type_isc",
  "trig_type_osc",
  "trim_clin",
  "trim_dgps",
  "trim_enc",
  "trim_mag",
  "trim_null",
  "trim_ss",
  "tr_m4",
  "tr_m5",
  "tr_optbox_filt",
  "t_rw",
  "t_sbsc",
  "t_second_sf",
  "t_serial",
  "t_set_gy",
  "t_set_sbsc",
  "t_star_back",
  "t_star_ss",
  "t_strut_bot",
  "t_strut_side",
  "t_wd_flc",
  "usec_sbsc",
  "v1_1_pss",
  "v1_2_pss",
  "v_12_ss",
  "v2_1_pss",
  "v2_2_pss",
  "v3_1_pss",
  "v3_2_pss",
  "v4_1_pss",
  "v4_2_pss",
  "v_5_ss",
  "v_arr_cc",
  "v_batt_cc",
  "v_batt_ss",
  "vel_act",
  "vel_az_mc",
  "vel_az_p",
  "vel_calc_piv",
  "vel_dps_az",
  "vel_el_mc",
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
  "v_targ_cc",
  "wait_sf",
  "w_p",
  "xel_if_clin",
  "x_lim_stage",
  "x_mag",
  "x_off_isc",
  "x_off_osc",
  "x_p",
  "x_stage",
  "x_stp_stage",
  "x_str_stage",
  "x_vel_stage",
  "y_lim_stage",
  "y_mag",
  "y_off_isc",
  "y_off_osc",
  "y_p",
  "y_stage",
  "y_stp_stage",
  "y_str_stage",
  "y_vel_stage",
  "z_mag",
  //"n22c02",
  //"n18c02",
  //"n30c01",
  //"n26c02",
  //"n31c12",
  //"n29c11",
  //"n22c00",
  //"n25c23",
  //"n26c00",
  //"n31c00",
  //"n19c22",
  //"n27c22",
  //"n31c22",
  //"n17c21",
  //"n25c21",
  ""
};


//*********************************************************
// check if the name referes to a bolometer
//*********************************************************
int isBoloField(char *field) {
  if (field[0]!='n') return 0;
  if (strlen(field)!=6) return 0;
  if (field[3]!='c') return 0;
  if (field[1]<'1') return 0;
  if (field[1]>'3') return 0;

  return 1;
}

//*********************************************************
// return a pointer to a channel struct describing the field.
// if its a bolometer, make a struct with default values.
// WARNING: rw, bus, node, and addr are not set for bolo fields
//*********************************************************
struct ChannelStruct *GetChannelStruct(char *name) {
  int i_ch;

  struct ChannelStruct * cs;

  // search the slow channels
  for (i_ch = 0; SlowChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, SlowChannels[i_ch].field)==0) {
      return (SlowChannels+i_ch);
    }
  }

  // search the slow wide channels
  for (i_ch = 0; WideSlowChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, WideSlowChannels[i_ch].field)==0) {
      return (WideSlowChannels+i_ch);
    }
  }

  // search the fast channels
  for (i_ch = 0; FastChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, FastChannels[i_ch].field)==0) {
      return (FastChannels+i_ch);
    }
  }

  // search the wide fast channels
  for (i_ch = 0; WideFastChannels[i_ch].field[0]!='\0'; i_ch++) {
    if (strcmp(name, WideFastChannels[i_ch].field)==0) {
      return (WideFastChannels+i_ch);
    }
  }

  if (isBoloField(name)) {
    cs = (struct ChannelStruct *)malloc(sizeof(struct ChannelStruct));
    strcpy(cs->field, name);
    cs->type = 'U';
    cs->m_c2e = LOCKIN_C2V;
    cs->b_e2e = LOCKIN_OFFSET;
    cs->quantity[0] = '\0';
    cs->units[0] = '\0';
    return(cs);
  }

  return (NULL);
}

