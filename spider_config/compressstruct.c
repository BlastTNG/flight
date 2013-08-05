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
  {"bi0_fifo_size", 1, 1, NOAVG, NODX, 16, SLOW},
  {"bbc_fifo_size", 1, 1, NOAVG, NODX, 16, SLOW},
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
  {"step_1_el", 3, 1, NOAVG, NODX, 8, SLOW},
  {"step_2_el", 3, 1, NOAVG, NODX, 8, SLOW},
  {"dest_el_mc", 5, 1, NOAVG, NODX, 8, SLOW},
  {"dac_rw", 256, 1, NOAVG, NODX, 8, SLOW},
  {"dac_piv", 256, 1, NOAVG, NODX, 8, SLOW},
  {"i_el", 256, 1, NOAVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 1, NOAVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 1, NOAVG, NODX, 8, SLOW},
  {"term_i_az", 256, 1, NOAVG, NODX, 8, SLOW},
  {"term_p_az", 256, 1, NOAVG, NODX, 8, SLOW},
  {"term_p_t_rw_piv", 256, 1, NOAVG, NODX, 8, SLOW},
  {"term_p_v_req_az_piv", 256, 1, NOAVG, NODX, 8, SLOW},
  {"term_p_v_rw_piv", 256, 1, NOAVG, NODX, 8, SLOW},
  {"res_piv", 1, 1, NOAVG, DX, 8, SLOW},
  {"vel_ser_rw", 1, 1, NOAVG, DX, 8, SLOW},
  
  {"ofpch_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofpch_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  
  {"mce000",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce001",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce002",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce003",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce004",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce005",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce006",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce007",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce008",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce009",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce010",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce011",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce012",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce013",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce014",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce015",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce016",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce017",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce018",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce019",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce020",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce021",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce022",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce023",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce024",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce025",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce026",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce027",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce028",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce029",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce030",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce031",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce032",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce033",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce034",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce035",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce036",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce037",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce038",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce039",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce040",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce041",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce042",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce043",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce044",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce045",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce046",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce047",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce048",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce049",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce050",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce051",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce052",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce053",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce054",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce055",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce056",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce057",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce058",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce059",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce060",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce061",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce062",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce063",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce064",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce065",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce066",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce067",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce068",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce069",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce070",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce071",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce072",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce073",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce074",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce075",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce076",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce077",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce078",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce079",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce080",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce081",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce082",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce083",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce084",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce085",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce086",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce087",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce088",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce089",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce090",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce091",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce092",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce093",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce094",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce095",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce096",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce097",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce098",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce099",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce100",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce101",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce102",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce103",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce104",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce105",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce106",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce107",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce108",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce109",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce110",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce111",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce112",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce113",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce114",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce115",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce116",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce117",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce118",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce119",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce120",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce121",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce122",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce123",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce124",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce125",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce126",1, 5, NOAVG, NODX, 16, SLOW},
  {"mce127",1, 5, NOAVG, NODX, 16, SLOW},
  
END_OF_STREAM
  },
  
  {  // field set 1
    {"time",1,1,NOAVG,NODX,8,SLOW},
    {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},
    {"mce000",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce001",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce002",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce003",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce004",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce005",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce006",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce007",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce008",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce009",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce010",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce011",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce012",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce013",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce014",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce015",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce016",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce017",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce018",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce019",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce020",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce021",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce022",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce023",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce024",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce025",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce026",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce027",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce028",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce029",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce030",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce031",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce032",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce033",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce034",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce035",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce036",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce037",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce038",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce039",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce040",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce041",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce042",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce043",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce044",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce045",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce046",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce047",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce048",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce049",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce050",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce051",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce052",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce053",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce054",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce055",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce056",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce057",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce058",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce059",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce060",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce061",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce062",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce063",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce064",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce065",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce066",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce067",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce068",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce069",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce070",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce071",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce072",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce073",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce074",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce075",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce076",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce077",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce078",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce079",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce080",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce081",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce082",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce083",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce084",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce085",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce086",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce087",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce088",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce089",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce090",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce091",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce092",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce093",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce094",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce095",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce096",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce097",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce098",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce099",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce100",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce101",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce102",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce103",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce104",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce105",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce106",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce107",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce108",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce109",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce110",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce111",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce112",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce113",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce114",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce115",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce116",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce117",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce118",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce119",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce120",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce121",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce122",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce123",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce124",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce125",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce126",1, 5, NOAVG, NODX, 16, SLOW},
    {"mce127",1, 5, NOAVG, NODX, 16, SLOW},
    
    END_OF_STREAM
  }
};

char *frameList[] = {
#include "framelist.inc"
  ""
};

