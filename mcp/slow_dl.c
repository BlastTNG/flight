#include "slow_dl.h"

struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  {"t_dpm_3v", SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"cpu_time", SLOWDL_U_MASK,    16, 0.0, -1, -1, -1},
  {"t_gybox",  SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"gyro1",    SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"t_reac",   SLOWDL_FORCE_INT, 7,  0.0, -1, -1, -1},
  {"g_p_el",   SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"i_el",     SLOWDL_TAKE_BIT,  3,  0.0, -1, -1, -1}
};
