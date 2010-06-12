#include "compressstruct.h"

char *frameList[] = {
  "time",
  "t_cpu_flc",
  "t_gy",
  ""
};

struct fieldStreamStruct streamList[] = {
  {"gy_ifel", 0, 10, AVG, DX, 4},
  {"gy_ifel1", 0, 10, AVG, DX, 4},
  END_OF_STREAM
};

