#ifndef HWPR_H
#define HWPR_H

#include "ezstep.h"

#define HWPR_STEPS_PER_MOTENC (64)
#define HWPR_REV_PER_MOTREV (24./100.)

#define HWPR_NAME "HWPR Motor"
#define HWPR_ADDR EZ_WHO_S13
#define HWPR_PREAMBLE "aE64000"

void DoHWPR(struct ezbus* bus);

#endif
