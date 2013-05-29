/* MPC: MCE-PCM communicator
 *
 * Copyright (c) 2013, D. V. Wiebe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * This software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall the copyright holder or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */

#ifndef MPC_H
#define MPC_H

#include "blast.h"
#include <stdlib.h>
#include <stdint.h>

/* MPC globals */
extern int nmce;
extern int in_turnaround;
extern int fake;
extern int data_mode;

#define MPC_ETC_DIR "/data/mas/etc"

/* the MAS-MPC interface and its fake counterpart */
void *mas_data(void *dummy);

/* The frame-processing loop */
void do_frame(const uint32_t *frame, size_t frame_size);

/* Script routines */
int run_simple_script(const char *path, char *argv[]);
#endif
