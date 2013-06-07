/* fake.c: MAS-like stubs for fake MAS operation
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include "mce_frame.h"
#include "mpc.h"
#include "mputs.h"
#include <stdlib.h>
#include <unistd.h>

/* make and send fake data -- we only make data mode 11 data, which is static */
#define FAKE_DATA_RATE   6667 /* microseconds -- this is approximate */
void *fake_data(void *dummy)
{
  uint32_t framenum = 0x37183332;
  int i, j;
  uint32_t mode11[NUM_COL * NUM_ROW + MCE_HEADER_SIZE];

  nameThread("FAKE");
  bprintf(info, "Running FAKE data at %.1f Hz", 1e6 / FAKE_DATA_RATE);

  /* generate the mode 11 data -- this is simply:
   *
   *  0000 0000 0000 0000 0000 000r rrrr rccc
   *
   *  where :
   *
   *  r = the row number
   *  c = the column number
   */
  for (i = 0; i < NUM_COL; ++i)
    for (j = 0; j < NUM_ROW; ++j)
      mode11[MCE_HEADER_SIZE + i + j * NUM_COL] = (i & 0x7) | ((j & 0x3F) << 3);

  /* wait for initialisation from PCM */
  while (init)
    sleep(1);

  /* Run the frame loop */
  for (;;) {
    do_frame(mode11, (MCE_HEADER_SIZE + NUM_COL * NUM_ROW) * sizeof(uint32_t));
    usleep(FAKE_DATA_RATE);
    framenum++;
  }

  return NULL;
}

/* FAKE-only stubs */
#ifdef FAKE_MAS
void *mas_data(void *dummy)
{
  /* Can only get here if mpc.c is compiled with the wrong CPP defines */
  bputs(tfatal, "Unhandled MAS execution attempted.");
}
#endif
