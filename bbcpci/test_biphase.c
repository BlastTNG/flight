/* test_biphase: sample program for the bbc_pci driver
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * test_biphase is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * test_biphase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with test_biphase; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define DEFAULT_FRAME_FILE              "frame.dat"
#define DEFAULT_BIT_ERRORS_PER_FRAME    0
#define DEFAULT_BLACK_OUT_PERIOD        0
#define DEFAULT_BLACK_OUT_LENGTH        10
#define BYTEBITS                        (8 * sizeof(unsigned short))
#define WAITFRAMES                      4
#define TOLERANCE                       5

int main(int argc, char *argv[]) {
  int fp, framelen, bitsperframe, biterrorperiod, badbitnum, numbiterrs;
  int badsyncs, numbadsyncs, numfalsesyncs;
  int blackoutperiod, blackoutlen, avgblackoutlen, numblackouts;
  double biterrorsperframe;
  FILE *f;
  unsigned short *sendbuf, sendword, checkword;
  unsigned int i, j, k, l, m; 
  char framefile[80], tmpstr[80];
  char waiter[4] = {'-', '\\', '|', '/'};
  char thisbitbad, thisblackout;

  srand(time(NULL));
 
  strcpy(framefile, DEFAULT_FRAME_FILE);
  biterrorsperframe = DEFAULT_BIT_ERRORS_PER_FRAME;
  blackoutperiod = DEFAULT_BLACK_OUT_PERIOD;
  blackoutlen = DEFAULT_BLACK_OUT_LENGTH;
  
  if (argc == 2 && !strcmp(argv[1], "--help")) {
    printf("test_biphase - send fake frames through BBC PCI card\n");
    printf("\n");
    printf("test_biphase [-Eerrorrate] [-Bblackoutrate] [-Lblackoutlength]\n");
    printf("             [frame file]\n");
    printf("\n");
    printf("-E  average number of bad bits per frame to simulate\n");
    printf("-B  simulate about how many frames apart the telemetry fails;\n");
    printf("    biphase is blacked out and only zeros are sent down\n");
    printf("-L  how many frames, on average, the black outs last\n");

    return 1;
  }
  
  for (i = 1; i < argc; i++) {
    strcpy(tmpstr, argv[i]);
    tmpstr[2] = '\0';
    if (!strcmp(tmpstr, "-E"))
      biterrorsperframe = atof(argv[i] + 2);
    else if (!strcmp(tmpstr, "-B"))
      blackoutperiod = atoi(argv[i] + 2);
    else if (!strcmp(tmpstr, "-L"))
      blackoutlen = atoi(argv[i] + 2);
    else
      strcpy(framefile, argv[i]);
  }
  
  fp = open("/dev/bi0_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening bi0_pci.\n");
    exit(0);
  }
  
  if ((f = fopen(framefile, "r")) == NULL) {
    printf("Could not open %s for reading.  Exiting.\n", framefile);
    return 0;
  }

  if (fscanf(f, "%d", &framelen) == EOF) {
    printf("Bad frame file (%s).  Couldn't read frame length.  Exiting.\n",
           framefile);
    return 0;
  }

  sendbuf = (unsigned short *)malloc(framelen * sizeof(unsigned short));

  sendbuf[0] = (unsigned short)BBC_BI0_SYNC;
  for (i = 1; i < framelen; i++) {
    if (fscanf(f, "%x", sendbuf + i) == EOF) {
      printf("Bad frame file (%s).  Reached EOF at line %d; was expecting a "
             "frame of size %d.  Exiting.\n", framefile, i, framelen);
      return 0;
    }
  }

  fclose(f);

  bitsperframe = framelen * BYTEBITS * BYTEBITS;
  biterrorperiod = (int)((double)bitsperframe / biterrorsperframe);
 
  system("clear");
  printf("NIOS program version is %8x.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  printf("Using frame in file %s.\n", framefile);
  printf("Generating %g bit errors per frame.\n", biterrorsperframe);
  if (blackoutperiod)
    printf("On average, generating blackouts every %d frames, lasting about %d "
           "frames.\n", blackoutperiod, blackoutlen);
  else
    printf("Generating no blackouts.\n");
 
  numbiterrs = 0;
  badsyncs = 0;
  numbadsyncs = 0;
  numfalsesyncs = 0;
  numblackouts = 0;
  avgblackoutlen = 0;
  thisblackout = 0;
  for (i = 0, j = 0; ; i++) {
    sendbuf[0] = ~sendbuf[0];
    for (k = 0; k < framelen; k++) {
      if (!thisblackout) {
        sendword = sendbuf[k];
        if (blackoutperiod) {
          if ((int)((double)rand() / RAND_MAX * blackoutperiod * framelen) 
              == 0) {
            thisblackout = 1;
            numblackouts++;
          }
        }
      }
      else {
        sendword = 0;
        avgblackoutlen++;
        if ((int)((double)rand() / RAND_MAX * blackoutlen * framelen) == 0)
          thisblackout = 0;
      }
     
      if (!thisblackout) {
      // Insert flakey bits.
        thisbitbad = 0;
        for (l = 0; l < BYTEBITS; l++) {
          if ((badbitnum = (int)((double)rand() / RAND_MAX * biterrorperiod)) < 
              BYTEBITS) {
            // Swap a bit.
            thisbitbad = 1;
            if ((sendword >> badbitnum) & 1)
              sendword &= ~(1 << badbitnum);
            else
              sendword |= (1 << badbitnum);
            numbiterrs++;
          }
        }

        // Count bad sync words.
        if (k == 0) {
          if (thisbitbad)
            badsyncs++;
          else
            badsyncs--;
        }

        if (badsyncs >= TOLERANCE) {
          numbadsyncs++;
          badsyncs = TOLERANCE;
        }
        else if (badsyncs < 0)
          badsyncs = 0;

        // Check for spurious 0xeb90 / 0x146f
        for (l = 0; l < BYTEBITS; l++) {
          checkword = 0;
          checkword = sendword << l;
          m = k + 1;
          if (m >= framelen)
            m = 0;
          checkword |= sendbuf[m] >> (BYTEBITS - l);
          if ((checkword == BBC_BI0_SYNC || ~checkword == BBC_BI0_SYNC) &&
              !(l == 0 && k == 0))
            numfalsesyncs++;
        }
      }
      write(fp, (void *)(&sendword), sizeof(unsigned short));
    }
    if (i % 100 == 0) {
      printf("%c[6;1H", 0x1B);
      printf("Bit errors = %-10d     > %1d consecutive syncs = %-10d\n", 
             numbiterrs, TOLERANCE, numbadsyncs);
      printf("Blackouts = %-10d      Avg. blackout length (frames) = %-10g\n",
             numblackouts, (double)(avgblackoutlen) / 
                           (double)((numblackouts + 1) * framelen));
      printf("Spurious sync words = %-10d\n", numfalsesyncs);
      printf("\n                               %c\n", waiter[j++]);
      if (j >= WAITFRAMES)
        j = 0;
    }
  }
   
  return 1;
}
