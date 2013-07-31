/* MCCview
 *
 * Copyright (C) 2013 D. V. Wiebe
 *
 * Usage of this software is permitted provided that this license is retained
 * with the software, so that any entity that uses this software is notified of
 * this license.
 *
 * DISCLAIMER: THIS SOFTWARE IS WITHOUT WARRANTY.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <getdata.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <ncurses.h>

#define DRIVE0_UNMAP 0x0004
#define DRIVE0_MASK  0x0007
#define DRIVE1_UNMAP 0x0020
#define DRIVE1_MASK  0x0038
#define DRIVE2_UNMAP 0x0080
#define DRIVE2_MASK  0x00C0

struct fft { const char *fmt; gd_type_t type; };

#define NF 17
static const struct fft ff[NF] = {
  {"TIME_MCC%i", GD_UINT64},  /* 0 */
  {"TILE_HEATER_MCE%i", GD_UINT16}, /* 1 */
  {"DF_0_MCC%i", GD_FLOAT64}, /* 2 */
  {"DF_1_MCC%i", GD_FLOAT64}, /* 3 */
  {"DF_2_MCC%i", GD_FLOAT64}, /* 4 */
  {"DF_3_MCC%i", GD_FLOAT64}, /* 5 */
  {"state_mpc%i", GD_UINT16}, /* 6 */
  {"GOAL_MPC%i", GD_UINT16},  /* 7 */
  {"task_mpc%i", GD_UINT16},  /* 8 */
  {"DTASK_MPC%i", GD_UINT16}, /* 9 */
  {"T_MCC%i", GD_FLOAT64},    /* 10 */
  {"T_MCE%i", GD_FLOAT64},    /* 11 */
  {"dead_count_mce%i", GD_UINT16}, /* 12 */
  {"DRIVE_MAP_MPC%i", GD_UINT16}, /* 13 */
  {"last_tune_mpc%i", GD_UINT16}, /* 14 */
  {"last_iv_mpc%i", GD_UINT16}, /* 15 */
  {"used_tune_mpc%i", GD_UINT16}, /* 16 */
};

#define NGF 12
static const struct fft gf[NGF] = {
  {"mce_blob", GD_UINT16}, /* 0 */
  {"blob_num_mpc", GD_UINT16}, /* 1 */
  {"reporting_mpcs", GD_UINT16}, /* 2 */
  {"alive_mpcs", GD_UINT16}, /* 3 */
  {"squid_veto", GD_UINT16}, /* 4 */
  {"BSET_NUM", GD_UINT16}, /* 5 */
  {"data_mode", GD_UINT16}, /* 6 */
  {"UPPER_START_DMB", GD_UINT16}, /* 7 */
  {"UPPER_NBITS_DMB", GD_UINT16}, /* 8 */
  {"LOWER_START_DMB", GD_UINT16}, /* 9 */
  {"LOWER_NBITS_DMB", GD_UINT16}, /* 10 */
  {"mce_power", GD_UINT16}, /* 11 */
};

union du {
  double f64;
  uint64_t u64;
};
static union du d[6][NF];
static union du gd[NGF];

const char *dtasks[] = {"idle", "setdir", "dsp_rst", "mce_rst", "reconfig",
  "start_acq", "fakestop", "empty", "status", "acq_cnf", "tuning", "del_acq",
  "iv_curve", "stop", "stop_mce", "lcloop", "bstep", "bramp",
};
const char *goals[] = {"pause", "tune", "iv", "stop", "lcloop", "cycle",
  "acq", "bstep", "bramp"};
const char *modes[] = {"none", "tuning", "iv_curve", "lcloop", "running",
  "bstep", "bramp"};
#define N_STATES 7
const char *states[N_STATES] = {"drives", "active", "mcecom", "syncon",
  "config", "acqcnf", "retdat"};

static char drivemap(uint64_t map, int n)
{
  if (n == 0) {
    if (map & DRIVE0_UNMAP)
      return '!';
    return (map & DRIVE0_MASK) + '0';
  } else if (n == 1) {
    if (map & DRIVE1_UNMAP)
      return '!';
    return ((map & DRIVE1_MASK) >> 3) + '0';
  } else {
    if (map & DRIVE2_UNMAP)
      return '!';
    return ((map & DRIVE2_MASK) >> 6) + '0';
  }
}

static char bits[NGF][7];
char *mcebits(int n)
{
  int i;

  for (i = 0; i < 6; ++i)
    bits[n][i] = (gd[n].u64 & (1U << i)) ? i + '1' : '.';
  return bits[n];
}

const char *mce_power(int x)
{
  if (x == 1 || x == 2)
    return (gd[11].u64 & 0x1) ? "off" : "on";
  if (x == 3 || x == 5)
    return (gd[11].u64 & 0x2) ? "off" : "on";
  return (gd[11].u64 & 0x4) ? "off" : "on";
}

int main(int argc, char **argv)
{
  DIRFILE *D = gd_open((argc > 1) ? argv[1] : "/data/etc/defile.lnk", GD_RDONLY | GD_VERBOSE);
  if (!D)
    return 1;
  if (gd_error(D))
    return 1;

  initscr();

  for (;;) {
    int f, x;
    size_t n;
    off_t fn = gd_nframes64(D) - 5;
    printw("Frame: %lli  blob#%-6llu  mce_blob:0x%04llX  reporting:%s  "
        "alive:%s  veto:%s  bset:%03llu  dmode:%02llu  "
        "bits:%02llu+%02llu/%02llu+%02llu\n", (long long)fn, gd[1].u64,
        gd[0].u64, mcebits(2), mcebits(3), mcebits(4), gd[5].u64,
        gd[6].u64, gd[7].u64, gd[8].u64, gd[9].u64, gd[10].u64);
    char field[100];

    for (f = 0; f < NGF; ++f) {
      sprintf(field, gf[f].fmt, x + 1);
      if (gf[f].type == GD_UINT16)
        n = gd_getdata(D, field, fn, 0, 0, 1, GD_UINT64, &gd[f].u64);
      else if (gf[f].type == GD_UINT64)
        n = gd_getdata(D, field, fn, 0, 0, 1, GD_UINT64, &gd[f].u64);
      else if (gf[f].type == GD_FLOAT64)
        n = gd_getdata(D, field, fn, 0, 0, 1, GD_FLOAT64, &gd[f].f64);

      if (gd_error(D)) {
        endwin();
        return 1;
      }
    }

    for (x = 0; x < 6; ++x) {
      for (f = 0; f < NF; ++f) {
        sprintf(field, ff[f].fmt, x + 1);
        if (ff[f].type == GD_UINT16)
          n = gd_getdata(D, field, fn, 0, 0, 1, GD_UINT64, &d[x][f].u64);
        else if (ff[f].type == GD_UINT64)
          n = gd_getdata(D, field, fn, 0, 0, 1, GD_UINT64, &d[x][f].u64);
        else if (ff[f].type == GD_FLOAT64)
          n = gd_getdata(D, field, fn, 0, 0, 1, GD_FLOAT64, &d[x][f].f64);

        if (gd_error(D)) {
          endwin();
          return 1;
        }
        if (n == 0)
          d[x][f].u64 = 0;
      }
    }

    for (x = 0; x < 6; ++x)
      printw("            X%i           ", x + 1);
    printw("\n");

    for (x = 0; x < 6; ++x) {
      time_t t = d[x][0].u64;
      char ct[100];
      strcpy(ct, ctime(&t));
      ct[20] = 0;
      printw("   %-22s", ct);
    }
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" MCEpwr: %9s       ", mce_power(x));
    printw("\n");

    for (f = 0; f < 4; ++f) {
      for (x = 0; x < 6; ++x)
        printw(" free%i: %11.3f GB   ", f, d[x][2 + f].f64 / 1e9);
      printw("\n");
    }

    for (x = 0; x < 6; ++x)
      printw(" d map:          %c%c%c     ",
          drivemap(d[x][13].u64, 0),
          drivemap(d[x][13].u64, 1),
          drivemap(d[x][13].u64, 2));
    printw("\n");

    for (f = 0; f < N_STATES; ++f) {
      unsigned b = 1 << f;
      for (x = 0; x < 6; ++x)
        printw(" %6s:       ---%c---   ", states[f],
            (d[x][6].u64 & b) ? 'X' : '-');
      printw("\n");
    }

    for (x = 0; x < 6; ++x)
      printw("  mode: %12s     ", modes[d[x][6].u64 >> 8]);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  goal: %12s     ", goals[d[x][7].u64]);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  task:       0x%04X     ", d[x][8].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" dtask: %12s     ", dtasks[d[x][9].u64]);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" T mcc:  %10.2f oC   ", d[x][10].f64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" T mce:  %10.2f oC   ", d[x][11].f64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  dead:         %3llu      ", d[x][12].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("used tune:    %5llu      ", d[x][16].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("last tune:    %5llu      ", d[x][14].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("last iv:      %5llu      ", d[x][15].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("heater: %11.3f V    ", f, d[x][1].f64);
    printw("\n");

    refresh();
    usleep(500000);
    clear();
  }
}
