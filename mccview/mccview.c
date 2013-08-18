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

int first = 1;

struct fft { const char *fmt; gd_type_t type; };

#define NF 21
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
  {"ramp_count_mce%i", GD_UINT16}, /* 12 */
  {"DRIVE_MAP_MPC%i", GD_UINT16}, /* 13 */
  {"last_tune_mpc%i", GD_UINT16}, /* 14 */
  {"last_iv_mpc%i", GD_UINT16}, /* 15 */
  {"used_tune_mpc%i", GD_UINT16}, /* 16 */
  {"clamp_count_mce%i", GD_UINT16}, /* 17 */
  {"ref_tune_mpc%i", GD_UINT16}, /* 18 */
  {"tune_stat_mpc%i", GD_UINT16}, /* 19 */
  {"UPTIME_MPC%i", GD_FLOAT64}, /* 20 */
};

#define NGF 14
static const struct fft gf[NGF] = {
  {"mce_blob", GD_UINT16}, /* 0 */
  {"blob_num_mpc", GD_UINT16}, /* 1 */
  {"reporting_mpcs", GD_UINT16}, /* 2 */
  {"alive_mpcs", GD_UINT16}, /* 3 */
  {"squid_veto_mpc", GD_UINT16}, /* 4 */
  {"BSET_NUM", GD_UINT16}, /* 5 */
  {"data_mode_mce", GD_UINT16}, /* 6 */
  {"UPPER_START_DMB", GD_UINT16}, /* 7 */
  {"UPPER_NBITS_DMB", GD_UINT16}, /* 8 */
  {"LOWER_START_DMB", GD_UINT16}, /* 9 */
  {"LOWER_NBITS_DMB", GD_UINT16}, /* 10 */
  {"mce_power", GD_UINT16}, /* 11 */
  {"sync_veto_mpc", GD_UINT16}, /* 12 */
  {"therm_veto_mpc", GD_UINT16}, /* 13 */
};

union du {
  double f64;
  uint64_t u64;
};
static union du d[6][NF];
static union du gd[NGF];

#define N_DTASKS 19
const char *dtasks[] = {"idle", "setdir", "dsp_rst", "mce_rst", "reconfig",
  "start_acq", "fakestop", "empty", "status", "acq_cnf", "tuning", "del_acq",
  "iv_curve", "stop", "stop_mce", "lcloop", "bstep", "bramp", "kick"
};
#define N_GOALS 9
const char *goals[] = {"pause", "tune", "iv", "stop", "lcloop", "cycle",
  "acq", "bstep", "bramp"};
#define N_MODES 7
const char *modes[] = {"none", "tuning", "iv_curve", "lcloop", "running",
  "bstep", "bramp"};
#define N_STATES 9
const char *states[N_STATES] = {"POW", "DRI", "ACT", "COM", "SYN", "CFG",
  "BIA", "ACF", "RET"};

int sslen;

#define NSS 16
const char *ssf[NSS] = {
  "bc2:flux_fb(16)",
  "bc2:flux_fb(17)",
  "bc2:flux_fb(18)",
  "bc2:flux_fb(19)",
  "bc2:flux_fb(20)",
  "bc2:flux_fb(21)",
  "bc2:flux_fb(22)",
  "bc2:flux_fb(23)",
  "bc2:flux_fb(24)",
  "bc2:flux_fb(25)",
  "bc2:flux_fb(26)",
  "bc2:flux_fb(27)",
  "bc2:flux_fb(28)",
  "bc2:flux_fb(29)",
  "bc2:flux_fb(30)",
  "bc2:flux_fb(31)"
};
int ssn[NSS][6];
int sso;
uint16_t *ssv;

int ptot_count = 0;
#define PTOT (NF * 6 + NGF + 0 * NSS)
#define update_ptot() do { \
  if (first) { printf("\rinit %i/%i  ", ++ptot_count, PTOT); fflush(stdout); } \
} while(0)

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

int mce_power(int x)
{
  if (x == 1 || x == 2)
    return !(gd[11].u64 & 0x1);
  if (x == 3 || x == 5)
    return !(gd[11].u64 & 0x2);
  return !(gd[11].u64 & 0x4);
}

static char bits[NGF][7];
char *vetobits(int q, int t)
{
  int i;

  for (i = 0; i < 6; ++i)
    bits[q][i] = '.';

  /* power veto */
  for (i = 0; i < 6; ++i)
    if (!mce_power(i))
      bits[q][i] = 'p';

  /* thermal veto */
  for (i = 0; i < 6; ++i)
    if (gd[t].u64 & (1U << i))
      bits[q][i] = 't';

  /* user veto */
  for (i = 0; i < 6; ++i)
    if (gd[q].u64 & (1U << i))
      bits[q][i] = 'V';

  return bits[q];
}

char *mcebits(int n)
{
  int i;

  for (i = 0; i < 6; ++i)
    bits[n][i] = (gd[n].u64 & (1U << i)) ? i + '1' : '.';
  return bits[n];
}

void ss_init(void)
{
#if 0
  int i;
  FILE *stream = fopen("/data/etc/spider/format.mce_mplex", "r");
  char buffer[1024];

  if (stream == NULL) {
    perror("Unable to read format.mce_mplex");
    exit(1);
  }

  while (fgets(buffer, 1024, stream)) {
    if (buffer[0] == '#' || buffer[0] == '/')
      continue;

    /* initialise the length */
    if (sslen == 0) {
      sslen = atoi(buffer + 42);
      if (sslen % 20)
        sslen = (1 + sslen / 20);
      else
        sslen = sslen / 20;

      ssv = malloc(sizeof(uint16_t) * sslen * 20);
      memset(ssv, 0, sizeof(uint16_t) * sslen * 20);
      update_ptot();
    }

    /* is this a match? */
    for (i = 0; i < NSS; ++i) {
      size_t len = strlen(ssf[i]);
      if (strncmp(buffer + 3, ssf[i], len) == 0) {
        ssn[i][buffer[1] - '1'] = atoi(buffer + 30 + len);
        update_ptot();
      }
    }
  }

  fclose(stream);
#endif
}

void update_ss(DIRFILE *D, off_t lf, off_t fn)
{
}

char Abuffer[100];
const char *A(const char **v, int i, int l)
{
  if (i >= l) {
    sprintf(Abuffer, "(%i)", i);
    return Abuffer;
  }
  return v[i];
}

char delta_t_buffer[100];
const char *delta_t(double hours)
{
  int d = hours / 24;
  int h = hours - d * 24;
  int m = (hours - d * 24 - h) * 60;
  if (d > 0)
    sprintf(delta_t_buffer, "% 2i %02i:%02i", d, h, m);
  else
    sprintf(delta_t_buffer, "   %02i:%02i", h, m);

  return delta_t_buffer;
}

int main(int argc, char **argv)
{
  unsigned state[6];
  char field[100];
  int f, x;
  size_t n;
  off_t fn, lf = 0;

  DIRFILE *D = gd_open((argc > 1) ? argv[1] : "/data/etc/defile.lnk",
      GD_RDONLY | GD_VERBOSE);
  if (!D)
    return 1;
  if (gd_error(D))
    return 1;

  ss_init();

  for (;;) {
    fn = gd_nframes64(D) - 4;
    if (fn < 1) {
      fprintf(stderr, "\nNo data.\n");
      return 1;
    }
    update_ss(D, lf, fn);

    for (f = 0; f < NGF; ++f) {
      if (gf[f].type == GD_UINT16)
        n = gd_getdata(D, gf[f].fmt, fn, 0, 0, 1, GD_UINT64, &gd[f].u64);
      else if (gf[f].type == GD_UINT64)
        n = gd_getdata(D, gf[f].fmt, fn, 0, 0, 1, GD_UINT64, &gd[f].u64);
      else if (gf[f].type == GD_FLOAT64)
        n = gd_getdata(D, gf[f].fmt, fn, 0, 0, 1, GD_FLOAT64, &gd[f].f64);

      update_ptot();

      if (gd_error(D)) {
        if (!first)
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

        update_ptot();

        if (gd_error(D)) {
          if (!first)
            endwin();
          return 1;
        }
        if (n == 0)
          d[x][f].u64 = 0;
      }
    }

    if (first) {
      puts("");
      initscr();
      noecho();
      keypad(stdscr, TRUE);
      cbreak();
      first = 0;
    }

    printw("Frame: %lli  blob#%-6llu  mce_blob:0x%04llX  reporting:%s  "
        "alive:%s  veto:%s  sync_veto:%s  bset:%03llu  dmode:%02llu  "
        "bits:%02llu+%02llu/%02llu+%02llu\n", (long long)fn, gd[1].u64,
        gd[0].u64, mcebits(2), mcebits(3), vetobits(4, 13), mcebits(12),
        gd[5].u64, gd[6].u64, gd[7].u64, gd[8].u64, gd[9].u64, gd[10].u64);
    for (x = 0; x < 6; ++x)
      printw("            X%i           ", x + 1);
    printw("\n");

    for (x = 0; x < 6; ++x) {
      time_t t = d[x][0].u64;
      char ct[100];
      strcpy(ct, asctime(gmtime(&t)));
      ct[20] = 0;
      printw("   %-22s", ct);
    }
    printw("\n");

    /* add MCE power to the bottom of the state bitmap */
    for (x = 0; x < 6; ++x)
      state[x] = (unsigned)(d[x][6].u64 << 1) | (mce_power(x) ? 1 : 0);

    for (x = 0; x < 6; ++x)
      printw("uptime:  %12s    ", delta_t(d[x][20].f64));
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

    for (f = 0; f < N_STATES; f += 3) {
      unsigned b = 1 << f;
      for (x = 0; x < 6; ++x)
        printw(" %6s    %3s %3s %3s   ",
            (f == 3) ? "state:" : "",
            (state[x] & b) ? states[f] : "...",
            (state[x] & (b << 1)) ? states[f + 1] : "...",
            (state[x] & (b << 2)) ? states[f + 2] : "...");
      printw("\n");
    }

    for (x = 0; x < 6; ++x)
      printw("  mode: %12s     ", A(modes, d[x][6].u64 >> 8, N_MODES));
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  goal: %12s     ", A(goals, d[x][7].u64, N_GOALS));
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  task:       0x%04X     ", d[x][8].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" dtask: %12s     ", A(dtasks, d[x][9].u64, N_DTASKS));
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" T mcc:  %10.2f oC   ", d[x][10].f64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" T mce:  %10.2f oC   ", d[x][11].f64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("  ramp:         %3llu      ", d[x][12].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" clamp:         %3llu      ", d[x][17].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("used tune:    %5llu      ", d[x][16].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("last tune:    %5llu      ", d[x][14].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw(" ref tune:    %5llu      ", d[x][18].u64);
    printw("\n");

    for (x = 0; x < 6; ++x)
      printw("tune stat:     %04llx      ", d[x][19].u64);
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
