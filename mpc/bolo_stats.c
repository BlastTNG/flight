#include <stdio.h>
#include <math.h>

#include "mpc.h"
#include "mce_frame.h"

/* filter parameters for band-passed data */
#define NUM_FILT_COEFF   5  // length of coefficient arrays
double filt_coeffa[NUM_FILT_COEFF]; // denominator coefficients
double filt_coeffb[NUM_FILT_COEFF]; // numerator coefficients

uint8_t bolo_stat_buff[N_STAT_TYPES][NUM_ROW * NUM_COL];

/* buffers */
uint32_t frame_offset[NUM_ROW * NUM_COL];
double frame_filt[FB_SIZE][NUM_ROW * NUM_COL];
double frame_sum[NUM_ROW * NUM_COL];
double frame_sum2[NUM_ROW * NUM_COL];
double frame_fsum2[NUM_ROW * NUM_COL];

int sb_top = 0;

/* extract data from frame (assumes data mode 10!) */
#define FRAME_EXTRACT(frm,idx) (double)((frm[idx + MCE_HEADER_SIZE] >> 7) << 3)

static void set_filter_coeffs(const double fsamp, const double flow, const double fup)
{

  /* warp frequencies */
  double oml = tan(M_PI * flow / fsamp);
  double omu = tan(M_PI * fup / fsamp);
  double om02 = oml*omu;
  double om04 = om02*om02;
  double dom = omu - oml;
  double rt2dom = sqrt(2.)*dom;
  double dom2 = dom*dom;

  /* filter normalization */
  double norm = 1. + rt2dom + 2.*om02 + dom2 + rt2dom*om02 + om04;

  /* update coefficients -- 2-pole butterworth bandpass */
  filt_coeffb[0] = dom2/norm;
  filt_coeffb[1] = 0.;
  filt_coeffb[2] = -2.*dom2/norm;
  filt_coeffb[3] = 0.;
  filt_coeffb[4] = dom2/norm;

  filt_coeffa[0] = 1.;
  filt_coeffa[1] = (-2. - rt2dom + rt2dom*om02 + 2.*om04)*2./norm;
  filt_coeffa[2] = (3. - 2.*om02 - dom2 + 3.*om04)*2./norm;
  filt_coeffa[3] = (-2. + rt2dom - rt2dom*om02 + 2.*om04)*2./norm;
  filt_coeffa[4] = (1. - rt2dom - rt2dom*om02 + 2.*om02 + dom2 + om04)/norm;

  bprintf(info, "Science band: %.2f Hz center, %.2f Hz bandwidth, at sample rate of %.2f",
      (flow + fup) / 2., fup - flow, fsamp);
  bprintf(info, "Science band filter: b = [ %.4f, %.4f, %.4f, %.4f, %.4f ]",
      filt_coeffb[0], filt_coeffb[1], filt_coeffb[2], filt_coeffb[3], filt_coeffb[4]);
  bprintf(info, "Science band filter: a = [ %.4f, %.4f, %.4f, %.4f, %.4f ]",
      filt_coeffa[0], filt_coeffa[1], filt_coeffa[2], filt_coeffa[3], filt_coeffa[4]);
}

void update_stats(const uint32_t *curr_frame, size_t frame_size, uint32_t frameno)
{
  const struct mas_header *header = (const struct mas_header *)curr_frame;

  static double loc_filt_freq, loc_filt_bw;
  static int loc_filt_len;
  static double loc_bs_gain[N_STAT_TYPES];
  static int loc_bs_offset[N_STAT_TYPES];

  int ii, jj, sb_idx[NUM_FILT_COEFF], fb_idx[NUM_FILT_COEFF];
  size_t ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE - 1;
  double datum, datum2, fdatum, dmean, dsigma, dnoise, sgn;

  double fsamp = 50.e6 / (double)(header->row_len 
      * header->data_rate 
      * header->num_rows);

  if (ndata > NUM_COL * NUM_ROW) ndata = NUM_COL * NUM_ROW;

  if (stat_reset == 1) {

    loc_filt_len = memory.bolo_filt_len;
    loc_filt_freq = memory.bolo_filt_freq;
    loc_filt_bw = memory.bolo_filt_bw;
    for (ii = 0; ii < N_STAT_TYPES; ii++) {
      loc_bs_gain[ii] = memory.bolo_stat_gain[ii];
      loc_bs_offset[ii] = memory.bolo_stat_offset[ii];
    }

    /* recalculate filter coefficients */
    set_filter_coeffs((const double) fsamp,
        (const double) loc_filt_freq * ( 1. - loc_filt_bw / 2. ),
        (const double) loc_filt_freq * ( 1. + loc_filt_bw / 2. ));

    bprintf(info, "Statistics: buffer %d", loc_filt_len);
    bprintf(info, "Statistics: gains M = %.2f, S = %.2f, N = %.2f",
        loc_bs_gain[bs_mean],
        loc_bs_gain[bs_sigma],
        loc_bs_gain[bs_noise]);
    bprintf(info, "Statistics: offsets M = %i, S = %i, N = %i",
        loc_bs_offset[bs_mean],
        loc_bs_offset[bs_sigma],
        loc_bs_offset[bs_noise]);

    /* reset buffers */
    sb_top = 0;
    for (ii = 0; ii < ndata; ii++) {
      frame_sum[ii] = 0;
      frame_sum2[ii] = 0;
      frame_fsum2[ii] = 0;
      frame_offset[ii] = FRAME_EXTRACT(curr_frame, ii);

      for (jj = 0; jj < FB_SIZE; jj++) frame_filt[jj][ii] = 0;

      bolo_stat_buff[bs_mean][ii] = 128;
      bolo_stat_buff[bs_sigma][ii] = 0;
      bolo_stat_buff[bs_noise][ii] = 0;
    }

    stat_reset = 0;
  }

  /* neighboring buffer elements */
  fb_idx[0] = (fb_top + 1) % FB_SIZE; // oldest element
  sb_idx[0] = (sb_top + 1) % loc_filt_len; // oldest element
  // previous elements for filtering
  for (jj = 1; jj < NUM_FILT_COEFF; jj++) {
    fb_idx[jj] = (fb_top - jj) % FB_SIZE;
    if (fb_idx[jj] < 0) fb_idx[jj] += FB_SIZE;
    sb_idx[jj] = (sb_top - jj) % loc_filt_len;
    if (sb_idx[jj] < 0) sb_idx[jj] += loc_filt_len;
  }

  /* add new values */
  for (ii = 0; ii < ndata; ii++) {
    datum = (double) (FRAME_EXTRACT(curr_frame, ii) - frame_offset[ii]);
    frame_sum[ii] += datum;
    frame_sum2[ii] += datum * (double) datum;

    /* apply filter */
    fdatum = filt_coeffb[0] * datum;
    for (jj = 1; jj < NUM_FILT_COEFF; jj++) {
      datum2 = (double) (FRAME_EXTRACT(frame[fb_idx[jj]], ii) - frame_offset[ii]);
      fdatum += (filt_coeffb[jj] * datum2 -
          filt_coeffa[jj] * frame_filt[sb_idx[jj]][ii]);
    }
    fdatum /= filt_coeffa[0];
    frame_fsum2[ii] += fdatum*fdatum;
    frame_filt[sb_top][ii] = fdatum;

    /* update statistics */    
    dmean = frame_sum[ii] / (double)(loc_filt_len);
    dsigma = sqrt((frame_sum2[ii] - dmean * dmean * (double) loc_filt_len )
        / (double)(loc_filt_len - 1));

    sgn = (dmean > 0) - (dmean < 0);
    if ( fabs(dmean) < loc_bs_offset[bs_mean] ) dmean = loc_bs_offset[bs_mean];
    dmean = sgn * loc_bs_gain[0] * log (1. + fabs(dmean) - loc_bs_offset[0]);
    bolo_stat_buff[bs_mean][ii] = dmean + 128;

    if ( dsigma < loc_bs_offset[bs_sigma] ) dsigma = loc_bs_offset[bs_sigma];
    dsigma = loc_bs_gain[bs_sigma] * log ( 1. + dsigma - loc_bs_offset[bs_sigma] );
    bolo_stat_buff[bs_sigma][ii] = dsigma;

    dnoise = sqrt(frame_fsum2[ii] / (double)(loc_filt_len - 1) /
        (double)(loc_filt_freq * loc_filt_bw));
    if ( dnoise < loc_bs_offset[bs_noise] ) dnoise = loc_bs_offset[bs_noise];
    dnoise = loc_bs_gain[bs_noise] * log ( 1. + dnoise - loc_bs_offset[bs_noise] );
    bolo_stat_buff[bs_noise][ii] = dnoise;

    /* subtract oldest values from buffers */
    datum = (double) (FRAME_EXTRACT(frame[fb_idx[0]], ii) - frame_offset[ii]);
    frame_sum[ii] -= datum;
    frame_sum2[ii] -= datum * datum;
    fdatum = frame_filt[sb_idx[0]][ii];
    frame_fsum2[ii] -= fdatum * fdatum;
  }

  sb_top = (sb_top + 1) % loc_filt_len;
}
