#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mpc.h"
#include "mce_frame.h"

/* filter parameters for band-passed data */
#define NUM_FILT_COEFF   5  // length of coefficient arrays
double filt_coeffa[NUM_FILT_COEFF]; // denominator coefficients
double filt_coeffb[NUM_FILT_COEFF]; // numerator coefficients

uint8_t bolo_stat_buff[N_STAT_TYPES][NUM_ROW * NUM_COL];

/* buffers */
int32_t frame_offset[NUM_ROW * NUM_COL];
double frame_filt[FB_SIZE][NUM_ROW * NUM_COL];
double frame_local[FB_SIZE][NUM_ROW * NUM_COL];
double frame_sum[NUM_ROW * NUM_COL];
double frame_sum2[NUM_ROW * NUM_COL];
double frame_fsum2[NUM_ROW * NUM_COL];

int sb_top = 0;

#define STAT_RESET_TIMEOUT 10

/* extract data from frame (assumes data mode 10!) */
#define FRAME_EXTRACT(frm,idx) (int32_t)((frm[idx + MCE_HEADER_SIZE] >> 7) << 3)

/* log rescale */
#define RESCALE_LOG(x,g,o) (g) * log( (1. + ((x)<(o) ? (o) : (x))) / (1. + (o)) )

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
  static int reset_count = 0;
  static int scount = 0;
  // static int count = 0;
  
  int ii, jj, sb_idx[NUM_FILT_COEFF];
  size_t ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE - 1;
  double vthis, vlast, vfilt, vflast, v, dmean, dsigma, dnoise, norm;

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
    memset(frame_sum, 0, sizeof(double) * NUM_ROW * NUM_COL);
    memset(frame_sum2, 0, sizeof(double) * NUM_ROW * NUM_COL);
    memset(frame_fsum2, 0, sizeof(double) * NUM_ROW * NUM_COL);
    memset(frame_filt, 0, sizeof(double) * NUM_ROW * NUM_COL * FB_SIZE);
    memset(frame_local, 0, sizeof(double) * NUM_ROW * NUM_COL * FB_SIZE);

    memset(bolo_stat_buff[bs_sigma], 0, sizeof(uint8_t) * NUM_ROW * NUM_COL);
    memset(bolo_stat_buff[bs_noise], 0, sizeof(uint8_t) * NUM_ROW * NUM_COL);

    for (ii = 0; ii < ndata; ii++) {
      frame_offset[ii] = FRAME_EXTRACT(curr_frame, ii);
      bolo_stat_buff[bs_mean][ii] = 128;
    }

    stat_reset = 0;
    scount = 0;
    reset_count = 0;
  }
  
  /* ignore the first few frames after a reset */
  if (reset_count < STAT_RESET_TIMEOUT) {
    reset_count++;
    return;
  }
  
  norm = loc_filt_len;

  /* neighboring buffer elements */
  sb_idx[0] = (sb_top + 1) % loc_filt_len; // oldest element
  // previous elements for filtering
  for (jj = 1; jj < NUM_FILT_COEFF; jj++) {
    sb_idx[jj] = (sb_top - jj) % loc_filt_len;
    if (sb_idx[jj] < 0) sb_idx[jj] += loc_filt_len;
  }

  /* add new values */
  for (ii = 0; ii < ndata; ii++) {
    /* update statistics */
    vthis = FRAME_EXTRACT(curr_frame, ii) - frame_offset[ii];
    vlast = frame_local[sb_idx[0]][ii];
    frame_local[sb_top][ii] = vthis;

    frame_sum[ii] += (vthis - vlast);
    dmean = frame_sum[ii] / norm;

    frame_sum2[ii] += (vthis * vthis - vlast * vlast);
    dsigma = sqrt((frame_sum2[ii] - frame_sum[ii] * frame_sum[ii] / norm )
		  / (norm - 1.));

    /* apply filter */
    vfilt = filt_coeffb[0] * vthis;
    for (jj = 1; jj < NUM_FILT_COEFF; jj++) {
      vfilt += (filt_coeffb[jj] * frame_local[sb_idx[jj]][ii] -
		filt_coeffa[jj] * frame_filt[sb_idx[jj]][ii]);
    }
    vfilt /= filt_coeffa[0];
    vflast = frame_filt[sb_idx[0]][ii];
    frame_fsum2[ii] += (vfilt * vfilt - vflast * vflast);
    frame_filt[sb_top][ii] = vfilt;
    dnoise = sqrt(frame_fsum2[ii] / (norm - 1.) / (loc_filt_freq * loc_filt_bw));

    /* compress */
    v = 128 + ((dmean > 0) - (dmean < 0)) * 
      RESCALE_LOG(fabs(dmean), loc_bs_gain[bs_mean], loc_bs_offset[bs_mean]);
    bolo_stat_buff[bs_mean][ii] = (v < 0) ? 0 : ( (v > 255) ? 255 : v );

    v = RESCALE_LOG(dsigma, loc_bs_gain[bs_sigma], loc_bs_offset[bs_sigma]);
    bolo_stat_buff[bs_sigma][ii] = (v < 0) ? 0 : ( (v > 255) ? 255 : v );

    v = RESCALE_LOG(dnoise, loc_bs_gain[bs_noise], loc_bs_offset[bs_noise]);
    bolo_stat_buff[bs_noise][ii] = (v < 0) ? 0 : ( (v > 255) ? 255 : v );

#if 0
    if (ii==41) {
      if (count==loc_filt_len || scount < loc_filt_len + 5) {
        bprintf(info, "(%d) Last: off %d / this %d / last %d", scount, frame_offset[ii],
            (int) vthis, (int) vlast);
        bprintf(info, "(%d) Mean: %f / %d", scount, dmean, bolo_stat_buff[bs_mean][ii]);
        bprintf(info, "(%d) Sigma: %f / %f / %d", scount, frame_sum2[ii], dsigma, 
            bolo_stat_buff[bs_sigma][ii]);
        bprintf(info, "(%d) Noise: %f / %d", scount, dnoise, 
            bolo_stat_buff[bs_noise][ii]);
        count = 0;
      } else {
        count++;
      }
    }
#endif

  }

  scount++;
  sb_top = (sb_top + 1) % loc_filt_len;
}
