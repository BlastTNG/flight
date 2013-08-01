#include "mpc.h"
#include "mce_frame.h"
#include "bolo_stats.h"

/* filter parameters for band-passed data */
#define FILT_FREQ  5. // center frequency in Hz
#define FILT_BW    1.2 // bandwidth in fraction of center frequency
#define FILT_LEN   5  // length of coefficient arrays
double filt_coeffa[FILT_LEN]; // denominator coefficients
double filt_coeffb[FILT_LEN]; // numerator coefficients

/* buffers */
uint32_t frame_offset[NUM_ROW * NUM_COL];
double frame_filt[FB_SIZE][NUM_ROW * NUM_COL];
double frame_sum[NUM_ROW * NUM_COL];
double frame_sum2[NUM_ROW * NUM_COL];
double frame_fsum2[NUM_ROW * NUM_COL];

/* extract data from frame (assumes data mode 10!) */
#define FRAME_EXTRACT(frm,idx) (double)(frm[idx + MCE_HEADER_SIZE] >> 7)

static void set_filter_coeffs(const double fsamp,const double flow, const double fup)
{
  /* warp frequencies */
  double oml = tan(M_PI*flow/fsamp);
  double omu = tan(M_PI*fup/fsamp);
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
  
  bprintf(info, "Science band: %.2f Hz to %.2f Hz, at sample rate of %.2f",
	  flow, fup, fsamp);
  bprintf(info, "Science band filter: b = [ %.4f, %.4f, %.4f, %.4f, %.4f ]",
	  filt_coeffb[0], filt_coeffb[1], filt_coeffb[2], filt_coeffb[3], filt_coeffb[4]);
  bprintf(info, "Science band filter: a = [ %.4f, %.4f, %.4f, %.4f, %.4f ]",
	  filt_coeffa[0], filt_coeffa[1], filt_coeffa[2], filt_coeffa[3], filt_coeffa[4]);
}

void update_stats(const uint32_t *curr_frame, size_t frame_size, uint32_t frameno)
{
  const struct mas_header *header = (const struct mas_header *)curr_frame;
  
  int ii, jj, fb_idx[FILT_LEN];
  size_t ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE - 1;
  double datum, datum2, fdatum, dmean, dsigma, dnoise;
  
  double fsamp = 50.e6 / (double)(header->row_len 
				  * header->data_rate 
				  * header->num_rows);
  
  if (ndata > NUM_COL * NUM_ROW) ndata = NUM_COL * NUM_ROW;
  
  if (stat_reset == 1) {
    /* recalculate filter coefficients */
    set_filter_coeffs((const double) fsamp,
		      (const double) (FILT_FREQ*(1. - FILT_BW/2.)),
		      (const double) (FILT_FREQ*(1. + FILT_BW/2.)));
    
    /* reset buffers */
    for (ii = 0; ii < ndata; ii++) {
      frame_sum[ii] = 0;
      frame_sum2[ii] = 0;
      frame_fsum2[ii] = 0;
      frame_offset[ii] = FRAME_EXTRACT(curr_frame, ii);
      
      for (jj = 0; jj < FB_SIZE; jj++) frame_filt[jj][ii] = 0;
      
      mean[ii] = (uint8_t) MEAN_OFFSET;
      sigma[ii] = (uint8_t) SIGMA_OFFSET;
      noise[ii] = (uint8_t) NOISE_OFFSET;
    }
    
    stat_reset = 0;
  }
  
  /* neighboring buffer elements */
  fb_idx[0] = (fb_top + 1) % FB_SIZE; // oldest element
  // previous elements for filtering
  for (jj = 1; jj < FILT_LEN; jj++) {
    fb_idx[jj] = (fb_top - jj) % FB_SIZE;
    if (fb_idx[jj] < 0) fb_idx[jj] += FB_SIZE;
  }
  
  /* add new values */
  for (ii = 0; ii < ndata; ii++) {
    datum = (double) (FRAME_EXTRACT(curr_frame, ii) - frame_offset[ii]);
    frame_sum[ii] += datum;
    frame_sum2[ii] += datum * (double) datum;
    
    /* apply filter */
    fdatum = filt_coeffb[0] * datum;
    for (jj = 1; jj < FILT_LEN; jj++) {
      datum2 = (double) (FRAME_EXTRACT(frame[fb_idx[jj]], ii) - frame_offset[ii]);
      fdatum += (filt_coeffb[jj] * datum2 -
		 filt_coeffa[jj] * frame_filt[fb_idx[jj]][ii]);
    }
    fdatum /= filt_coeffa[0];
    frame_fsum2[ii] += fdatum*fdatum;
    frame_filt[fb_top][ii] = fdatum;
    
    /* update statistics */
    dmean = frame_sum[ii] / (double)(FB_SIZE);
    mean[ii] = (uint8_t) RESCALE_MEAN(dmean);
    dsigma = sqrt((frame_sum2[ii] - dmean * dmean * (double) FB_SIZE )
		  / (double)(FB_SIZE-1));
    sigma[ii] = (uint8_t) RESCALE_SIGMA(dsigma);
    dnoise = sqrt(frame_fsum2[ii] / (double)(FB_SIZE-1) / (double)(FILT_BW*FILT_FREQ));
    noise[ii] = (uint8_t) RESCALE_NOISE(dnoise);
    
    /* subtract oldest values from buffers */
    datum = (double) (FRAME_EXTRACT(frame[fb_idx[0]], ii) - frame_offset[ii]);
    frame_sum[ii] -= datum;
    frame_sum2[ii] -= datum * datum;
    fdatum = frame_filt[fb_idx[0]][ii];
    frame_fsum2[ii] -= fdatum * fdatum;
  }
}
