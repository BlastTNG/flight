#include <stdio.h>
#include <math.h>

/* global statistics */
uint8_t mean[NUM_ROW * NUM_COL];
uint8_t sigma[NUM_ROW * NUM_COL];
uint8_t noise[NUM_ROW * NUM_COL];

/* gains and offsets for statistics */
#define MEAN_GAIN        7.96618566056
#define MEAN_OFFSET      -4.52173913043
#define SGN(x,c)         (double)(((x) > c) - ((x) < c))
#define RESCALE_MEAN(x)  SGN((x),0.) * ( log(1. + fabs((double)(x))) * MEAN_GAIN + MEAN_OFFSET) + 128.
#define LOOKUP_MEAN(x)   SGN((x),128.) * (exp((fabs((x) - 128.) - MEAN_OFFSET) / MEAN_GAIN) - 1.)
#define SIGMA_GAIN       15.2685225161
#define SIGMA_OFFSET     -9.58333333333
#define RESCALE_SIGMA(x) log(1. + (x)) * SIGMA_GAIN + SIGMA_OFFSET
#define LOOKUP_SIGMA(x)  exp(((x) - SIGMA_OFFSET) / SIGMA_GAIN) - 1.
#define NOISE_GAIN       15.2685225161
#define NOISE_OFFSET     -9.58333333333
#define RESCALE_NOISE(x) log(1. + (x)) * NOISE_GAIN + NOISE_OFFSET
#define LOOKUP_NOISE(x)  exp(((x) - NOISE_OFFSET) / NOISE_GAIN) - 1.
