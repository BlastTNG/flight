/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2005-2006 University of Toronto
 * (C) 2015-2016 University of Pennsylvania
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#include "blast.h"
#include "fir.h"

static double sinc(double x)
{
    return abs(x) <= DBL_EPSILON ? 1.0f : sin(M_PI * x) / (M_PI * x);
}
static double rect(double n, double N, double alpha)
{
    (void)alpha;
    return n >= 0.0 && n < N ? 1.0 : 0.0;
}
static double hann(double n, double N, double alpha)
{
    (void)alpha;
    return 0.5 * (1.0 - cos(2.0 * M_PI * n / (N - 1.0)));
}
static double hamming(double n, double N, double alpha)
{
    (void)alpha;
    return 0.54 - 0.46 * cos(2.0 * M_PI * n / (N - 1.0));
}
static double lanczos(double n, double N, double alpha)
{
    (void)alpha;
    return sinc(2.0 * n / (N - 1.0) - 1.0);
}
static double gauss(double n, double N, double o)
{
    return exp(- 1.0/2.0 * pow((n - (N - 1.0) / 2.0) / (o * (N - 1.0) / 2.0), 2.0));
}
static double blackman(double n, double N, double alpha)
{
    return 0.5 * (1.0 - alpha) - 0.5 * cos(2.0 * M_PI * n / (N - 1.0)) + 0.5 * alpha * cos(4.0 * M_PI * n / (N - 1.0));
}
static double old_blast(double n, double N, double alpha)
{
    (void) alpha;
    return exp(- pow((n - N / 2.0) / (N / 4.5), 2.0));
}

typedef double (*window_fn)(double, double, double);
static window_fn window_filt[] = {
        [wind_oldblast] = old_blast,
        [wind_blackman] = blackman,
        [wind_gauss] = gauss,
        [wind_lanczos] = lanczos,
        [wind_hamming] = hamming,
        [wind_hann] = hann,
        [wind_rect] = rect
};

void init_fir(fir_t *fs, int N, int window, double alpha)
{
    int i;
    double sw = 0.0;

    for (i = 0; i < NSTAGE; i++) {
        fs->sum[i] = 0;
        fs->i_w[i] = i * N / NSTAGE;
    }

    fs->out = 0;
    fs->ns = N;

    fs->w = (double *) balloc(fatal, N * sizeof(double));

    if (window < wind_oldblast || window > wind_rect) window = wind_oldblast;
    for (i = 0; i < N; i++) {
        fs->w[i] = window_filt[window](i, N, alpha);
        sw += fs->w[i];
    }
    for (i = 0; i < N; i++) {
        fs->w[i] /= sw;
    }
}

void deinit_fir(fir_t *fs)
{
    bfree(err, fs->w);
}

double fir_filter(double x, fir_t *fs)
{
    int i, i_stage;

    for (i_stage = 0; i_stage < NSTAGE; i_stage++) {
        i = fs->i_w[i_stage];

        fs->sum[i_stage] += fs->w[i] * x;

        i++;
        if (i >= fs->ns) {
            fs->out = fs->sum[i_stage];
            i = 0;
            fs->sum[i_stage] = 0;
        }
        fs->i_w[i_stage] = i;
    }
    return (fs->out);
}

#if 0
int main()
{
    double x;
    struct FirStruct fs;
    int i;

    init_fir(&fs, 100.0 * 30.0 * 60.0);

    for (i = 0; i < 1000000; i++) {
        x = sin(2.0 * M_PI * (double) i / 4000) + sin(2.0 * M_PI * (double) i / 8000);
        printf("%g %g\n", x, fir_filter(x, &fs));
    }
    return (0);
}
#endif
