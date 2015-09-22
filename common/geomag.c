//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//
// GEOMAG
//
// Downloaded from http://www.ngdc.noaa.gov/seg/potfld/DoDWMM.shtml
//
// This code is an abomination (presumably translated straight from FORTRAN),
// and ugly to wade through but it seems efficient enough. I have changed a
// few minor things (mainly in terms of formatting) and I renamed the last
// two functions, which used to be called geomag & geomg1.
//
// I also got rid of main() so that it could be included in our mcp.
// MagModelInit should have the value of '12' passed to it.
//
// N.B. that this file needs the file WMM.COF which gives it necessary
// coefficients for the algorithm.  The file is good until 2005.
//
// The claim is that it is good to 30 arcminutes, with the usual warnings about
// magnetic storms and localised magnetic anomolies.
//
// Adam Hincks, 21 August 2002
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------



/* PROGRAM MAGPOINT (GEOMAG DRIVER) */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mcp.h"

static void E0000(int IENTRY, int *maxdeg, float alt, float glat, float glon,
    float time, float *dec, float *dip, float *ti, float *gv,
    const char* wmmFile) {

  static int maxord, i, icomp, n, m, j, D1, D2, D3, D4;
  static float c[13][13], cd[13][13], tc[13][13], dp[13][13], snorm[169],
  sp[13], cp[13], fn[13], fm[13], pp[13], k[13][13], pi, dtr, a, b, re,
  a2, b2, c2, a4, b4, c4, epoch, gnm, hnm, dgnm, dhnm, flnmj, otime, oalt,
  olat, olon, dt, rlon, rlat, srlon, srlat, crlon, crlat, srlat2,
  crlat2, q, q1, q2, ct, st, r2, r, d, ca, sa, aor, ar, br, bt, bp, bpp,
  par, temp1, temp2, parp, bx, by, bz, bh;
  static char model[20], c_str[81], c_new[5];
  static float *p = snorm;

  FILE *wmmdat;

  switch(IENTRY){case 0: goto GEOMAG; case 1: goto GEOMG1;}

GEOMAG:
  if ((wmmdat = fopen(wmmFile, "r")) == NULL) {
    berror(err, "World Magnetic Model: Error opening WMM.COF");
    return;
  }

  /* INITIALIZE CONSTANTS */
  maxord = *maxdeg;
  sp[0] = 0.0;
  cp[0] = *p = pp[0] = 1.0;
  dp[0][0] = 0.0;
  a = 6378.137;
  b = 6356.7523142;
  re = 6371.2;
  a2 = a * a;
  b2 = b * b;
  c2 = a2 - b2;
  a4 = a2 * a2;
  b4 = b2 * b2;
  c4 = a4 - b4;

  /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS */
  c[0][0] = 0.0;
  cd[0][0] = 0.0;
  if (fgets(c_str, 80, wmmdat) == NULL) {
    berror(err, "World Magnetic Model: Error while reading WMM.COF");
    return;
  }
  sscanf(c_str, "%f%s", &epoch, model);
S3:
  if (fgets(c_str, 80, wmmdat) == NULL) {
    berror(err, "World Magnetic Model: Error while reading WMM.COF");
    return;
  }
  /* CHECK FOR LAST LINE IN FILE */
  for (i = 0; i < 4 && (c_str[i] != '\0'); i++)
  {
    c_new[i] = c_str[i];
    c_new[i + 1] = '\0';
  }
  icomp = strcmp("9999", c_new);
  if (icomp == 0)
    goto S4;
  /* END OF FILE NOT ENCOUNTERED, GET VALUES */
  sscanf(c_str, "%d%d%f%f%f%f", &n, &m, &gnm, &hnm, &dgnm, &dhnm);
  if (m <= n)
  {
    c[m][n] = gnm;
    cd[m][n] = dgnm;
    if (m != 0)
    {
      c[n][m - 1] = hnm;
      cd[n][m - 1] = dhnm;
    }
  }
  goto S3;

  /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */
S4:
  *snorm = 1.0;
  for (n = 1; n <= maxord; n++)
  {
    *(snorm + n) = *(snorm + n - 1) * (float)(2 * n - 1) / (float)n;
    j = 2;
    for (m = 0, D1 = 1, D2 =(n - m + D1) / D1; D2 > 0; D2--, m += D1)
    {
      k[m][n] = (float)(((n - 1) * (n - 1)) - (m * m)) / (float)((2 * n - 1) *
          (2 * n - 3));
      if (m > 0)
      {
        flnmj = (float)((n - m + 1) * j) / (float)(n + m);
        *(snorm + n + m * 13) = *(snorm + n + (m - 1) * 13) * sqrt(flnmj);
        j = 1;
        c[n][m - 1] = *(snorm + n + m * 13) * c[n][m - 1];
        cd[n][m - 1] = *(snorm + n + m * 13) * cd[n][m - 1];
      }
      c[m][n] = *(snorm + n + m * 13) * c[m][n];
      cd[m][n] = *(snorm + n + m * 13) * cd[m][n];
    }
    fn[n] = (float)(n + 1);
    fm[n] = (float)n;
  }
  k[1][1] = 0.0;

  otime = oalt = olat = olon = -1000.0;
  fclose(wmmdat);
  return;

  /*************************************************************************/

GEOMG1:

  dt = time - epoch;
  if (otime < 0.0 && (dt < 0.0 || dt > 5.0))
  {
    blast_err("World Magnetic Model: !!!! Mag. model needs a year between "
        "%f and %f !!!!\n", epoch, epoch + 5.0);
    return;
  }

  pi = 3.14159265359;
  dtr = pi / 180.0;
  rlon = glon * dtr;
  rlat = glat * dtr;
  srlon = sin(rlon);
  srlat = sin(rlat);
  crlon = cos(rlon);
  crlat = cos(rlat);
  srlat2 = srlat * srlat;
  crlat2 = crlat * crlat;
  sp[1] = srlon;
  cp[1] = crlon;

  /* CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS. */
  if (alt != oalt || glat != olat)
  {
    q = sqrt(a2 - c2 * srlat2);
    q1 = alt * q;
    q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2));
    ct = srlat / sqrt(q2 * crlat2 + srlat2);
    st = sqrt(1.0 - (ct * ct));
    r2 = (alt * alt) + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q);
    r = sqrt(r2);
    d = sqrt(a2 * crlat2 + b2 * srlat2);
    ca = (alt + d) / r;
    sa = c2 * crlat * srlat / (r * d);
  }
  if (glon != olon)
  {
    for (m = 2; m <= maxord; m++)
    {
      sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
      cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
    }
  }
  aor = re / r;
  ar = aor * aor;
  br = bt = bp = bpp = 0.0;
  for (n = 1; n <= maxord; n++)
  {
    ar = ar * aor;
    for (m = 0, D3 = 1, D4 = (n + m + D3) / D3; D4 > 0; D4--, m += D3)
    {
      /*
         COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
         AND DERIVATIVES VIA RECURSION RELATIONS
         */
      if (alt != oalt || glat != olat)
      {
        if (n == m)
        {
          *(p + n + m * 13) = st * *(p + n - 1 + (m - 1) * 13);
          dp[m][n] = st * dp[m - 1][n - 1] + ct * *(p + n - 1 + (m - 1) * 13);
          goto S50;
        }
        if (n == 1 && m == 0)
        {
          *(p + n + m * 13) = ct * *(p + n - 1 + m * 13);
          dp[m][n] = ct * dp[m][n - 1] - st * *(p + n - 1 + m * 13);
          goto S50;
        }
        if (n > 1 && n != m)
        {
          if (m > n - 2)
            *(p + n - 2 + m * 13) = 0.0;
          if (m > n - 2)
            dp[m][n - 2] = 0.0;
          *(p + n + m * 13) = ct * *(p + n - 1 + m * 13) -
            k[m][n] * *(p + n - 2 + m * 13);
          dp[m][n] = ct * dp[m][n - 1] - st * *(p + n - 1 + m * 13) - k[m][n] *
            dp[m][n - 2];
        }
      }
S50:
      /*
         TIME ADJUST THE GAUSS COEFFICIENTS
         */
      if (time != otime)
      {
        tc[m][n] = c[m][n] + dt * cd[m][n];
        if (m != 0)
          tc[n][m - 1] = c[n][m - 1] + dt * cd[n][m - 1];
      }
      /*
         ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
         */
      par = ar * *(p + n + m * 13);
      if (m == 0)
      {
        temp1 = tc[m][n] * cp[m];
        temp2 = tc[m][n] * sp[m];
      }
      else
      {
        temp1 = tc[m][n] * cp[m] + tc[n][m - 1] * sp[m];
        temp2 = tc[m][n] * sp[m] - tc[n][m - 1] * cp[m];
      }
      bt = bt - ar * temp1 * dp[m][n];
      bp += (fm[m] * temp2 * par);
      br += (fn[n] * temp1 * par);
      /*
         SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
         */
      if (st == 0.0 && m == 1)
      {
        if (n == 1)
          pp[n] = pp[n - 1];
        else pp[n] = ct * pp[n - 1] - k[m][n] * pp[n - 2];
        parp = ar * pp[n];
        bpp += (fm[m] * temp2 * parp);
      }
    }
  }
  if (st == 0.0)
    bp = bpp;
  else bp /= st;
  /*
     ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
     GEODETIC COORDINATES
     */
  bx = -bt * ca - br * sa;
  by = bp;
  bz = bt * sa - br * ca;
  /*
     COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
     TOTAL INTENSITY (TI)
     */
  bh = sqrt((bx * bx) + (by * by));
  *ti = sqrt((bh * bh) + (bz * bz));
  *dec = atan2(by, bx) / dtr;
  *dip = atan2(bz, bh) / dtr;
  /*
     COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
     GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
     (I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)

     OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
     */
  *gv = -999.0;
  if (fabs(glat) >= 55.)
  {
    if (glat > 0.0 && glon >= 0.0)
      *gv = *dec - glon;
    if (glat > 0.0 && glon < 0.0)
      *gv = *dec + fabs(glon);
    if (glat < 0.0 && glon >= 0.0)
      *gv = *dec + glon;
    if (glat < 0.0 && glon < 0.0)
      *gv = *dec - fabs(glon);
    if (*gv > +180.0)
      *gv -= 360.0;
    if (*gv < -180.0)
      *gv += 360.0;
  }
  otime = time;
  oalt = alt;
  olat = glat;
  olon = glon;
  return;
}

/*************************************************************************/

void MagModelInit(int maxdeg, const char* wmmFile)
{
  int dummy;

  dummy = maxdeg;

  E0000(0, &dummy, 0.0, 0.0, 0.0, 0.0, NULL, NULL, NULL, NULL, wmmFile);
}

/*************************************************************************/

void GetMagModel(float alt, float glat, float glon, float time,
    float *dec, float *dip, float *ti, float *gv)
{
  E0000(1, NULL, alt, glat, glon, time, dec, dip, ti, gv, NULL);
}
/*************************************************************************/
