/* pyramid.cc: PYRAMID is
 * an implementation of 
 * D. Mortari, et al., "the PYRAMID star identification Techinque"
 *
 * This file is copyright (C) 2006 E. Pascale 
 *
 * PYRAMID is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * PYRAMID is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Deacon; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#pragma pack(2) 

#include <iostream>
#include <fstream>
#include <list>
#include <math.h>
#include <time.h>
//FIXME including stdio.h,stdlib.h - exit,srand,rand,qsort; cstdlib - RAND_MAX; stdio.h, string.h - memcpy
//cstdlib - exit,srand,rand,qsort
//cstring - memcpy
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "pyramid.h"

using namespace std;

//flag to activate display of debugging information
#define PYRAMID_DEBUG 0


//comparison function for sorting pointers to gsc_t (first by dec, then by ra)
// static int decl_sort(const void *x, const void *y)
// {
//   gsc_t *gsc_0 = (gsc_t *)x;
//   gsc_t *gsc_1 = (gsc_t *)y;
// 
//   int retval;
// 
//   if(gsc_0->dec < gsc_1->dec) {
//     retval = -1;
//   } else if(gsc_0->dec > gsc_1->dec) {
//     retval = 1;
//   } else if(gsc_0->ra <= gsc_1->ra) {
//     retval = -1;
//   } else retval = 1;
// 
//   return retval;
// }

//comparison function for sorting pointers to rec_t by distance (d)
static int dist_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->d <= r1->d) return -1;
  return 1;
}

//comparison function for sorting pointers of type rec_t by I (catalog index of first member of pair?)
static int rec_I_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->I <   r1->I) return -1;
  if(r0->I ==  r1->I) return  0;
  return 1;
}

//comparison function for sorting pointers of type rec_t by J (catalog index of second member of pair?)
static int rec_J_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->J <   r1->J) return -1;
  if(r0->J ==  r1->J) return  0;
  return 1;
}

//comparison function between pointer to unsigned long (x) and to I member of pointer to rect_t (y)
static int I_match(const void *x, const void *y)
{
  const unsigned long *I0    = (unsigned long *)x;
  const rec_t *r1  = (rec_t *)y;
  return (*I0 > r1->I) - (*I0 < r1->I);
}
//comparison function between pointer to unsigned long (x) and to J member of pointer to rect_t (y)
static int J_match(const void *x, const void *y)
{
  const unsigned long *I0    = (unsigned long *)x;
  const rec_t *r1  = (rec_t *)y;
  return (*I0 > r1->J) - (*I0 < r1->J);
}

/* Perform a binary search for KEY in BASE which has NMEMB elements
   of SIZE bytes each.  The comparisons are done by (*COMPAR)().  */
void *
pbsearch (const void *key, const void *base, size_t nmemb, size_t size,
          int (*compar) (const void *, const void *))
{
  size_t l, u, idx;
  const void *p;
  int comparison;

  l = 0;
  u = nmemb;
  while (l < u)
    {
      idx = (l + u) / 2;
      p = (void *) (((const char *) base) + (idx * size));
      comparison = (*compar) (key, p);
      if (comparison < 0)
        u = idx;
      else if (comparison > 0)
        l = idx + 1;
      else {
        do {
          if(idx == 0) return (void *)p;
          p = (void *) (((const char *) base) + (--idx * size));
        } while((*compar) (key, p) == 0);
        p = (void *) (((const char *) base) + (++idx * size));
        return (void *) p;
      }
    }

  return NULL;
}

//default constructor: must call Init before using the object
Pyramid::Pyramid()
{
	ngsc = nrec = nsolution = 0;
	gsc = NULL;
	rec = NULL;
	solution = NULL;
	catalog = katalog = "";
}
/* alternate constructor: calls Init
   must specify fov (longest of length or width in radians of sky)
                catalogname (star catalog filename)
                katalogname (catalogue of star pairs & distances (?))
*/
Pyramid::Pyramid(double fov, char *catalogname, char *katalogname)
{
	gsc = NULL;
	rec = NULL;
	solution = NULL;
	Init(fov, catalogname, katalogname);
}

void Pyramid::Init(double fov, char *catalogname, char *katalogname)
{
  unsigned long k;
  this->fov = fov;
  this->c_fov = cos(this->fov);

  this->catalog = catalogname;
  this->katalog = katalogname;

#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: opening file: " << this->catalog << endl;
#endif
  ifstream file(this->catalog, ios::in|ios::binary);
 
  if(!file.is_open()) {
#if PYRAMID_DEBUG
    cerr << "[Pyramid debug]: unable to open file catalog " << this->catalog << endl;
#endif
    exit(0);
  }

  file.read ((char *)&ngsc, sizeof(ngsc));
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: reading star catalogue data, ngsc = " << ngsc << endl;
#endif
  if (gsc) delete[] gsc; gsc = new gsc_t[ngsc];  
  
  for(k = 0; k < ngsc; k++) {
    file.read((char *)&gsc[k].ra, sizeof(gsc[k].ra));
    file.read((char *)&gsc[k].dec, sizeof(gsc[k].dec));
    file.read((char *)&gsc[k].mag, sizeof(gsc[k].mag));
  }
  file.close();  
  
  
  // load kvector catalog
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: opening file: " << this->katalog  << endl;
#endif
  file.open(this->katalog, ios::in|ios::binary);
  
  if(!file.is_open()) {
#if PYRAMID_DEBUG
    cerr << "[Pyramid debug]: unable to open file catalog " << this->katalog << endl;
#endif
    exit(0);
  }
  
  file.read ((char *)&this->m, sizeof(double));
  file.read ((char *)&this->q, sizeof(double));
  file.read ((char *)&this->nrec, sizeof(unsigned long));

#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: reading k-vector catalog, nrec = " << this->nrec << endl;
#endif
  
  if (rec) delete[] rec; rec = new rec_t[this->nrec];
  
  for(k = 0; k < this->nrec; k++) {
    file.read((char *)&rec[k].I, sizeof(rec[k].I));
    file.read((char *)&rec[k].J, sizeof(rec[k].J));
    file.read((char *)&rec[k].K, sizeof(rec[k].K));
    file.read((char *)&rec[k].d, sizeof(rec[k].d));
  }

  file.close();

  if (solution) delete[] solution; solution = NULL;
  nsolution = 0;
  return;
}

//destructor: unallocates arrays
Pyramid::~Pyramid()
{
  ngsc = 0;
  if (gsc) delete[] gsc;

  nrec = 0;
  if(rec) delete[] rec;

  nsolution = 0;
  if(solution) delete[] solution;
}

//finds the distance between two points with (RA, dec) in radians of (a0, d0) and (a1, d1)
//this distance is the cosine of the great-circle distance separating the points
double Pyramid::dist(double& a0, double& a1, double& d0, double& d1)
{
  double temp;

  temp  = cos(a0-a1);
  temp *= cos(d0)*cos(d1);
  temp += sin(d0)*sin(d1);
    
  return temp;
}


//finds (cosine of) distance between two tangent plane points
// Note: I cannot use tangent plane coordinates to calculate the inter-star
// distances; dicrepancies are order of 20" on 3.4 deg fov
double Pyramid::cdist(double& x0, double& x1, double& y0, double& y1){
  double temp;
  double denom;

  denom  = x0*x0 + y0*y0 + 1.0;
  denom *= x1*x1 + y1*y1 + 1.0;

  temp = (x0*x1 + y0*y1 + 1.0);
  temp /= sqrt(denom);

  return temp;
}

//a small number that is used to avoid division by zero errors
#define SMALLNUMBER 1.0E-9
//next two functions convert between spherical coordinates (ra, dec) and tangent plane coordinates (x, y)
//(raz, decz) is the location of the tangent point in spherical coordinates
//s2tp returns a status code depending on what denominator rounding might have been necessary
int Pyramid::s2tp(double ra, double dec, double raz, double decz, double& x, double& y)
{
  double sdecz, sdec, cdecz, cdec, radif, sradif, cradif, denom;
  int retval;
  
  // Trig functions 
  sdecz  = sin(decz);
  sdec   = sin(dec);
  cdecz  = cos(decz);
  cdec   = cos(dec);
  radif  = ra - raz;
  sradif = sin(radif);
  cradif = cos(radif);
  
  // Reciprocal of star vector length to tangent plane 
  denom = sdec * sdecz + cdec * cdecz * cradif;

  // Handle vectors too far from axis 
  if (denom > SMALLNUMBER) {
    retval = 0;
  } else if (denom >= 0.0) {
    retval = 1;
    denom = SMALLNUMBER;
  } else if (denom > -SMALLNUMBER) {
    retval = 2;
    denom = -SMALLNUMBER;
  } else {
    retval = 3;
  }
  
  // Compute tangent plane coordinates (even in dubious cases) 
  x  = cdec * sradif / denom;
  y = (sdec * cdecz - cdec * sdecz * cradif) / denom;

  return retval;
}

void Pyramid::tp2s(double& ra, double& dec, double raz, double decz, double x, double y)
{
   double sdecz, cdecz, denom, radif;

   sdecz = sin(decz);
   cdecz = cos(decz);

   denom = cdecz - y * sdecz;
   radif = atan2(x, denom);

   ra = radif + raz;
   dec = atan2(sdecz + y*cdecz, sqrt (x*x + denom*denom));
}
#undef SMALLNUMBER


//maximum number of stars to find in GetTestStars function
//finds the catalogue index (id) and coordinates (xx and yy) in the tagnet plane centered at ra0, dec0
//of up to n stars within half fov from tangent point (id, xx, and yy must be at least n elements long)
//when less than n stars are found, randomly chooses which ones to save, returns the number of stars returned
#define MAXSTARS 10000
int Pyramid::GetTestStars(double ra0, double dec0,
                          double *xx, double *yy, 
                          unsigned long *id,
                          unsigned long n)
{
  double c_fov_2;
  int star[MAXSTARS];

  unsigned long nstar;
  unsigned long star_idx;
  unsigned long k;
  
  c_fov_2 = cos(0.5*fov);
  for(k = 0, nstar = 0; k < ngsc; k++) {
    if(dist(ra0, gsc[k].ra, dec0, gsc[k].dec) < c_fov_2) {
      star[nstar] = k;
      if(++nstar == MAXSTARS) break;
    }
  }
  
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: Found " << nstar << "/" << n << " stars\n"; 
#endif
  if(nstar < n) n = nstar;

  srand((unsigned int)time(NULL));
  for(k = 0; k < n; k++) {
    do {
      star_idx = (int) ( (nstar-1)*((double)rand()/(double)RAND_MAX));
    } while(star[star_idx] == -1 && star_idx < nstar);
    
    s2tp(gsc[star[star_idx]].ra,gsc[star[star_idx]].dec,
         ra0, dec0, xx[k], yy[k]);

    id[k] = star[star_idx];
    
    star[star_idx] = -1;
  }


  return n;
}
#undef MAXSTARS


//finds the star catalgue indexes (id0 and id1) surrounding the distance val with an error ftol
//returns -1 on error, or 0 on success
int Pyramid::GetIdx(double val, double ftol, 
                    unsigned long& id0, unsigned long& id1)
{
  double error;
  double val_m, val_M;

  error = ftol*(sqrt(1 - val*val) + 0.5*val*ftol); 

  val_m = val - error; 
  val_M = val + error; 
  
  if (val_m < rec[0].d) {
#if PYRAMID_DEBUG
	  cerr << "[Pyramid debug]: GetIdx has val_m too low" << endl;
#endif
	  val_m = rec[0].d;
	  val_M = val_m + 2*error;
  }
  if (val_M > rec[nrec-1].d) {
#if PYRAMID_DEBUG
	  cerr << "[Pyramid debug]: GetIdx has val_M too high" << endl;
#endif
	  val_M = rec[nrec-1].d;
	  val_m = val_M - 2*error;
  }
//   if( val_m < rec[0].d || val_M > rec[nrec-1].d) {
// #if PYRAMID_DEBUG
//     cerr << "[Pyramid debug]: warning, index out of range" << endl;
// #endif
//     return -1;
//   }

  id0 = (unsigned long)floor((val_m - this->q) / this->m) - 1;
  id1 = (unsigned long) ceil((val_M - this->q) / this->m) - 1;


  if(id0 >= this->nrec || id1 >= this->nrec) return -1;
  
  id0 = rec[id0].K+1;
  id1 = rec[id1].K;

  return 0;
}

//when called with n>=3, creates lists of all possible 3-point combinations (n total points)
//after (n<3), it will return (ii, jj, and kk) one such combination and step to the next (returns 1)
//when all combinations have been exhausted, returns 0 again
int Pyramid::GetTriangle(unsigned long& ii, unsigned long& jj, 
                         unsigned long& kk, unsigned long n/*=0*/) 
{
  static unsigned long *Ti = NULL;
  static unsigned long *Tj = NULL;
  static unsigned long *Tk = NULL;
  static unsigned long ntriangles = 0;
  static unsigned long Tidx = 0;

  if(n >= 3) {
    unsigned long dj, dk, i;
    ntriangles = (n*(n-1)*(n-2)) / 6;      //n "choose" 3
    Tidx = 0;
    if(Ti != NULL) delete[] Ti; Ti = new unsigned long [ntriangles];
    if(Tj != NULL) delete[] Tj; Tj = new unsigned long [ntriangles];
    if(Tk != NULL) delete[] Tk; Tk = new unsigned long [ntriangles];

    for(dj = 1; dj <= (n-2); dj++) {
      for(dk = 1; dk <= (n - dj -1); dk++) {
        for(i = 1; i <= (n - dj - dk); i++) {
          Ti[Tidx] = i - 1;
          Tj[Tidx] = i + dj - 1;
          Tk[Tidx] = Tj[Tidx] + dk;
          Tidx++;
        }
      }
    }
    Tidx = 0;

    return 0;
  }

  if(Tidx < ntriangles) {
    ii = Ti[Tidx];
    jj = Tj[Tidx];
    kk = Tk[Tidx];
    Tidx++;
    return 1;
  }

  return 0;
}


int Pyramid::CheckSpecularTriangle(double* x, double* y, solution_t* S)
{
  double b0[3], b1[3], b2[3];
  double s0[3], s1[3], s2[3];
  double b_vol, s_vol;
  double tcos;

  b0[0] = x[S->B[0]]; b0[1] = y[S->B[0]]; b0[2] = 1.0;
  b1[0] = x[S->B[1]]; b1[1] = y[S->B[1]]; b1[2] = 1.0;
  b2[0] = x[S->B[2]]; b2[1] = y[S->B[2]]; b2[2] = 1.0;

  tcos = cos(gsc[S->I[0]].dec);
  s0[0] = cos(gsc[S->I[0]].ra)*tcos; s0[1] = sin(gsc[S->I[0]].ra)*tcos; s0[2] = sin(gsc[S->I[0]].dec); 
  tcos = cos(gsc[S->I[1]].dec);
  s1[0] = cos(gsc[S->I[1]].ra)*tcos; s1[1] = sin(gsc[S->I[1]].ra)*tcos; s1[2] = sin(gsc[S->I[1]].dec); 
  tcos = cos(gsc[S->I[2]].dec);
  s2[0] = cos(gsc[S->I[2]].ra)*tcos; s2[1] = sin(gsc[S->I[2]].ra)*tcos; s2[2] = sin(gsc[S->I[2]].dec); 
  
  b_vol  = b0[0]*(b1[1]*b2[2] - b1[2]*b2[1]);
  b_vol += b0[1]*(b1[2]*b2[0] - b1[0]*b2[2]);
  b_vol += b0[2]*(b1[0]*b2[1] - b1[1]*b2[0]);

  s_vol  = s0[0]*(s1[1]*s2[2] - s1[2]*s2[1]);
  s_vol += s0[1]*(s1[2]*s2[0] - s1[0]*s2[2]);
  s_vol += s0[2]*(s1[0]*s2[1] - s1[1]*s2[0]);

  return ((b_vol*s_vol > 0.0) ? 1 : 0);
}


//finds all pairs of stars that might match (within ftol) those in the x and y arrays (of length n)
//stores matches in sol (length nsol) and returns 0 if there are solutions (-1 if there aren't)
int Pyramid::StarPair(double* x, double* y, unsigned long n, double ftol,
                      solution_t* sol, unsigned long& nsol)
  
{
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: in StarPair function" << endl;
#endif
  unsigned long i,j,k;
  unsigned long idx_m, idx_M;
  double d;
  solution_t* S;
                                                
  nsol = 0;
  for(i = 0; i < n-1; i++) {
    for(j = i+1; j < n; j++) {
      d = cdist(x[i],x[j],y[i],y[j]);
      if(GetIdx(d, ftol, idx_m, idx_M)) continue;
      
      for(k = idx_m; k <= idx_M; k++) {
        S = &sol[nsol];
        S->n = 0;
        S->flag = 1;
        S->I[S->n] = rec[k].I;
        S->B[S->n++] = i;
        S->I[S->n] = rec[k].J;
        S->B[S->n++] = j;
        if(++nsol == MAXSOLUTION) return 0;
      }
    }
  }

  return ( (nsol) ? 0 : -1 );
}

//checks catalogue for star pairs separated by d1, d2, d3 (within ftol) that form triangle
//if so, those stars (indexes i, j, and k) are added to sol and nsol is incremented for each
//returns 0 if a solution is found or if nsol >= MAXSOLUTION, otherwise -1
int Pyramid::StarTriangle(unsigned long i, unsigned long j, unsigned long k,
                          double d1, double d2, double d3, double ftol,
                          solution_t* sol, unsigned long& nsol)
{
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: in StarTriangle function, indexes: (" << i << "," 
       << j << "," << k << ")" << endl;
#endif
  unsigned long l, p;
  unsigned long idx1_m, idx1_M, idx2_m, idx2_M, idx3_m, idx3_M;
  
  list<solution_t> pivot;
  list<solution_t>::iterator i_piv;
  solution_t P;

  int retval = -1;

  if( nsol >= MAXSOLUTION ) return 0;

  if(GetIdx(d1, ftol, idx1_m, idx1_M)) return retval; 
  if(GetIdx(d2, ftol, idx2_m, idx2_M)) return retval;     
  if(GetIdx(d3, ftol, idx3_m, idx3_M)) return retval;     
  
  unsigned long nrec2 = idx2_M - idx2_m + 1;
  rec_t *rec2 = new rec_t [nrec2];

  memcpy(rec2, &rec[idx2_m], nrec2*sizeof(rec_t));
  qsort(rec2, nrec2, sizeof(rec_t), rec_I_sort);

 
  rec_t *R;

  //search list of pairs separated by d2 (rec2) for an I that matches a star in a pair separated by d1
  //for all matches found, push the possible solution struct onto pivot
  for(l = idx1_m; l <= idx1_M; l++) {    
    // I in I
    if( (R = (rec_t *)pbsearch(&rec[l].I, rec2, nrec2, sizeof(rec_t), I_match)) != NULL) {
	  do {
        P.n    = 3;
        P.flag = 0;
        P.I[0] = rec[l].I;
        P.B[0] = i;
        P.I[1] = rec[l].J;
        P.B[1] = j;
        P.I[2] = R->J;
        P.B[2] = k;
        
        pivot.push_back(P);
        
        R++;
      } while(rec[l].I == R->I); 
    } 

    // J in I
    if( (R = (rec_t *)pbsearch(&rec[l].J, rec2, nrec2, sizeof(rec_t), I_match)) != NULL) {
	  do {
        P.n    = 3;
        P.flag = 0;
        P.I[0] = rec[l].J;
        P.B[0] = i;
        P.I[1] = rec[l].I;
        P.B[1] = j;
        P.I[2] = R->J;
        P.B[2] = k;
        
        pivot.push_back(P);
        
        R++;
      } while(rec[l].J == R->I); 
    }
  }  // loop on l

  //search list of pairs separated by d2 (rec2) for a J that matches a star in a pair separated by d1
  //for all matches found, push the possible solution struct onto pivot
  qsort(rec2, nrec2, sizeof(rec_t), rec_J_sort);
  for(l = idx1_m; l <= idx1_M; l++) {    
    // I in J
    if( (R = (rec_t *)pbsearch(&rec[l].I, rec2, nrec2, sizeof(rec_t), J_match)) != NULL) {
	  do {
        P.n    = 3;
        P.flag = 0;
        P.I[0] = rec[l].I;
        P.B[0] = i;
        P.I[1] = rec[l].J;
        P.B[1] = j;
        P.I[2] = R->I;
        P.B[2] = k;
        
        pivot.push_back(P);
        
        R++;
      } while(rec[l].I == R->J); 
    } 
    // J in J
    if( (R = (rec_t *)pbsearch(&rec[l].J, rec2, nrec2, sizeof(rec_t), J_match)) != NULL) {
	  do {
        P.n    = 3;
        P.flag = 0;
        P.I[0] = rec[l].J;
        P.B[0] = i;
        P.I[1] = rec[l].I;
        P.B[1] = j;
        P.I[2] = R->I;
        P.B[2] = k;
        
        pivot.push_back(P);
        
        R++;
      } while(rec[l].J == R->J); 
    }
  
  } // loop on l

  delete[] rec2;

  //for every solution in the pivot list, check if the two unmatched endpoints separated by d3
  //if so, then a solution is found and it is added to the array for return
  for(i_piv = pivot.begin(); i_piv != pivot.end(); i_piv++) {
    for(p = idx3_m; p <= idx3_M; p++) {
      if( (i_piv->I[2] == rec[p].I && i_piv->I[1] == rec[p].J) || 
          (i_piv->I[2] == rec[p].J && i_piv->I[1] == rec[p].I) ) {
        // Found triangle
        sol[nsol] = *i_piv;
        retval = 0;        
        if(++nsol == MAXSOLUTION) return retval;
        break;
      }
    }

  }

  return retval;
}


//checks a star of blob index r, that is d1, d2, and d3 away from the 3 stars in S
//if 4th star fits it is added to the solution and 0 is returned, otherwise -1 is returned
int Pyramid::StarPyramid(unsigned long r, 
                         double d1, double d2, double d3, double ftol,
                         solution_t* S)
{
  unsigned long l,m,p;
  unsigned long p1, p2, p3;
  unsigned long idx1_m, idx1_M, idx2_m, idx2_M, idx3_m, idx3_M;
  int retval = -1;
  
  if(S->n == MAXSOLUTION) return retval;

  if(GetIdx(d1, ftol, idx1_m, idx1_M)) return retval;
  if(GetIdx(d2, ftol, idx2_m, idx2_M)) return retval;
  if(GetIdx(d3, ftol, idx3_m, idx3_M)) return retval;

  for(l = idx1_m; l <= idx1_M; l++) {
    if(rec[l].I == S->I[0]) {
      p1 = rec[l].J;
    } else if(rec[l].J == S->I[0]) {
      p1 = rec[l].I;
    } else {
      continue;
    }
    for(m = idx2_m; m <= idx2_M; m++) {
      if(rec[m].I == S->I[1]) {
        p2 = rec[m].J;
      } else if(rec[m].J == S->I[1]) {
        p2 = rec[m].I;
      } else {
        continue;
      }
      for(p = idx3_m; p <= idx3_M; p++) {
        if(rec[p].I == S->I[2]) {
          p3 = rec[p].J;
        } else if(rec[p].J == S->I[2]) {
          p3 = rec[p].I;
        } else {
          continue;
        }
        
        // Candidate triad
        if(p1 == p2 && p1 == p3) {
          S->I[S->n] = p1;
          S->B[S->n++] = r;
          return 0; // Match achieved: quit here.
        }
      }
    }
  }
  
  return retval;
}


//seraches tangent plane coordinates (x and y with length n) of stars for 
//pattern matches in star catalog (within tolerance ftol)
int Pyramid::Match(double* x, double* y, double ftol, unsigned long n)

{
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: in Match function" << endl;
#endif
  unsigned long i, j, k, r;
  double d1, d2, d3;

  solution_t sol_[MAXSOLUTION];
  solution_t *S;
  unsigned long nsol_ = 0;
  unsigned long nsol = 0;
  unsigned long sol_idx, sol_start;
  int pyr_cnt = 0, pyr_flag;

  int retval = -1;

  if(n > MAXBLOBS) n = MAXBLOBS;
  switch (n) {
  case 0:  retval = 0; nsol = 0; break;
  case 1:  retval = 0; nsol = 0; break;
  case 2: 
    StarPair(x, y, n, ftol, sol_, nsol_);
    nsol = nsol_;
    retval = (nsol) ? 2 : 0;
    break;
  case 3: 
    i = 0; j = 1; k = 2;
    d1 = cdist(x[i], x[j], y[i], y[j]);
    d2 = cdist(x[i], x[k], y[i], y[k]);
    d3 = cdist(x[j], x[k], y[j], y[k]);
    
    if(StarTriangle(i,j,k,d1,d2,d3, ftol, sol_, nsol_) ) {
      
      for(sol_idx = 0; sol_idx < nsol_; sol_idx++) {
        S = &sol_[sol_idx];
        if(CheckSpecularTriangle(x, y, S)) {
          S->flag = 1;
          nsol++;
        }
      }
    }
    
    if(nsol == 0) { // no valid solutions found
      StarPair(x, y, n, ftol, sol_, nsol_);
      nsol = nsol_;
      retval = (nsol) ? 2 : 0;
    } else {
      retval = 3;
    }

    break;
    
  default: // More than three candidate stars
    GetTriangle(i,j,k,n);        //creates a list of possible triangles
  M0: 
    if(GetTriangle(i,j,k) == 0) {         //steps through list of triangles (assigns i,j,k)
#if PYRAMID_DEBUG
		cerr << "[Pyramid debug]: Checked all triangles\n";
#endif
      //check if pyramids were matched, if so continue
      if (pyr_cnt > 0) break;
      // check if there were some matched triangles and use those
      for(sol_idx = 0; sol_idx < nsol_; sol_idx++) {
        S = &sol_[sol_idx];
        if(CheckSpecularTriangle(x, y, S)) {
          S->flag = 1;
          nsol++;
        } 
      }
      if(nsol == 0) { // no valid solutions found
        StarPair(x, y, n, ftol, sol_, nsol_);
        nsol = nsol_;
        retval = (nsol) ? 2 : 0;
      } else {
        retval = 3;
      }
      break;
    }
	//else, this is a possible triangle so try to match to it

    d1 = cdist(x[i], x[j], y[i], y[j]);
    d2 = cdist(x[i], x[k], y[i], y[k]);
    d3 = cdist(x[j], x[k], y[j], y[k]);

    sol_start = nsol_;
	if(StarTriangle(i,j,k,d1,d2,d3, ftol, sol_, nsol_) ) {
#if PYRAMID_DEBUG
		cerr << "[Pyramid debug]: Failed to match triangle\n";
#endif
		goto M0;  //pattern matching failed, try next triangle
	}
#if PYRAMID_DEBUG
    cerr << "[Pyramid debug]: Found " << (nsol_ - sol_start) << " triangle matches" << endl;
#endif
    // Now I have some candidate triangles, look for the pyramid
    for(sol_idx = sol_start, pyr_flag = 0; sol_idx < nsol_; sol_idx++) {
      S = &sol_[sol_idx];
      for(r = 0; r < n; r++) {
        if( r == i || r == j || r == k) continue;
        
        d1 = cdist(x[i], x[r], y[i], y[r]);
        d2 = cdist(x[j], x[r], y[j], y[r]);
        d3 = cdist(x[k], x[r], y[k], y[r]);
        
        if(StarPyramid(r, d1, d2, d3, ftol, S) == 0) {
#if PYRAMID_DEBUG
          cerr << "[Pyramid debug]: Matched a pyramid!!\n";
#endif
          pyr_cnt++;
          S->flag = 1;
		  retval = 4;
		  nsol++;
        }

      } // loop on piramid r star
    } // loop on sol_ 

	if(!pyr_flag) {
#if PYRAMID_DEBUG
		cerr << "[Pyramid debug]: Failed to match pyramid\n";
#endif
		goto M0; // No matching pyramid found, start over
	}
  }
  
  if(solution != NULL) delete[] solution;
  solution = new solution_t [nsol];

  for(sol_idx = 0, k = 0; sol_idx < nsol_; sol_idx++) {
    if(sol_[sol_idx].flag) {
      memcpy(&solution[k], &sol_[sol_idx], sizeof(solution_t));
      
	  //memset(solution[k].C, 0, n*sizeof(gsc_t*));
      for(i = 0; i < solution[k].n; i++) {
        solution[k].C[i] = &gsc[solution[k].I[i]];
      }
      if(++k == nsol) break;
    }
  }

  nsolution = k;

  return retval;
}



//main function to perform pattern matching
//finds stars that match those with tangent plane coordinates x and y (nblobs of them), within ftol
//matches are added to array (sol, length nsol) (these don't need to be allocated before calling)
//ra0, dec0, and r0 currently do nothing, so this method is more or less a wrapper for the Match function
//returns 2, 3, or 4 for a pair, triangle, or pyramid match, 0 for no match and -1 for error
int Pyramid::GetSolution(double ftol, double* x, double* y,  int nblobs, 
                         solution_t** sol, int* nsol,
                         double ra0, double dec0, double r0)
{ 
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: in GetSolution function" << endl;
#endif
  int retval = 0;

#ifdef __TIMING__
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
#endif

  retval = Match(x, y, ftol, nblobs);
#if PYRAMID_DEBUG
  cerr << "[Pyramid debug]: pattern matching completed with return value: " << retval << "\n";
#endif
  
#ifdef __TIMING__
  gettimeofday(&t1, NULL);
  cerr << "[Pyramid debug]: It took " << ((t1.tv_sec + t1.tv_usec*1.0E-6) 
		  -(t0.tv_sec + t0.tv_usec*1.0E-6)) *1.0E3 << endl; 
#endif

//   if(retval > 3) lis = 0;

//   if(retval < 2) return retval;

//   if(retval == 2 && r0 > 0.0) {
//     solution_t sol_[MAXSOLUTION];
//     int nsol_;
//     int k;
//     double c_r0 = cos(r0);
//     for(k = 0, nsol_ = 0; k < nsolution; k++) {
//       if(dist(gsc[solution[k].I[0]].ra, ra0, gsc[solution[k].I[0]].dec, dec0) > c_r0) {
//      memcpy(sol_ + nsol_, &solution[k], sizeof(solution_t));
//      nsol_++;
//       }
//     }
//     delete solution;
//     solution = new solution_t [nsol_];
//     memcpy(solution, sol_, nsol_*sizeof(solution_t));
//     nsolution = nsol_;
//   }
  
  *sol = this->solution;
  *nsol = this->nsolution;
  

  return retval;
}

