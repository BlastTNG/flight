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

#include "pyramid.h"

using namespace std;


static int decl_sort(const void *x, const void *y)
{
  gsc_t *gsc_0 = (gsc_t *)x;
  gsc_t *gsc_1 = (gsc_t *)y;

  int retval;

  if(gsc_0->dec < gsc_1->dec) {
    retval = -1;
  } else if(gsc_0->dec > gsc_1->dec) {
    retval = 1;
  } else if(gsc_0->ra <= gsc_1->ra) {
    retval = -1;
  } else retval = 1;

  return retval;
}

static int dist_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->d <= r1->d) return -1;
  return 1;
}

static int rec_I_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->I <   r1->I) return -1;
  if(r0->I ==  r1->I) return  0;
  return 1;
}

static int rec_J_sort(const void *x, const void *y)
{
  rec_t *r0 = (rec_t *)x;
  rec_t *r1 = (rec_t *)y;

  if(r0->J <   r1->J) return -1;
  if(r0->J ==  r1->J) return  0;
  return 1;
}

static int I_match(const void *x, const void *y)
{
  const unsigned long *I0    = (unsigned long *)x;
  const rec_t *r1  = (rec_t *)y;
  return (*I0 > r1->I) - (*I0 < r1->I);
}
static int J_match(const void *x, const void *y)
{
  const unsigned long *I0    = (unsigned long *)x;
  const rec_t *r1  = (rec_t *)y;
  return (*I0 > r1->J) - (*I0 < r1->J);
}
// static int IJ_match(const void *x, const void *y)
// {
//   long long IJ0 = ((const rec_t *)x)->I << 8 |  
//   const unsigned long *I0    = (unsigned long *)x;
//   const rec_t *r1  = (rec_t *)y;
//   return (*I0 > r1->J) - (*I0 < r1->J);
// }
  

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

Pyramid::Pyramid(double fov)
{
  unsigned long k;
  this->fov = fov;
  this->c_fov = cos(this->fov);

  ifstream file(CATALOG, ios::in|ios::binary);
 
  if(!file.is_open()) {
    cerr << "Pyramid: unable to open file catalog " << CATALOG << endl;
    exit(0);
  }

  cerr << "Pyramid: reading data" << endl;

  file.read ((char *)&ngsc, sizeof(ngsc));
  gsc = new gsc_t [ngsc];  
  
  for(k = 0; k < ngsc; k++) {
    file.read((char *)&gsc[k].ra, sizeof(gsc[k].ra));
    file.read((char *)&gsc[k].dec, sizeof(gsc[k].dec));
    file.read((char *)&gsc[k].mag, sizeof(gsc[k].mag));
  }
  file.close();  
  
  
  // load kvector catalog
  cerr << "Pyramid: load " << KATALOG  << endl;
  file.open(KATALOG, ios::in|ios::binary);
  
  if(!file.is_open()) {
    cerr << "Pyramid: unable to open file catalog " << KATALOG << endl;
    exit(0);
  }
  
  file.read ((char *)&this->m, sizeof(double));
  file.read ((char *)&this->q, sizeof(double));
  file.read ((char *)&this->nrec, sizeof(unsigned long));

  cerr << "Pyramid: NREC IS NOW " << this->nrec << endl;
  
  this->rec = new rec_t [this->nrec];
  
  for(k = 0; k < this->nrec; k++) {
    file.read((char *)&rec[k].I, sizeof(rec[k].I));
    file.read((char *)&rec[k].J, sizeof(rec[k].J));
    file.read((char *)&rec[k].K, sizeof(rec[k].K));
    file.read((char *)&rec[k].d, sizeof(rec[k].d));
  }

  file.close();  

  solution = NULL;
}

Pyramid::~Pyramid()
{
  ngsc = 0;
  delete gsc;

  nrec = 0;
  if(rec) delete rec;

  nsolution = 0;
  if(solution != NULL) delete solution;
}


double Pyramid::dist(double& a0, double& a1, double& d0, double& d1)
{
  double temp;

  temp  = cos(a0-a1);
  temp *= cos(d0)*cos(d1);
  temp += sin(d0)*sin(d1);
    
  return temp;
}


// Note: I cannot use tangent plane coordinates to calculate the inter-star
// distances; dicrepancies are order of 20" on 3.4 deg fov
double Pyramid::cdist(double& x0, double& x1, double& y0, double& y1){
  double temp;
  double denom;

  denom  = x0*x0 + y0*y0 + 1.0;
  denom *= x1*x1 + y1*y1 + 1.0;

  temp = (x0*x1 + y0*y1 + 1.0)/sqrt(denom);

  return temp;
}

#define SMALLNUMBER 1.0E-9
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




#define MAXREC 8000000UL
int Pyramid::BuildCatalog(double ra0, double dec0, double r0)
{
  unsigned long i, i0, i1; 
  unsigned long j; 
  double dec_min, dec_max;
  double d;
  double c_r0;

  c_r0 = cos(r0);
  
  rec_t *rec_ = new rec_t [MAXREC]; 
  nrec = 0;
  
  dec_max = dec0 + 0.5*r0;
  dec_min = dec0 - 0.5*r0;
  if(dec_max > M_PI_2) {
    dec_max = M_PI_2;
    dec_min = dec_max - r0;
  } else if(dec_min < -M_PI_2) {
    dec_min = -M_PI_2;
    dec_max = dec_min + r0;
  }
//   if(dec_max > M_PI_2) {
//     dec_max = M_PI_2;
//   } else if(dec_min < -M_PI_2) {
//     dec_min = -M_PI_2;
//   }

  cerr << dec_min*180.0/M_PI << " " << dec_max*180./M_PI << endl;

  i0 = 0;
  while(gsc[i0].dec < dec_min) 
    if(++i0 == ngsc) break;
  
  i1 = i0;
  while(gsc[i1].dec <= dec_max) 
    if(++i1 == ngsc) break;

  cerr << "Pyramid: building distances catalog" << endl;
  for(i = i0; i < i1-1; i++) {
    d = dist(gsc[i].ra, ra0, gsc[i].dec, dec0);
    if(d < c_r0) continue;
    
    for(j = i+1; j < i1; j++) {      

      d = dist(gsc[i].ra, gsc[j].ra, gsc[i].dec, gsc[j].dec);
      if(d > c_fov) {
	if(i < j) {
	  rec_[nrec].I = i;
	  rec_[nrec].J = j;
	} else {
	  rec_[nrec].I = j;
	  rec_[nrec].J = i;
	}
 	rec_[nrec].d = d;
	if(++nrec == MAXREC) {
	  cerr << "Pyramid: too many star pairs" << endl;
	  delete rec_;
	  return -1;
	}
      } else if(cos(gsc[i].dec - gsc[j].dec) < c_fov) {
      	break;
      }
    }
  }

  if(rec != NULL) delete rec;
  rec = new rec_t [nrec];
  memcpy(rec, rec_, nrec*sizeof(rec_t));
  delete rec_;
  
  qsort((void *)rec, nrec, sizeof(*rec), dist_sort);
  
  cerr << "Pyramid: distance catalog has " << nrec << " elemenst" << endl;

  // Now build K vector
  this->m = (rec[nrec-1].d - rec[0].d + 2*CSI) / (double)(nrec - 1);
  this->q = rec[0].d - this->m - CSI;

  rec[0].K = 0;
  for(i = 1, j=0; i < nrec-1; i++) {
    while( ((rec[j].d > (m*(i+1) + q)) || (rec[j+1].d  <=  (m*(i+1) + q))) && j < nrec ) j++;
    rec[i].K = j;
  }
  rec[nrec-1].K = nrec-1;

  return 0;
}
#undef MAXREC


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
  
  c_fov_2 = cos(0.5*acos(c_fov));
  for(k = 0, nstar = 0; k < ngsc; k++) {
    if(dist(ra0, gsc[k].ra, dec0, gsc[k].dec) > c_fov_2) {
      star[nstar] = k;
      if(++nstar == MAXSTARS) break;
    }
  }
  
  cerr << "Found " << nstar << "/" << n << " stars\n"; 
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


int Pyramid::GetIdx(double val, double ftol, 
		    unsigned long& id0, unsigned long& id1)
{
  double error;
  double val_m, val_M;

  error = ftol*(sqrt(1 - val*val) + 0.5*val*ftol); 

  val_m = val - error; 
  val_M = val + error; 

  if( val_m < rec[0].d || val_M > rec[nrec-1].d) {
    cerr << "Pyramid: warning, index out of range" << endl;
    return -1;
  }

  id0 = (unsigned long)floor((val_m - this->q) / this->m) - 1;
  id1 = (unsigned long) ceil((val_M - this->q) / this->m) - 1;


  if(id0 >= this->nrec || id1 >= this->nrec) return -1;
  
  id0 = rec[id0].K+1;
  id1 = rec[id1].K;

  return 0;
}

int Pyramid::GetTriangle(unsigned long& ii, unsigned long& jj, 
			 unsigned long& kk, unsigned long n) 
{
  static unsigned long *Ti = NULL;
  static unsigned long *Tj = NULL;
  static unsigned long *Tk = NULL;
  static unsigned long ntriangles = 0;
  static unsigned long Tidx = 0;

  if(n >= 3) {
    unsigned long dj, dk, i;
    ntriangles = (n*(n-1)*(n-2)) / 6;
    Tidx = 0;
    if(Ti != NULL) delete Ti; Ti = new unsigned long [ntriangles];
    if(Tj != NULL) delete Tj; Tj = new unsigned long [ntriangles];
    if(Tk != NULL) delete Tk; Tk = new unsigned long [ntriangles];

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


int Pyramid::StarPair(double* x, double* y, unsigned long n, double ftol,
		      solution_t* sol, unsigned long& nsol)
  
{
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

int Pyramid::StarTriangle(unsigned long i, unsigned long j, unsigned long k,
			  double d1, double d2, double d3, double ftol,
			  solution_t* sol, unsigned long& nsol)
{
  unsigned long l, p;
  unsigned long idx1_m, idx1_M, idx2_m, idx2_M, idx3_m, idx3_M;
  
  list<solution_t> pivot;
  list<solution_t>::iterator i_piv;
  solution_t P;

  int retval = -1;

  if(GetIdx(d1, ftol, idx1_m, idx1_M)) return retval; 
  if(GetIdx(d2, ftol, idx2_m, idx2_M)) return retval;     
  if(GetIdx(d3, ftol, idx3_m, idx3_M)) return retval;     
  
  unsigned long nrec2 = idx2_M - idx2_m + 1;
  rec_t *rec2 = new rec_t [nrec2];

  memcpy(rec2, &rec[idx2_m], nrec2*sizeof(rec_t));
  qsort(rec2, nrec2, sizeof(rec_t), rec_I_sort);

 
  rec_t *R;

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

//   unsigned long nrec3 = idx3_M - idx3_m + 1;
//   rec_t *rec3 = new (rec_t) [nrec3];

//   memcpy(rec3, &rec[idx3_m], nrec3*sizeof(rec_t));
//   qsort(rec3, nrec3, sizeof(rec_t), rec_I_sort);

//   for(i_piv = pivot.begin(); i_piv != pivot.end(); i_piv++) {
//     if(pbsearch(i_piv, rec3, nrec3, sizeof(rec_t), IJ_match) == NULL) continue;
//     sol[nsol] = *i_piv;
//     retval = 0;
//     if(++nsol == MAXSOLUTION) return retval;   
//   }

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
	  //cerr << "ULLLALLLAAAAA Match is acieved (" << S->B[0] << " " << S->B[1] << " " << S->B[2] << " " << S->B[3] << " " << S->B[S->n-1]<< "\n";
	  return 0; // Match achieved: quit here.
	}
      }
    }
  }
  
  return retval;
}



int Pyramid::Match(double* x, double* y, double ftol, unsigned long n)

{
  unsigned long i, j, k, r;
  double d1, d2, d3;

  solution_t sol_[MAXSOLUTION];
  solution_t *S;
  unsigned long nsol_ = 0;
  unsigned long nsol = 0;
  unsigned long sol_idx, sol_start;
  int pyramid;

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
    GetTriangle(i,j,k,n);
  M0: 
    if(GetTriangle(i,j,k) == 0) {
      // There are no more triangles to be scanned
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
    
    d1 = cdist(x[i], x[j], y[i], y[j]);
    d2 = cdist(x[i], x[k], y[i], y[k]);
    d3 = cdist(x[j], x[k], y[j], y[k]);

    sol_start = nsol_;
    if(StarTriangle(i,j,k,d1,d2,d3, ftol, sol_, nsol_) ) goto M0;
    cerr << "Found " << nsol_ << " triangles" << endl;
    // Now I have some candidate triangles, look for the pyramid
    for(sol_idx = sol_start, pyramid = 0; sol_idx < nsol_ && !pyramid; sol_idx++) {
      S = &sol_[sol_idx];
      for(r = 0; r < n; r++) {
	if( r == i || r == j || r == k) continue;
	
	d1 = cdist(x[i], x[r], y[i], y[r]);
	d2 = cdist(x[j], x[r], y[j], y[r]);
	d3 = cdist(x[k], x[r], y[k], y[r]);
	
	if(StarPyramid(r, d1, d2, d3, ftol, S) == 0) {
	  pyramid = 1;
	  S->flag = 1;
	}

      } // loop on piramid r star
    } // loop on sol_ 

    if(!pyramid) goto M0; // No matching pyramid found, start over
    nsol++;
    retval = S->n;
    break;
  }
  
  if(solution != NULL) delete solution;
  solution = new solution_t [nsol];

  for(sol_idx = 0, k = 0; sol_idx < nsol_; sol_idx++) {
    if(sol_[sol_idx].flag) {
      memcpy(&solution[k], &sol_[sol_idx], sizeof(solution_t));
      
      memset(solution[k].C, 0, n*sizeof(gsc_t));
      for(i = 0; i < solution[k].n; i++) {
	solution[k].C[solution[k].B[i]] = &gsc[solution[k].I[solution[k].B[i]]];
      }
      if(++k == nsol) break;
    }
  }

  nsolution = k;

  return retval;
}



int Pyramid::GetSolution(double ftol, double* x, double* y,  int nblobs, 
			 solution_t** sol, int* nsol,
			 double ra0, double dec0, double r0)
{ 
  int retval = 0;

#ifdef __TIMING__
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
#endif

  cerr << "start\n";
  retval = Match(x, y, ftol, nblobs);
  cerr << "blaaaa\n";
  cerr << retval << endl;
  
#ifdef __TIMING__
  gettimeofday(&t1, NULL);
  cerr << "It took " << ((t1.tv_sec + t1.tv_usec*1.0E-6) -(t0.tv_sec + t0.tv_usec*1.0E-6)) *1.0E3 << endl; 
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
// 	memcpy(sol_ + nsol_, &solution[k], sizeof(solution_t));
// 	nsol_++;
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

