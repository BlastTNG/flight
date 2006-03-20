#ifndef __PYRAMID__H
#define __PYRAMID__H

#define CATALOG "D:\\catalog\\gsc_mag08_res22.bin"
#define KATALOG "D:\\catalog\\k.bin"

#define FOV (3.4 * M_PI/180.0)    // SC field of view

#define CSI 1.0E-10
#define MAXSOLUTION 1000
#define MAXBLOBS 100

typedef struct {
  double ra;
  double dec;
  float mag;
} gsc_t;

typedef struct {
  unsigned long I, J;
  unsigned long K;
  double d;
} rec_t;

typedef struct {
  unsigned long I[MAXBLOBS];
  unsigned long B[MAXBLOBS];
  gsc_t* C[MAXBLOBS];
  int flag;
  unsigned long n;
} solution_t;


typedef struct {
  rec_t *r;
  unsigned long flag;
} match_t;

class Pyramid {
 public:
  Pyramid(double fov = FOV);
  ~Pyramid();  
  
  int BuildCatalog(double ra0, double dec0, double r0);


  int Match(double*, double*, double, unsigned long);
  int GetSolution(double, double*, double*, int, solution_t**, int*, 
		  double ra0=0.0, double dec0=0.0, double r0=-1.0);

  int GetTestStars(double, double, double *, double *, 
		   unsigned long *, unsigned long);
  
  double Ra(int I) {return gsc[I].ra;}
  double Dec(int I) {return gsc[I].dec;}
 private:
  
  double dist(double&, double&, double&, double&);
  double cdist(double&, double&, double&, double&);
  int    GetIdx(double, double, unsigned long&, unsigned long&);
  int    GetTriangle(unsigned long&, unsigned long&, 
		     unsigned long&, unsigned long n = 0); 
  int    s2tp(double, double, double, double, double&, double&);
  void   tp2s(double&, double&, double, double, double, double);

  int    StarPair(double* x, double*, unsigned long, double, solution_t*, unsigned long&); 

  int    StarTriangle(unsigned long, unsigned long, unsigned long,
			double, double, double, double, solution_t*, unsigned long&);

  int    Pyramid::StarPyramid(unsigned long , double, double, double, double, solution_t*);


  int    CheckSpecularTriangle(double*, double*, solution_t*);
  // ***********************************************

  unsigned long ngsc;          // number of starts in catalog
  gsc_t *gsc;                  // pointd to the guide star catalog
                               // sorted in increasing declination

  unsigned long nrec;
  rec_t *rec;


  solution_t *solution;
  int nsolution;

  int nkmap;
  int kidx;
  
  double c_fov;                // cos(fov);
  double fov;


  double m, q;

};











#endif
