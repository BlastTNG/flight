#include "stdint.h"

#ifndef __PYRAMID__H
#define __PYRAMID__H

#ifdef _WINDOWS_  // If in windows change packing order of frames to have
#pragma pack(2)    // same packing as Linux
#endif

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#define CSI 1.0E-10
#define MAXSOLUTION 200
#define MAXBLOBS 100

//type for entries in the star catalogue
typedef struct {
  double ra;                          //right ascension
  double dec;                         //declination
  float mag;                          //(apparent) magnitude
} gsc_t;

//type for entries in star pair & distance catalogue
typedef struct {
  //uint32_t I, J;                //catalog indexes of each star in pair
  //uint32_t K;                   // ?????
  uint32_t I, J;                //catalog indexes of each star in pair
  uint32_t K;                   // ?????
  double d;                          //distance between stars (on focal plane?)
} rec_t;

//type for pattern matching solutions
typedef struct {
  //uint32_t I[MAXBLOBS];       // Ignore - index into master catalogue
  //uint32_t B[MAXBLOBS];       // Blob index corresponding to C 
  uint32_t I[MAXBLOBS];       // Ignore - index into master catalogue
  uint32_t B[MAXBLOBS];       // Blob index corresponding to C 
  gsc_t* C[MAXBLOBS];              // Matched Star catalogue
  int flag;
  //uint32_t n;
  uint32_t n;
} solution_t;

//type for ???????
typedef struct {
  rec_t *r;
  uint32_t flag;
} match_t;


class Pyramid {
 public:
  Pyramid();
  Pyramid(double fov, const char *catalogname, const char *katalogname);
  void Init(double fov, const char *catalogname, const char *katalogname);
  ~Pyramid();  
  
  int BuildCatalog(double ra0, double dec0, double r0);


  int Match(double*, double*, double, uint32_t);
  int GetSolution(double, double*, double*, int, solution_t**, int*, 
                  double*, double*, double*);

  int GetTestStars(double, double, double *, double *, 
                   uint32_t *, uint32_t);
  
  double Ra(int I) {return gsc[I].ra;}
  double Dec(int I) {return gsc[I].dec;}
 private:
  void   SwapColumns(double *matrix, int ,int ,int, int);
  double* Product(double *matrix1, double *matrix2, int ,int , int);
  void   Transpose(double *matrix, int ,int);
  double Determinant3x3(double *matrix);
  void   GetCenter(double*, double*, int ,double&, double&, double&);
  double dist(double&, double&, double&, double&);
  double cdist(double&, double&, double&, double&);
  int    GetIdx(double, double, uint32_t&, uint32_t&);
  int    GetTriangle(uint32_t&, uint32_t&, 
                     uint32_t&, uint32_t n = 0); 
  int    s2tp(double, double, double, double, double&, double&);
  void   tp2s(double&, double&, double, double, double, double);

  int    StarPair(double* x, double*, uint32_t, double, solution_t*, uint32_t&); 

  int    StarTriangle(uint32_t, uint32_t, uint32_t,
                        double, double, double, double, solution_t*, uint32_t&);

  int    StarPyramid(uint32_t , double, double, double, double, solution_t*);


  int    CheckSpecularTriangle(double*, double*, solution_t*);
  // ***********************************************

  //uint32_t ngsc;          // number of starts in catalog
  uint32_t ngsc;          // number of starts in catalog
  gsc_t *gsc;                  // pointd to the guide star catalog
                               // sorted in increasing declination

  //uint32_t nrec;         //size of pair & distance catalogue
  uint32_t nrec;         //size of pair & distance catalogue
  rec_t *rec;                 // the catalogue itself


  solution_t *solution;       //array of possible pattern-matching solutions
  int nsolution;              //number of possible solutions

  int nkmap;
  int kidx;
  
  double c_fov;                // cos(fov);
  double fov;                  // field of view (intest of length or width) in radians of sky

  double m, q;                 //coefficients of a linear distance vs index trend for the rec array (?)

  char *catalog;               //star catalogue filename
  char *katalog;               //pair & distance catalogue filename

};


#endif
