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

// for ground operations, in seconds
#define timeout_pyramid 10. //**LORENZO**

//type for entries in the star catalogue
typedef struct {
  double ra;                          //right ascension
  double dec;                         //declination
  float mag;                          //(apparent) magnitude
} gsc_t;

//type for entries in star pair & distance catalogue
typedef struct {
  unsigned long I, J;                //catalog indexes of each star in pair
  unsigned long K;                   // ?????
  double d;                          //distance between stars (on focal plane?)
} rec_t;

//type for pattern matching solutions
typedef struct {
  unsigned long I[MAXBLOBS];       // Ignore - index into master catalogue
  unsigned long B[MAXBLOBS];       // Blob index corresponding to C 
  gsc_t* C[MAXBLOBS];              // Matched Star catalogue
  int flag;
  unsigned long n;
} solution_t;

//type for ???????
typedef struct {
  rec_t *r;
  unsigned long flag;
} match_t;


class Pyramid {
 public:
  Pyramid();
  Pyramid(double fov, char *catalogname, char *katalogname);
  void Init(double fov, char *catalogname, char *katalogname);
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

  unsigned long nrec;         //size of pair & distance catalogue
  rec_t *rec;                 // the catalogue itself


  solution_t *solution;       //array of possible pattern-matching solutions
  int nsolution;              //number of possible solutions

  int nkmap;
  int kidx;
  
  double c_fov;                // cos(fov);
  double fov;                  // field of view (longest of length or width) in radians of sky

  double m, q;                 //coefficients of a linear distance vs index trend for the rec array (?)

  char *catalog;               //star catalogue filename
  char *katalog;               //pair & distance catalogue filename

};


#endif
