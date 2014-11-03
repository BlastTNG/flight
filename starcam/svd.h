#include <iostream>
#include <stdlib.h>

/* max(A,B) - larger (most +ve) of two numbers (generic) */
#define gmax(A,B) ((A)>(B)?(A):(B))

/* min(A,B) - smaller (least +ve) of two numbers (generic) */
#define gmin(A,B) ((A)<(B)?(A):(B))

/* dsign(A,B) - magnitude of A with sign of B (double) */
#define dsign(A,B) ((B)<0.0?-(A):(A))

/* logicals */
#if !defined(FALSE) || ((FALSE)!=0)
#define FALSE 0
#endif
#if !defined(TRUE) || ((TRUE)!=1)
#define TRUE 1
#endif

double rms ( double a, double b );
void svd ( int m, int n, int mp, int np, double *a, double *w, double *v, double *work, int *jstat );
