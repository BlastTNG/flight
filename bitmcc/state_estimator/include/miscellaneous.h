#ifndef MISC_H_
#define MISC_H_

#include <math.h>
#include <stdio.h>
#include <gsl/gsl_blas.h>


void print_matrix(const gsl_matrix * );
void print_vector(const gsl_vector * );
void C_x(gsl_matrix *, const double );
void C_y(gsl_matrix *, const double );
void C_z(gsl_matrix *, const double );
void xmat(gsl_matrix *, const gsl_vector *);
void unxmat(gsl_vector *, const gsl_matrix *);
void axis2rot(gsl_matrix *, const gsl_vector *, const double );
void horiz_concat(gsl_matrix * , const gsl_matrix * , const gsl_matrix * );
void vert_concat(gsl_matrix * , const gsl_matrix * , const gsl_matrix * );
void vec_concat(gsl_vector * , const gsl_vector * , const gsl_vector * );
void quadratic_mult_pd(const double , gsl_matrix * , gsl_matrix * , const double , gsl_matrix * );
void parameterize_312_rotation(gsl_vector * , const gsl_matrix * );
void compute_312_rotation_matrix(gsl_matrix * , const gsl_vector * );

#endif /* MISC_H_ */
