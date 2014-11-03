/*
 * state_estimator.h
 *
 *  Created on: 2013-09-30
 *      Author: javier
 */

#ifndef STATE_ESTIMATOR_H_
#define STATE_ESTIMATOR_H_

struct predictor
{
	// buffer variables
	int n; // state size
	int m; // buffer size
	int order; // order of predictor
	double dt; // time period
	int sat; // buffer saturation

	gsl_matrix * buffer;
	gsl_matrix * fit_matrix;
	gsl_matrix * meas_matrix;
	gsl_matrix * coeff;
	gsl_matrix * b_temp;
	gsl_vector * value;
	gsl_vector * time;
};

struct estimator
{
	// state variables
	int n; // size of the state
	int n_int; // size of interoceptive measurement
	int n_ext; // size of exteroceptive measurement
	int m; // size of the buffer
	double dt; // time step of the estimator
	int sat; // buffer saturation
	gsl_vector * x_k; // state
	gsl_matrix * C_k; // rotation matrix state
	gsl_matrix * P_k; // state covariance
	gsl_vector * e; // state error (for entire buffer)
	gsl_vector * dx; // state correction (for entire buffer)
	gsl_matrix * H; // state Jacobian (for entire buffer)
	gsl_matrix * H_t; // state Jacobian (for entire buffer)
	gsl_matrix * T; // noise Jacobian (for entire buffer);
	gsl_matrix * inv; // inverse matrix for GN algorithm

	gsl_matrix * x_k_buffer; // state buffer
	gsl_matrix * y_k_buffer; // exteroceptive measurement buffer
	gsl_matrix * u_k_buffer; // interoceptive measurement buffer

	// temporary variables
	gsl_matrix * AA;
	gsl_matrix * BB;
	gsl_matrix * CC;
	gsl_matrix * DD;
	gsl_vector * v;
	gsl_vector * w;
};

// some type definitions
#define arcsec 4.848136805555555555555555555e-6

typedef void (*state_trans)(gsl_vector *, gsl_vector *, gsl_vector *, double );
typedef void (*meas_trans)(gsl_vector *, gsl_vector *, gsl_vector *, double );

// define some function prototypes
void init_estimator(struct estimator *, int , int , int , int , double );

void init_predictor(struct predictor *, int , int , int , double );
int evaluate_predictor(struct predictor *, double );
void update_predictor_coeff(struct predictor *);
void update_predictor(struct predictor *, gsl_vector *);

void ekf_estimator(struct estimator *, gsl_vector *, gsl_vector *, gsl_matrix * , gsl_matrix * );
void sliding_window_estimator(struct estimator *, gsl_vector *, gsl_vector *, gsl_matrix *, gsl_matrix *);

/*void ekf_estimator(struct estimator *, gsl_vector *, gsl_vector *,
		void (*state_trans)(gsl_vector *, gsl_vector *, gsl_vector *, double),
		void (*meas_trans)(gsl_vector *, gsl_vector *, gsl_vector *, double ),
		gsl_matrix * , gsl_matrix * );*/

#endif /* STATE_ESTIMATOR_H_ */
