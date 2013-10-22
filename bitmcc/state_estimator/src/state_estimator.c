/*
 * state_estimator.c
 *
 *  Created on: 2013-09-30
 *      Author: javier
 */

// standard includes
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>
#include <hw/inout.h> // raw PC104 I/O routines
#include <sys/neutrino.h> // Neutrino kernel/thread control
#include <gsl/gsl_blas.h> // GSL math stuff
#include <gsl/gsl_linalg.h> // GSL math stuff

#include "miscellaneous.h" // linear algebra and rotation stuff
#include "state_estimator.h"

// defines the temporary workspace for the state estimator for state of size n, buffer size n, and sampling period dt
void init_estimator(struct estimator *est, int n, int n_int, int n_ext, int m, double dt)
{
	est->n = n;
	est->n_int = n_int;
	est->n_ext = n_ext;
	est->m = m;
	est->dt = dt;
	est->x_k = gsl_vector_calloc(n);
	est->C_k = gsl_matrix_calloc(3,3);
	est->P_k = gsl_matrix_calloc(n,n);
	est->H = gsl_matrix_calloc(m*(n+n_ext),(m+1)*n);
	est->H_t = gsl_matrix_calloc((m+1)*n,m*(n+n_ext));
	est->inv = gsl_matrix_calloc((m+1)*n,(m+1)*n);
	est->T = gsl_matrix_calloc(m*(n+n_ext),m*(n+n_ext));

	est->x_k_buffer = gsl_matrix_calloc(n,m+1); // m+1 because includes current state x_k and initial state x_0
	est->u_k_buffer = gsl_matrix_calloc(n_int,m);
	est->y_k_buffer = gsl_matrix_calloc(n_ext,m);

	est->e = gsl_vector_calloc((n+n_ext)*m);
	est->dx = gsl_vector_calloc(n*(m+1));
	est->sat = 0;

	est->AA = gsl_matrix_calloc(2*n,2*n);
	est->BB = gsl_matrix_calloc(2*n,2*n);
	est->CC = gsl_matrix_calloc(2*n,2*n);
	est->DD = gsl_matrix_calloc(2*n,2*n);
	est->v = gsl_vector_calloc(100);
	est->w = gsl_vector_calloc(100);
}

// initializes a predictor with state size n, buffer size m, and timestep dt
void init_predictor(struct predictor *pred, int n, int m, int order, double dt)
{
	pred->n = n;
	pred->m = m;
	pred->order = order;
	pred->dt = dt;
	pred->buffer = gsl_matrix_calloc(n,m);
	pred->meas_matrix = gsl_matrix_calloc(order+1,m);
	pred->fit_matrix = gsl_matrix_calloc(order+1,order+1);
	pred->coeff = gsl_matrix_calloc(order+1,n);
	pred->value = gsl_vector_calloc(n);
	pred->b_temp = gsl_matrix_calloc(pred->order+1,pred->n);
	pred->time = gsl_vector_calloc(order+1);
	pred->sat = 0;

	// pre-compute measurement and fit matrices
	int i,j;
	for (i=0;i<m;i++)
	{
		for (j=0;j<(order+1);j++)
		{
			gsl_matrix_set(pred->meas_matrix,j,i,pow((i+1),order-j));
		}
	}
	gsl_blas_dgemm(CblasNoTrans,CblasTrans, 1.0, pred->meas_matrix, pred->meas_matrix, 0.0, pred->fit_matrix);

	gsl_linalg_cholesky_decomp(pred->fit_matrix);
	gsl_linalg_cholesky_invert(pred->fit_matrix);
}


// propagates the predictor and generates a prediction t seconds ahead if the buffer is full (return 1);
// otherwise, returns 0
int evaluate_predictor(struct predictor *pred, double t)
{
	int i;
	double proj_t = t/pred->dt;
	if (pred->sat >= pred->m)
	{
		for (i=0;i<=pred->order;i++) gsl_vector_set(pred->time,i,pow(proj_t,pred->order-i));
		gsl_blas_dgemv(CblasTrans, 1.0, pred->coeff, pred->time, 0.0, pred->value);
		return 1;
	}
	else return 0;
}

// updates the predictor buffer and sets the predictor as saturated if the buffer has saturated
void update_predictor(struct predictor * pred, gsl_vector * meas)
{
	if (meas->size != pred->n) // size mismatch
	{
		printf("Update predictor measurement dimension mismatch.\n");
		return;
	}

	int i;

	if (pred->sat < pred->m) // fill the buffer
	{
		gsl_matrix_set_col(pred->buffer,pred->sat,meas);
		(pred->sat)++;

	}
	else // add measurement to buffer and dump oldest measurement
	{
		for (i=1; i<pred->m; i++)
		{
			gsl_matrix_swap_columns(pred->buffer,i-1,i);
		}
		gsl_matrix_set_col(pred->buffer,pred->m-1,meas);
	}
	update_predictor_coeff(pred);
}

// generate fit with measurements if buffer is full
void update_predictor_coeff(struct predictor * pred)
{
	if (pred->sat >= pred->m)
	{
		gsl_blas_dgemm(CblasNoTrans,CblasTrans, 1.0, pred->meas_matrix, pred->buffer, 0.0, pred->b_temp);
		gsl_blas_dgemm(CblasNoTrans,CblasNoTrans, 1.0, pred->fit_matrix, pred->b_temp, 0.0, pred->coeff);
	}
}

void ekf_estimator(struct estimator *est, gsl_vector * u_k, gsl_vector * y_k, gsl_matrix * Q_k, gsl_matrix * R_k)
{
	/* Definitions
	 * -------------------
	 * x_k = [orientation; biases]
	 * u_k = w_bit;
	 * y_k = orientation measurement from camera
	 */


	gsl_matrix_view A, B, C, D;
	gsl_vector_view v, w;
    gsl_vector_view phi_k, b_k;

    phi_k = gsl_vector_subvector(est->x_k,0,3);
    b_k = gsl_vector_subvector(est->x_k,3,3);

	double psi_k_mag;

	double dt = est->dt;
	int n = est->n;

	// compute the PSI matrix
	v = gsl_vector_subvector(est->v,0,3); // v = w_bit
	gsl_vector_memcpy(&v.vector,u_k);
	gsl_vector_add(&v.vector,&b_k.vector);
	gsl_vector_scale(&v.vector,dt);
	psi_k_mag = gsl_blas_dnrm2(&v.vector);
	A = gsl_matrix_submatrix(est->AA,0,0,3,3); // A = PSI_k
	if (psi_k_mag == 0) gsl_matrix_set_identity(&A.matrix);
	else
	{
		gsl_vector_scale(&v.vector,1.0/psi_k_mag);
		axis2rot(&A.matrix,&v.vector,psi_k_mag);
	}

	// compute predicted state
	B = gsl_matrix_submatrix(est->BB,0,0,3,3); // B = C_km1
	compute_312_rotation_matrix(&B.matrix,&phi_k.vector);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &A.matrix, &B.matrix, 0.0, est->C_k);

	// compute state transition and noise Jacobian
	gsl_matrix_view tl = gsl_matrix_submatrix(est->BB, 0, 0, 3, 3);
	gsl_matrix_view tr = gsl_matrix_submatrix(est->BB, 0, 3, 3, 3);
	gsl_matrix_view br = gsl_matrix_submatrix(est->BB, 3, 3, 3, 3);
	gsl_matrix_view bl = gsl_matrix_submatrix(est->BB, 3, 0, 3, 3);

	B = gsl_matrix_submatrix(est->BB,0,0,6,6); // B = H_xk

	gsl_matrix_memcpy(&tl.matrix, &A.matrix);
	gsl_matrix_set_identity(&tr.matrix);
	gsl_matrix_set_identity(&br.matrix);
	gsl_matrix_set_zero(&bl.matrix);

	A = gsl_matrix_submatrix(est->AA,0,0,n,3); // A = H_wk
	gsl_matrix_view t = gsl_matrix_submatrix(&A.matrix, 0, 0, 3, 3);
	gsl_matrix_view b = gsl_matrix_submatrix(&A.matrix, 3, 0, 3, 3);

	gsl_matrix_set_identity(&t.matrix);
	gsl_matrix_scale(&t.matrix, dt);
	gsl_matrix_set_identity(&b.matrix);

	// compute the predicted covariance
	C = gsl_matrix_submatrix(est->CC,0,0,n,n); // C = P_km1
	gsl_matrix_memcpy(&C.matrix,est->P_k);
	quadratic_mult_pd(1.0, &B.matrix, &C.matrix, 0.0, est->P_k);
	quadratic_mult_pd(1.0, &A.matrix, Q_k, 1.0, est->P_k); // P_k = P_km

	// compute the measurement error
	B = gsl_matrix_submatrix(est->BB,0,0,3,3); // B = E_k
	A = gsl_matrix_submatrix(est->AA,0,0,3,3); // A = C_meas
	compute_312_rotation_matrix(&A.matrix,y_k);
	gsl_matrix_set_identity(&B.matrix);
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, -1.0, &A.matrix, est->C_k, 1.0, &B.matrix);
	unxmat(&v.vector,&B.matrix); // v = e_k

	// compute measurement Jacobians
	A = gsl_matrix_submatrix(est->AA,0,0,3,n); // A = G_xk;
	gsl_matrix_set_identity(&A.matrix);
	gsl_matrix_set_identity(&B.matrix); // B = G_nk
	gsl_matrix_scale(&B.matrix,-1.0);

	// calculate Kalman gain
	C = gsl_matrix_submatrix(est->CC,0,0,3,3); // C = inv
	quadratic_mult_pd(1.0, &A.matrix, est->P_k, 0.0, &C.matrix);
	quadratic_mult_pd(1.0, &B.matrix, R_k, 1.0, &C.matrix);

	int status = gsl_linalg_cholesky_decomp(&C.matrix);
	if (status != GSL_SUCCESS) printf("poly_ekf: non-positive definite matrix\n");
	gsl_linalg_cholesky_invert(&C.matrix);
	B = gsl_matrix_submatrix(est->BB,0,0,n,3); // B = K_k
	D = gsl_matrix_submatrix(est->DD,0,0,n,3); // D = P_km1*G_xk'
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, est->P_k, &A.matrix, 0.0, &D.matrix);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &D.matrix, &C.matrix, 0.0, &B.matrix);

	// update the state and covariance based on Kalman gain
	double dphi_k_mag;

	w = gsl_vector_subvector(est->w,0,n); // w = dx_k
	gsl_blas_dgemv(CblasNoTrans, 1.0, &B.matrix, &v.vector, 0.0, &w.vector);
	gsl_vector_view dphi_k = gsl_vector_subvector(est->w,0,3);
	gsl_vector_view db_k = gsl_vector_subvector(est->w,3,3);

	dphi_k_mag = gsl_blas_dnrm2(&dphi_k.vector);
	if (dphi_k_mag == 0) gsl_matrix_set_identity(&C.matrix); // C = PHI_k
	else
	{
		gsl_vector_scale(&dphi_k.vector,1.0/dphi_k_mag);
		axis2rot(&C.matrix,&dphi_k.vector,dphi_k_mag);
	}

	D = gsl_matrix_submatrix(est->DD,0,0,3,3); // D = C_km
	gsl_matrix_memcpy(&D.matrix,est->C_k);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &C.matrix, &D.matrix, 0.0, est->C_k);
	parameterize_312_rotation(&phi_k.vector,est->C_k);

	C = gsl_matrix_submatrix(est->CC,0,0,n,n); // C = P_km
	D = gsl_matrix_submatrix(est->DD,0,0,n,n); // D = I-K_k*G_xk
	gsl_matrix_memcpy(&C.matrix,est->P_k);
	gsl_matrix_set_identity(&D.matrix);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, &B.matrix, &A.matrix, 1.0, &D.matrix);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &D.matrix, &C.matrix, 0.0, est->P_k);
	gsl_vector_add(&b_k.vector,&db_k.vector);

	/*
	// state prediction (RK4)
	A = gsl_matrix_submatrix(est->AA, 0, 0, n, n);

	k1 = gsl_matrix_column(&A.matrix,0);
	k2 = gsl_matrix_column(&A.matrix,1);
	k3 = gsl_matrix_column(&A.matrix,2);
	k4 = gsl_matrix_column(&A.matrix,3);

	gsl_matrix_memcpy(est->v, est->x_k); // v = x_km

	(*state_trans)(k1.vector,est->x_k,u_k,dt); // k1
	gsl_vector_scale(k1.vector,dt/6); // k1*h/6
	gsl_vector_add(est->v,k1.vector); // v += k1*h/6
	gsl_vector_scale(k1.vector,3); // k1*h/2
	gsl_vector_add(k1.vector,est->x_k); //x_km1+k1*h/2

	(*state_trans)(k2.vector,k1.vector,u_k,dt); // k2
	gsl_vector_scale(k2.vector,dt/3); // k2*h/3
	gsl_vector_add(est->v,k2.vector); // v += k2*h/3
	gsl_vector_scale(k2.vector,3/2); // k2*h/2
	gsl_vector_add(k2.vector,est->x_k); //x_km1+k2*h/2

	(*state_trans)(k3.vector,k2.vector,u_k,dt); // k3
	gsl_vector_scale(k3.vector,dt/3); // k1*h/3
	gsl_vector_add(est->v,k3.vector); // v += k3*h/3
	gsl_vector_scale(k3.vector,3); // k3*h
	gsl_vector_add(k3.vector,est->x_k); //x_km1+k3*h

	(*state_trans)(k4.vector,k3.vector,u_k,dt); // k4
	gsl_vector_scale(k4.vector,dt/6); // k1*h/6
	gsl_vector_add(est->v,k4.vector); // v += k4*h/6
	*/




}

void sliding_window_estimator(struct estimator *est, gsl_vector * U_K, gsl_vector * Y_K, gsl_matrix * Q_k, gsl_matrix * R_k)
{
	int i, status;
	gsl_matrix_view A, B, C, D;
	gsl_matrix_view C_k, C_km1;
	gsl_vector_view phi_k, phi_km1, b_k, b_km1;
	gsl_vector_view u_k, y_k;
	gsl_vector_view x_km1, x_k;
	gsl_vector_view v, w;

	double dt = est->dt;
	int n = est->n;

	if (est->sat < est->m) // buffer is not saturated => keep filling buffer
	{
		gsl_matrix_set_col(est->x_k_buffer,est->sat,est->x_k); // x_km1 state

		ekf_estimator(est,U_K,Y_K,Q_k,R_k); // propagate using EKF

		gsl_matrix_set_col(est->x_k_buffer,est->sat+1,est->x_k); // x_k state
		gsl_matrix_set_col(est->u_k_buffer,est->sat,U_K);
		gsl_matrix_set_col(est->y_k_buffer,est->sat,Y_K);
		est->sat++;
	}
	else // update the buffer
	{
		for (i=1;i<est->m;i++) // shift the buffer and throw away x_0 state
		{
			gsl_matrix_swap_columns(est->x_k_buffer,i-1,i);
			gsl_matrix_swap_columns(est->u_k_buffer,i-1,i);
			gsl_matrix_swap_columns(est->y_k_buffer,i-1,i);
		}

		// fill x_km1 state
		gsl_matrix_set_col(est->x_k_buffer,est->m-1,est->x_k); // x_km1 state

		ekf_estimator(est,U_K,Y_K,Q_k,R_k); // propagate using EKF

		gsl_matrix_set_col(est->x_k_buffer,est->m,est->x_k); // x_k state
		gsl_matrix_set_col(est->u_k_buffer,est->m-1,U_K);
		gsl_matrix_set_col(est->y_k_buffer,est->m-1,Y_K);
	}
	if (est->sat >= est->m) // buffer is saturated => Gauss-Newton algorithm
	{
		gsl_matrix_view H_xk, T_wk, G_xk, T_nk, I;
		gsl_vector_view e_k_int, e_k_int_phi, e_k_int_b, e_k_ext;

		gsl_matrix_view tl;
		gsl_matrix_view tr;
		gsl_matrix_view br;
		gsl_matrix_view bl;
		gsl_matrix_view t;
		gsl_matrix_view b;

		double psi_k_mag;
		double sse = 100;
		double limit = sqrt(pow(0.0001,2.0)*n*(est->m+1));
		while (sse > limit)
			{
			for (i=0;i<est->m;i++)
			{
				// get current and previous states
				x_km1 = gsl_matrix_column(est->x_k_buffer,i);
				phi_km1 = gsl_vector_subvector(&x_km1.vector,0,3);
				b_km1 = gsl_vector_subvector(&x_km1.vector,3,3);

				x_k = gsl_matrix_column(est->x_k_buffer,i+1);
				phi_k = gsl_vector_subvector(&x_k.vector,0,3);
				b_k = gsl_vector_subvector(&x_k.vector,3,3);

				// get current measurements
				u_k = gsl_matrix_column(est->u_k_buffer,i); // interoceptive
				y_k = gsl_matrix_column(est->y_k_buffer,i); // exteroceptive

				// get locations of Jacobians and error
				H_xk = gsl_matrix_submatrix(est->H,i*(n+3),i*n,n,n); // -H_xk
				G_xk = gsl_matrix_submatrix(est->H,i*(n+3)+n,(i+1)*n,3,n); // -G_xk
				I = gsl_matrix_submatrix(est->H,i*(n+3),(i+1)*n,n,n); // I
				T_wk = gsl_matrix_submatrix(est->T,i*(n+3),i*(n+3),n,n); // T_wk = H_wk*Q_k*H_wk'
				T_nk = gsl_matrix_submatrix(est->T,i*(n+3)+n,i*(n+3)+n,3,3); // T_nk = G_nk*R_k*G_nk'
				e_k_int = gsl_vector_subvector(est->e,i*(n+3),n); // interoceptive error
				e_k_int_phi = gsl_vector_subvector(&e_k_int.vector,0,3); // phi error
				e_k_int_b = gsl_vector_subvector(&e_k_int.vector,3,3); // bias error
				e_k_ext = gsl_vector_subvector(est->e,i*(n+3)+n,3); // exteroceptive error

				// compute the PSI matrix
				v = gsl_vector_subvector(est->v,0,3); // v = w_bit
				gsl_vector_memcpy(&v.vector,&u_k.vector);
				gsl_vector_add(&v.vector,&b_k.vector);
				gsl_vector_scale(&v.vector,dt);
				psi_k_mag = gsl_blas_dnrm2(&v.vector);
				A = gsl_matrix_submatrix(est->AA,0,0,3,3); // A = PSI_k
				if (psi_k_mag == 0) gsl_matrix_set_identity(&A.matrix);
				else
				{
					gsl_vector_scale(&v.vector,1.0/psi_k_mag);
					axis2rot(&A.matrix,&v.vector,psi_k_mag);
				}

				// compute state transition and noise Jacobian
				tl = gsl_matrix_submatrix(&H_xk.matrix, 0, 0, 3, 3);
				tr = gsl_matrix_submatrix(&H_xk.matrix, 0, 3, 3, 3);
				br = gsl_matrix_submatrix(&H_xk.matrix, 3, 3, 3, 3);
				bl = gsl_matrix_submatrix(&H_xk.matrix, 3, 0, 3, 3);

				gsl_matrix_memcpy(&tl.matrix, &A.matrix);
				gsl_matrix_set_identity(&tr.matrix);
				gsl_matrix_set_identity(&br.matrix);
				gsl_matrix_set_zero(&bl.matrix);
				gsl_matrix_scale(&H_xk.matrix,-1.0);

				B = gsl_matrix_submatrix(est->BB,0,0,3,3); // B = I
				gsl_matrix_set_identity(&B.matrix);
				t = gsl_matrix_submatrix(&T_wk.matrix, 0, 0, 3, 3);
				b = gsl_matrix_submatrix(&T_wk.matrix, 3, 3, 3, 3);
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, Q_k, 0.0, &b.matrix);
				gsl_matrix_scale(&B.matrix, pow(dt,2));
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, Q_k, 0.0, &t.matrix);

				status = gsl_linalg_cholesky_decomp(&T_wk.matrix);
				if (status != GSL_SUCCESS) printf("non-positive definite matrix\n");
				gsl_linalg_cholesky_invert(&T_wk.matrix); // invert T_wk

				gsl_matrix_set_identity(&I.matrix);
				gsl_matrix_scale(&I.matrix,-1.0);

				// compute interoceptive error
				C = gsl_matrix_submatrix(est->CC,0,0,3,3); // C = C_km1
				B = gsl_matrix_submatrix(est->BB,0,0,3,3); // B = E_k
				D = gsl_matrix_submatrix(est->AA,0,0,3,3); // D = C_km
				compute_312_rotation_matrix(&C.matrix,&phi_km1.vector);
				compute_312_rotation_matrix(est->C_k,&phi_k.vector); // C_k

				gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &A.matrix, &C.matrix, 0.0, &D.matrix);
				gsl_matrix_set_identity(&B.matrix);
				gsl_blas_dgemm(CblasNoTrans, CblasTrans, -1.0, est->C_k, &D.matrix, 1.0, &B.matrix);
				unxmat(&e_k_int_phi.vector,&B.matrix); // e_k_ext

				gsl_vector_memcpy(&e_k_int_b.vector,&b_k.vector);
				gsl_vector_sub(&e_k_int_b.vector,&b_km1.vector);

				// compute measurement Jacobians
				gsl_matrix_set_identity(&G_xk.matrix);
				gsl_matrix_scale(&G_xk.matrix,-1.0);

				D = gsl_matrix_submatrix(est->DD,0,0,3,3);
				gsl_matrix_set_identity(&D.matrix); // D = G_nk
				quadratic_mult_pd(-1.0,&D.matrix,R_k,0.0,&T_nk.matrix);
				status = gsl_linalg_cholesky_decomp(&T_nk.matrix);
				if (status != GSL_SUCCESS) printf("non-positive definite matrix\n");
				gsl_linalg_cholesky_invert(&T_nk.matrix); // invert T_nk

				// compute exteroceptive error
				B = gsl_matrix_submatrix(est->BB,0,0,3,3); // B = E_k
				A = gsl_matrix_submatrix(est->AA,0,0,3,3); // A = C_meas
				compute_312_rotation_matrix(&A.matrix,&y_k.vector);
				gsl_matrix_set_identity(&B.matrix);
				gsl_blas_dgemm(CblasNoTrans, CblasTrans, -1.0, &A.matrix, est->C_k, 1.0, &B.matrix);
				unxmat(&e_k_ext.vector,&B.matrix); // e_k_ext

			}
			// compute the correction
			gsl_matrix_transpose_memcpy(est->H_t,est->H); // H'
			quadratic_mult_pd(1.0,est->H_t,est->T,0.0,est->inv); // inv = H'/T*H

			status = gsl_linalg_cholesky_decomp(est->inv);
			if (status != GSL_SUCCESS) printf("non-positive definite matrix\n");
			gsl_linalg_cholesky_invert(est->inv); // invert inv

			v = gsl_vector_subvector(est->w,0,(n+est->n_ext)*(est->m));
			w = gsl_vector_subvector(est->w,0,(est->m+1)*n);

			gsl_blas_dgemv(CblasNoTrans, -1.0, est->T, est->e, 0.0, &v.vector); // -Tinv*e
			gsl_blas_dgemv(CblasNoTrans, 1.0, est->H_t, &v.vector, 0.0, &w.vector); // -H'*Tinv*e
			gsl_blas_dgemv(CblasNoTrans, 1.0, est->inv, &w.vector, 0.0, est->dx); // dx = -inv*H'*Tinv*e

			// correct the state
			double dphi_k_mag;
			C = gsl_matrix_submatrix(est->CC,0,0,3,3);
			C_km1 = gsl_matrix_submatrix(est->AA,0,0,3,3);
			C_k = gsl_matrix_submatrix(est->BB,0,0,3,3);


			for (i=0;i<=est->m;i++)
			{
				// get state
				x_km1 = gsl_matrix_column(est->x_k_buffer,i);
				phi_km1 = gsl_vector_subvector(&x_km1.vector,0,3);
				b_km1 = gsl_vector_subvector(&x_km1.vector,3,3);

				// get corrections
				v = gsl_vector_subvector(est->dx,i*n,3); // v = dphi_k
				w = gsl_vector_subvector(est->dx,i*n+3,3); // w = db_k

				compute_312_rotation_matrix(&C_km1.matrix,&phi_km1.vector);

				dphi_k_mag = gsl_blas_dnrm2(&v.vector);

				if (dphi_k_mag == 0) gsl_matrix_set_identity(&C.matrix); // C = PHI_k
				else
				{
					gsl_vector_scale(&v.vector,1.0/dphi_k_mag);
					axis2rot(&C.matrix,&v.vector,dphi_k_mag);
				}
				gsl_vector_scale(&v.vector,dphi_k_mag);

				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &C.matrix, &C_km1.matrix, 0.0, &C_k.matrix);
				parameterize_312_rotation(&phi_km1.vector,&C_k.matrix);

				gsl_vector_add(&b_km1.vector,&w.vector);

			}
			sse = gsl_blas_dnrm2(est->dx);
			printf("Residual: %g\n",sse);
			//print_vector(est->dx);
			//sleep(1);
		}


	}

}


