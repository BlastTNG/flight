

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

#include "PWM_board.h"
#include "miscellaneous.h" // linear algebra and rotation stuff
#include "state_estimator.h"

struct PWM_board pwm;
double CLK_FREQ = 10000.0;
int test = 0;

// interrupt handler for PWM clock
const struct sigevent * intHandlerPWM (void *arg, int id)
{
	test = test+1;
	if (check_IRQ(&pwm)) clear_IRQ(&pwm);
	return NULL;
}

int main()
{
	struct sigevent event;
	int interruptID;
	float time;

	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread
	InterruptEnable();

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	init_PWM(&pwm,PWM_ADDR_DEFAULT);
	enable_IRQ(&pwm);
	set_IRQ_freq(&pwm,CLK_FREQ);

	interruptID = InterruptAttach (PWM_IRQ_DEFAULT, intHandlerPWM, &event, sizeof (event), 0);

	if (interruptID == -1)
	{
		printf ("Can't attach to IRQ.");
		perror (NULL);
		exit (1);
	}

	struct predictor pred;
	struct estimator est;
	int i;
	double point;
	gsl_vector * meas = gsl_vector_alloc(6);

	printf("Start predictor test:\n");

	init_predictor(&pred,6,5+1,2,0.001);

	time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
	printf("Start time: %f\n",time);

	for (i=0;i<15;i++)
	{
		point =(i-2)*(i+5);
		gsl_vector_set_all(meas,point);
		update_predictor(&pred,meas);

		if (evaluate_predictor(&pred,0.007))
		{
			printf("%f\n----------\n",point);
			print_vector(pred.value);
		}
		time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
		printf("%f\n",time);
	}
	time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
	printf("End time: %f\n",time);

	printf("Start estimator test:\n");

	double dt = 0.001;
	init_estimator(&est,6,3,3,5,dt);

	// set test values for EKF estimator
	gsl_vector_set_zero(est.x_k); // initial orientation estimate
	gsl_matrix_set_identity(est.C_k); //initial orientation estimate
	gsl_matrix_set_identity(est.P_k);
	gsl_matrix_scale(est.P_k,10.0); // initial covariance estimate

	gsl_vector * phi_meas = gsl_vector_alloc(3);
	gsl_vector * omega_meas = gsl_vector_alloc(3);
	gsl_matrix * Q_k = gsl_matrix_alloc(3,3);
	gsl_matrix * R_k = gsl_matrix_alloc(3,3);

	gsl_vector_set(phi_meas,0,0.0033);
	gsl_vector_set(phi_meas,1,0.0033);
	gsl_vector_set(phi_meas,2,0.0017);
	gsl_vector_set(omega_meas,0,-0.0024);
	gsl_vector_set(omega_meas,1,-0.0029);
	gsl_vector_set(omega_meas,2,-3.48e-7);

	gsl_matrix_set_identity(Q_k);
	gsl_matrix_scale(Q_k,pow(arcsec/3.0/sqrt(50.0)*4.0,2.0));
	gsl_vector * temp = gsl_vector_alloc(3);

	R_k = gsl_matrix_alloc(3,3);
	gsl_matrix_set_identity(R_k);
	gsl_matrix_scale(R_k,pow(arcsec/3.0/5.0,2.0));

	time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
	printf("Start time: %f\n",time);

	pred.sat = pred.m;

	for (i=0;i<20;i++)
	{
		gsl_vector_memcpy(temp,omega_meas);
		gsl_vector_scale(temp,dt);
		gsl_vector_add(phi_meas,temp);
		sliding_window_estimator(&est,omega_meas,phi_meas,Q_k,R_k);
		if (est.sat >= est.m)
		{
			pred.buffer = est.x_k_buffer;
			update_predictor_coeff(&pred);

			time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
			printf("Time: %f\n",time);
			if (evaluate_predictor(&pred,0.007)) print_vector(pred.value);
		}

	}
	time = ((float) test)/CLK_FREQ; // get the elapsed time [s]
	printf("End time: %f\n",time);

	disable_IRQ(&pwm);
	close_PWM(&pwm);

	return 0;
}
