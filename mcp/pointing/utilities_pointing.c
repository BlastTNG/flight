#include "utilities_pointing.h"

#include <math.h>
#include <time.h>

/* General astronomy related routines */

/* Check out Xephem for some inspiration: */
/* http://www.clearskyinstitute.com/xephem/xephem-3.7.1.tar.gz */
/* In particular the codes mjd.c, misc.c, aa_hadec.c, plmoon.c */


/* Public function definitions */

//TODO:Unify multiple AZ/EL Conversion Functions
void azel_2_hadec(double az[], double el[], int n, double cosL, double sinL,
		  double *ha, double *dec, double *cosdec) 
{
  /*Purpose: convert Az, El in radians to HA, Dec in radians and cos(Dec)*/

  double sindec, cosha, sinel,coselcosL;
  int ii;

  for (ii=0; ii<=n-1; ii++) range_fast(&az[ii],TWOPI);

  for (ii=0;ii<=n-1;ii++){ 
    sinel = sin(el[ii]);
    coselcosL= cos(el[ii])*cosL;

    sindec = sinel*sinL + coselcosL*cos(az[ii]);
    dec[ii] = asin(sindec);
    cosdec[ii] = cos(dec[ii]);
    cosha = (sinel - sinL*sindec) / (cosL*cosdec[ii]);

    if(cosha > 1.0) cosha=1.0;
    if(cosha < -1.0) cosha= -1.0;

    ha[ii] = acos(cosha);
    if (az[ii] < PI) ha[ii] = TWOPI - ha[ii];    
  }
}


void rael_2_az(double RA, double el, double lst, double cosL, double sinL,
	       double *az) 
     /*Purpose: Given a target RA and current elevation in radians, LST in hours,
       work out the two possible azimuth solutions in radians.

       LST = GST + long/15. */
{     
     int ii;
     double ha,cc,bb,aa,sinel,cosha,delta;
     double *sin_dec,*cos_dec,*azimuth;
     sin_dec=(double *)(calloc(2,sizeof(double)));   
     cos_dec=(double *)(calloc(2,sizeof(double)));   
     azimuth=(double *)(calloc(2,sizeof(double)));   

     ha = lst*HR2RAD - RA;     
     sinel=sin(el);
     cosha=cos(ha);

     cc = sinel*sinel-cosha*cosha*cosL*cosL;
     bb = -2.*sinel*sinL;
     aa = sinL*sinL + cosha*cosha*cosL*cosL;
     delta = bb*bb - 4.*aa*cc;

     if (delta < 0.0) delta = 0.;
     
     sin_dec[0] = (-bb - sqrt(delta))/(2.*aa) ;
     sin_dec[1] = (-bb + sqrt(delta))/(2.*aa) ; 

     for (ii=0;ii<2;ii++)
       {
	 if (sin_dec[ii] <= 1.) cos_dec[ii] = sqrt(1.-sin_dec[ii]*sin_dec[ii]);
       }

     for (ii=0;ii<2;ii++)
       {
	 azimuth[ii] = PI + atan2(cos_dec[ii]*sin(ha),(-sin_dec[ii]*cosL+cos_dec[ii]*cosha*sinL));
	 range_fast(&azimuth[ii],TWOPI);
       }
     az[0]=azimuth[0];
     az[1]=azimuth[1];

     free(sin_dec); free(cos_dec);free(azimuth);
}



void ha2ra(double ha[], double lst[], long int n, double *ra)
{
  /*Purpose: given HA and lst, both in radians, converts to RA, in radians. */
  long int sample;

  for(sample = 0 ; sample < n ; sample++)
    {
      ra[sample] = lst[sample] - ha[sample];
      range_fast(&ra[sample],TWOPI);
    }
}




