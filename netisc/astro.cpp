#include "astro.h"

#define __astro_NEWDIST (0.5*PI/180.) // distance from cen. before reloading
#define __astro_EPSILON 0.000001      // tolerance for special case in rad

#define __astro_MAX_ITER 10           // max iter. in map centre solver

int *__astro_sortindex=NULL;
double *__astro_cat_ra=NULL;          // The current sub-catalogue
double *__astro_cat_dec=NULL;
double *__astro_cat_mag=NULL;        
int __astro_ncat=0;
double __astro_cat_ra_0=0;
double __astro_cat_dec_0=0;
double __astro_cat_radius=0;
double __astro_cat_maglimit=0;

HTMCatalog *__astro_catalog=NULL;

Pyramid *__astro_pyr;                 // Make global to avoid re-loading    


// Sort array indices by magnitudes in the reduced star catalogie
int compare( const void *arg1, const void *arg2 ) {
  double diff = __astro_cat_mag[*(int *)arg1] - 
    __astro_cat_mag[*(int *)arg2];
  return (diff < 0) ? -1 : (diff == 0) ? 0 : 1;
}

// Convert tanplane offsets to RA and DEC given coordinates of tan. point
// and the tangent plane offsets of the blobs from the centre. 
//
// ra_0    = ra of map centre in radians
// dec_0   = dec "  "    "    "  "
// ra_off  = array of tan. plane offsets parallel to the +ra axis
// dec_off =  "    "   "       "     "       "       "  " +dec "   
// ra      = OUTPUT: pointer to array of ra in rad
// dec     = OUTPUT: pointer to array of dec in rad
// noffsets= # of tangent plane offsets in the above arrays

void tan2radec( double ra_0, double dec_0, double *ra_off, 
                double *dec_off, double *ra, double *dec, int nblobs ) {
  int i;
  
  for( i=0; i<nblobs; i++ ) {
    slaDtp2s ( ra_off[i], dec_off[i], ra_0, dec_0,
               &ra[i], &dec[i] );
  }
}

// calculate the tan plane offsets given the RA and DEC of the map cen, 
// and the RA and DEC for each blob
//
// ra_0    = ra of map centre in radians
// dec_0   = dec "  "    "    "  "
// ra      = array of ra in rad
// dec     = array of dec in rad
// ra_off  = OUTPUT: array tanplane off. parallel to the +ra axis(deg) 
// dec_off = OUTPUT:  "    "   "       "     "       "   +dec 
// nblobs  = # of blobs
void radec2tan( double ra_0, double dec_0, double *ra, double *dec, 
                double *ra_off, double *dec_off, int nblobs) {
  int i, status;
  
  for( i=0; i<nblobs; i++ ) {
    slaDs2tp ( ra[i], dec[i], ra_0, dec_0,
               &ra_off[i], &dec_off[i], &status );
  }
}

// calculate the greenwich sidereal time given the east longitude and
// current UT
//
// year    = UT year on gregorian calendar
// month   = "  month
// day     = "  day
// dayfrac = "  day fraction
// lon     = east longitude (radians) 
// Return: -999 for error, else 

double get_gst( int year, int month, int day, double dayfrac ) {
  double djm;
  int status;

  slaCaldj( year, month, day, &djm, &status );

  if( status != 0 ) return -999;

  return slaGmsta( djm, dayfrac );
}

// Calculate the pixel coordinates of stars in the CCD tangent plane 
// (offsets from centre pixel) using a pointing model. 
// All angular quantities in radians.
//
// ra,dec           = coordinates of the stars matched to the blobs
// az_0, el_0, rot  = pointing solution
// platescale       = CCD platescale in arcsec/pixel
// x_star           = pointer to array that will hold the star az offsets
// y_star           = "

void model_tanplane( double *ra, double *dec, int nblobs, 
                     double lat, double lst, 
                     double az_0, double el_0, 
                     double rot, double platescale,
                     double *x_star, double *y_star ) {
  int i;
  double cos_theta = cos(rot); // in the radec -> azel direction
  double sin_theta = sin(rot);  
  
  double *az_star = new double[nblobs];         // az/el of stars
  double *el_star = new double[nblobs];
  double *az_star_off = new double[nblobs];     // az/el of stars
  double *el_star_off = new double[nblobs];
  
  // calculate the az and el of each star
  for( i=0; i<nblobs; i++ )
    calc_alt_az( ra[i], dec[i], lat, lst, &el_star[i], &az_star[i] );
  
  // az/el tanplane offsets for each star
  radec2tan( az_0, el_0, az_star, el_star, az_star_off, el_star_off, 
             nblobs);
  
  //printf("model: %lf %lf %lf\n",az_0*180./PI, el_0*180./PI, 
  //rot*180./PI);
  
  for( i=0; i<nblobs; i++ ) {
    // CCD coordinates of the star
    x_star[i] = (az_star_off[i]*cos_theta-el_star_off[i]*sin_theta) /
      (platescale/3600.*PI/180.);
    y_star[i] = (az_star_off[i]*sin_theta+el_star_off[i]*cos_theta) / 
      (platescale/3600.*PI/180.);
    
    //d_star[i] = sqrt( x_star[i]*x_star[i] + y_star[i]*y_star[i] );
    //printf("  %i: (%lf %lf) %lf %lf   -->   %lf %lf %lf\n", i,
    //           ra[i]*180./PI/15., dec[i]*180./PI, 
    //           az_star[i]*180./PI, el_star[i]*180./PI, 
    //           x_star[i]*180./PI, y_star[i]*180./PI, d_star[i]*180./PI );
  }
  
  // clean up
  delete[] az_star;
  delete[] el_star;
  delete[] az_star_off;
  delete[] el_star_off;
  
}


// Map centre: remember, all quantities in radians!
// Calculations done in az/el tangent plane coordinates
// Using a different least squares method
// Model paramaters: coordinates of centre + rotation
//                   platescale is FIXED
//
// x       = ccd x pixel offsets
// y       = ccd y pixel offsets
// flux    = fluxes
// nblobs  = number of blobs in the array
// ra      = true right ascensions of each blob
// dec     = "    declinations          "
// lst     = local sidereal time (radians) USED FOR RADEC->AZEL transform
// lat     = latitude in radians             "    "    "     "
// ra_0    = OUTPUT: ra of map centre
// dec_0   = OUTPUT: dec of map centre
// rot     = INPUT/OUTPUT: rotation from azel -> CCD (input is a guess)
// platescale = INPUT: CCD platescale arcsec/pixel
// var     = estimate of the variance in the solution (in rad^2)
// return: 1 for success, 0 for FAILURE, -1 for max iterations

int map_centre( double *x, double *y, double *flux, int nblobs, 
                double *ra, double *dec, 
                double lst, double lat, double *ra_0, 
                double *dec_0, double *rot, double platescale, 
                double *var ) {
  int i,j;
  double az_0, el_0;             // the map centre in az.el
  double az_1, el_1;             // extra solutions from map centre
  int nsol;                      // # solutions for map centre
  
  double az, el, az_off, el_off; // temporary star/blob coordinates
  
  double cos_theta = cos(*rot);  // in the radec -> azel direction
  double sin_theta = sin(*rot);  
  
  double *x_star = new double[nblobs];     // model star CCD coordinates
  double *y_star = new double[nblobs];     //  "
  double *x_star2 = new double[nblobs];    // model star CCD coordinates
  double *y_star2 = new double[nblobs];    //  "

  double *res = new double[nblobs*2];      // model residuals
  double *G_0 = new double[nblobs*2];      // grad WRT model par. 0
  double *G_1 = new double[nblobs*2];      // grad WRT model par. 1
  double *G_2 = new double[nblobs*2];      // grad WRT model par. 2

  double step_az, step_el, step_rot;       // step in model parameters
  double v_a, v_b, v_c;                    // vector components
  double m_a,m_b,m_c,m_d,m_e,m_f,m_g,m_h,m_i; // matrix components
  double det_r;                            // reciprocal determinant of m
  double mi_a,mi_b,mi_c,mi_d,mi_e,mi_f,mi_g,mi_h,mi_i;// inverse of m

  // the amount to step in az/el/rot to determine the gradient
  double delt_az; 
  double delt_el; 
  double delt_rot;

  // calculate the starting model using the first blob.
  calc_alt_az( ra[0], dec[0], lat, lst, &el, &az );  // theory az & el

  az_off = (x[0]*cos_theta + y[0]*sin_theta)* // measured az/el offsets
    (platescale/3600.*PI/180.);
  el_off = (-x[0]*sin_theta + y[0]*cos_theta)*
    (platescale/3600.*PI/180.);

  slaTps2c( az_off, el_off, az, el, &az_0, &el_0, &az_1, &el_1, &nsol );
  
  //printf("off: %lf %lf  coord: %lf %lf  cen: %lf %lf\n",
  //         az_off*180./PI, el_off*180./PI, az*180./PI, el*180./PI, 
  //         az_0*180./PI, el_0*180./PI);
  
  // Main iteration loop for the map centre -----------------------------
  for( i=0; i<__astro_MAX_ITER; i++ ) {
    // positions of stars in the model tangent plane
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0, *rot, platescale, 
                    x_star, y_star );
    
    //printf("model: %lf %lf %lf\n",az_0*180./PI,el_0*180./PI,
    //*rot*180./PI);
  
    // Calculate the residuals (in CCD pixel units)
    *var = 0;
    for( j=0; j<nblobs; j++ ) {
      res[j*2]   = x[j] - x_star[j];
      res[j*2+1] = y[j] - y_star[j];
      
      // calculate variance in pixels^2
      *var += res[j*2]*res[j*2] + res[j*2+1]*res[j*2+1];
    }
    //printf("\n");
    *var /= nblobs;

    // convert variance to rad^2
    *var *= (platescale/3600.*PI/180.)*(platescale/3600.*PI/180.);
    
    // Calculate the gradient matrix:
    
    delt_az=0.02/3600.*PI/180.*cos(el_0); 
    delt_el=0.02/3600.*PI/180.; 
    delt_rot=0.0002*PI/180.;
    
    // az_0
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0-delt_az, el_0, *rot, platescale, 
                    x_star, y_star );
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0+delt_az, el_0, *rot, platescale, 
                    x_star2, y_star2 );
    
    for( j=0; j<nblobs; j++ ) {
      G_0[j*2]   = (x_star2[j] - x_star[j])/(2.*delt_az);
      G_0[j*2+1] = (y_star2[j] - y_star[j])/(2.*delt_az);
    }
    
    // el_0
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0-delt_el, *rot, platescale, 
                    x_star, y_star );
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0+delt_el, *rot, platescale, 
                    x_star2, y_star2 );

    for( j=0; j<nblobs; j++ ) {
      G_1[j*2]   = (x_star2[j] - x_star[j])/(2.*delt_el);
      G_1[j*2+1] = (y_star2[j] - y_star[j])/(2.*delt_el);
    }

    // rot
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0, *rot-delt_rot, platescale, 
                    x_star, y_star );
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0, *rot+delt_rot, platescale, 
                    x_star2, y_star2 );
    
    for( j=0; j<nblobs; j++ ) {  
      G_2[j*2]   = (x_star2[j] - x_star[j])/(2.*delt_rot);
      G_2[j*2+1] = (y_star2[j] - y_star[j])/(2.*delt_rot);
    }
    
    // Calculate the next step
    v_a=0; v_b=0; v_c=0;
    m_a=0; m_b=0; m_c=0; 
    m_d=0; m_e=0; m_f=0;
    m_g=0; m_h=0; m_i=0;
    
    for( j=0; j<nblobs*2; j++ ) {
      v_a += G_0[j]*res[j];
      v_b += G_1[j]*res[j];
      v_c += G_2[j]*res[j];
      
      m_a += G_0[j]*G_0[j];
      m_b += G_0[j]*G_1[j];
      m_c += G_0[j]*G_2[j];
      
      m_e += G_1[j]*G_1[j];
      m_f += G_1[j]*G_2[j];
      
      m_i += G_2[j]*G_2[j];
    }
    
    m_d = m_b;
    m_g = m_c;
    m_h = m_f;
    
    det_r = 1/(m_a*m_e*m_i + m_b*m_f*m_g + m_c*m_d*m_h - 
               m_c*m_e*m_g - m_h*m_f*m_a - m_i*m_d*m_b);
    
    mi_a = det_r*(m_e*m_i-m_f*m_h);
    mi_b = det_r*(m_c*m_h-m_b*m_i);
    mi_c = det_r*(m_b*m_f-m_c*m_e);
    mi_d = det_r*(m_f*m_g-m_d*m_i);
    mi_e = det_r*(m_a*m_i-m_c*m_g);
    mi_f = det_r*(m_c*m_d-m_a*m_f);
    mi_g = det_r*(m_d*m_h-m_e*m_g);
    mi_h = det_r*(m_b*m_g-m_a*m_h);
    mi_i = det_r*(m_a*m_e-m_b*m_d);
    
    step_az  = mi_a*v_a + mi_b*v_b + mi_c*v_c;
    step_el  = mi_d*v_a + mi_e*v_b + mi_f*v_c;
    step_rot = mi_g*v_a + mi_h*v_b + mi_i*v_c;
    
    // Update the model
    az_0 += step_az;
    el_0 += step_el;
    *rot += step_rot;

    // Decide if we can quit
    if( (fabs(step_az) <= 5*delt_az) && 
        (fabs(step_el) <= 5*delt_el) &&
        (fabs(step_rot) <= 5*delt_rot) )
      i = __astro_MAX_ITER; 
  }

  // transform the map centre back into RA and Dec
  calc_ra_dec( az_0, el_0, lat, lst, ra_0, dec_0 );

  // clean up
  delete[] x_star;
  delete[] y_star;
  delete[] x_star2;
  delete[] y_star2;
  delete[] res;
  delete[] G_0;
  delete[] G_1;
  delete[] G_2;

  return 1;
}

// Map centre3: remember, all quantities in radians!
// Calculations done in az/el tangent plane coordinates
// Using a different least squares method
// Model paramaters: coordinates of centre
//                   rotation and platescale are FIXED
//
// x       = ccd x pixel offsets
// y       = ccd y pixel offsets
// flux    = fluxes
// nblobs  = number of blobs in the array
// ra      = true right ascensions of each blob
// dec     = "    declinations          "
// lst     = local sidereal time in radians USED FOR RADEC->AZEL xform
// lat     = latitude in radians             "    "    "     "
// ra_0    = OUTPUT: ra of map centre
// dec_0   = OUTPUT: dec of map centre
// rot     = INPUT: rotation from azel -> CCD
// platescale = INPUT: CCD platescale arcsec/pixel
// var     = estimate of the variance in the solution (in rad^2)
// return: 1 for success, 0 for FAILURE, -1 for max iterations

int map_centre3( double *x, double *y, double *flux, int nblobs, 
                 double *ra, double *dec, double lst, double lat, 
                 double *ra_0, double *dec_0, 
                 double rot, double platescale, 
                 double *var ) {
  int i,j;
  double az_0, el_0;             // the map centre in az.el
  double az_1, el_1;             // extra solutions from map centre
  int nsol;                      // # solutions for map centre

  double az, el, az_off, el_off; // temporary star/blob coordinates

  double cos_theta = cos(rot);   // in the radec -> azel direction
  double sin_theta = sin(rot);  

  double *x_star = new double[nblobs];     // model star CCD coordinates
  double *y_star = new double[nblobs];     //  "
  double *x_star2 = new double[nblobs];    // model star CCD coordinates
  double *y_star2 = new double[nblobs];    //  "

  double *res = new double[nblobs*2];      // model residuals
  double *G_0 = new double[nblobs*2];      // grad WRT model par. 0
  double *G_1 = new double[nblobs*2];      // grad WRT model par. 1

  double step_az, step_el;                 // step in model parameters
  double v_a, v_b;                         // vector components
  double m_a,m_b,m_c,m_d;                  // matrix components
  double det_r;                            // reciprocal determinant of m
  double mi_a,mi_b,mi_c,mi_d;              // inverse of m

  // the amount to step in az/el/rot to determine the gradient
  double delt_az; 
  double delt_el; 

  // calculate the starting model using the first blob.
  calc_alt_az( ra[0], dec[0], lat, lst, &el, &az );  // theory az & el

  az_off = (x[0]*cos_theta + y[0]*sin_theta)* // measured az/el offsets
    (platescale/3600.*PI/180.);
  el_off = (-x[0]*sin_theta + y[0]*cos_theta)*
    (platescale/3600.*PI/180.);
  
  slaTps2c( az_off, el_off, az, el, &az_0, &el_0, &az_1, &el_1, &nsol );
  
  //printf("off: %lf %lf  coord: %lf %lf  cen: %lf %lf\n",
  //         az_off*180./PI, el_off*180./PI, az*180./PI, el*180./PI, 
  //         az_0*180./PI, el_0*180./PI);

  // Main iteration loop for the map centre -----------------------------
  for( i=0; i<__astro_MAX_ITER; i++ ) {
    // positions of stars in the model tangent plane
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0, rot, platescale, 
                    x_star, y_star );

    //printf("model: %lf %lf %lf\n",az_0*180./PI,el_0*180./PI,
    //*rot*180./PI);

    // Calculate the residuals (in CCD pixel units)
    *var = 0;
    for( j=0; j<nblobs; j++ ) {
      res[j*2]   = x[j] - x_star[j];
      res[j*2+1] = y[j] - y_star[j];
      
      // calculate variance in pixels^2
      *var += res[j*2]*res[j*2] + res[j*2+1]*res[j*2+1];
    }
    //printf("\n");
    *var /= nblobs;

    // convert variance to rad^2
    *var *= (platescale/3600.*PI/180.)*(platescale/3600.*PI/180.);

    // Calculate the gradient matrix:

    delt_az=0.02/3600.*PI/180.*cos(el_0); 
    delt_el=0.02/3600.*PI/180.; 

    // az_0
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0-delt_az, el_0, rot, platescale, 
                    x_star, y_star );
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0+delt_az, el_0, rot, platescale, 
                    x_star2, y_star2 );

    for( j=0; j<nblobs; j++ ) {
      G_0[j*2]   = (x_star2[j] - x_star[j])/(2.*delt_az);
      G_0[j*2+1] = (y_star2[j] - y_star[j])/(2.*delt_az);
    }

    // el_0
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0-delt_el, rot, platescale, 
                    x_star, y_star );
    model_tanplane( ra, dec, nblobs, lat, lst, 
                    az_0, el_0+delt_el, rot, platescale, 
                    x_star2, y_star2 );

    for( j=0; j<nblobs; j++ ) {
      G_1[j*2]   = (x_star2[j] - x_star[j])/(2.*delt_el);
      G_1[j*2+1] = (y_star2[j] - y_star[j])/(2.*delt_el);
    }

    // Calculate the next step
    v_a=0; v_b=0;
    m_a=0; m_b=0;
    m_c=0; m_d=0;

    for( j=0; j<nblobs*2; j++ ) {
      v_a += G_0[j]*res[j];
      v_b += G_1[j]*res[j];
      
      m_a += G_0[j]*G_0[j];
      m_b += G_0[j]*G_1[j];
      
      m_d += G_1[j]*G_1[j];      
    }
    m_c = m_b;

    det_r = 1/(m_a*m_d - m_b*m_c);
    
    mi_a = det_r*m_d;
    mi_b = -det_r*m_b;
    mi_c = -det_r*m_c;
    mi_d = det_r*m_a;
    
    step_az  = mi_a*v_a + mi_b*v_b;
    step_el  = mi_c*v_a + mi_d*v_b;

    // Update the model
    az_0 += step_az;
    el_0 += step_el;

    // Determine if we can quit
    if( (fabs(step_az) <= 5*delt_az) && 
        (fabs(step_el) <= 5*delt_el) )
      i = __astro_MAX_ITER; 
  }

  // transform the map centre back into RA and Dec
  calc_ra_dec( az_0, el_0, lat, lst, ra_0, dec_0 );
  
  // clean up
  delete[] x_star;
  delete[] y_star;
  delete[] x_star2;
  delete[] y_star2;
  delete[] res;
  delete[] G_0;
  delete[] G_1;

  return 1;
}

// Calculate the RA and DEC given azimuth, elevation, latitude and LST
void calc_ra_dec( double az, double el, double lat, double lst, 
                  double *ra, double *dec ) {
  double ha;
  
  // Get the Dec and hour angle
  slaDh2e ( az, el, lat, &ha, dec );

  // get the ra from the ha and normalize 0-2PI
  *ra = lst - ha;
  *ra = slaDranrm ( *ra );

}

// Calculate the Az and El given RA, Dec, Lat and LST
void calc_alt_az( double ra, double dec, double lat, double lst, 
                  double *el, double *az ) {
  double ha;

  ha = lst - ra;

  slaDe2h ( ha, dec, lat, az, el );
}

// Calculate the parallactic angle given LST, RA, DEC, LAT, EL 
// (rotation of az/el wrt ra/dec)
double calc_parallactic( double lst, double ra, double dec, double lat ) {
  double ha;

  ha = lst - ra;

  return slaPa( ha, dec, lat );
}

// Convert between celestial and cartesian coordinates on the unit sphere
void cel2vec( double ra, double dec, double *x, double *y, double *z ) {
  double theta = ra;
  double phi = PI/2 - dec;
  
  *x = cos(theta)*sin(phi);
  *y = sin(theta)*sin(phi);
  *z = cos(phi);
}

void vec2cel( double x, double y, double z, double *ra, double *dec ) {
  double theta = atan2(y,x);
  double phi = acos(z);
  
  *ra = theta;
  *dec = PI/2 - phi;
}

// Extract a sub-catalogue from the all-sky star catalogue in some search
// radius down to a limiting magnitude. Catalogue is sorted by 
// increasing magnitude. Output buffers are delete[]ed first 
// (so if empty, set to NULL!)
//
// ra_0   = right ascension of search region  (J2000)
// dec_0  = declination of search region         "
// radius = angular radius of the search region
// mag_0  = magnitude limit of the catalogue
// ra     = right ascensions of the catalogue stars    
// dec    = declination      "  "    "        "
// mag    = magnitudes       "  "    "        "
// ncat   = # stars in the catalogue

void catalogue( double ra_0, double dec_0, double radius, double mag_0, 
                double **ra, double **dec, double **mag, int *ncat ) {
  double tan_x, tan_y, tan_z;
  double tan_xold, tan_yold, tan_zold;

  // Decide if a new catalogue needs to be extracted
  cel2vec( ra_0, dec_0, &tan_x, &tan_y, &tan_z );
  cel2vec( __astro_cat_ra_0, __astro_cat_dec_0, &tan_xold, &tan_yold, 
           &tan_zold );
  double sep = acos( tan_x*tan_xold + tan_y*tan_yold + tan_z*tan_zold );

  if( (sep < radius*__astro_NEWDIST) && (mag_0 == __astro_cat_maglimit) 
      && (__astro_ncat != 0) && (radius == __astro_cat_radius) ) {
    // Return the old catalogue because it is sufficiently similar
    *ra = __astro_cat_ra;          // sorted catalogue
    *dec = __astro_cat_dec;
    *mag = __astro_cat_mag;
    *ncat = __astro_ncat;

    return;
  }

  // Get the new catalogue

  __astro_cat_ra_0 = ra_0;
  __astro_cat_dec_0 = dec_0;
  __astro_cat_radius = radius;
  __astro_cat_maglimit = mag_0;

  // Free-up the previous catalogue
  delete[] __astro_cat_ra;
  delete[] __astro_cat_dec;
  delete[] __astro_cat_mag;

  // Initialize star catalog
  //HTMCatalog catalog("/mnt/win_d/catalog/gsc1");
  //catalog.init();
  //qDebug("Initialized catalog");

  // Define the search region 
  Point fieldCenter(tan_x, tan_y, tan_z);
  MinorCircleRegion bigSearch( fieldCenter, Radian(radius) ); 
                
  // Extract star positions from catalogue
  bigSearch.setMagnitudeLimit(mag_0);

  QValueList<Star> star = __astro_catalog->find( bigSearch );
  QValueList<Star>::iterator it;        
  __astro_ncat = (int)star.count();

  double *temp_ra = new double[__astro_ncat];  // temp. cat. unsorted
  double *temp_dec = new double[__astro_ncat];
  double *temp_mag = new double[__astro_ncat];

  __astro_cat_ra = new double[__astro_ncat];
  __astro_cat_dec = new double[__astro_ncat];
  __astro_cat_mag = new double[__astro_ncat];
        
  int i = 0;
  for (it = star.begin(); it != star.end(); it++) {
    temp_ra[i] = (double) (*it).getRa( Radian() );
    temp_dec[i] = (double) (*it).getDec( Radian() );
    temp_mag[i] = (double) (*it).getMagnitude();
    i++;
  }
  //printf("Stars in search region: %i\n",__astro_ncat);

  // Sort the catalogue in order of increasing magnitude
  __astro_sortindex = new int[__astro_ncat];
  for(i=0;i<__astro_ncat;i++) __astro_sortindex[i] = i; //init sort index
  qsort(__astro_sortindex, __astro_ncat, 
        sizeof(__astro_sortindex[0]), compare);
        
  for(i=0;i<__astro_ncat;i++) {
    __astro_cat_ra[i] = temp_ra[i];
    __astro_cat_dec[i] = temp_dec[i];
    __astro_cat_mag[i] = temp_mag[i];
    //printf("%i mag: %lf, %lf %lf\n",i,__astro_cat_mag[i],
    //__astro_cat_ra[i]*180/PI/15,__astro_cat_dec[i]*180/PI);
  }

  delete[] temp_ra;
  delete[] temp_dec;
  delete[] temp_mag;
  delete[] __astro_sortindex;

  *ra = __astro_cat_ra;
  *dec = __astro_cat_dec;
  *mag = __astro_cat_mag;
  *ncat = __astro_ncat;

}

// Multi-star frame matching algorithm
// NOTE: quit tolerance is ignored for small numbers of blob IDs
//
// ra_0_guess    = guess for RA centre of frame
// dec_0_guess   =  "        DEC
// size          = search diameter of region in catalogue
// maglimit      = how deep to search in the map
// tolerance     = angular tolerance for a matched star (should be set) 
//                 like the FWHM of the PSF)
// lst           = lst in radians
// lat           = latitude in radians
// rot           = approximate rotation
// platescale    = CCD platescale (arcsec/pixel)
// sig_tol       = tolerance for uncertainty in the rough pointing 
//                 solution from the primary+secondary blobs
// match_tol     = fraction of blobs that must be matched
// quit_tol      = if this match fraction is achieved, the routine exits
// rot_tol       = rotational tolerance of frame WRT parallactic angle 
//                 (a few degrees)
// x             = CCD pixel offsets of blobs
// y             =  "
// flux          = flux of blob
// q             = rotation between CCD and RA/DEC coordinates
// ra            = OUTPUT: coordinates of the match (-999 if no match)
// dec           = OUTPUT: "
// mag           = OUTPUT: magnitude of the blob
// nblobs        = number of blobs
// abtflag       = pointer to semafore that indicates we are aborting
// brightStarMode= If set, add a star to the catalogue at:
// brightRA      =   right ascension
// brightDEC     =   declination
//
// return: # matched blobs for success, 0 for failure

int match_frame( double ra_0_guess, double dec_0_guess, double size, 
                 double maglimit, double tolerance, 
                 double lst, double lat, double rot, 
                 double platescale, double sig_tol, 
                 double match_tol, double quit_tol, double rot_tol,
                 double *x, double *y, double *flux, double *ra, 
                 double *dec, double *mag, int nblobs, int *abtflag,
                 int brightStarMode, double brightRA, double brightDEC) {
  int i, j, k, l, m, ncat;
  double *source_catra, *source_catdec, *source_catmag;// sorted reduced 
                                                       // catalogue b4 
                                                       // adding sources
  double *catra, *catdec, *catmag;
  
  // catalogue + extra sources inserted
  double *cat_x, *cat_y, *cat_z;
  double rot_temp;
  double temp_x[2], temp_y[2], temp_flux[2];
  double *temp_az, *temp_el, temp_ra[2], temp_dec[2];
  double temp_az_0, temp_el_0;
  double temp_ra_0, temp_dec_0;
  double *temp_el_off, *temp_az_off;
  double var, cos_theta, sin_theta;
  double d_squared, tol_squared;
  double *blob_az_off;        // tangent plane offsets for the blobs
  double *blob_el_off;

  tol_squared = tolerance*tolerance;     // square of the separation tol
  double closest_dist;                   // distance to closest star
  int closest;                           // index of star closest to blob
  int matchcount;                        // number of blobs with matches 
  int bestmatchcount;                    // matchcount for the best frame
  int *matchframe;                       // index of stars matched
  int *bestframe;                        // the best frame
  double chi_squared;                    // chi_squared of frame model 
  double best_chi_squared;               // chi_squared of the best frame
  int quitflag=0;                        // set if found a quittable match
  int matchflag=0;                       // set if one sufficient match
  double sig_tol_squared = sig_tol*sig_tol;
  
  //printf("Rot: %lf\n",rot);

  // Return if we don't have enough blobs to match a frame
  if( nblobs < 2 ) return 0;

  // Update the catalogue
  catalogue( ra_0_guess, dec_0_guess, size, maglimit, &source_catra, 
             &source_catdec, &source_catmag, &ncat );
  
  // If adding an extra bright object to the front of the catalogue, 
  // do it here
  if( brightStarMode ) {
    ncat++;  
    catra = new double[ncat];
    catdec = new double[ncat];
    catmag = new double[ncat];
    
    // put the extra star in first at an arbitrary magnitude of 0
    catra[0] = brightRA;
    catdec[0] = brightDEC;
    catmag[0] = 0;
    
    // memcpy over the rest of the catalogue
    memcpy(&catra[1],source_catra,(ncat-1)*sizeof(source_catra[0]));
    memcpy(&catdec[1],source_catdec,(ncat-1)*sizeof(source_catdec[0]));
    memcpy(&catmag[1],source_catmag,(ncat-1)*sizeof(source_catmag[0]));
  }
  // Otherwise the final catalogue is just the source catalogue
  else {
    catra = source_catra;
    catdec = source_catdec;
    catmag = source_catmag;
  }
  
  // return if less than 2 stars in the catalogue
  if( ncat < 2 ) return 0;
  
  // Get the cartesian unit vectors for each catalogue star so we can
  // calculate angular separations with dot products
  cat_x = new double[ncat];
  cat_y = new double[ncat];
  cat_z = new double[ncat];
  for(i=0; i<ncat; i++ )
    cel2vec(catra[i],catdec[i],&cat_x[i],&cat_y[i],&cat_z[i]);
  
  // Find the frame
  int primaryIndex, secondaryIndex; // blob indices primary/secondary 
  int primaryStar, secondaryStar;   // star "
  double sep, ref_sep;                                
        
  temp_az_off = new double[ncat];
  temp_el_off = new double[ncat];
  temp_az = new double[ncat];
  temp_el = new double[ncat];
  blob_az_off = new double[nblobs];
  blob_el_off = new double[nblobs];

  matchframe = new int[nblobs];
  bestframe = new int[nblobs];
  best_chi_squared = 1.e40;         // initialized to some large number
  bestmatchcount = 0;

  primaryIndex = 0;
  secondaryIndex = 1;

  while( !quitflag ) {        
    //printf("---- %i --- %i --------------\n",
    //primaryIndex,secondaryIndex);
    
    // ref separation converted from pixel units to radians in tanplane
    ref_sep = (sqrt( pow(x[primaryIndex]-x[secondaryIndex],2) + 
                     pow(y[primaryIndex]-y[secondaryIndex],2))) *
      (platescale/3600.*PI/180.);
    
    //printf("Reference separation: %lf\n",ref_sep*180./PI);
    
    // Now loop over all the possible primary and secondary stars
    // to compare with the primary and secondary blob.
    for(i=0; i<ncat-1; i++) {    // primary catalogue star
      for(j=i+1; j<ncat; j++ ) { // secondary catalogue star
        // Seperation angle is acos of the dot product  unit vectors
        sep = acos( cat_x[i]*cat_x[j] + cat_y[i]*cat_y[j] + 
                    cat_z[i]*cat_z[j] );
        
        // If the seperation angle within primary-secondary tolerance 
        // seperation, get a rough pointing solution, and project the 
        // catalogue stars onto the tangent plane to see if we can find 
        // matches for enough of the blobs
        if( fabs(sep-ref_sep) <= tolerance ) {
          primaryStar = i;
          secondaryStar = j;
          
          for( k=0; k<2; k++ ) { // second time swap primary/secondary
            // Get a map centre from the two blobs
            temp_x[0] = x[primaryIndex]; 
            temp_x[1] = x[secondaryIndex];
            temp_y[0] = y[primaryIndex]; 
            temp_y[1] = y[secondaryIndex];
            temp_flux[0] = flux[primaryIndex]; 
            temp_flux[1] = flux[secondaryIndex];
            
            temp_ra[0] = catra[primaryStar]; 
            temp_ra[1] = catra[secondaryStar];
            temp_dec[0] = catdec[primaryStar]; 
            temp_dec[1] = catdec[secondaryStar];
            
            rot_temp = rot;
            
            if( temp_y == NULL ) printf("OOOOOOGA!\n");
              
            map_centre( temp_x, temp_y, temp_flux, 2,
                        temp_ra, temp_dec, lst, lat,
                        &temp_ra_0, &temp_dec_0, &rot_temp, platescale,  
                        &var );
            
            //printf("%lf %lf\n",lst,lat);
            
            //printf("p: %i s: %i ",primaryStar,secondaryStar);
            //printf("ra: %lf dec: %lf rot: %lf rot_temp: %lf sigma: %lf",
            //           temp_ra_0*180./PI/15., temp_dec_0*180./PI, 
            //           rot*180./PI, rot_temp*180./PI, 
            //           sqrt(var)*180./PI*3600.);
            
            
            // This primary/secondary pair is reasonable if uncertainty 
            // in the map centre wasn't bigger than sig_tol from the 
            // two points, and the rotation of the field is less than 
            // rot_tol degrees from the initial guess

            //printf("sigma=%lf  tol=%lf\n",
            //           sqrt(var)*180./PI,
            //           sig_tol*180./PI);

            if( (var < sig_tol_squared) && 
                (fabs(rot_temp - rot) < rot_tol) ) {
              //printf(" ***\n");
              
              //printf("p:%i s:%i ",primaryStar,secondaryStar);
              //printf("centre: %lf %lf uncertaint: %lf rot: %lf\n",
              //       temp_ra_0*180/PI/15,temp_dec_0*180/PI,
              //     3600*sqrt(var)*180/PI,rot*180/PI);
              
              // Project the catalogue on to this tangent plane
              
              // az/el of the tangent point
              calc_alt_az( temp_ra_0, temp_dec_0, lat, lst, 
                           &temp_el_0, &temp_az_0 );
              
              // calculate the az and el of each star
              for( l=0; l<ncat; l++ ) {  
                calc_alt_az( catra[l], catdec[l], lat, lst, &temp_el[l], 
                             &temp_az[l] );
              }

              // az/el tanplane offsets for each star
              radec2tan( temp_az_0, temp_el_0, temp_az, temp_el, 
                         temp_az_off, temp_el_off, ncat);

              //for( l=0; l<ncat; l++ )
              //  printf("%lf %lf, %lf %lf, %lf %lf\n",
              //         temp_ra_0, temp_dec_0, 
              //         catra[l],catdec[l], 
              //         temp_az_off[l], temp_el_off[l]);
              
                
              cos_theta = cos(rot_temp); // in the CCD -> azel direction
              sin_theta = sin(rot_temp);  

              matchcount = 0;  // assume no blobs have star matches
              chi_squared = 0.;  

              for( l=0; l<nblobs; l++ ) {
                // rotate into blob az/el tanplane offsets (rad)
                blob_az_off[l] =  (x[l]*cos_theta + y[l]*sin_theta) *
                  (platescale/3600.*PI/180.);
                
                blob_el_off[l] = (-x[l]*sin_theta + y[l]*cos_theta) *
                  (platescale/3600.*PI/180.);
                
                //printf("(x,y)=%lf %lf, (blobx,bloby)=%lf %lf 
                // plate=%lf cos_theta=%lf sin_theta=%lf\n",x[l],y[l],
                //         blob_az_off[l],blob_el_off[l],
                // platescale,cos_theta, sin_theta);
                
                matchframe[l] = -1;  // initialize to no match
                closest = -1;        // index to closest star (if within 
                                     // tolerance)
                closest_dist = 2*PI; // distance^2 to closest star 
                
                // Find the closest projected catalogue star to blob 
                // positions
                for( m=0; m<ncat; m++ ) {
                  d_squared = (blob_az_off[l]-temp_az_off[m])*
                    (blob_az_off[l]-temp_az_off[m]) +
                    (blob_el_off[l]-temp_el_off[m]) * 
                    (blob_el_off[l]-temp_el_off[m]);
                  
                  if( d_squared < closest_dist ) {
                    closest_dist = d_squared;

                    if( d_squared < tol_squared ) 
                      closest = m;

                    //printf("%lf %lf, %lf %lf\n",
                    //            blob_az_off[l],blob_el_off[l],
                    //           temp_az_off[m],temp_el_off[m]);
                    
                  }
                }

                //printf("Closest: %lf\n",
                //sqrt(closest_dist)*180./PI*3600. );
                    
                if( closest != -1 ) {  // found a match for this blob!
                  matchframe[l] = closest;
                  matchcount ++;
                  
                  // add this seperation to the running chi_squared
                  chi_squared += closest_dist;  
                }
              } 
              
              // If the frame was a good match
              if( (((double)matchcount/nblobs >= match_tol) &&
                   (matchcount >= 2)) || (matchcount >= 4) ) {

                chi_squared = chi_squared/(double)matchcount/tol_squared;
                
                //printf("Matched %i blobs of %i, chisquared=%lf: ",
                //         matchcount,nblobs,chi_squared);
                //printf("%lf %lf rot: %lf\n",temp_ra_0*180/PI/15,
                //         temp_dec_0*180/PI,rot*180/PI);
                
                // Is this the best frame?
                //if( chi_squared < best_chi_squared )

                if( (matchcount > bestmatchcount) ||  
                    ( (chi_squared < best_chi_squared) &&
                      matchcount == bestmatchcount) ) {

                  //printf("this:%i best:%i\n",matchcount,
                  // bestmatchcount);

                  memcpy( bestframe, matchframe, 
                          sizeof(matchframe[0])*nblobs );
                  
                  best_chi_squared = chi_squared;
                  bestmatchcount = matchcount;
                }
                  
                matchflag = 1;

                // We can quit if quit_tol fraction of matched blobs
                // or if we got four matched
                
                //printf("frac: %lf quittol: %lf\n",
                //      (double)matchcount/nblobs, quit_tol);

                if( (((double)matchcount/nblobs >= quit_tol)) ||
                    (matchcount >= 5) ) {

                  //printf("QUIT!!!!\n");
                  quitflag = 1;
                  i = ncat;             // get out of the current for loops
                  j = ncat;
                  k = 2;
                }
              } else {
                //printf("Only matched %i/%i blobs.\n", 
                // matchcount, nblobs);
              }
            }
            //else printf("\n");
            
            primaryStar = j;             // swap the IDs second time around
            secondaryStar = i;
          }
        }
        
        // check the quit semafore if we're aborting
        if( *abtflag == 1 ) {
          quitflag = 1;
          matchflag = 0;
          i = ncat;                // get out of the inner loops
          j = ncat;
        }
      }
    }
    
    // Change the reference primary and secondary blobs. Irrelevant 
    // if quit was already set.
    
    // Change the secondary blob in the frame
    if(secondaryIndex < nblobs-1) secondaryIndex++;

    // Change the primary blob
    else if(primaryIndex < nblobs-2) {
      primaryIndex++;
      secondaryIndex = primaryIndex+1;
    }
    else quitflag = 1;   // we've tried all combinations
  }

  int retval;
  
  // If matching frame, place the RA and DEC (J2000) into the output 
  // array and return
  if( matchflag ) {
    for(i=0;i<nblobs;i++) 
      if( bestframe[i] != -1) {
        ra[i] = catra[bestframe[i]];
        dec[i] = catdec[bestframe[i]];
        mag[i] = catmag[bestframe[i]];
      } else {
        ra[i] = -999;
        dec[i] = -999;
        mag[i] = -999;
      }
    retval = bestmatchcount;
  }
  else retval = 0;
  
  // Clean up before returning

  delete[] cat_x;
  delete[] cat_y;
  delete[] cat_z;
  delete[] temp_az;
  delete[] temp_el;
  delete[] temp_az_off;
  delete[] temp_el_off;
  delete[] blob_az_off;
  delete[] blob_el_off;
  delete[] matchframe;
  delete[] bestframe;
  
  if( brightStarMode ) {
    delete[] catra;
    delete[] catdec;
    delete[] catmag;
  }
  
  return retval;
}        

// Single/double-star frame matching / pointing solution calculation
//
// ra_0_guess    = guess for RA centre of frame
// dec_0_guess   =  "        DEC
// size          = search diameter of region in catalogue
// maglimit      = how deep to search in the map
// tolerance     = angular tolerance for a matched star (should be set 
//                 like the FWHM of the PSF)
// lst           = lst in radians
// lat           = latitude in radians
// rot           = the rotation of the CCD frame (fixed!)
// platescale    = CCD platescale (arcsec/pixel)
// x             = CCD pixel offsets of blobs
// y             =  "
// q             = rotation between CCD and RA/DEC coordinates
// ra            = OUTPUT: the coordinates of the match (-999 no match)
// dec           = OUTPUT: "                J2000!          
// mag           = OUTPUT: magnitude of the blob
// nblobs        = number of blobs
// epoch         = epoch of observation in years
//
// return: 1 for success, 0 for failure

int match_frame_simple( double ra_0_guess, double dec_0_guess, 
                        double size, double maglimit, double tolerance, 
                        double lst, double lat, double rot, 
                        double platescale, 
                        double *x, double *y, double *ra, 
                        double *dec, double *mag, int nblobs, 
                        double epoch ) {
  int i, j, ncat, retval=0;
  double *source_catra, *source_catdec, *source_catmag;// sorted reduced 
                                                       // catalogue b4  
                                                       // adding sources
  double *catra, *catdec, *catmag;
  double *cat_x, *cat_y, *cat_z;
  double az_0_guess, el_0_guess;
  double cos_theta, sin_theta;
  double *blob_az_off, *blob_el_off;  // tanplane offsets for the blobs
  double *blob_az, *blob_el;

  double thisra, thisdec;
  double blob_cel_x, blob_cel_y, blob_cel_z;
  double sep;

  double closest_dist;                // angular distance to closest star
  int closest;                        // index of star closest to blob

  // allocate space for buffers
  blob_az_off = new double[nblobs];
  blob_el_off = new double[nblobs];
  blob_az = new double[nblobs];
  blob_el = new double[nblobs];

  // Update the catalogue - make deeper if necessary!
  ncat = 0;
  while( ncat < 7 ) {  
    catalogue( ra_0_guess, dec_0_guess, size, maglimit, &source_catra, 
               &source_catdec, &source_catmag, &ncat );

    //cout << maglimit << ": " << ncat << endl;
    maglimit = maglimit + 0.5;
  }

  catra = source_catra;
  catdec = source_catdec;
  catmag = source_catmag;
  
  // return if less than 1 star in the catalogue
  if( ncat < 1 ) return 0;
  
  // Precess tangent point coordinates to apparent
  slaPreces ( "FK5", 2000, epoch, &ra_0_guess, &dec_0_guess );

  // Get cartesian unit vectors for each catalogue star so that we can 
  // calculate angular separations with dot products
  cat_x = new double[ncat];
  cat_y = new double[ncat];
  cat_z = new double[ncat];

  for(i=0; i<ncat; i++ ) {
    // precess star coordinates to apparent before getting
    // unit vectors
    thisra = catra[i];
    thisdec = catdec[i];
    slaPreces ( "FK5", 2000, epoch, &thisra, &thisdec );
    cel2vec(thisra,thisdec,&cat_x[i],&cat_y[i],&cat_z[i]);
  }
    
  // calculate the az/el corresponding to ra/dec guess for tanpoint
  calc_alt_az( ra_0_guess, dec_0_guess, lat, lst,
               &el_0_guess, &az_0_guess );

  // using the guess pointing solution, calculate the RA + Dec of
  // each blob and then the cartesian unit vectors

  cos_theta = cos(rot); // in the CCD -> azel direction
  sin_theta = sin(rot);  

  for( i=0; i<nblobs; i++ ) {

    // rotate into azel tanplane offsets (rad)
    blob_az_off[i] =  (x[i]*cos_theta + y[i]*sin_theta) *
      (platescale/3600.*PI/180.);

    blob_el_off[i] = (-x[i]*sin_theta + y[i]*cos_theta) *
      (platescale/3600.*PI/180.);
  }

  // calculate the azel for each blob
  tan2radec( az_0_guess, el_0_guess, blob_az_off, blob_el_off,
             blob_az, blob_el, nblobs );

  // calculate the ra + dec for each blob, unit vector, closest star
  // in catalogue
  closest = 0;             // index to closest star in catalogue
  closest_dist = 0;        // distance to closest star        

  for( i=0; i<nblobs; i++ ) {

    calc_ra_dec( blob_az[i], blob_el[i], lat, lst, &thisra, &thisdec );
    cel2vec(thisra,thisdec,&blob_cel_x,&blob_cel_y,&blob_cel_z);

    // calculate distance to each star in catalogue
    for( j=0; j<ncat; j++ ) {
      sep = acos( cat_x[j]*blob_cel_x + cat_y[j]*blob_cel_y + 
                  cat_z[j]*blob_cel_z );

      if( (j == 0) || (sep < closest_dist) ) {
        closest = j;
        closest_dist = sep;
      }
    }

    // if close enough, copy matched star information to output
    if( closest_dist <= tolerance ) {
      ra[i] = catra[closest];
      dec[i] = catdec[closest];
      mag[i] = catmag[closest];
      
      retval++;
    } else {
      ra[i] = -999;
      dec[i] = -999;
      mag[i] = -999;
    }
  }
  
  // clean up

  delete[] cat_x;
  delete[] cat_y;
  delete[] cat_z;
  delete[] blob_az_off;
  delete[] blob_el_off;
  delete[] blob_az;
  delete[] blob_el;
  
  return retval;
}        


// Calculate current epoch from the system date (return value in years)
double calc_epoch( void ) {
  // calculate the current epoch
  time_t ltime_s;                 // time in seconds
  time( &ltime_s );
  struct tm *ltime;
  ltime = localtime( &ltime_s );  // full date/time structure
  int year = ltime->tm_year + 1900;
  int month = ltime->tm_mon + 1;
  int day = ltime->tm_mday;
  
  double mjd;        // modified julian date
  int flag;
  slaCaldj( year, month, day, &mjd, &flag );
  
  double epoch = slaEpj(mjd);
  //printf("Current epoch: %lf\n",epoch);
  
  return epoch;
}

// Calculate a pointing solution - all angular quantities in radians
//
// ra_0_guess    = guess for centre of the field in apparent coordinates
// dec_0_guess   =  "
// lost          = if set use pyramid frame matching algorithm
// epoch         = epoch of the field (in years)
// lat           = latitude of telescope (radians)
// lst           = local sidereal time (radians)
// x             = array of CCD pixel offsets from centre (+az direction)
// y             = "           "     "               "    (+el direction)
// flux          = measured fluxes
// nblobs        = number of blobs
// minplateblobs = min number of blobs to fit platescale ***IGNORED***
// radius        = angular radius of search region in catalogue 
//                 (should be about 2X size of CCD FOV)
// maglimit      = limiting magnitude of earch region
// tolerance     = angular tolerance for a match (should be a few pixels)
// tol_simp      = tolerance for simple solution cases (should be wider)
// sig_tol       = tolerance for solution from primary/secondary blob 
// match_tol     = fraction of blobs required for a frame to be matched
// quit_tol      = match_tol significantly high that once achieved match 
//                 may quit immediately
// rot_tol       = acceptable tolerance in the field rotation w.r.t. 
//                 the input rotation (parallactic angle)
// ra_0          = OUTPUT: pointing solution in apparent coordinates
// dec_0         = OUTPUT: "        "
// var           = OUTPUT: estimate variance in the pointing (rad^2)
// rot           = INPUT/OUTPUT: Rotation of the field WRT input rot
// platescale    = INPUT/OUTPUT: arcsec/pixel of the frame
// star_ra       = OUTPUT: apparent ra of each blob (-999 if not matched)
// star_dec      = OUTPUT: "        dec   "
// star_mag      = OUTPUT: magnitude of   "
// abtflag       = pointer to semafore indicates we are aborting the 
//                 solution calculation (when set)
// brightStarMode= If set, add a star to the catalogue at:
// brightRA      =   right ascension
// brightDEC     =   declination
//
// return: # blobs with matches used in pointing solution, 0 if failed

int calc_pointing( double ra_0_guess, double dec_0_guess, 
                   int lost, double epoch,
                   double lat, double lst, double *x, double *y, 
                   double *flux, int nblobs, int minplateblobs,  
                   double radius, double maglimit, 
                   double tolerance, double tol_simp, 
                   double sig_tol, double match_tol, 
                   double quit_tol, double rot_tol, double *ra_0, 
                   double *dec_0, double *var, double *rot, 
                   double *platescale, double *star_ra, 
                   double *star_dec, double *star_mag, 
                   int *abtflag, int brightStarMode, 
                   double brightRA, double brightDEC ) {

  int i, j, matchcount=0;
  double *temp_x, *temp_y, *temp_flux, *temp_ra, *temp_dec;
  int nsol;
  double sin_theta, cos_theta;
  double blob_az_off, blob_el_off;
  double az_blob, el_blob;
  double az_0, el_0, az_1, el_1;

  //printf("Apparent RA and DEC guess: %lf %lf\n",ra_0_guess*180/PI/15,
  //dec_0_guess*180/PI);  //jk removed comment

  // If we don't have at least one blob
  if( nblobs < 1 ) return 0;
  
  // If the lost flag was set & at least 4 blobs use the pyramid algorithm
  if( lost && (nblobs >= 4) ) {

    printf("Inside LOST IN SPACE pointing solution code\n");

    solution_t *pyrsol=NULL;
    int pyrnsol;

    temp_x = new double[nblobs];
    temp_y = new double[nblobs];

    for( i=0; i<nblobs; i++ ) {
      // convert the pixel offsets into tanplane offsets
      temp_x[i] = x[i] * (*platescale) / 206265.;
      temp_y[i] = y[i] * (*platescale) / 206265.;

      // Assume no stars will get matched
      star_ra[i] = -999;
      star_dec[i] = -999;
      matchcount = 0;
    }
    
    // Call pyramid frame match
    if( __astro_pyr->GetSolution( tolerance,
                                  temp_x, temp_y, nblobs,
                                  &pyrsol, &pyrnsol ) >= 4 ) {
      
      // Extract matched ra/dec if only 1 solution
      if( pyrnsol == 1 ) {
        matchcount = pyrsol->n;
        for( j=0; j<matchcount; j++ ) {
          star_ra[pyrsol->B[j]] = (pyrsol->C)[j]->ra;
          star_dec[pyrsol->B[j]]= (pyrsol->C)[j]->dec;
          star_mag[pyrsol->B[j]] = (pyrsol->C)[j]->mag; 
        }
      }    
    }
    
    // delete because they get used again later
    delete[] temp_x;
    delete[] temp_y;
  } 
  
  // Otherwise use the old algorithm
  else {
    // precess map centre coordinates to J2000
    slaPreces ( "FK5", epoch, 2000, &ra_0_guess, &dec_0_guess );
    
    // Match the frame. If a match was found, continue with the solution
    // printf("POINTING: match frame... %i, %lf\n",nblobs,*rot);
    
    matchcount = match_frame( ra_0_guess, dec_0_guess, radius, maglimit, 
                              tolerance, lst, lat, *rot, *platescale, 
                              sig_tol, match_tol, quit_tol, rot_tol,
                              x, y, flux, star_ra, star_dec, star_mag, 
                              nblobs, abtflag, brightStarMode, brightRA, 
                              brightDEC);
  
    // simple pointing solution for 1 or 2 stars
    if( (nblobs <= 2) || (matchcount <= 2) ) {
      matchcount = match_frame_simple( ra_0_guess, dec_0_guess, radius, 
                                       maglimit, tol_simp, lst, lat, *rot, 
                                       *platescale, x, y, 
                                       star_ra, star_dec, 
                                       star_mag, nblobs, epoch ); 
    }
  }
    
  // Pointing solution from matched blobs
  if( matchcount >= 1 ) {
    // Create arrays of subsets of blobs that had good matches
    temp_x = new double[matchcount];
    temp_y = new double[matchcount];
    temp_flux = new double[matchcount];
    temp_ra = new double[matchcount];
    temp_dec = new double[matchcount];
    int index=0;

    for( i=0; i<nblobs; i++ ) { 
      if( star_ra[i] != -999 ) {
          
        //printf("blob %i: %lf %lf %lf\n",i, star_ra[i]*180./PI/15.,
        // star_dec[i]*180./PI, star_mag[i]);
          
        // precess the star catalogue positions to apparent coordinates
        slaPreces ( "FK5", 2000, epoch, &star_ra[i], &star_dec[i] );
        temp_x[index] = x[i];
        temp_y[index] = y[i];
        temp_flux[index] = flux[i];
        temp_ra[index] = star_ra[i];
        temp_dec[index] = star_dec[i];
        index++;
      }
      //printf("blob: %i  ra:%lf  dec:%lf mag:%lf\n",i,
      // star_ra[i]*180./PI/15., star_dec[i]*180./PI,star_mag[i]);  
    }
     
    // One star case
    if( matchcount == 1 ) {
      // assuming a fixed roll calculate the tangent point of the CCD
      cos_theta = cos(*rot); // in the CCD -> azel direction
      sin_theta = sin(*rot);  
      
      blob_az_off = (temp_x[0]*cos_theta + temp_y[0]*sin_theta) *
                    (*platescale/3600.*PI/180.);
      
      blob_el_off = (-temp_x[0]*sin_theta + temp_y[0]*cos_theta) *
                    (*platescale/3600.*PI/180.);
      
      calc_alt_az( temp_ra[0], temp_dec[0], lat, lst,
                   &el_blob, &az_blob );
      
      slaTps2c( blob_az_off, blob_el_off, az_blob, el_blob, 
                            &az_0, &el_0, &az_1, &el_1, &nsol );
      
      // !!! I'm not bothering to check for two solutions !!!
      calc_ra_dec( az_0, el_0, lat, lst, ra_0, dec_0 ); 
      
      *var = 10./3600.*PI/180.;  // variance is meaningless in this case
      *var *= *var;
    } else if( matchcount == 2 ) {
      // Just find ra/dec without letting rotation vary
      map_centre3( temp_x, temp_y, temp_flux, matchcount, 
                   temp_ra, temp_dec,
                   lst, lat, ra_0, dec_0, *rot, *platescale, var );
    } else { 
      // Find ra/dec AND rotation
      map_centre( temp_x, temp_y, temp_flux, matchcount, 
                  temp_ra, temp_dec,
                  lst, lat, ra_0, dec_0, rot, *platescale, var );
    }
    
    // Clean up
    delete[] temp_x;
    delete[] temp_y;
    delete[] temp_flux;
    delete[] temp_ra;
    delete[] temp_dec;              
  }

  /*
  // return the guess in apparent coordinates if no new solution found
  else {
    *ra_0 = ra_0_guess;
    *dec_0 = dec_0_guess;
    slaPreces ( "FK5", 2000, epoch, ra_0, dec_0 );
  }
  */

  return matchcount;
}

// These two routines need to be called before and after doing pointing
// solution calculations respectively

void astro_init_catalogue( const char *pathname, char *catalogname,
                           char *katalogname ) {
  __astro_catalog = new HTMCatalog(pathname);
  __astro_catalog->init();

  __astro_pyr = new Pyramid( (3.4 * M_PI/180.0), catalogname, katalogname );
}

void astro_close_catalogue( void ) {
  delete __astro_catalog;
  delete __astro_pyr;
}
