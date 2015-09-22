
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <blast.h>

#include "geomag.h"

static WMMtype_MagneticModel *MagneticModel;
static WMMtype_MagneticModel *TimedMagneticModel;
static WMMtype_Ellipsoid Ellip;
static WMMtype_CoordSpherical CoordSpherical;
static WMMtype_CoordGeodetic CoordGeodetic;
static WMMtype_Date UserDate;
static WMMtype_GeoMagneticElements GeoMagneticElements;
static WMMtype_Geoid Geoid;

/*
 * ABSTRACT
 *
 *    The purpose of WMM Subroutine Library is to support the World Magnetic Model (WMM) 2010-2015.
 *
 *
 *
 * REUSE NOTES
 *
 *  WMM Subroutine Library is intended for reuse by any application that requires
 * Computation of Geomagnetic field from WMM model.
 *
 * REFERENCES
 *
 *    Further information on Geoid can be found in the WMM Technical Documents.
 *
 *
 * LICENSES
 *
 *  The WMM source code is in the public domain and not licensed or under copyright.
 *	The information and software may be used freely by the public. As required by 17 U.S.C. 403,
 *	third parties producing copyrighted works consisting predominantly of the material produced by
 *	U.S. government agencies must provide notice with such work(s) identifying the U.S. Government material
 *	incorporated and stating that such material is not subject to copyright protection.
 *
 * RESTRICTIONS
 *
 *    WMM Subroutine library has no restrictions.
 *
 * ENVIRONMENT
 *
 *    WMM Subroutine library was tested in the following environments
 *
 *    1. Red Hat Linux  with GCC Compiler
 *    2. MS Windows XP with CodeGear C++ compiler
 *    3. Sun Solaris with GCC Compiler
 *
 *
 * MODIFICATIONS
 *
 *    Date                 Version
 *    ----                 -----------
 *    Jul 15, 2009         0.1
 *    Nov 17, 2009         0.2
	Nov 23, 2009	   0.3
	Jan 28, 2010  	   1.0

*	Contact Information


*  Sponsoring Government Agency
*  National Geospatial-Intelligence Agency
*  PRG / CSAT, M.S. L-41
*  3838 Vogel Road
*  Arnold, MO 63010
*  Attn: Craig Rollins
*  Phone:  (314) 263-4186
*  Email:  Craig.M.Rollins@Nga.Mil


*  National Geophysical Data Center
*  NOAA EGC/2
*  325 Broadway
*  Boulder, CO 80303 USA
*  Attn: Susan McLean
*  Phone:  (303) 497-6478
*  Email:  Susan.McLean@noaa.gov


*  Software and Model Support
*  National Geophysical Data Center
*  NOAA EGC/2
*  325 Broadway"
*  Boulder, CO 80303 USA
*  Attn: Manoj Nair or Stefan Maus
*  Phone:  (303) 497-6522 or -6522
*  Email:  Manoj.C.Nair@noaa.gov or Stefan.Maus@noaa.gov
*  URL: http://www.ngdc.noaa.gov/Geomagnetic/WMM/DoDWMM.shtml


*  For more details on the subroutines, please consult the WMM
*  Technical Documentations at
*  http://www.ngdc.noaa.gov/Geomagnetic/WMM/DoDWMM.shtml

*  Nov 23, 2009
*  Written by Manoj C Nair and Adam Woods
*  Manoj.C.Nair@Noaa.Gov

*/

int WMM_AssociatedLegendreFunction(WMMtype_CoordSpherical CoordSpherical, int nMax, WMMtype_LegendreFunction *LegendreFunction)

	/* Computes  all of the Schmidt-semi normalized associated Legendre
	functions up to degree nMax. If nMax <= 16, function WMM_PcupLow is used.
	Otherwise WMM_PcupHigh is called.
	INPUT  CoordSpherical 	A data structure with the following elements
							double lambda; ( longitude)
							double phig; ( geocentric latitude )
							double r;  	  ( distance from the center of the ellipsoid)
			nMax        	integer 	 ( Maxumum degree of spherical harmonic secular model)
			LegendreFunction Pointer to data structure with the following elements
							double *Pcup;  (  pointer to store Legendre Function  )
							double *dPcup; ( pointer to store  Derivative of Lagendre function )

	OUTPUT  LegendreFunction  Calculated Legendre variables in the data structure

	 */

	{
	double sin_phi;
	int FLAG = 1;

	sin_phi =  sin ( WMM_DEG2RAD ( CoordSpherical.phig ) );       /* sin  (geocentric latitude) */

	if (nMax <= 16 || (1 - fabs(sin_phi)) < 1.0e-10 ) 	/* If nMax is less tha 16 or at the poles */
		FLAG = WMM_PcupLow(LegendreFunction->Pcup,LegendreFunction->dPcup,sin_phi, nMax);
	else FLAG = WMM_PcupHigh(LegendreFunction->Pcup,LegendreFunction->dPcup,sin_phi, nMax);
	if (FLAG == 0) /* Error while computing  Legendre variables*/
			return false;


	return true;
	} /*WMM_AssociatedLegendreFunction */

int WMM_CalculateGeoMagneticElements(WMMtype_MagneticResults *MagneticResultsGeo, WMMtype_GeoMagneticElements *GeoMagneticElements)

	/* Calculate all the Geomagnetic elements from X,Y and Z components
	INPUT     MagneticResultsGeo   Pointer to data structure with the following elements
				double Bx;    ( North )
				double By;	  ( East )
				double Bz;    ( Down )
	OUTPUT    GeoMagneticElements    Pointer to data structure with the following elements
				double Decl; (Angle between the magnetic field vector and true north, positive east)
				double Incl; Angle between the magnetic field vector and the horizontal plane, positive down
				double F; Magnetic Field Strength
				double H; Horizontal Magnetic Field Strength
				double X; Northern component of the magnetic field vector
				double Y; Eastern component of the magnetic field vector
				double Z; Downward component of the magnetic field vector
	CALLS : none
	*/
	{
	GeoMagneticElements->X = MagneticResultsGeo->Bx;
	GeoMagneticElements->Y = MagneticResultsGeo->By;
	GeoMagneticElements->Z = MagneticResultsGeo->Bz;

	GeoMagneticElements->H = sqrt (MagneticResultsGeo->Bx* MagneticResultsGeo->Bx + MagneticResultsGeo->By * MagneticResultsGeo->By);
	GeoMagneticElements->F = sqrt (GeoMagneticElements->H*GeoMagneticElements->H + MagneticResultsGeo->Bz * MagneticResultsGeo->Bz);
	GeoMagneticElements->Decl = WMM_RAD2DEG(atan2 (GeoMagneticElements->Y , GeoMagneticElements->X));
	GeoMagneticElements->Incl = WMM_RAD2DEG(atan2 (GeoMagneticElements->Z , GeoMagneticElements->H));

	return true;
	}  /*WMM_CalculateGeoMagneticElements */

int WMM_CalculateGridVariation(WMMtype_CoordGeodetic location, WMMtype_GeoMagneticElements *elements)

	/*Computes the grid variation for |latitudes| > WMM_MAX_LAT_DEGREE

	Grivation (or grid variation) is the angle between grid north and
	magnetic north. This routine calculates Grivation for the Polar Stereographic
	projection for polar locations (Latitude => |55| deg). Otherwise, it computes the grid
	variation in UTM projection system. However, the UTM projection codes may be used to compute
	the grid variation at any latitudes.

	INPUT    location    Data structure with the following elements
			double lambda; (longitude)
			double phi; ( geodetic latitude)
			double HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
			double HeightAboveGeoid;(height above the Geoid )
	OUTPUT  elements Data  structure with the following elements updated
			double GV; ( The Grid Variation )
	CALLS : WMM_GetTransverseMercator

	*/

	{
	WMMtype_UTMParameters UTMParameters;
	if(location.phi >= WMM_PS_MAX_LAT_DEGREE)
	{
		elements->GV = elements->Decl - location.lambda;
		return 1;
	}
	else if(location.phi <= WMM_PS_MIN_LAT_DEGREE)
	{
		elements->GV = elements->Decl + location.lambda;
		return 1;
	}

	else
	{
	WMM_GetTransverseMercator(location, &UTMParameters);
        elements->GV = elements->Decl - UTMParameters.ConvergenceOfMeridians;
	}
	return 0;
	} /*WMM_CalculateGridVariation*/

int WMM_CalculateSecularVariation(WMMtype_MagneticResults MagneticVariation, WMMtype_GeoMagneticElements *MagneticElements)
/*This takes the Magnetic Variation in x, y, and z and uses it to calculate the secular variation of each of the Geomagnetic elements.
	INPUT     MagneticVariation   Data structure with the following elements
				double Bx;    ( North )
				double By;	  ( East )
				double Bz;    ( Down )
	OUTPUT   MagneticElements   Pointer to the data  structure with the following elements updated
			double Decldot; Yearly Rate of change in declination
			double Incldot; Yearly Rate of change in inclination
			double Fdot; Yearly rate of change in Magnetic field strength
			double Hdot; Yearly rate of change in horizontal field strength
			double Xdot; Yearly rate of change in the northern component
			double Ydot; Yearly rate of change in the eastern component
			double Zdot; Yearly rate of change in the downward component
			double GVdot;Yearly rate of chnage in grid variation
	CALLS : none

*/
{
	MagneticElements->Xdot = MagneticVariation.Bx;
	MagneticElements->Ydot = MagneticVariation.By;
	MagneticElements->Zdot = MagneticVariation.Bz;
	MagneticElements->Hdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot) / MagneticElements->H; //See equation 19 in the WMM technical report
	MagneticElements->Fdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot + MagneticElements->Z * MagneticElements->Zdot) / MagneticElements->F;
	MagneticElements->Decldot = 180.0 / M_PI * (MagneticElements->X * MagneticElements->Ydot - MagneticElements->Y * MagneticElements->Xdot) / (MagneticElements->H * MagneticElements->H);
	MagneticElements->Incldot = 180.0 / M_PI * (MagneticElements->H * MagneticElements->Zdot - MagneticElements->Z * MagneticElements->Hdot) / (MagneticElements->F * MagneticElements->F);
	MagneticElements->GVdot = MagneticElements->Decldot;
	return true;
} /*WMM_CalculateSecularVariation*/

int WMM_ComputeSphericalHarmonicVariables(	 WMMtype_Ellipsoid  Ellip, WMMtype_CoordSpherical CoordSpherical, int nMax, WMMtype_SphericalHarmonicVariables *SphVariables)

   /* Computes Spherical variables
	  Variables computed are (a/r)^(n+2), cos_m(lamda) and sin_m(lambda) for spherical harmonic
	  summations. (Equations 10-12 in the WMM Technical Report)
	  INPUT   Ellip  data  structure with the following elements
				double a; semi-major axis of the ellipsoid
				double b; semi-minor axis of the ellipsoid
				double fla;  flattening
				double epssq; first eccentricity squared
				double eps;  first eccentricity
				double re; mean radius of  ellipsoid
			CoordSpherical 	A data structure with the following elements
				double lambda; ( longitude)
				double phig; ( geocentric latitude )
				double r;  	  ( distance from the center of the ellipsoid)
			nMax   integer 	 ( Maxumum degree of spherical harmonic secular model)\

	OUTPUT  SphVariables  Pointer to the   data structure with the following elements
		double RelativeRadiusPower[WMM_MAX_MODEL_DEGREES+1];   [earth_reference_radius_km  sph. radius ]^n
		double cos_mlambda[WMM_MAX_MODEL_DEGREES+1]; cp(m)  - cosine of (mspherical coord. longitude)
		double sin_mlambda[WMM_MAX_MODEL_DEGREES+1];  sp(m)  - sine of (mspherical coord. longitude)
	CALLS : none
	  */

	{
	double cos_lambda, sin_lambda;
	int m, n;
	cos_lambda = cos(WMM_DEG2RAD(CoordSpherical.lambda));
	sin_lambda = sin(WMM_DEG2RAD(CoordSpherical.lambda));
	/* for n = 0 ... model_order, compute (Radius of Earth / Spherica radius r)^(n+2)
	for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
	SphVariables->RelativeRadiusPower[0] = (Ellip.re / CoordSpherical.r) * (Ellip.re / CoordSpherical.r);
	for (n = 1; n <= nMax; n++)
	{
		SphVariables->RelativeRadiusPower[n] = SphVariables->RelativeRadiusPower[n-1] * (Ellip.re  / CoordSpherical.r);
	}

  /*
   Compute cos(m*lambda), sin(m*lambda) for m = 0 ... nMax
	 cos(a + b) = cos(a)*cos(b) - sin(a)*sin(b)
	 sin(a + b) = cos(a)*sin(b) + sin(a)*cos(b)
  */
	SphVariables->cos_mlambda[0] = 1.0;
	SphVariables->sin_mlambda[0] = 0.0;

	SphVariables->cos_mlambda[1] = cos_lambda;
	SphVariables->sin_mlambda[1] = sin_lambda;
	for (m = 2; m <= nMax; m++)
	{
		SphVariables->cos_mlambda[m] = SphVariables->cos_mlambda[m-1]*cos_lambda - SphVariables->sin_mlambda[m-1]*sin_lambda;
		SphVariables->sin_mlambda[m] = SphVariables->cos_mlambda[m-1]*sin_lambda + SphVariables->sin_mlambda[m-1]*cos_lambda;
	}
	return true;
	}  /*WMM_ComputeSphericalHarmonicVariables*/



void WMM_Error(int control)

	/*This prints WMM errors.
	INPUT     control     Error look up number
	OUTPUT	  none
	CALLS : none

	*/

	{
	switch(control)
	{
		case 1:
			berror(err,"Error allocating in WMM_LegendreFunctionMemory.");
			break;
		case 2:
			berror(err,"Error allocating in WMM_AllocateModelMemory.");
			break;
		case 3:
			berror(err,"Error allocating in WMM_InitializeGeoid");
			break;
		case 4:
			berror(err,"Error in setting default values.");
			break;
		case 5:
			berror(err,"Error initializing Geoid.");
			break;
		case 6:
			berror(err,"Error opening WMM.COF.");
			break;
		case 7:
			berror(err,"Error opening WMMSV.COF.");
			break;
		case 8:
			berror(err,"Error reading Magnetic Model.");
			break;
		case 9:
			berror(err,"Error printing Command Prompt introduction.");
			break;
		case 10:
			berror(err,"Error converting from geodetic co-ordinates to spherical co-ordinates.");
			break;
		case 11:
			berror(err,"Error in time modifying the Magnetic model");
			break;
		case 12:
			berror(err,"Error in Geomagnetic");
			break;
		case 13:
			berror(err,"Error printing user data");
			break;
		case 14:
			berror(err,"Error allocating in WMM_SummationSpecial");
			break;
		case 15:
			berror(err,"Error allocating in WMM_SecVarSummationSpecial");
			break;
		case 16:
			berror(err,"Error in opening EGM9615.BIN file");
			break;
		case 17:
			berror(err,"Error: Latitude OR Longitude out of range in WMM_GetGeoidHeight");
			break;
		case 18:
			berror(err,"Error allocating in WMM_PcupHigh");
			break;
		case 19:
			berror(err,"Error allocating in WMM_PcupLow");
			break;
		case 20:
			berror(err,"Error opening coefficient file");
			break;
		case 21:
			berror(err,"Error: UnitDepth too large");
			break;
	}
	} /*WMM_Error*/

int WMM_FreeMemory(WMMtype_MagneticModel *MagneticModel, WMMtype_MagneticModel *TimedMagneticModel, WMMtype_LegendreFunction *LegendreFunction)

	/* Free memory used by WMM functions. Only to be called at the end of the main function.
	INPUT :  MagneticModel	pointer to data structure with the following elements

				double EditionDate;
				double epoch;       Base time of Geomagnetic model epoch (yrs)
				char  ModelName[20];
				double *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				double *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				int nMax;  Maximum degree of spherical harmonic model
				int nMaxSecVar; Maxumum degree of spherical harmonic secular model
				int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

			TimedMagneticModel 	Pointer to data structure similar to the first input.
			LegendreFunction Pointer to data structure with the following elements
							double *Pcup;  (  pointer to store Legendre Function  )
							double *dPcup; ( pointer to store  Derivative of Lagendre function )

	OUTPUT  none
	CALLS : none

	*/

	{
		if (MagneticModel->Main_Field_Coeff_G)
		{
			free(MagneticModel->Main_Field_Coeff_G);
			MagneticModel->Main_Field_Coeff_G = NULL;
		}
		if (MagneticModel->Main_Field_Coeff_H)
		{
			free(MagneticModel->Main_Field_Coeff_H);
			MagneticModel->Main_Field_Coeff_H = NULL;
		}
		if (MagneticModel->Secular_Var_Coeff_G)
		{
			free(MagneticModel->Secular_Var_Coeff_G);
			MagneticModel->Secular_Var_Coeff_G = NULL;
		}
		if (MagneticModel->Secular_Var_Coeff_H)
		{
			free(MagneticModel->Secular_Var_Coeff_H);
			MagneticModel->Secular_Var_Coeff_H = NULL;
		}
	 if (MagneticModel)
		{
			free(MagneticModel);
			MagneticModel = NULL;
		}

		if (TimedMagneticModel->Main_Field_Coeff_G)
		{
			free(TimedMagneticModel->Main_Field_Coeff_G);
			TimedMagneticModel->Main_Field_Coeff_G = NULL;
		}
		if (TimedMagneticModel->Main_Field_Coeff_H)
		{
			free(TimedMagneticModel->Main_Field_Coeff_H);
			TimedMagneticModel->Main_Field_Coeff_H = NULL;
		}
		if (TimedMagneticModel->Secular_Var_Coeff_G)
		{
			free(TimedMagneticModel->Secular_Var_Coeff_G);
			TimedMagneticModel->Secular_Var_Coeff_G = NULL;
		}
		if (TimedMagneticModel->Secular_Var_Coeff_H)
		{
			free(TimedMagneticModel->Secular_Var_Coeff_H);
			TimedMagneticModel->Secular_Var_Coeff_H = NULL;
		}

		if (TimedMagneticModel)
		{
			free(TimedMagneticModel);
			TimedMagneticModel = NULL;
		}

		if(LegendreFunction->Pcup)
		{
			free(LegendreFunction->Pcup);
			LegendreFunction->Pcup = NULL;
		}
		if(LegendreFunction->dPcup)
		{
			free(LegendreFunction->dPcup);
			LegendreFunction->dPcup = NULL;
		}
		if(LegendreFunction)
		{
			free(LegendreFunction);
			LegendreFunction = NULL;
		}

	 return true;
	} /*WMM_FreeMemory */


int WMM_FreeMagneticModelMemory(WMMtype_MagneticModel *MagneticModel)

	/* Free the magnetic model memory used by WMM functions.
	INPUT :  MagneticModel	pointer to data structure with the following elements

				double EditionDate;
				double epoch;       Base time of Geomagnetic model epoch (yrs)
				char  ModelName[20];
				double *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				double *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				int nMax;  Maximum degree of spherical harmonic model
				int nMaxSecVar; Maxumum degree of spherical harmonic secular model
				int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

	OUTPUT  none
	CALLS : none

	*/

	{
		if (MagneticModel->Main_Field_Coeff_G)
		{
			free(MagneticModel->Main_Field_Coeff_G);
			MagneticModel->Main_Field_Coeff_G = NULL;
		}
		if (MagneticModel->Main_Field_Coeff_H)
		{
			free(MagneticModel->Main_Field_Coeff_H);
			MagneticModel->Main_Field_Coeff_H = NULL;
		}
		if (MagneticModel->Secular_Var_Coeff_G)
		{
			free(MagneticModel->Secular_Var_Coeff_G);
			MagneticModel->Secular_Var_Coeff_G = NULL;
		}
		if (MagneticModel->Secular_Var_Coeff_H)
		{
			free(MagneticModel->Secular_Var_Coeff_H);
			MagneticModel->Secular_Var_Coeff_H = NULL;
		}
	 if (MagneticModel)
		{
			free(MagneticModel);
			MagneticModel = NULL;
		}

			 return true;
	} /*WMM_FreeMagneticModelMemory */


int WMM_FreeLegendreMemory(WMMtype_LegendreFunction *LegendreFunction)

	/* Free the Legendre Coefficients memory used by WMM functions.
	INPUT : LegendreFunction Pointer to data structure with the following elements
							double *Pcup;  (  pointer to store Legendre Function  )
							double *dPcup; ( pointer to store  Derivative of Lagendre function )

	OUTPUT  none
	CALLS : none

	*/

     {
		if(LegendreFunction->Pcup)
		{
			free(LegendreFunction->Pcup);
			LegendreFunction->Pcup = NULL;
		}
		if(LegendreFunction->dPcup)
		{
			free(LegendreFunction->dPcup);
			LegendreFunction->dPcup = NULL;
		}
		if(LegendreFunction)
		{
			free(LegendreFunction);
			LegendreFunction = NULL;
		}

	 return true;
	} /*WMM_FreeLegendreMemory */


int WMM_GeodeticToSpherical(WMMtype_Ellipsoid Ellip, WMMtype_CoordGeodetic CoordGeodetic, WMMtype_CoordSpherical *CoordSpherical)

	/* Converts Geodetic coordinates to Spherical coordinates

	  INPUT   Ellip  data  structure with the following elements
				double a; semi-major axis of the ellipsoid
				double b; semi-minor axis of the ellipsoid
				double fla;  flattening
				double epssq; first eccentricity squared
				double eps;  first eccentricity
				double re; mean radius of  ellipsoid

			CoordGeodetic  Pointer to the  data  structure with the following elements updates
				double lambda; ( longitude )
				double phi; ( geodetic latitude )
				double HeightAboveEllipsoid; ( height above the WGS84 ellipsoid (HaE) )
				double HeightAboveGeoid; (height above the EGM96 Geoid model )

	 OUTPUT		CoordSpherical 	Pointer to the data structure with the following elements
				double lambda; ( longitude)
				double phig; ( geocentric latitude )
				double r;  	  ( distance from the center of the ellipsoid)

	CALLS : none

	*/
	{
	double CosLat, SinLat, rc, xp, zp; /*all local variables */

	/*
	** Convert geodetic coordinates, (defined by the WGS-84
	** reference ellipsoid), to Earth Centered Earth Fixed Cartesian
	** coordinates, and then to spherical coordinates.
	*/

	CosLat = cos(WMM_DEG2RAD(CoordGeodetic.phi));
	SinLat = sin(WMM_DEG2RAD(CoordGeodetic.phi));

	/* compute the local radius of curvature on the WGS-84 reference ellipsoid */

	rc = Ellip.a / sqrt(1.0 - Ellip.epssq * SinLat * SinLat);

	/* compute ECEF Cartesian coordinates of specified point (for longitude=0) */

	xp = (rc + CoordGeodetic.HeightAboveEllipsoid) * CosLat;
	zp = (rc*(1.0 - Ellip.epssq) + CoordGeodetic.HeightAboveEllipsoid) * SinLat;

	/* compute spherical radius and angle lambda and phi of specified point */

	CoordSpherical->r = sqrt(xp * xp + zp * zp);
	CoordSpherical->phig = WMM_RAD2DEG(asin(zp / CoordSpherical->r));     /* geocentric latitude */
	CoordSpherical->lambda = CoordGeodetic.lambda;                   /* longitude */

	return true;
	}/*WMM_GeodeticToSpherical*/

	/*Geoid Functions   */
int WMM_InitializeGeoid(const char *m_filename, WMMtype_Geoid *Geoid)
	/*
	 * The function reads Geoid data from the file EMG9615.BIN in
	 * the current directory and builds the Geoid grid from it.
	 * If the Geoid file can not be found or accessed, an error is printed
	 * and function returns false code. If the file is incomplete
	 * or improperly formatted, an error is printed
	 * and function returns false code.

	 INPUT  Pointer to data structure Geoid with the following elements
			int NumbGeoidCols ;   ( 360 degrees of longitude at 15 minute spacing )
			int NumbGeoidRows ;   ( 180 degrees of latitude  at 15 minute spacing )
			int NumbHeaderItems ;    ( min, max lat, min, max long, lat, long spacing )
			int	ScaleFactor;    ( 4 grid cells per degree at 15 minute spacing  )
			float *GeoidHeightBuffer;   (Pointer to the memory to store the Geoid elevation data )
			int NumbGeoidElevs;    (number of points in the gridded file )
			int  Geoid_Initialized ;  ( indicates successful initialization )

	OUPUT  Pointer to data structure Geoid with the following elements updated
			int NumbGeoidCols ;   ( 360 degrees of longitude at 15 minute spacing )
			int NumbGeoidRows ;   ( 180 degrees of latitude  at 15 minute spacing )
			int NumbHeaderItems ;    ( min, max lat, min, max long, lat, long spacing )
			int	ScaleFactor;    ( 4 grid cells per degree at 15 minute spacing  )
			float *GeoidHeightBuffer;   (Pointer to the memory to store the Geoid elevation data )
			int NumbGeoidElevs;    (number of points in the gridded file )
	CALLS : none
	 */
	{
  int   ElevationsRead;
  FILE  *GeoidHeightFile;


  if (Geoid->Geoid_Initialized)
  {
	return (true);
  }
  else
  {

	Geoid->GeoidHeightBuffer = ( float *) malloc ( (Geoid->NumbGeoidElevs + 1) * sizeof(float) );
	if (!Geoid->GeoidHeightBuffer){
								WMM_Error(3);
								//   printf("error allocating in WMM_InitializeGeoid\n");
								return (false);
							}

   /*  Open the File READONLY, or Return Error Condition.	EMG9615.BIN is binary
	dump of the ascii file WW15MGH.GRD. This file contains  EGM96 Geoid heights
	in 15x15 min resolution. The binary file supplied with the WMM package is
	Little Endian. Now check the system to determine its Endianness*/




  if (( GeoidHeightFile = fopen( m_filename , "rb" ) ) == NULL)
  {
	WMM_Error(16);
	return (false);
  }

  ElevationsRead = fread(Geoid->GeoidHeightBuffer, sizeof(float), Geoid->NumbGeoidElevs, GeoidHeightFile);

   if (ElevationsRead != Geoid->NumbGeoidElevs)
  {

	WMM_Error(3);
	//printf("Error in Geoid initilazation\n");
	return ( false );

  }
  fclose(GeoidHeightFile);

  Geoid->Geoid_Initialized = 1;
  }
  return ( true );
	}  /*WMM_InitializeGeoid*/


int WMM_GetGeoidHeight (double Latitude,
						double Longitude,
						double *DeltaHeight ,
						WMMtype_Geoid *Geoid)
/*
 * The  function WMM_GetGeoidHeight returns the height of the
 * EGM96 geiod above or below the WGS84 ellipsoid,
 * at the specified geodetic coordinates,
 * using a grid of height adjustments from the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    DeltaHeight         : Height Adjustment, in meters.          (output)
 *    Geoid				  : WMMtype_Geoid with Geoid grid		   (input)
	CALLS : none
 */
{
  long   Index;
  double DeltaX, DeltaY;
  double ElevationSE, ElevationSW, ElevationNE, ElevationNW;
  double OffsetX, OffsetY;
  double PostX, PostY;
  double UpperY, LowerY;
  int Error_Code = 0;

  if (!Geoid->Geoid_Initialized)
  {
	WMM_Error(5);
	//printf("Geoid not initialized\n");
	return (false);
  }
  if ((Latitude < -90) || (Latitude > 90))
  { /* Latitude out of range */
	Error_Code |= 1;
  }
  if ((Longitude < -180) || (Longitude > 360))
  { /* Longitude out of range */
	Error_Code |= 1;
  }

  if (!Error_Code)
  { /* no errors */
	/*  Compute X and Y Offsets into Geoid Height Array:                          */

	if (Longitude < 0.0)
	{
	  OffsetX = ( Longitude + 360.0 ) *Geoid->ScaleFactor;
	}
	else
	{
	  OffsetX = Longitude * Geoid->ScaleFactor;
	}
	OffsetY = ( 90.0 - Latitude ) * Geoid->ScaleFactor;

	/*  Find Four Nearest Geoid Height Cells for specified Latitude, Longitude;   */
	/*  Assumes that (0,0) of Geoid Height Array is at Northwest corner:          */

	PostX = floor( OffsetX );
	if ((PostX + 1) == Geoid->NumbGeoidCols)
	  PostX--;
	PostY = floor( OffsetY );
	if ((PostY + 1) == Geoid->NumbGeoidRows)
	  PostY--;

	Index = (long)(PostY * Geoid->NumbGeoidCols + PostX);
	ElevationNW = ( double ) Geoid->GeoidHeightBuffer[ Index ];
	ElevationNE = ( double ) Geoid->GeoidHeightBuffer[ Index+ 1 ];

	Index = (long)((PostY + 1) * Geoid->NumbGeoidCols + PostX);
	ElevationSW = ( double ) Geoid->GeoidHeightBuffer[ Index ];
	ElevationSE = ( double ) Geoid->GeoidHeightBuffer[ Index + 1 ];

	/*  Perform Bi-Linear Interpolation to compute Height above Ellipsoid:        */

	DeltaX = OffsetX - PostX;
	DeltaY = OffsetY - PostY;

	UpperY = ElevationNW + DeltaX * ( ElevationNE - ElevationNW );
	LowerY = ElevationSW + DeltaX * ( ElevationSE - ElevationSW );

	*DeltaHeight = UpperY + DeltaY * ( LowerY - UpperY );
  }

  else
  {
	WMM_Error(17);
	//printf("Latitude OR Longitude out of range in WMM_GetGeoidHeight\n");
  return (false);
  }
  return true;
}  /*WMM_GetGeoidHeight*/


int WMM_ConvertGeoidToEllipsoidHeight (WMMtype_CoordGeodetic *CoordGeodetic, WMMtype_Geoid *Geoid)

/*
 * The function Convert_Geoid_To_Ellipsoid_Height converts the specified WGS84
 * Geoid height at the specified geodetic coordinates to the equivalent
 * ellipsoid height, using the EGM96 gravity model.
 *
  *   CoordGeodetic->phi        : Geodetic latitude in degress           (input)
 *    CoordGeodetic->lambda     : Geodetic longitude in degrees          (input)
 *    CoordGeodetic->HeightAboveEllipsoid	     : Ellipsoid height, in kilometers         (output)
 *    CoordGeodetic->HeightAboveGeoid: Geoid height, in kilometers           (input)
 *
	CALLS : WMM_GetGeoidHeight (

 */
{
  double  DeltaHeight;
  int Error_Code;

  if (Geoid->UseGeoid == 1) {      /* Geoid correction required */
			Error_Code = WMM_GetGeoidHeight ( CoordGeodetic->phi, CoordGeodetic->lambda, &DeltaHeight, Geoid );
			CoordGeodetic->HeightAboveEllipsoid = CoordGeodetic->HeightAboveGeoid + DeltaHeight / 1000; /*  Input and output should be kilometers,
			However WMM_GetGeoidHeight returns Geoid height in meters - Hence division by 1000 */
						}
	else     /* Geoid correction not required, copy the MSL height to Ellipsoid height */
	{
	CoordGeodetic->HeightAboveEllipsoid = CoordGeodetic->HeightAboveGeoid;
	Error_Code = true;
    }
  return ( Error_Code );
} /* WMM_ConvertGeoidToEllipsoidHeight*/



WMMtype_LegendreFunction *WMM_AllocateLegendreFunctionMemory(int NumTerms)

	/* Allocate memory for Associated Legendre Function data types.
	   Should be called before computing Associated Legendre Functions.

	 INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


	 OUTPUT:    Pointer to data structure WMMtype_LegendreFunction with the following elements
				double *Pcup;  (  pointer to store Legendre Function  )
				double *dPcup; ( pointer to store  Derivative of Lagendre function )

				false: Failed to allocate memory

	CALLS : none

	*/

	{
	WMMtype_LegendreFunction * LegendreFunction;

	LegendreFunction =  (WMMtype_LegendreFunction * ) calloc(1, sizeof(WMMtype_LegendreFunction));

	if (!LegendreFunction) {
		WMM_Error(1);
		//printf("error allocating in WMM_AllocateLegendreFunctionMemory\n");
		return false;
					}
	LegendreFunction->Pcup = (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );
	if (LegendreFunction->Pcup == 0)
	{
		WMM_Error(1);
		//printf("error allocating in WMM_AllocateLegendreFunctionMemory\n");
		return false;
	}
	LegendreFunction->dPcup = (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );
	if (LegendreFunction->dPcup == 0)
	{
		WMM_Error(1);
		//printf("error allocating in WMM_AllocateLegendreFunctionMemory\n");
		return false;
	}
	return LegendreFunction;
	}  /*WMMtype_LegendreFunction*/


WMMtype_MagneticModel *WMM_AllocateModelMemory(int NumTerms)

	/* Allocate memory for WMM Coefficients
	* Should be called before reading the model file *

	  INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


	 OUTPUT:    Pointer to data structure WMMtype_MagneticModel with the following elements
				double EditionDate;
				double epoch;       Base time of Geomagnetic model epoch (yrs)
				char  ModelName[20];
				double *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				double *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				int nMax;  Maximum degree of spherical harmonic model
				int nMaxSecVar; Maxumum degree of spherical harmonic secular model
				int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

				false: Failed to allocate memory
	CALLS : none
	*/
	{
	WMMtype_MagneticModel *MagneticModel;


	MagneticModel =  (WMMtype_MagneticModel * ) calloc(1, sizeof(WMMtype_MagneticModel));

	if (MagneticModel == NULL) {
		WMM_Error(2);
		//printf("error allocating in WMM_AllocateModelMemory\n");
		return false;
					}

	MagneticModel->Main_Field_Coeff_G =  (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );

	if (MagneticModel->Main_Field_Coeff_G == NULL)
	{
		WMM_Error(2);
		//printf("error allocating in WMM_AllocateModelMemory\n");
		return false;
	}

	MagneticModel->Main_Field_Coeff_H =  (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );

	if (MagneticModel->Main_Field_Coeff_H == NULL)
	{
		WMM_Error(2);
		//printf("error allocating in WMM_AllocateModelMemory\n");
		return false;
	}
	MagneticModel->Secular_Var_Coeff_G =  (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );
	if (MagneticModel->Secular_Var_Coeff_G == NULL)
	{
		WMM_Error(2);
		//printf("error allocating in WMM_AllocateModelMemory\n");
		return false;
	}
	MagneticModel->Secular_Var_Coeff_H =  (double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );
	if (MagneticModel->Secular_Var_Coeff_H == NULL)
	{
		WMM_Error(2);
		//printf("error allocating in WMM_AllocateModelMemory\n");
		return false;
	}
	return MagneticModel;

	} /*WMM_AllocateModelMemory*/

int WMM_PcupHigh(double *Pcup, double *dPcup, double x, int nMax)

/*	This function evaluates all of the Schmidt-semi normalized associated Legendre
	functions up to degree nMax. The functions are initially scaled by
	10^280 sin^m in order to minimize the effects of underflow at large m
	near the poles (see Holmes and Featherstone 2002, J. Geodesy, 76, 279-299).
	Note that this function performs the same operation as WMM_PcupLow.
	However this function also can be used for high degree (large nMax) models.

	Calling Parameters:
		INPUT
			nMax:	 Maximum spherical harmonic degree to compute.
			x:		cos(colatitude) or sin(latitude).

		OUTPUT
			Pcup:	A vector of all associated Legendgre polynomials evaluated at
					x up to nMax. The lenght must by greater or equal to (nMax+1)*(nMax+2)/2.
		  dPcup:   Derivative of Pcup(x) with respect to latitude

		CALLS : none
	Notes:



  Adopted from the FORTRAN code written by Mark Wieczorek September 25, 2005.

  Manoj Nair, Nov, 2009 Manoj.C.Nair@Noaa.Gov

  Change from the previous version
  The prevous version computes the derivatives as
  dP(n,m)(x)/dx, where x = sin(latitude) (or cos(colatitude) ).
  However, the WMM Geomagnetic routines requires dP(n,m)(x)/dlatitude.
  Hence the derivatives are multiplied by sin(latitude).
  Removed the options for CS phase and normalizations.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.

  The derivates can't be computed for latitude = |90| degrees.
	*/
	{
	double  pm2, pm1, pmm, plm, rescalem, z, scalef;
	double *f1, *f2, *PreSqr;
	int k, kstart, m, n, NumTerms;

	NumTerms = ( ( nMax + 1 ) * ( nMax + 2 ) / 2 );


	if (fabs(x) == 1.0)
	{
	  printf("Error in PcupHigh: derivative cannot be calculated at poles\n");
	  return false;
	}


	f1  =   	(double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );
	if (f1 == 0)
	{
		WMM_Error(18);
		//printf("error allocating in WMM_PcupHigh\n");
		return false;
	}


	PreSqr = 	(double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );

	if (PreSqr == 0)
	{
		WMM_Error(18);
		//printf("error allocating in WMM_PcupHigh\n");
		return false;
	}

	f2  = 		(double *) 	malloc	( (NumTerms +1) * sizeof ( double ) );

	if (f2 == 0)
	{
		WMM_Error(18);
		//printf("error allocating in WMM_PcupHigh\n");
		return false;
	}

	scalef = 1.0e-280;

	for(n = 0 ; n <= 2*nMax+1 ; ++n )
	{
		PreSqr[n] = sqrt((double)(n));
	}

	k = 2;

	for(n=2 ; n<=nMax ; n++)
	{
		k = k + 1;
		f1[k] = (double)(2*n-1) /(double)(n);
		f2[k] = (double)(n-1) /(double)(n);
		for(m=1 ; m<=n-2 ; m++)
		{
			k = k+1;
			f1[k] = (double)(2*n-1) / PreSqr[n+m] / PreSqr[n-m];
			f2[k] = PreSqr[n-m-1] * PreSqr[n+m-1] / PreSqr[n+m] / PreSqr[n-m];
		}
		k = k + 2;
	}

	/*z = sin (geocentric latitude) */
	z = sqrt((1.0-x)*(1.0+x));
	pm2  = 1.0;
	Pcup[0] = 1.0;
	dPcup[0] = 0.0;
	if (nMax == 0)
		return false;
	pm1  		= x;
	Pcup[1] 	= pm1;
	dPcup[1] 	= z;
	k = 1;

	for(n = 2; n <= nMax; n++ )
	{
		k = k+n;
		plm = f1[k]*x*pm1-f2[k]*pm2;
		Pcup[k] = plm;
		dPcup[k] = (double)(n) * (pm1 - x * plm) / z;
		pm2  = pm1;
		pm1  = plm;
	}

	pmm = PreSqr[2]*scalef;
	rescalem = 1.0/scalef;
	kstart = 0;

	for(m = 1; m <= nMax - 1; ++m)
	{
		rescalem = rescalem*z;

		/* Calculate Pcup(m,m)*/
		kstart = kstart+m+1;
		pmm =  pmm * PreSqr[2*m+1] / PreSqr[2*m];
		Pcup[kstart] = pmm*rescalem / PreSqr[2*m+1];
		dPcup[kstart] = -((double)(m) * x * Pcup[kstart] / z);
		pm2 = pmm/PreSqr[2*m+1];
		/* Calculate Pcup(m+1,m)*/
		k = kstart+m+1 ;
		pm1 = x * PreSqr[2*m+1] * pm2;
		Pcup[k] = pm1*rescalem;
		dPcup[k] =   ((pm2*rescalem) * PreSqr[2*m+1] - x * (double)(m+1) * Pcup[k]) / z;
		/* Calculate Pcup(n,m)*/
		for(n = m+2; n <= nMax; ++n)
		{
			k = k+n;
			plm  = x*f1[k]*pm1-f2[k]*pm2;
			Pcup[k] = plm*rescalem;
			dPcup[k] = (PreSqr[n+m] * PreSqr[n-m] * (pm1 * rescalem) - (double)(n) * x * Pcup[k] ) / z;
			pm2  = pm1;
			pm1  = plm;
		}
	}

	/* Calculate Pcup(nMax,nMax)*/
	rescalem = rescalem*z;
	kstart = kstart+m+1;
	pmm =  pmm  / PreSqr[2*nMax];
	Pcup[kstart] = pmm * rescalem;
	dPcup[kstart] = -(double)(nMax) * x * Pcup[kstart] / z;
	free(f1);
	free(PreSqr);
	free(f2);

	return true ;
} /* WMM_PcupHigh */

int WMM_PcupLow( double *Pcup, double *dPcup, double x, int nMax)

/*   This function evaluates all of the Schmidt-semi normalized associated Legendre
	functions up to degree nMax.

	Calling Parameters:
		INPUT
			nMax:	 Maximum spherical harmonic degree to compute.
			x:		cos(colatitude) or sin(latitude).

		OUTPUT
			Pcup:	A vector of all associated Legendgre polynomials evaluated at
					x up to nMax.
		   dPcup: Derivative of Pcup(x) with respect to latitude

	Notes: Overflow may occur if nMax > 20 , especially for high-latitudes.
	Use WMM_PcupHigh for large nMax.

   Writted by Manoj Nair, June, 2009 . Manoj.C.Nair@Noaa.Gov.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.
*/
{
	int n, m, index, index1, index2, NumTerms;
	double k, z, *schmidtQuasiNorm;
	Pcup[0] = 1.0;
	dPcup[0] = 0.0;
		/*sin (geocentric latitude) - sin_phi */
	z = sqrt( ( 1.0 - x ) * ( 1.0 + x ) ) ;

	NumTerms = ( ( nMax + 1 ) * ( nMax + 2 ) / 2 );
	schmidtQuasiNorm  =   	(double *) 	malloc	( (NumTerms +1 )* sizeof ( double ) );

	if (schmidtQuasiNorm == 0)
	{
		WMM_Error(19);
		//printf("error allocating in WMM_PcupLow\n");
		return false;
	}

	/*	 First,	Compute the Gauss-normalized associated Legendre  functions*/
	for (n = 1; n <=  nMax; n++)
	{
		for (m=0;m<=n;m++)
		{
		index = (n * (n + 1) / 2 + m);
			if (n == m)
			{
				index1 = ( n - 1 ) * n / 2 + m -1;
				Pcup [index]  = z * Pcup[index1];
				dPcup[index] = z *  dPcup[index1] + x *  Pcup[index1];
			}
			else if (n == 1 && m == 0)
			{
				index1 = ( n - 1 ) * n / 2 + m;
				Pcup[index]  = x *  Pcup[index1];
				dPcup[index] = x *  dPcup[index1] - z *  Pcup[index1];
			}
			else if (n > 1 && n != m)
			{
				index1 = ( n - 2 ) * ( n - 1 ) / 2 + m;
				index2 = ( n - 1) * n / 2 + m;
				if (m > n - 2)
				{
					Pcup[index]  = x *  Pcup[index2];
					dPcup[index] = x *  dPcup[index2] - z *  Pcup[index2];
				}
				else
				{
					k = (double)( ( ( n - 1 ) * ( n - 1 ) ) - ( m * m ) ) / ( double ) ( ( 2 * n - 1 ) * ( 2 * n - 3 ) );
					Pcup[index]  = x *  Pcup[index2]  - k  *  Pcup[index1];
					dPcup[index] = x *  dPcup[index2] - z *  Pcup[index2] - k *  dPcup[index1];
				}
			}
		}
	}
/*Compute the ration between the Gauss-normalized associated Legendre
  functions and the Schmidt quasi-normalized version. This is equivalent to
  sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */

	schmidtQuasiNorm[0] = 1.0;
	for (n = 1; n <= nMax; n++)
	{
		index = (n * (n + 1) / 2);
		index1 = (n - 1)  * n / 2 ;
		/* for m = 0 */
		schmidtQuasiNorm[index] =  schmidtQuasiNorm[index1] * (double) (2 * n - 1) / (double) n;

		for ( m = 1; m <= n; m++)
		{
			index = (n * (n + 1) / 2 + m);
			index1 = (n * (n + 1) / 2 + m - 1);
			schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrt( (double) ((n - m + 1) * (m == 1 ? 2 : 1)) / (double) (n + m));
		}

	}

/* Converts the  Gauss-normalized associated Legendre
	  functions to the Schmidt quasi-normalized version using pre-computed
	  relation stored in the variable schmidtQuasiNorm */

	for (n = 1; n <=  nMax; n++)
	{
		for (m=0;m<=n;m++)
		{
			 index = (n * (n + 1) / 2 + m);
			 Pcup[index]  = Pcup[index]  *  schmidtQuasiNorm[index];
			 dPcup[index] =  - dPcup[index] *  schmidtQuasiNorm[index];
			 /* The sign is changed since the new WMM routines use derivative with respect to latitude
			 insted of co-latitude */
		}
	}

	if(schmidtQuasiNorm)
		free(schmidtQuasiNorm);
	return true;
}   /*WMM_PcupLow */


int WMM_readMagneticModel(const char *filename, WMMtype_MagneticModel * MagneticModel)
{

/* READ WORLD Magnetic MODEL SPHERICAL HARMONIC COEFFICIENTS (WMM.cof)
   INPUT :  filename
   	MagneticModel : Pointer to the data structure with the following fields required as inputs
				nMax : 	Number of static coefficients
   UPDATES : MagneticModel : Pointer to the data structure with the following fields populated
				char  *ModelName;
				double epoch;       Base time of Geomagnetic model epoch (yrs)
				double *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
				double *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
				double *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
	CALLS : none

*/

	FILE *WMM_COF_File;
	char c_str[81], c_new[5];   /*these strings are used to read a line from coefficient file*/
	int i, icomp, m, n, EOF_Flag = 0, index;
	double epoch, gnm, hnm, dgnm, dhnm;
	WMM_COF_File = fopen(filename,"r");
	if (WMM_COF_File == NULL)
	{
		WMM_Error(20);
		//printf("Error in opening %s File\n",filename);
		return false;
		/* should we have a standard error printing routine ?*/
	}
	MagneticModel->Main_Field_Coeff_H[0] = 0.0;
	MagneticModel->Main_Field_Coeff_G[0] = 0.0;
	MagneticModel->Secular_Var_Coeff_H[0] = 0.0;
	MagneticModel->Secular_Var_Coeff_G[0] = 0.0;
	fgets(c_str, 80, WMM_COF_File);
	sscanf(c_str,"%lf%s",&epoch, MagneticModel->ModelName);
	MagneticModel->epoch = epoch;
	while (EOF_Flag == 0)
	{
		fgets(c_str, 80, WMM_COF_File);
		/* CHECK FOR LAST LINE IN FILE */
		for (i=0; i<4 && (c_str[i] != '\0'); i++)
		{
			c_new[i] = c_str[i];
			c_new[i+1] = '\0';
		}
		icomp = strcmp("9999", c_new);
		if (icomp == 0)
		{
			EOF_Flag = 1;
			break;
		}
		/* END OF FILE NOT ENCOUNTERED, GET VALUES */
		sscanf(c_str,"%d%d%lf%lf%lf%lf",&n,&m,&gnm,&hnm,&dgnm,&dhnm);
		if (m <= n)
		{
			index = (n * (n + 1) / 2 + m);
			MagneticModel->Main_Field_Coeff_G[index] = gnm;
			MagneticModel->Secular_Var_Coeff_G[index] = dgnm;
			MagneticModel->Main_Field_Coeff_H[index] = hnm;
			MagneticModel->Secular_Var_Coeff_H[index] = dhnm;
		}
	}

	fclose(WMM_COF_File);
	return true;
} /*WMM_readMagneticModel */



int WMM_RotateMagneticVector(WMMtype_CoordSpherical CoordSpherical, WMMtype_CoordGeodetic CoordGeodetic, WMMtype_MagneticResults MagneticResultsSph, WMMtype_MagneticResults *MagneticResultsGeo)
	/* Rotate the Magnetic Vectors to Geodetic Coordinates
	Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
	Equation 16, WMM Technical report

	INPUT : CoordSpherical : Data structure WMMtype_CoordSpherical with the following elements
				double lambda; ( longitude)
				double phig; ( geocentric latitude )
				double r;  	  ( distance from the center of the ellipsoid)

			CoordGeodetic : Data structure WMMtype_CoordGeodetic with the following elements
				double lambda; (longitude)
				double phi; ( geodetic latitude)
				double HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
				double HeightAboveGeoid;(height above the Geoid )

			MagneticResultsSph : Data structure WMMtype_MagneticResults with the following elements
				double Bx;     North
				double By;	   East
				double Bz;    Down

	OUTPUT: MagneticResultsGeo Pointer to the data structure WMMtype_MagneticResults, with the following elements
				double Bx;     North
				double By;	   East
				double Bz;    Down

	CALLS : none

	*/
	{
	double  Psi;
		 /* Difference between the spherical and Geodetic latitudes */
	Psi =  ( M_PI/180 ) * ( CoordSpherical.phig - CoordGeodetic.phi );

		 /* Rotate spherical field components to the Geodeitic system */
		MagneticResultsGeo->Bz =     MagneticResultsSph.Bx *  sin(Psi) + MagneticResultsSph.Bz * cos(Psi);
		MagneticResultsGeo->Bx =     MagneticResultsSph.Bx *  cos(Psi) - MagneticResultsSph.Bz * sin(Psi);
		MagneticResultsGeo->By =     MagneticResultsSph.By;
	return true;
	}   /*WMM_RotateMagneticVector*/

int WMM_SetDefaults(WMMtype_Ellipsoid *Ellip, WMMtype_MagneticModel *MagneticModel, WMMtype_Geoid *Geoid)

/*
	Sets default values for WMM subroutines.

	UPDATES : Ellip
			MagneticModel
			Geoid

	CALLS : none




*/
{

		/* Sets WGS-84 parameters */
		Ellip->a	=			6378.137; /*semi-major axis of the ellipsoid in */
		Ellip->b	=			6356.7523142;/*semi-minor axis of the ellipsoid in */
		Ellip->fla	=			1/298.257223563;/* flattening */
		Ellip->eps	=			sqrt(1- ( Ellip->b *	Ellip->b) / (Ellip->a * Ellip->a ));  /*first eccentricity */
		Ellip->epssq	=			(Ellip->eps * Ellip->eps);   /*first eccentricity squared */
		Ellip->re	=			6371.2;/* Earth's radius */

	   /* Sets Magnetic Model parameters */

		MagneticModel->nMax = WMM_MAX_MODEL_DEGREES;
		MagneticModel->nMaxSecVar = WMM_MAX_SECULAR_VARIATION_MODEL_DEGREES;

	   /* Sets EGM-96 model file parameters */
		Geoid->NumbGeoidCols = 1441;   /* 360 degrees of longitude at 15 minute spacing */
		Geoid->NumbGeoidRows = 721;   /* 180 degrees of latitude  at 15 minute spacing */
		Geoid->NumbHeaderItems = 6;    /* min, max lat, min, max long, lat, long spacing*/
		Geoid->ScaleFactor = 4;    /* 4 grid cells per degree at 15 minute spacing  */
		Geoid->NumbGeoidElevs  = Geoid->NumbGeoidCols * Geoid->NumbGeoidRows;
		Geoid->Geoid_Initialized  = 0; /*  Geoid will be initialized only if this is set to zero */
		Geoid->UseGeoid = WMM_USE_GEOID;


		return true;
}  /*WMM_SetDefaults */

int WMM_SecVarSummation(WMMtype_LegendreFunction *LegendreFunction, WMMtype_MagneticModel *MagneticModel, WMMtype_SphericalHarmonicVariables SphVariables, WMMtype_CoordSpherical CoordSpherical, WMMtype_MagneticResults *MagneticResults)
{
	/*This Function sums the secular variation coefficients to get the secular variation of the Magnetic vector.
	INPUT :  LegendreFunction
			MagneticModel
			SphVariables
			CoordSpherical
	OUTPUT : MagneticResults

	CALLS : WMM_SecVarSummationSpecial

	*/
	int m, n, index;
	double cos_phi;
	MagneticModel->SecularVariationUsed = true;
	MagneticResults->Bz = 0.0;
	MagneticResults->By = 0.0;
	MagneticResults->Bx = 0.0;
	for (n = 1; n <=  MagneticModel->nMaxSecVar; n++)
	{
		for (m=0;m<=n;m++)
		{
			index = (n * (n + 1) / 2 + m);

/*		    nMax  	(n+2) 	  n     m            m           m
	Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
			n=1      	      m=0   n            n           n  */
/*  Derivative with respect to radius.*/
			MagneticResults->Bz -= 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Secular_Var_Coeff_G[index]*SphVariables.cos_mlambda[m] +
						MagneticModel->Secular_Var_Coeff_H[index]*SphVariables.sin_mlambda[m]	)
						* (double) (n+1) * LegendreFunction-> Pcup[index];

/*		  1 nMax  (n+2)    n     m            m           m
	By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1             m=0   n            n           n  */
/* Derivative with respect to longitude, divided by radius. */
			MagneticResults->By += 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Secular_Var_Coeff_G[index]*SphVariables.sin_mlambda[m] -
						MagneticModel->Secular_Var_Coeff_H[index]*SphVariables.cos_mlambda[m]  )
						* (double) (m) * LegendreFunction-> Pcup[index];
/*		   nMax  (n+2) n     m            m           m
	Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1         m=0   n            n           n  */
/* Derivative with respect to latitude, divided by radius. */

			MagneticResults->Bx -= 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Secular_Var_Coeff_G[index]*SphVariables.cos_mlambda[m]  +
						MagneticModel->Secular_Var_Coeff_H[index]*SphVariables.sin_mlambda[m]  )
						* LegendreFunction-> dPcup[index];
		}
	}
	cos_phi = cos ( WMM_DEG2RAD ( CoordSpherical.phig ) );
	if ( fabs(cos_phi) > 1.0e-10 )
	{
		MagneticResults->By = MagneticResults->By / cos_phi ;
	}
	else
	/* Special calculation for component By at Geographic poles */
	{
		WMM_SecVarSummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
	}
	return true;
} /*WMM_SecVarSummation*/

int WMM_SecVarSummationSpecial(WMMtype_MagneticModel *MagneticModel, WMMtype_SphericalHarmonicVariables SphVariables, WMMtype_CoordSpherical CoordSpherical, WMMtype_MagneticResults *MagneticResults)
{
	/*Special calculation for the secular variation summation at the poles.


	INPUT: MagneticModel
		   SphVariables
		   CoordSpherical
	OUTPUT: MagneticResults
	CALLS : none


	*/
	int n, index;
	double k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

	PcupS = (double *) malloc ( (MagneticModel->nMaxSecVar +1) * sizeof (double) );

	if (PcupS == 0)
	{
		WMM_Error(15);
		//printf("error allocating in WMM_SummationSpecial\n");
		return false;
	}

	PcupS[0] = 1;
	schmidtQuasiNorm1 = 1.0;

	MagneticResults->By = 0.0;
	sin_phi = sin ( WMM_DEG2RAD ( CoordSpherical.phig ) );

	for (n = 1; n <=  MagneticModel->nMaxSecVar; n++)
	{
		index = (n * (n + 1) / 2 + 1);
		schmidtQuasiNorm2 = schmidtQuasiNorm1 * (double) (2 * n - 1) / (double) n;
		schmidtQuasiNorm3 = schmidtQuasiNorm2 *  sqrt( (double) (n * 2) / (double) (n + 1));
		schmidtQuasiNorm1 = schmidtQuasiNorm2;
		if (n == 1)
		{
			PcupS[n] = PcupS[n-1];
		}
		else
		{
			k = (double)( ( (n - 1) * (n - 1) ) - 1) / ( double ) ( (2 * n - 1) * (2 * n - 3) );
			PcupS[n] =     sin_phi * PcupS[n-1] - k * PcupS[n-2];
		}

/*		  1 nMax  (n+2)    n     m            m           m
	By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1             m=0   n            n           n  */
/* Derivative with respect to longitude, divided by radius. */

		MagneticResults->By += 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Secular_Var_Coeff_G[index]*SphVariables.sin_mlambda[1] -
						MagneticModel->Secular_Var_Coeff_H[index]*SphVariables.cos_mlambda[1]  )
						*  PcupS[n] * schmidtQuasiNorm3;
	}

	if (PcupS)
		free(PcupS);
	return true;
}/*SecVarSummationSpecial*/

int WMM_Summation(WMMtype_LegendreFunction *LegendreFunction, WMMtype_MagneticModel *MagneticModel, WMMtype_SphericalHarmonicVariables SphVariables, WMMtype_CoordSpherical CoordSpherical, WMMtype_MagneticResults *MagneticResults)
{
	/* Computes Geomagnetic Field Elements X, Y and Z in Spherical coordinate system using
	spherical harmonic summation.


	The vector Magnetic field is given by -grad V, where V is Geomagnetic scalar potential
	The gradient in spherical coordinates is given by:

			 dV ^     1 dV ^        1     dV ^
	grad V = -- r  +  - -- t  +  -------- -- p
			 dr       r dt       r sin(t) dp


	INPUT :  LegendreFunction
			MagneticModel
			SphVariables
			CoordSpherical
	OUTPUT : MagneticResults

	CALLS : WMM_SummationSpecial



   Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
   */
	int m, n, index;
	double cos_phi;
	MagneticResults->Bz = 0.0;
	MagneticResults->By = 0.0;
	MagneticResults->Bx = 0.0;
	for (n = 1; n <=  MagneticModel->nMax; n++)
	{
		for (m=0;m<=n;m++)
		{
			index = (n * (n + 1) / 2 + m);

/*		    nMax  	(n+2) 	  n     m            m           m
	Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
			n=1      	      m=0   n            n           n  */
/* Equation 12 in the WMM Technical report.  Derivative with respect to radius.*/
			MagneticResults->Bz -= 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Main_Field_Coeff_G[index]*SphVariables.cos_mlambda[m] +
						MagneticModel->Main_Field_Coeff_H[index]*SphVariables.sin_mlambda[m]	)
						* (double) (n+1) * LegendreFunction-> Pcup[index];

/*		  1 nMax  (n+2)    n     m            m           m
	By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1             m=0   n            n           n  */
/* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
			MagneticResults->By += 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Main_Field_Coeff_G[index]*SphVariables.sin_mlambda[m] -
						MagneticModel->Main_Field_Coeff_H[index]*SphVariables.cos_mlambda[m]  )
						* (double) (m) * LegendreFunction-> Pcup[index];
/*		   nMax  (n+2) n     m            m           m
	Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1         m=0   n            n           n  */
/* Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius. */

			MagneticResults->Bx -= 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Main_Field_Coeff_G[index]*SphVariables.cos_mlambda[m]  +
						MagneticModel->Main_Field_Coeff_H[index]*SphVariables.sin_mlambda[m]  )
						* LegendreFunction-> dPcup[index];



		}
	}

	cos_phi = cos ( WMM_DEG2RAD ( CoordSpherical.phig ) );
	if ( fabs(cos_phi) > 1.0e-10 )
	{
		MagneticResults->By = MagneticResults->By / cos_phi ;
	}
	else
	/* Special calculation for component - By - at Geographic poles.
	* If the user wants to avoid using this function,  please make sure that
	* the latitude is not exactly +/-90. An option is to make use the function
	* WMM_CheckGeographicPoles.
	*/

	{
		WMM_SummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
	}
	return true;
}/*WMM_Summation */

int WMM_SummationSpecial(WMMtype_MagneticModel *MagneticModel, WMMtype_SphericalHarmonicVariables SphVariables, WMMtype_CoordSpherical CoordSpherical, WMMtype_MagneticResults *MagneticResults)
	/* Special calculation for the component By at Geographic poles.
	Manoj Nair, June, 2009 manoj.c.nair@noaa.gov
    INPUT: MagneticModel
		   SphVariables
		   CoordSpherical
	OUTPUT: MagneticResults
	CALLS : none
	See Section 1.4, "SINGULARITIES AT THE GEOGRAPHIC POLES", WMM Technical report

	*/
	{
	int n, index;
	double k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

	PcupS = (double *) malloc ( (MagneticModel->nMax +1) * sizeof (double) );
	if (PcupS == 0)
	{
		WMM_Error(14);
		//printf("error allocating in WMM_SummationSpecial\n");
		return false;
	}

	PcupS[0] = 1;
	schmidtQuasiNorm1 = 1.0;

	MagneticResults->By = 0.0;
	sin_phi = sin ( WMM_DEG2RAD ( CoordSpherical.phig ) );

	for (n = 1; n <=  MagneticModel->nMax; n++)
	{

	/*Compute the ration between the Gauss-normalized associated Legendre
  functions and the Schmidt quasi-normalized version. This is equivalent to
  sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */

		index = (n * (n + 1) / 2 + 1);
		schmidtQuasiNorm2 = schmidtQuasiNorm1 * (double) (2 * n - 1) / (double) n;
		schmidtQuasiNorm3 = schmidtQuasiNorm2 *  sqrt( (double) (n * 2) / (double) (n + 1));
		schmidtQuasiNorm1 = schmidtQuasiNorm2;
		if (n == 1)
		{
			PcupS[n] = PcupS[n-1];
		}
		else
		{
			k = (double)( ( (n - 1) * (n - 1) ) - 1) / ( double ) ( (2 * n - 1) * (2 * n - 3) );
			PcupS[n] =     sin_phi * PcupS[n-1] - k * PcupS[n-2];
		}

/*		  1 nMax  (n+2)    n     m            m           m
	By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
		   n=1             m=0   n            n           n  */
/* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */

		MagneticResults->By += 	SphVariables.RelativeRadiusPower[n] *
					(	MagneticModel->Main_Field_Coeff_G[index]*SphVariables.sin_mlambda[1] -
						MagneticModel->Main_Field_Coeff_H[index]*SphVariables.cos_mlambda[1]  )
						*  PcupS[n] * schmidtQuasiNorm3;
	}

	if (PcupS)
		free(PcupS);
	return true;
	}/*WMM_SummationSpecial */

int WMM_TimelyModifyMagneticModel(WMMtype_Date UserDate, WMMtype_MagneticModel *MagneticModel,  WMMtype_MagneticModel *TimedMagneticModel)

	/* Time change the Model coefficients from the base year of the model using secular variation coefficients.
	Store the coefficients of the static model with their values advanced from epoch t0 to epoch t.
	Copy the SV coefficients.  If input "t�" is the same as "t0", then this is merely a copy operation.
	If the address of "TimedMagneticModel" is the same as the address of "MagneticModel", then this procedure overwrites
	the given item "MagneticModel".

	INPUT: UserDate
		   MagneticModel
	OUTPUT:TimedMagneticModel
	CALLS : none
	*/

	{
	int n, m, index, a, b;
	TimedMagneticModel->EditionDate = MagneticModel->EditionDate;
	TimedMagneticModel->epoch	   = MagneticModel->epoch;
        TimedMagneticModel->nMax	   	   = MagneticModel->nMax;
	TimedMagneticModel->nMaxSecVar = MagneticModel->nMaxSecVar;
	a = TimedMagneticModel->nMaxSecVar;
	b = (a * (a + 1) / 2 + a);
	strcpy(TimedMagneticModel->ModelName,MagneticModel->ModelName);
	for (n = 1; n <=  MagneticModel->nMax; n++)
	{
		for (m=0;m<=n;m++)
		{
			index = (n * (n + 1) / 2 + m);
			if(index <= b)
			{
				TimedMagneticModel->Main_Field_Coeff_H[index]   = MagneticModel->Main_Field_Coeff_H[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_H[index];
				TimedMagneticModel->Main_Field_Coeff_G[index]   = MagneticModel->Main_Field_Coeff_G[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_G[index];
				TimedMagneticModel->Secular_Var_Coeff_H[index]  = MagneticModel->Secular_Var_Coeff_H[index]; // We need a copy of the secular var coef to calculate secular change
				TimedMagneticModel->Secular_Var_Coeff_G[index]  = MagneticModel->Secular_Var_Coeff_G[index];
			}
			else
			{
				TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index];
				TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index];
			}
		}
	}
	return true;
	} /* WMM_TimelyModifyMagneticModel */


int WMM_Geomag(WMMtype_Ellipsoid Ellip,  WMMtype_CoordSpherical CoordSpherical, WMMtype_CoordGeodetic CoordGeodetic,
	WMMtype_MagneticModel *TimedMagneticModel, WMMtype_GeoMagneticElements  *GeoMagneticElements)
   /*
   The main subroutine that calls a sequence of WMM sub-functions to calculate the magnetic field elements for a single point.
   The function expects the model coefficients and point coordinates as input and returns the magnetic field elements and
   their rate of change. Though, this subroutine can be called successively to calculate a time series, profile or grid
   of magnetic field, these are better achieved by the subroutine WMM_Grid.

   INPUT: Ellip
		 CoordSpherical
		 CoordGeodetic
		 TimedMagneticModel

   OUTPUT : GeoMagneticElements

   CALLS:  	WMM_AllocateLegendreFunctionMemory(NumTerms);  ( For storing the ALF functions )
			WMM_ComputeSphericalHarmonicVariables( Ellip, CoordSpherical, TimedMagneticModel->nMax, &SphVariables); (Compute Spherical Harmonic variables  )
			WMM_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction);  	Compute ALF
			WMM_Summation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSph);  Accumulate the spherical harmonic coefficients
			WMM_SecVarSummation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSphVar); Sum the Secular Variation Coefficients
			WMM_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); Map the computed Magnetic fields to Geodeitic coordinates
			WMM_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSphVar, &MagneticResultsGeoVar);  Map the secular variation field components to Geodetic coordinates
			WMM_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements);   Calculate the Geomagnetic elements
			WMM_CalculateSecularVariation(MagneticResultsGeoVar, GeoMagneticElements); Calculate the secular variation of each of the Geomagnetic elements

   */



	{
	WMMtype_LegendreFunction *LegendreFunction;
	WMMtype_SphericalHarmonicVariables SphVariables;
	int NumTerms;
	WMMtype_MagneticResults MagneticResultsSph, MagneticResultsGeo, MagneticResultsSphVar, MagneticResultsGeoVar;

	NumTerms = ( ( WMM_MAX_MODEL_DEGREES + 1 ) * ( WMM_MAX_MODEL_DEGREES + 2 ) / 2 );    /* MAXDEGREEOFMODEL is defined in WMMHEADER.H */
	LegendreFunction 		   = WMM_AllocateLegendreFunctionMemory(NumTerms);  /* For storing the ALF functions */

	WMM_ComputeSphericalHarmonicVariables( Ellip, CoordSpherical, TimedMagneticModel->nMax, &SphVariables); /* Compute Spherical Harmonic variables  */
	WMM_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction);  	/* Compute ALF  */
	WMM_Summation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSph); /* Accumulate the spherical harmonic coefficients*/
	WMM_SecVarSummation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSphVar); /*Sum the Secular Variation Coefficients  */
	WMM_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); /* Map the computed Magnetic fields to Geodeitic coordinates  */
	WMM_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSphVar, &MagneticResultsGeoVar); /* Map the secular variation field components to Geodetic coordinates*/
	WMM_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements);   /* Calculate the Geomagnetic elements, Equation 18 , WMM Technical report */
	WMM_CalculateSecularVariation(MagneticResultsGeoVar, GeoMagneticElements); /*Calculate the secular variation of each of the Geomagnetic elements*/

	WMM_FreeLegendreMemory(LegendreFunction);

    return true;
	}





int WMM_GetTransverseMercator(WMMtype_CoordGeodetic CoordGeodetic, WMMtype_UTMParameters *UTMParameters)
   /* Gets the UTM Parameters for a given Latitude and Longitude.

   INPUT: CoordGeodetic : Data structure WMMtype_CoordGeodetic.
   OUTPUT : UTMParameters : Pointer to data structure WMMtype_UTMParameters with the following elements
			double Easting;  (X) in meters
			double Northing; (Y) in meters
			int Zone; UTM Zone
			char HemiSphere ;
			double CentralMeridian; Longitude of the Central Meridian of the UTM Zone
			double ConvergenceOfMeridians;  Convergence of Meridians
			double PointScale;
*/


   {

   double Eps, Epssq;
   double Acoeff[8];
   double Lam0, K0, falseE, falseN;
   double K0R4, K0R4oa;
   double Lambda, Phi;
   int XYonly;
   double X, Y, pscale, CoM;
   int Zone;
   char Hemisphere;



/*   Get the map projection  parameters */

   Lambda = WMM_DEG2RAD (CoordGeodetic.lambda);
   Phi = WMM_DEG2RAD (CoordGeodetic.phi);

   WMM_GetUTMParameters (Phi, Lambda, &Zone, &Hemisphere, &Lam0);
   K0 =  0.9996;



	if(Hemisphere=='n' || Hemisphere=='N')
	{
		falseN=0;
	}
	if(Hemisphere=='s' || Hemisphere=='S')
	{
		falseN=10000000;
	}

	falseE=500000;


   //WGS84 ellipsoid

	 Eps=0.081819190842621494335;
	 Epssq=0.0066943799901413169961;
	 K0R4=6367449.1458234153093;
	 K0R4oa=0.99832429843125277950;


	Acoeff[0] = 8.37731820624469723600E-04;
	Acoeff[1] = 7.60852777357248641400E-07;
	Acoeff[2] = 1.19764550324249124400E-09;
	Acoeff[3] = 2.42917068039708917100E-12;
	Acoeff[4] = 5.71181837042801392800E-15;
	Acoeff[5] = 1.47999793137966169400E-17;
	Acoeff[6] = 4.10762410937071532000E-20;
	Acoeff[7] = 1.21078503892257704200E-22;

	//WGS84 ellipsoid


/*   Execution of the forward T.M. algorithm  */

	  XYonly = 0;

	  WMM_TMfwd4 (Eps, Epssq, K0R4, K0R4oa, Acoeff,
			 Lam0, K0, falseE, falseN,
			 XYonly,
			 Lambda, Phi,
			&X, &Y, &pscale, &CoM);

/*   Report results  */

	UTMParameters->Easting = X; /* UTM Easting (X) in meters*/
	UTMParameters->Northing = Y;/* UTM Northing (Y) in meters */
	UTMParameters->Zone = Zone;/*UTM Zone*/
	UTMParameters->HemiSphere = Hemisphere;
	UTMParameters->CentralMeridian = WMM_RAD2DEG (Lam0);  /* Central Meridian of the UTM Zone */
	UTMParameters->ConvergenceOfMeridians = WMM_RAD2DEG (CoM) ;  /* Convergence of meridians of the UTM Zone and location */
	UTMParameters->PointScale = pscale;

   return 0;
   }


 int  WMM_GetUTMParameters (  double Latitude,
							  double Longitude,
							  int   *Zone,
							  char   *Hemisphere,
							  double *CentralMeridian)
{
/*
 * The function WMM_GetUTMParameters converts geodetic (latitude and
 * longitude) coordinates to UTM projection parameters (zone, hemisphere and central meridian)
 * If any errors occur, the error code(s) are returned
 * by the function, otherwise true is returned.
 *
 *    Latitude          : Latitude in radians                 (input)
 *    Longitude         : Longitude in radians                (input)
 *    Zone              : UTM zone                            (output)
 *    Hemisphere        : North or South hemisphere           (output)
 *    CentralMeridian	: Central Meridian of the UTM Zone in radians	   (output)
 */

  long Lat_Degrees;
  long Long_Degrees;
  long temp_zone;
  int  Error_Code = 0;



  if ((Latitude < WMM_DEG2RAD(WMM_UTM_MIN_LAT_DEGREE)) || (Latitude > WMM_DEG2RAD(WMM_UTM_MAX_LAT_DEGREE)))
  { /* Latitude out of range */
	WMM_Error(23);
	Error_Code = 1;
  }
  if ((Longitude < -M_PI) || (Longitude > (2*M_PI)))
  { /* Longitude out of range */
	 WMM_Error(24);
	 Error_Code = 1;
  }
  if (!Error_Code)
  { /* no errors */
    if (Longitude < 0)
	  Longitude += (2*M_PI) + 1.0e-10;
    Lat_Degrees = (long)(Latitude * 180.0 / M_PI);
    Long_Degrees = (long)(Longitude * 180.0 / M_PI);

    if (Longitude < M_PI)
      temp_zone = (long)(31 + ((Longitude * 180.0 / M_PI) / 6.0));
	else
      temp_zone = (long)(((Longitude * 180.0 / M_PI) / 6.0) - 29);
    if (temp_zone > 60)
      temp_zone = 1;
    /* UTM special cases */
    if ((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > -1)
        && (Long_Degrees < 3))
      temp_zone = 31;
    if ((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > 2)
        && (Long_Degrees < 12))
      temp_zone = 32;
    if ((Lat_Degrees > 71) && (Long_Degrees > -1) && (Long_Degrees < 9))
      temp_zone = 31;
    if ((Lat_Degrees > 71) && (Long_Degrees > 8) && (Long_Degrees < 21))
      temp_zone = 33;
    if ((Lat_Degrees > 71) && (Long_Degrees > 20) && (Long_Degrees < 33))
      temp_zone = 35;
    if ((Lat_Degrees > 71) && (Long_Degrees > 32) && (Long_Degrees < 42))
      temp_zone = 37;

	if (!Error_Code)
	{
      if (temp_zone >= 31)
		*CentralMeridian = (6 * temp_zone - 183) * M_PI / 180.0;
	  else
		*CentralMeridian = (6 * temp_zone + 177) * M_PI / 180.0;
	  *Zone = temp_zone;
	  if (Latitude < 0)	*Hemisphere = 'S';
	  else *Hemisphere = 'N';
	}
  } /* END OF if (!Error_Code) */
  return (Error_Code);
} /* END OF Convert_Geodetic_To_UTM */



void WMM_TMfwd4(double Eps, double Epssq, double K0R4, double K0R4oa,
		 double Acoeff[], double Lam0, double K0, double falseE,
		 double falseN, int XYonly, double Lambda, double Phi,
		 double *X, double *Y, double *pscale, double *CoM)
   {

/*  Transverse Mercator forward equations including point-scale and CoM
	=--------- =------- =--=--= ---------

   Algorithm developed by: C. Rollins   August 7, 2006
   C software written by:  K. Robins


   Constants fixed by choice of ellipsoid, choice of projection param.s
   --------- -----

	  Eps          Eccentricity (epsilon) of the ellipsoid
	  Epssq        Eccentricity squared
	( R4           Meridional isoperimetric radius   )
    ( K0           Central scale factor              )
	  K0R4         K0 times R4
      K0R4oa       K0 times Ratio of R4 over semi-major axis
	  Acoeff       Trig series coefficients, omega as a function of chi
	  Lam0         Longitude of the central meridian in radians
	  K0           Central scale factor, for example, 0.9996 for UTM
	  falseE       false easting, for example, 500000 for UTM
	  falseN       false northing

   Processing option
   ---------- ------

	  XYonly       If one (1), then only X and Y will be properly
				   computed.  Values returned for point-scale
				   and CoM will merely be the trivial values for
				   points on the central meridian

   Input items that identify the point to be converted
   ----- -----

	  Lambda       Longitude (from Greenwich) in radians
	  Phi          Latitude in radians

   Output items
   ------ -----

	  X            X coordinate (Easting) in meters
	  Y            Y coordinate (Northing) in meters
	  pscale       point-scale (dimensionless)
      CoM          Convergence-of-meridians in radians
*/

   double Lam, CLam, SLam, CPhi, SPhi;
   double P, part1, part2, denom, CChi, SChi;
   double U, V;
   double T, Tsq, denom2;
   double c2u, s2u, c4u, s4u, c6u, s6u, c8u, s8u;
   double c2v, s2v, c4v, s4v, c6v, s6v, c8v, s8v;
   double Xstar, Ystar;
   double sig1, sig2, comroo;

/*
   Ellipsoid to sphere
   --------- -- ------

   Convert longitude (Greenwhich) to longitude from the central meridian
   It is unnecessary to find the (-Pi, Pi] equivalent of the result.
   Compute its cosine and sine.
*/

   Lam = Lambda - Lam0;
   CLam = cos(Lam);
   SLam = sin(Lam);

/*   Latitude  */

   CPhi = cos(Phi);
   SPhi = sin(Phi);

/*   Convert geodetic latitude, Phi, to conformal latitude, Chi
     Only the cosine and sine of Chi are actually needed.        */

   P = exp(Eps * WMM_ATanH(Eps * SPhi));
   part1 = (1 + SPhi) / P;
   part2 = (1 - SPhi) * P;
   denom = 1 / (part1 + part2);
   CChi = 2 * CPhi * denom;
   SChi = (part1 - part2) * denom ;

/*
   Sphere to first plane
   ------ -- ----- -----

   Apply spherical theory of transverse Mercator to get (u,v) coord.s
   Note the order of the arguments in Fortran's version of ArcTan, i.e.
             atan2(y, x) = ATan(y/x)
   The two argument form of ArcTan is needed here.
*/

   T = CChi * SLam;
   U = WMM_ATanH(T);
   V = atan2(SChi, CChi * CLam);

/*
   Trigonometric multiple angles
   ------------- -------- ------

   Compute Cosh of even multiples of U
   Compute Sinh of even multiples of U
   Compute Cos  of even multiples of V
   Compute Sin  of even multiples of V
*/

   Tsq = T * T;
   denom2 = 1 / (1 - Tsq);
   c2u = (1 + Tsq) * denom2;
   s2u = 2 * T * denom2;
   c2v = (-1 + CChi * CChi * (1 + CLam * CLam)) * denom2;
   s2v = 2 * CLam * CChi * SChi * denom2;

   c4u = 1 + 2 * s2u * s2u;
   s4u = 2 * c2u * s2u;
   c4v = 1 - 2 * s2v * s2v;
   s4v = 2 * c2v * s2v;

   c6u = c4u * c2u + s4u * s2u;
   s6u = s4u * c2u + c4u * s2u;
   c6v = c4v * c2v - s4v * s2v;
   s6v = s4v * c2v + c4v * s2v;

   c8u = 1 + 2 * s4u * s4u;
   s8u = 2 * c4u * s4u;
   c8v = 1 - 2 * s4v * s4v;
   s8v = 2 * c4v * s4v;


/*   First plane to second plane
     ----- ----- -- ------ -----

     Accumulate terms for X and Y
*/

   Xstar =         Acoeff[3] * s8u * c8v;
   Xstar = Xstar + Acoeff[2] * s6u * c6v;
   Xstar = Xstar + Acoeff[1] * s4u * c4v;
   Xstar = Xstar + Acoeff[0] * s2u * c2v;
   Xstar = Xstar + U ;

   Ystar =         Acoeff[3] * c8u * s8v;
   Ystar = Ystar + Acoeff[2] * c6u * s6v;
   Ystar = Ystar + Acoeff[1] * c4u * s4v;
   Ystar = Ystar + Acoeff[0] * c2u * s2v;
   Ystar = Ystar + V ;

/*   Apply isoperimetric radius, scale adjustment, and offsets  */

   *X = K0R4 * Xstar + falseE;
   *Y = K0R4 * Ystar + falseN;


/*  Point-scale and CoM
    ----- ----- --- ---  */

   if (XYonly == 1)
      {
      *pscale = K0;
      *CoM = 0;
      }
   else
      {
      sig1 =        8 * Acoeff[3] * c8u * c8v;
      sig1 = sig1 + 6 * Acoeff[2] * c6u * c6v;
      sig1 = sig1 + 4 * Acoeff[1] * c4u * c4v;
      sig1 = sig1 + 2 * Acoeff[0] * c2u * c2v;
      sig1 = sig1 + 1;

      sig2 =        8 * Acoeff[3] * s8u * s8v;
      sig2 = sig2 + 6 * Acoeff[2] * s6u * s6v;
      sig2 = sig2 + 4 * Acoeff[1] * s4u * s4v;
      sig2 = sig2 + 2 * Acoeff[0] * s2u * s2v;

/*    Combined square roots  */
      comroo = sqrt((1 - Epssq * SPhi * SPhi) * denom2*
                  (sig1 * sig1 + sig2 * sig2));

      *pscale = K0R4oa * 2 * denom * comroo;
      *CoM = atan2(SChi * SLam, CLam) + atan2(sig2, sig1);
      }
   }

/*************************************************************************/

bool MagModelInit(const char *m_filename)
{
	const char *mag_model = (m_filename?m_filename:GEOMAG_WMM);
	int NumTerms = ( ( WMM_MAX_MODEL_DEGREES + 1 ) * ( WMM_MAX_MODEL_DEGREES + 2 ) / 2 );    /* WMM_MAX_MODEL_DEGREES is defined in WMM_Header.h */

	MagneticModel 	   = WMM_AllocateModelMemory(NumTerms);  /* For storing the WMM Model parameters */
	TimedMagneticModel  = WMM_AllocateModelMemory(NumTerms);  /* For storing the time modified WMM Model parameters */
	if(MagneticModel == NULL || TimedMagneticModel == NULL)
	{
		berror(err,"Could not initialize Magnetic Model!");
		return false;
	}

	WMM_SetDefaults(&Ellip, MagneticModel, &Geoid); /* Set default values and constants */
	if ( WMM_readMagneticModel(mag_model, MagneticModel) && WMM_InitializeGeoid(GEOMAG_GEOID, &Geoid))
	{
		blast_startup("Successfully initialized Magnetic Model");
		return true;
	}
	else return false;

}

/*************************************************************************/

void GetMagModel(double alt, double glat, double glon, double time,
		double *dec, double *dip)
{
	CoordGeodetic.lambda = glon;
	CoordGeodetic.phi = glat;
	CoordGeodetic.HeightAboveGeoid = alt;
	WMM_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &Geoid);
	UserDate.DecimalYear = time;

	WMM_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical);    /*Convert from geodeitic to Spherical Equations: 17-18, WMM Technical report*/
	WMM_TimelyModifyMagneticModel(UserDate, MagneticModel, TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
	WMM_Geomag(Ellip, CoordSpherical, CoordGeodetic, TimedMagneticModel, &GeoMagneticElements);   /* Computes the geoMagnetic field elements and their time change*/
	WMM_CalculateGridVariation(CoordGeodetic,&GeoMagneticElements);

	*dec = GeoMagneticElements.Decl;
	*dip = GeoMagneticElements.Incl;
}
/*************************************************************************/
