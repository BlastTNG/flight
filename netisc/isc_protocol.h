/*  isc_protocol.h: define data structures for communcation between the server (ISC) and clients (Sam,Frodo,...)

	Overview:

	The server program (netisc on the ISC computer) will connect several clients, each on a separate port. 
	
	Any client may send ISCStatusStruct data frames. The server takes the appropriate action to achieve that status.
	While the camera is busy changing the status, the server will continue accepting new frames but will ignore
	them until the previous changes have been made and it is ready to make further changes.

	The server sends ISCSolutionStruct data frames to all connected clients whenever a new image has been processed.

*/

#ifndef __ISC_PROTOCOL_H
#define __ISC_PROTOCOL_H

#ifdef _WINDOWS_	// If we're in windows need to change the packing order of the frames so that they have the
#pragma pack(2)		// same packing as in Linux
#endif

//#define _ISC_CCD     // using the the original ISC
//#define _OSC_CCD     // using the "other" ISC

#define MAX_ISC_BLOBS 20					// static # blobs to store in data frames
#define FOCUS_RANGE 2550					// # steps range for focus stepper
#define AP_RANGE 495						// # steps range for aperture stepper

#define ISC_CCD_X_PIXELS 1312					// pixel dimenions of the CCD
#define ISC_CCD_Y_PIXELS 1024					//   "        "     "   "   "

#define OSC_CCD_X_PIXELS 1360					// pixel dimenions of the CCD
#define OSC_CCD_Y_PIXELS 1036					//   "        "     "   "   "

#ifdef _ISC_CCD
#define CCD_X_PIXELS ISC_CCD_X_PIXELS
#define CCD_Y_PIXELS ISC_CCD_Y_PIXELS
#endif

#ifdef _OSC_CCD
#define CCD_X_PIXELS OSC_CCD_X_PIXELS
#define CCD_Y_PIXELS OSC_CCD_Y_PIXELS
#endif


typedef enum {full, roi, blob} ISCDisplayModeType;

struct ISCStatusStruct {
  // General server state
  int abort;			// 1 abort current execution thread   
  int pause;			// 1 is paused, 0 is pulse mode (doesn't apply to autofocus mode)
  int save;				// 1 save frames
  int autofocus;		// 1 auto focus on brightest blob in field
  int focus_pos;		// stepper position for focus
  int ap_pos;			// stepper position for aperture
  int MCPFrameNum;		// current frame number of MCP
  int shutdown;			// 0=nothing 1=shutdown 2=reboot computer 3=camera power cycle
  int hold_current;	    // the hold "heater" current (0-50)
  int exposure;			// *** new - exposure time in us REGARDLESS of self/hardware trigger ***

  // Display mode parameters
  ISCDisplayModeType display_mode;
  int roi_x;			// x pixel number for ROI
  int roi_y;			// y pixel number for ROI
  int blob_num;			// blob # for ROI
  double azBDA;			// az tangent plane offset from BDA centre for the CCD (radians)
  double elBDA;			// el   "      "     "    "    "    "    "  "  

  // telescope attitude
  double az;			// az in radians
  double el;			// el in radians
  double lst;			// lst in radians (!!)
  double lat;			// North Lat in radians

  // "brightest star is..." state
  int brightStarMode;	// 1 brightest star in field is at:
  double brightRA;		// RA: radians (apparent)
  double brightDEC;		// DEC: radians (apparent)

  // blob algorithm stuff
  double sn_threshold;
  int grid;				// ask ed
  int cenbox;			// ask ed
  int apbox;			// ask chapin
  int mult_dist;		// ed knows

  // *** new gain/offset stuff ***
  double gain;			// relative gain to the CCD factory default
  int offset;			// offset to the CCD factory default (in digitized units)

  // Matching algorithm
  int maxBlobMatch;		// maximum # blobs used in the frame matching
  double mag_limit;		// size of tires
  double norm_radius;	// search radius in normal mode
  double lost_radius;	// search radius when lost
  double tolerance;		// star asociation tolerance
  double match_tol;		// % of blobs required for a match
  double quit_tol;		// match_tol sufficient to exit from algorithm without testing all combinations
  double rot_tol;		// rotation tolerance for blob finding
};

struct ISCSolutionStruct {
  int cameraerr;		// 0 if everything is Kosher, otherwise problem...
  int framenum;			// ISC frame # (also file names for images)
  int MCPFrameNum;		// last MCP frame number received before image processing began
  int autoFocusPosition;// focus position determined from automatic procedure
  double mapMean;		// mean value of the map
  double ra;			// RA of CCD in radians (apparent)
  double dec;			// DEC   "        "        "
  double sigma;			// uncertainty of solution in radians (or 2PI on failure)

  double temp1;	        // temperature sensors
  double temp2;
  double temp3;
  double temp4;
  double pressure1;     // pressure sensor

  int n_blobs;
  double blob_x[MAX_ISC_BLOBS];
  double blob_y[MAX_ISC_BLOBS];
  int blob_flux[MAX_ISC_BLOBS];
  double blob_sn[MAX_ISC_BLOBS];
};

// -----------------------------------------------------------------------------------------------
// Stuff for INFOCUS flight
// -----------------------------------------------------------------------------------------------

/*
typedef struct { uint16_t data[4]; } clientCmdType;

typedef enum {NULLCMD    = 0,    // do nothing
	
	          ABORT      = 1, 
              PAUSE      = 2,    // toggle

              AZ         = 5,    // deg 
			  EL         = 6,    // deg
              LST        = 7,    // hr
              LAT        = 8,    // deg
              
              FOCUS      = 10,    // relative steps (+ away) or -1 go home
              AUTOFOCUS  = 11,   
              FSTOP      = 12,    // relative steps 
              DISPLAYMODE= 13,   // 0=full, 1=blob #
              EXPOSURE   = 14,   // ms
              SAVEDELAY  = 15,   // s
              
              SNTHRESH   = 20,   // floating point
              GRID       = 21,   // uint
              CENBOX     = 22,   // uint
              APBOX      = 23,   // uint
              MULTDIST   = 24,   // uint
              
              ROT_TOL    = 25,   // deg
              MAG_LIM    = 26,   // floating point
              RADIUS     = 27,   // deg - this sets both the normal and lost search radii for simplicity... maybe a bad idea
              ASSOC_TOL  = 28,   // floating point
              MATCH_TOL  = 29,   // floating point
			  MAXBLOB    = 30,   // int

			  REBOOT     = 48,
			  SHUTDOWN   = 49,
			  POWCYCLE   = 50

             } cmdType;
*/

#ifdef _WINDOWS_	
#pragma pack()		// go back to default packing
#endif

#endif
