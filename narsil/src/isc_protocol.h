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

#define MAX_ISC_BLOBS 20					// static # blobs to store in data frames
#define FOCUS_RANGE 2550					// # steps range for focus stepper
#define AP_RANGE 495						// # steps range for aperture stepper
#define CCD_X_PIXELS 1312					// pixel dimenions of the CCD
#define CCD_Y_PIXELS 1024					//   "        "     "   "   "

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
  int shutdown;			// If set, command the ISC computer to shut down (before a power cycle)

  // Display mode parameters
  ISCDisplayModeType display_mode;
  int roi_x;			// x pixel number for ROI
  int roi_y;			// y pixel number for ROI
  int blob_num;			// blob # for ROI
  double azBDA;			// az offset from CCD centre for the BDA (radians)
  double elBDA;			// el   "     "    "    "    "    "  "  

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

  int n_blobs;
  double blob_x[MAX_ISC_BLOBS];
  double blob_y[MAX_ISC_BLOBS];
  int blob_flux[MAX_ISC_BLOBS];
  double blob_sn[MAX_ISC_BLOBS];
};

#ifdef _WINDOWS_	
#pragma pack()		// go back to default packing
#endif

#endif
