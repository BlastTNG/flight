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
  int pause; // 1 is paused, 0 is pulse mode
  int save;  // 1: save frames
  int focus_pos;
  int ap_pos;
  ISCDisplayModeType display_mode;
  int roi_x;
  int roi_y;
  int blob_num;
  double az; // az in radians
  double el; // el in radians
  double lst; // lst in radians (!!)
  double lat; // North Lat in radians

  // blob algorithm stuffffff
  double sn_threshold;
  int grid; // ask ed
  int cenbox; // ask ec
  int apbox; // ask chaipin
  int mult_dist; // ed knows

  double mag_limit; // size of tires
  double norm_radius; // search radius in normal mode
  double lost_radius; // search radius when lost
  double tolerance; // star asociation tolerance
  double match_tol;
  double quit_tol;
  double rot_tol; // rotation tolerance for blob finding
};

struct ISCSolutionStruct {
  int framenum;
  int n_blobs;
  double blob_x[MAX_ISC_BLOBS];
  double blob_y[MAX_ISC_BLOBS];
  double blob_flux[MAX_ISC_BLOBS];
  double blob_sn[MAX_ISC_BLOBS];

  double ra;
  double dec;
  double sigma;
};

#ifdef _WINDOWS_	
#pragma pack()		// go back to default packing
#endif

#endif

/*
// enumerated type for the commands
typedef enum
{
	nocmd				= 0,	// null command, do nothing
	setTriggerMode		= 1,	// set trigger mode: 0=software, 1=edge, 2=pulse (par1)
	// If doing an expose or a freerun, setting par1=1 saves the images, par1=0 doesn't 
	expose				= 2,	// take a single exposure
	freerun				= 3,	// start continuous exposure loop until new command received	
	stepFocus			= 10,	// run the focus drive relative # of steps
	autoFocus			= 11,	// do an autofocus operation
	setFocus			= 12,	// set the focus to an absolute position (0-FOCUS_RANGE)
	stepAperture		= 20,   // run the aperture drive relative # of steps
	setAperture			= 21,	// set the aperture to an absolute position (0-AP_RANGE)
	updateSettings		= 30,   // set camera/search parameters from client frame data
	displayMode			= 31,	// display mode on server:
								// -1=fullscreen, (par1)
								// -2=keep centred over pixel(x, y) (par1,par2,par3)
								// otherwise ROI on blob # (par1)
	                            
	abortcmd			= 100,	// abort current command
	quit				= 200	// tell the server program (netisc) to quit
} isc_command;

// enumerated type for ISC error messages
typedef enum
{
	noerr				= 0,	// null error message
	success				= 1,	// command successfully completed
	failure				= 2,    // command failed (no further information)
	cambadcomm			= 10,	// communications error with the camera
	camtimeout			= 11,	// camera communication timed out
	apbadcomm			= 20,	// communications error with aperture stepper motor
	aptimeout			= 21,	// aperture stepper motor communication timed out
	matchbad			= 30,	// matching algorithm failed completely, pointing solution meaningless
	matchpoor			= 31,	// match was successful but probably poor solution
} isc_error;

// Client (command) data frame
typedef struct
{
	isc_command command;		// command for the server
		
	double az;					// radians for everything
	double el;
	double lst;
	double lat;

	unsigned long exposure;		// integration time (microseconds)
	int	gyro_speed;				// current gyro speed
	double platescale;			// # degrees/pixel on the CCD
	unsigned int gain;			// camera pre-amp gain 
	unsigned int offset;		// camera pre-amp offset
	unsigned int saturation;	// pixel saturation value
	double threshold;			// S/N threshold for blobs
	unsigned int grid;			// N pixels/side search grid
	unsigned int cenbox;		// "       "     centroiding box
	unsigned int apbox;			// "       "     photometry box
	unsigned int multiple_dist; // Distance threshold (pixels) to reject multiple sources
	int par1;					// additional command parameters
	int par2;
	int par3;
	int par4;
} client_frame;

// Server (ISC) response data frame
typedef struct
{
	isc_command response;		// command this frame is responding to
	int port;					// the port the command arrived on
	isc_error error;			// error code for the response

	unsigned int framenum;		// frame number
	double ra;					// pointing solution ra in radians (current epoch)
	double dec;					//   "      "        dec "    "        "      "
	double rot;					// field rotation WRT parallactic angle (radians)
	double lst;					// the lst used to find rot (radians)
	
	unsigned long exposure;		// integration time (microseconds) - meaningless in pulse triggered mode
	double platescale;			// degrees/pixel on the CCD
	unsigned int gain;			// camera pre-amp gain 
	unsigned int offset;		// camera pre-amp offset
	unsigned int saturation;	// pixel saturation value
	double mapmean;				// the mean pixel value in the map
	double threshold;			// S/N threshold for blobs
	unsigned int grid;			// N pixels/side search grid
	unsigned int cenbox;		// "       "     centroiding box
	unsigned int apbox;			// "       "     photometry box
	unsigned int multiple_dist; // Distance threshold (pixels) to reject multiple sources
	int focusPosition;			// Stepper position for the focus motor
	int aperturePosition;		// Stepper position for the aperture motor

	unsigned int nblobs;		// # blobs in frame < MAXBLOBS
	double x_blobs[MAXBLOBS];	// x tangent plane offset of blob (approx. +ve az)
	double y_blobs[MAXBLOBS];   // y   "       "     "    "   "      "      "  el 
	int flux_blobs[MAXBLOBS];   // flux of blob 
} server_frame;

*/
