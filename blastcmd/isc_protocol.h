/*  isc_protocol.h: define data structures for communcation between the server (ISC) and clients (Sam,Frodo,...)

	Overview:

	The server program (netisc on the ISC computer) will connect several clients, each on a separate port. Any client
	may send commands, but the server will select the most recent command for execution, and ignore all other commands
	until the previous one has completed.

	Connecting Clients:

	No special handshaking is required in order to connect to the server, but the client should send a nocmd
	frame so that the server responds with a frame containing the current settings. 

	Sending a command to the Server:

	Commands are sent at any time as a client_frame after a connection has been made.

	Receiving the response:

	The server will send out the response to the last completed command as a server_frame to all the connected
	clients. The server_frame indicates what command was executed, as well as the port on which it arrived
	(to determine which client had its question answered). Server will continue receiving commands, but ignore
	them while it is processing a command.
*/

#ifndef __ISC_PROTOCOL_H
#define __ISC_PROTOCOL_H

#ifdef _WINDOWS_	// If we're in windows need to change the packing order of the frames so that they have the
#pragma pack(2)		// same packing as in Linux
#endif

#define CLIENT_TIMEOUT 5000					// number of milliseconds for a client timeout  NOT CURRENTLY USED
#define SERVER_TIMEOUT 10000				//   "     "      "       "   " server "        NOT CURRENTLY USED

#define MAXBLOBS 50							// static # blobs to store in data frames
#define FOCUS_RANGE 2550					// # steps range for focus stepper
#define AP_RANGE 495						// # steps range for aperture stepper
#define CCD_X_PIXELS 1312					// pixel dimenions of the CCD
#define CCD_Y_PIXELS 1024					//   "        "     "   "   "

// enumerated type for the commands
typedef enum
{
	nocmd				= 0,	// null command, do nothing
	setTriggerMode		= 1,	// set trigger mode: 0=software, 1=edge, 2=pulse (par1)
	// If doing an expose or a freerun, setting par1=1 saves the images, par1=0 doesn't 
	expose				= 2,	// take a single exposure
	freerun				= 3,	// start continuous exposure loop until new command received	
	stepFocus			= 10,	// manually run the focus drive relative # of steps
	autoFocus			= 11,	// do an autofocus operation
	setFocus			= 12,	// set the focus to an absolute position (0-FOCUS_RANGE)
	stepAperture		= 20,   // manually run the aperture drive
	setAperture			= 21,	// set the aperture to an absolute position (0-AP_RANGE)
	updateSettings		= 30,   // set camera/search parameters
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
	
	//double ra;				// pointing ra in decimal degrees (J2000)
	//double dec;				//   "      dec "    "        "      "
	//double rot;				// field rotation (angle between -RA axis and +Az axis)
	
	double az;					// current pointing solution for the telescope (decimal degrees)
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
	
	double ra;					// pointing solution ra in decimal degrees (J2000)
	double dec;					//   "      "        dec "    "        "      "
	double rot;					// field rotation (parallactic angle)
	double lst;					// the lst used to find rot 
	
	unsigned long exposure;		// integration time (microseconds)
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
	unsigned int nblobs;		// # blobs in frame < MAXBLOBS
	int focusPosition;			// Stepper position for the focus motor
	int aperturePosition;		// Stepper position for the aperture motor

	double az_blobs[MAXBLOBS];	// blob information - static arrays, but only contain useful information
	double el_blobs[MAXBLOBS];  // up to nblobs-1. az and el are offsets in decimal pixel units from the 
	int flux_blobs[MAXBLOBS];   // bottom-left corner. Flux units are uncalibrated. 
} server_frame;

#ifdef _WINDOWS_	
#pragma pack()		// go back to default packing
#endif

#endif
