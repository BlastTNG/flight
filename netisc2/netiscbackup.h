/* netisc.h: Header file for ISC server. Change macros for #s of servers and to
            swap the aperture and focus motors.
*/

#ifndef __NETISC_H
#define __NETISC_H

//#define __NETISC_POINTING_MESSAGES

#include <iostream>
#include <time.h>                                
#include <windows.h>       // for threading
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <direct.h>
#include <conio.h>
#include <winbase.h>

#include "qcamapi.h"       // QCam interface
#include "isc_protocol.h"  // data structures for ISC tcp/ip communication
#include "FileTiff.h"      // TIFF file reading/writing
#include "bloblist.h"      // list of blobs
#include "frameblob.h"     // image frame to calculate blobs
#include "motorcmd.h"      // command motors over serial port
#include "csocket.h"       // wrapper for TCP/IP sockets
#include "astro.h"         // astrometry
#include "WinIo.h"         // Allow direct I/O for parallel port
//#include "readTemp.h"      // temp/pressure sensor + heater
#include "thumbnail.h"     // Save thumbnail images

// --- Macros ----------------------------------------------------------------

#define LOUD 1         // show millions of time stamps on the server

//#define AUTONOMOUS     // Set if not using client

#define FOCUS_MOTOR 2  // macro for the focus drive motor #
#define AP_MOTOR 1     //   "   "    "  aperture  "  "    "
#define NCLIENTS 3     // # of client connections to maintain

#define PARALLEL_BASE 0x378   // base address parallel port 
#define CAM_CYCLE_DELAY 20000 // # ms camera on before starting the camera 

#define ISC_SERVER_RESTART 144000 // # s before server automatically re-starts

#ifndef PI
#define PI 3.14159265358979
#endif

#ifndef RAD2DEG
#define RAD2DEG 57.29578
#endif

#ifndef DEG2RAD
#define DEG2RAD 0.017453
#endif

#ifndef HR2RAD
#define HR2RAD 0.26180
#endif

#ifndef CT2LST
#define CT2LST 1.0027
#endif

// Pointing
#define POINT_MAX_ERR 15    // # arcsec error in pointing to be considered bad
#define POINT_LOST_BLOBS 5  // # blobs required for lost mode
#define POINT_LOST_NBAD 5   // # bad solutions required for lost mode
#define POINT_EXCUR_NBAD 5  //   "        "       "     good despite excursion

// Controls for the display window
#define EYE_WIDTH 600       // Display window width in Pixels (evil eye)
#define EYE_HEIGHT 500      // "       height "     "  "
#define EYE_LEFT 20         // Coordinates of upper lefthand corner
#define EYE_TOP -20         //    "
#define EYE_ROI 200         // Length side region of interest pixels (even #!)
#define TICK_ARCMIN 2       // ROI tick marks every TICK_ARCMIN arcminutes
#define TICK_PIXLEN 4       // length of tick marks in pixels
#define BDA_THICK 3         // Thickness of the BDA outline
#define MARKER_THICK 1      // Thickness of box lines marking a blob location
#define MARKER_SIDE 15      // size (on a side) of box marking blob location
#define FONT_HEIGHT 30      // height and width of the text font
#define FONT_WIDTH 10       // pixels.
#define DISP_IMAGE_YFLIP -1 // 1 or -1: flip images in window in Y direction

// ONLY FOR GROUND TESTING! MUST BE SET TO 0 FOR FLIGHT!
#define BACKGROUND_KLUDGE 0	  // **LORENZO** if set to 1, loads and plugs into the frames
						                  // additional white noise read from "background.dat"

// autofocus algorithm
#define AUTOFOCUS_FINE_DELTA 5  // # steps between samples (+/-) (fine search)
#define AUTOFOCUS_FINE_NSAMPLES 100 // # steps in autofocus (fine search)
#define AUTOFOCUS_BOXCAR 5  // width of the moving boxcar smooth in samples

// Disk space
#define MIN_LOG_DISKSPACE 50.    // remaining megabytes to write a log entry
#define MIN_IMAGE_DISKSPACE 100. // "           "    "     "     an image

// Timeouts
#define CLIENT_TIMEOUT 10   // seconds to await packet before auto-trigger mode
#define TRIGGER_TIMEOUT 15  // seconds to await ext. trig. b4 temp. auto-trig 
#define TRIGGER_RETRY 60    // when auto-trig, every TRIGGER_RETRY sec attempt
                            //   external trigger

// --- Global variables ------------------------------------------------------

int NO_CAMERA=0;  // Test server without camera (load test image: 1=ISC, 2=OSC)
char FUDGEFILE[] = "test.tif";  // load this image if NO_CAMERA is set
int NO_CALC_POINTING=0;         // kludge - stop pointing soln. calculation

// filename
char settingsfilename[] = "settings.cam";       // general server settings file
char iscsettingsfilename[] = "settings_ISC.cam";// camera specific settings ISC
char oscsettingsfilename[] = "settings_OSC.cam";//          "               OSC
char iscbadpixfilename[] = "badpixels_ISC.cam"; // bad pixel coord. for ISC
char oscbadpixfilename[] = "badpixels_OSC.cam"; //            "         OSC

char stepperlogname[] = "stepperlog.cam";  // log the stepper motor positions
char framenumlogname[] = "fnumlog.cam";    // log the current frame number
char serverlogname[] = "serverlog.cam";    // main server log file
char imagePrefix[] = "images\\";           // name prefix for saved images 
char backgroundfilename[] = "background.dat";  // for BACKGROUND_KLUDGE

// client sockets / connection threads
int SERVER_PORTS[NCLIENTS];           // ports for accepting connections
CSocket *listen_sockets[NCLIENTS];    // pointers to listening sockets
CSocket *connected_sockets[NCLIENTS]; //   "    "     connected "
HANDLE thConnect[NCLIENTS];           // threads listening for connections
DWORD thConnectParam[NCLIENTS];       // connect thread i/o values
int thConnectState[NCLIENTS];         // connect thread states:
                                      // 0=disconnected, 1=connected, 
                                      // 2=awaiting connection

// receive threads (getting frames from clients)
HANDLE thRecv[NCLIENTS];        // threads that wait for client_frames
DWORD thRecvParam[NCLIENTS];    // receive thread parameters
int thRecvState[NCLIENTS];      // 0=not receiving, 1=receiving

// command execution thread
int newCmd=-1;    // -1 thCmd start / set to client # when client_frame rec'vd
HANDLE thCmd;     // handle for the command thread
DWORD thCmdParam; // parameter/return value for command thread
int thCmdState;   // 0=thread not running, 1=running, 
                  // 2=finished but not sent to clients

// exposure thread
HANDLE thExp;     // handle for the exposure thread
int thExpState;   // 0=not running, 1=running, 
                  // 2=finished but not ready for next grab, 3=ready for reset
int grabbingNow=0;// 0=not grabbing, 1=grabbing, 2=preparing to grab

// send threads (send response frames to clients)
HANDLE thSend[NCLIENTS];     // handles to response threads for all clients
DWORD thSendParam[NCLIENTS]; // parameter/return values for response threads
int thSendState[NCLIENTS];   // 0=not responding, 1=responding

// temperature sensor + heater thread
HANDLE thTemp;      // thread handle
DWORD thTempParam;  // thread parameter (not used)
int thTempState=0;  // 0=not running, 1=running

// Camera and frame information
QCam_CamListItem list[10];  // List of connected cameras  
unsigned long listlen = sizeof(list)/sizeof(list[0]);   
long CCD_X_PIXELS;  // dimenions of the CCD
long CCD_Y_PIXELS;  // "
int I_AM_ISC;       // 1=ISC, else OSC
QCam_Handle camhandle;    // Camera handle
QCam_Settings settings;   // Settings structure
int newSettings=1;        // new settings need to be sent to the camera
QCam_Frame QCFrame;       // Frame of camera data.
unsigned long FrameSize;  // Size of frame
int frameNum=0;
frameblob Frameblob;      // class for locating blobs
unsigned short ep_background[1392*1040];// for BACKGROUND_KLUDGE

// Display window variables
int eyeOn=1;              // by default the evil eye is updated
HWND hwndEye;             // Window handle for the evil eye
char *dispBuffer = NULL;  // Buffer to hold byte truncated display image
char roiBuffer[EYE_ROI*EYE_ROI]; // buffer to hold smaller ROI map
BITMAPINFO *BitMapInfo;   // structure storing bitmap information
HFONT hfntBold,hfntSmall; // fonts for the evil eye
char tiffFilename[512];   // filename of the current image
int eyeRefresh=1;         // semafore for evil eye refresh
int eyeMotor=0;           // semafore for motor command in eye refresh
ISCDisplayModeType lastMode=full; // previous display mode

// Time keeping
time_t server_start;  // at what time did this server start
long int frame_fname=0; // use time_t as file names for the frames
clock_t t1,t2,t3,t4,t5,t6,t7,t8;  // extra timers
SYSTEMTIME f1,f2,f3;  // fast timers
char timebuf[255];
time_t clock_newcmd=0;
time_t lastCmdRec;    // when last command was received
time_t lastTrigRec;   // when last external trigger was received

SYSTEMTIME exposureFinished; // when frame exposure finished

// The client frames. We don't queue up anything, so only NCLIENTS
// client_frames and 1 server_frame. execCmd is a copy of one of the
// client_data that is currently running, and does not get
// over-written while that command is running.

struct ISCStatusStruct client_data[NCLIENTS], execCmd;

// The server settings that may be changed by the client frames
int quitflag=0;      // when set, server quits
int shutdownFlag=0;  // 0=nothing, 1=shutdown, 2=reboot, 3=camera cycle
int saveFrameMode=0; // if set, save frames, if not, don't
int MCPFrameNum=-1;  // current MCP frame number
int pause=1;         // if set, not taking images
int abortFlag=0;     // if set, abort execution thread
int camcycleFlag=0;  // cycle the camera power when set
int autoFocusMode=0; // if set, in autofocus mode
int autoFocusStep=0; // make this global so that we can display it
int focusPosition=FOCUS_RANGE; // stepper position focus drive (starts at max)
int aperturePosition=AP_RANGE; // stepper position fstop drive (starts at max)
int current_focus; //**LORENZO**
ISCDisplayModeType eyeMode=full; // display mode
int xRoi, yRoi;      // centre ROI over pixel x, y
int eyeBlobRoi;      // blob # for ROI
double azBDA;        // az offset of CCD from BDA centre
double elBDA;        // el offset of CCD from BDA centre
double az=0;         // azimuth of the field (radians) 
double el=0;         // elevation   "    "      "
double lst=0.;       // local sidereal time     "
double lat=0.0;      // latitude of telescope   "
double lon=-0.0;     // E lon "
int focusOffset=0;   // offset to home position (for lab focusing)
double maxSlew=0.1*DEG2RAD;// max slew vel. of telescope for pointing rejection

int brightStarMode;  // 1 brightest star in field is at:
double brightRA;     // RA: radians (apparent)
double brightDEC;    // DEC: radians (apparent)

double sn_threshold; // blob finding
double grid;
double cenbox;
double apbox;
double mult_dist;

unsigned long ccd_exposure; // CCD exposure time (us)
unsigned long default_gain; // default CCD gain
unsigned long gain_res;     // resolution in the CCD gain control
signed long default_offset; // default offset for the CCD
double rel_gain;            // relative gain to the default (>0)
int rel_offset;             // relative offset to the default (+/-)
double noiseGain;           // noise model gain for default CCD settings
double noiseOffset;         //  "    offset      "
unsigned long coolerActive; // non-zero for camera's ccd cooler active
unsigned long highSensMode; // non-zero for camera's high sensitivity mode
unsigned long blackoutMode; // non-zero for camera's blackout mode

char catpath[255];          // UNIX-style path to the star catalogue
char catalogname[255];      // UNIX-style path to alternate star catalogues for
char katalogname[255];      //   pyramid matching algorithm.

double mag_limit=9.;        // magnitude limit of search catalogue
double norm_radius=2.*DEG2RAD; // radius of search catalogue in normal mode
double lost_radius=5.*DEG2RAD; //   "    "    "     "        when lost
double tolerance=20./3600.*DEG2RAD; // star-blob association angular tolerance
double rot_tol=1.*DEG2RAD;  // sky rot. tol. WRT CCD (offset from paral. angle)
double match_tol=0.5;       // match fraction tolerance for a frame
double quit_tol=1.0;        // match_tol sufficient to quit
int useLost=1;              // if set we can use pyramid solution
int maxBlobMatch=7;         // max blobs used in a match
int minBlobMatch=3;         // min blobs for match to be considered OK
double star_ra[MAX_ISC_BLOBS];  // values for the matched stars to each blob
double star_dec[MAX_ISC_BLOBS]; // "
double star_mag[MAX_ISC_BLOBS]; // "

// Pointing solution info sent back to client
struct ISCSolutionStruct server_data;  
double ra_0=0;     // current apparent RA
double dec_0=0;    // "       apparent DEC
double point_var;  // estimate of the variance in the pointing solution (rad^2)

// Other server settings
char comport[5];   // a string indicating serial port for motor communication
double platescale; // The current setting for the platescale (arcsec/pixel)
int motorSpeed;    // speed at which to operate the stepper motors
int newpointinglog=0; // new pointing solution to be logged
int triggertype=0;    // 0=software, 1=edge, 2=positive pulse, 3=negative pulse
struct _diskfree_t driveinfo; // for checking the remaining disk space
int hold_current=12;   // hold (heater) current for the motors
int camErr;           // current camera error message
time_t last_save=0;   // last time an image was saved
double ccdRotation=0; // CCD rot. WRT horizon (same sign  as parallactic angle)

int hereflag = 0;  // debugging flag

// Other pointing information
double ra_0_guess=0;   // guess centre of the search region in the catalogue
double dec_0_guess=0;  //  "             "              "                    "
double search_radius;  // current search radius in the catalogue
int pointing_quality=0;// 0=no solution, 1=good, -1=possibly bad, 2=LIS soln.
int pointing_nbad=0;   // number of bad solutions in a row
double q=0;            // field rotation (same sign parallactic angle)
double last_ra_0=0;    // last good solution 
double last_dec_0=0;   //   "
time_t last_time=0;    // time of last good solution
time_t frame_time=0;   // time of most recent frame was taken

SYSTEMTIME refSysTime; // system time last time the LST was set
double refLST;         // the last LST that was set by the client
int lost=0;            // set when lost

// Temperature readout
int tempControl = 1;
unsigned int tempSleeptime = 100;
float tempSetLimit = 3.0f;
float tempOffset = 0.10f;
float tempPressuregain;
float tempPressureoffset;
char tempstring[80];

// Variables for managing frame buffers
unsigned char *frameBuf1, *frameBuf2; // pointers to the frame buffers

// --- Function prototypes ----------------------------------------------------

int read_settings( void );
void server_log( int mode );
void time_stamp( char *buf, int bufsize );
int init_camera( void );
void framewrite( int mode );
DWORD WINAPI camera_grab( LPVOID parameter );
void expose_frame( void );
void pointingSolution( void );
void prep_image( void );
int step_motor( int motor, int steps );
void focus_home(void);
void absoluteMotor( int motor, int position );
void park_motors( void );
void calibrate_motors( void );
void doautofocus( void );
void disconnect_client(int client);
DWORD WINAPI connect_client( LPVOID parameter );
int update_connections( int mode );
DWORD WINAPI receive_frame( LPVOID parameter );
int update_receive( void );
DWORD WINAPI send_frame( LPVOID parameter );
int timeout_send( void );
DWORD WINAPI command_exec( LPVOID parameter );
int update_command( int abort );
DWORD WINAPI th_doTemp( LPVOID parameter );
void parse_coordinates( char *str );
LRESULT CALLBACK MainWndProc(HWND hwnd, UINT uMsg, WPARAM wParam, 
                             LPARAM lParam);
int initEyeWin();


DWORD WINAPI debugme( LPVOID parameter );
HANDLE thDebugme;

#endif
