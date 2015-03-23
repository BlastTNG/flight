/*  isc_protocol.h: define data structures for communcation between the
 *  server (ISC) and clients (Sam,Frodo,...)
 *
 * This software is copyright (C) 2003-2005 Edward Chapin
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
  Overview:

    The server program (netisc on the ISC computer) will connect
    several clients, each on a separate port.

    Any client may send ISCStatusStruct data frames. The server takes
    the appropriate action to achieve that status.  While the camera
    is busy changing the status, the server will continue accepting
    new frames but will ignore them until the previous changes have
    been made and it is ready to make further changes.

    The server sends ISCSolutionStruct data frames to all connected
    clients whenever a new image has been processed.

*/

#ifndef __ISC_PROTOCOL_H
#define __ISC_PROTOCOL_H

#ifdef _WINDOWS_  // If in windows change packing order of frames to have
#pragma pack(2)    // same packing as Linux
#endif

#define MAX_ISC_BLOBS 20      // static # blobs to store in data frames

#define ISC_CCD_X_PIXELS 1312 // pixel dimenions of the ISC CCD
#define ISC_CCD_Y_PIXELS 1024 //   "        "     "   "   "

#define OSC_CCD_X_PIXELS 1392 // pixel dimenions of the OSC CCD
#define OSC_CCD_Y_PIXELS 1040 //   "        "     "   "   "

#define FOCUS_RANGE 2550      // # steps range for focus stepper
#define AP_RANGE 495          // # steps range for aperture stepper


typedef enum {full, roi, blob} ISCDisplayModeType;

struct ISCStatusStruct {
  // General server state
  int abort;        // 1 abort current execution thread
  int pause;        // 1 paused, 0 continuous (not autofocus mode)
  int save;         // 1 save frames
  int autofocus;    // 1 auto focus on brightest blob in field
  int focus_pos;    // stepper position for focus
  int ap_pos;       // stepper position for aperture
  int MCPFrameNum;  // current frame number of MCP
  int shutdown;     // 0=null 1=shutdown 2=reboot pc 3=power cycle cam
  int hold_current; // the hold "heater" current (0-50)
  int exposure;     // exposure time in us REGARDLESS of trigger
  int triggertype;  // 0=software, 1=edge, 2=+ pulse, 3=- pulse
  int focusOffset;  // when camera focus ordered "home" step this far
  int eyeOn;        // if set update the window

  // Display mode parameters
  ISCDisplayModeType display_mode;
  int roi_x;        // x pixel number for ROI
  int roi_y;        // y pixel number for ROI
  int blob_num;     // blob # for ROI
  double azBDA;     // az tanplane offset BDA centre for the CCD (radians)
  double elBDA;     // el      "      "     "    "    "    "    "  "

  // telescope attitude
  double az;        // az in radians
  double el;        // el in radians
  double lst;       // lst in radians (!!)
  double lat;       // North Lat in radians
  double maxSlew;   // maximum telescope slew vel. in rad/sec

  // "brightest star is..." state
  int brightStarMode; // 1 brightest star in field is at:
  double brightRA;    // RA: radians (apparent)
  double brightDEC;   // DEC: radians (apparent)

  // blob algorithm stuff
  double sn_threshold;
  int grid;        // ask ed
  int mult_dist;   // ed knows

  // gain/offset
  double gain;    // relative gain to the CCD factory default
  int offset;     // offset to CCD factory default (in digitized units)

  // Matching algorithm
  int useLost;         // if set we can use the pyramid pointing solution
  int minBlobMatch;    // minimum # blobs used in the frame matching
  int maxBlobMatch;    // maximum # blobs used in the frame matching
  double mag_limit;    // size of tires
  double norm_radius;  // search radius in normal mode
  double lost_radius;  // search radius when lost
  double tolerance;    // star asociation tolerance
  double match_tol;    // % of blobs required for a match
  double quit_tol;     // match_tol sufficient to exit from algorithm w/o
                       // testing all combinations
  double rot_tol;      // rotation tolerance for blob finding
};

struct ISCSolutionStruct {
  int flag;        // 0=ready for pulse, 1=pointing solution
  int triggertype; // 0=software, 1=edge, 2=+ pulse, 3=- pulse
  int cameraerr;   // 0 if everything is Kosher, otherwise problem...
  int framenum;    // ISC frame # (also file names for images)
  int MCPFrameNum; // last MCP frame number received b/f image processing
  double mapMean;  // mean value of the map
  double ra;       // RA of CCD in radians (apparent)
  double dec;      // DEC   "        "        "
  double sigma;    // uncertainty of sol. in radians (or 2PI on failure)
  double rot;      // rot. of CCD field (rad) = par. angle + constant
  double az;       // az derived from ra, dec, lat, lon
  double el;       // el "

  double temp1;         // temperature sensors
  double temp2;
  double temp3;
  double temp4;
  double pressure1;     // pressure sensor
  int heaterOn;         // 1 if the heater is on

  double diskspace;     // MB space remaining on hard disk

  int autofocusOn;      // sends back to mcp the status of the autofocus
  int current_focus_pos;// keeps track of the current absolute position of the focus

  int n_blobs;
  double blob_x[MAX_ISC_BLOBS];
  double blob_y[MAX_ISC_BLOBS];
  int blob_flux[MAX_ISC_BLOBS];
  double blob_sn[MAX_ISC_BLOBS];
};

#ifdef _WINDOWS_
#pragma pack()    // go back to default packing
#endif

#endif
