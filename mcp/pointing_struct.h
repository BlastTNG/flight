/********************************************************************/
/*
                    Global data structures:
    Each of the structures below (except PointingDataStruct)
    is used by a thread to collect raw pointing data from some
    sensor or set of sensors.
    -The data are to be written to the downlinkframe (as needed)
     in tx.c::StoreData()
    -The data are to be read and collected into PointingData in pointing.c
     unless the raw data is actually what is wanted...
    -Where the data are written and read in different threads, a 3 way
     buffer is used.
    -r_index = GETREADINDEX(wr_index) can be used to get the readindex.
     it should be valid for atomic reads for at least 100ms.
    -index = INC_INDEX(index) can be used to increment the write index.
*/

// GETREADINDEX: converts circular buffer pointer to
#define GETREADINDEX(i) ((i+2) % 3)
#define INC_INDEX(i) ((i + 1) %3)

/**********************************************/
/*  VSCDataStruct                             */
/*  Purpose: Store VSC data                   */
/*   Source: VSC thread in starfind.c         */
/*     Used: Main thread                      */
struct VSCDataStruct {
  int sf_frame; // frame of found blob
  double col;
  double row;
  double mag;
  double az; // az based on ID'd star
  double el; // el based on ID'd star
  int id_frame; // frame of ID'd star
  time_t mcp_time; // time of ID'd star
};

/**********************************************/
/*  ACSDataStruct                             */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a curcular buffer...  */
struct ACSDataStruct {
  double mag_az;   // degrees
  double enc_elev; // degrees
  double gyro1;    // deg/s I think
  double gyro2;    // deg/s I think
  double gyro3;    // deg/s I think
  int mcp_frame;
  time_t t;
};

/**********************************************/
/*  SunSensorDataStruct                       */
/*  Purpose: Store raw sun sensor data        */
/*   Source: Sun Sensor thread                */
/*     Used: Main thread                      */
struct SunSensorDataStruct {
  short int raw_az; // uncalibrated
  short int raw_el; // uncalibrated
  short int prin;
};


/**********************************************/
/*  SIPDataStruct                             */
/*  Purpose: Store data from the SIP          */
/*   Source: Commands thread (commands.c)     */
/*     Used: Main thread                      */
struct GPSposStruct {
  double lat;   // probably degrees
  double lon;   // probably degrees
  double alt;
};

struct MKSaltStruct {
  float hi;
  float med;
  float lo;
};

struct TimeStruct {
  int UTC;
  int CPU;
};

struct MKScalStruct {
  float m_hi, m_med, m_lo;
  float b_hi, b_med, b_lo;
};

struct SIPDataStruct {
  struct GPSposStruct GPSpos;
  struct TimeStruct GPStime;
  struct MKSaltStruct MKSalt;
  char GPSstatus1;
  char GPSstatus2;
  struct MKScalStruct MKScal;
};

/**********************************************/
/*  PointingDataStruct                        */
/*  Purpose: Store derived pointing info      */
/*   Source: main thread; pointing.c          */
/*     Used: Main thread; VSC thread          */
struct PointingDataStruct {
  double az;        // degrees
  double el;        // degrees
  double gy1_offset;
  double gy_roll_amp;
  double lat;       // degrees
  double lon;       // degrees
  int mcp_frame;
  time_t t;
};

