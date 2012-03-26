/* ------------------------------------------------------------------------
  netisc: 
  
  Newer versions: March 11, 2006 version control done with CVS

  Version 1.0
  - Barth's improved blob centroid + improved Frameblob interface
  - Solved MAJOR Frameblob bug (was ignoring large portion of image)
  - replaced map centre solver with iterative least squares technique
  - two frame buffer system to speed things up
  - record the state of the heater
  
  Version 0.9
  - Handshaking with clients implemented (if handshake times out, continue 
    taking images)
  - Implemented external trigger timeout to self-trigger
  - Auto-detects which CCD is being used and loads appropriate CCD-dependent 
    settings
  - consolidated temperature in main settings / log files
  - clients can change the triggering mode
  - NO_CAMERA is now put in the settings.cam file
  - add constant rotational offset
  
  Version 0.8
  - works with ISC #2 (the OSC)
  - integrated temperature / pressure sensors + heater control
  - updated focus algorithm
  - fixed some astrometry special cases
  - integration time in server log
  - normalized gain + offset commanding
  
  Version 0.7 - June 19, 2004
  -between 0.6 and 0.7 a number of changes were made:
    o read, log and display ISC temperature sensors
    o display more general information on the screen
    o altered the behaviour of the stepper motor commands
    o new autofocus routine (although the display during
      focus is currently broken)
    o the pointing solution logic has been made more autonomous
  - in addition, the communications protocol was greatly
    simplified to work with the INFOCUS experiment;
    in this version the server has now been converted BACK
    TO THE COMMUNICATIONS PROTOCOL FOR BLAST
  - the temperature sensor readout is currently EXTREMELY kludgey;
    a separate program reads the sensors and logs the result to a
    file, which is periodically read by NETISC (a complicated
    batch file is required to get everything running properly -
    see master.bat).
  
  Version 0.6
  -communication protocol completely changed
  
  Version 0.5
  -absolute pointing solution
  
  Version 0.4
  -changed isc_protocol (byte packing + client frame sends different data)
  -Stepper motor position logging
  -Added pulse-width triggering
  -Added freerun mode. Once started, continues until a new command is received
  -changed all the floating point math to double precision
  
  Version 0.3 - August 12, 2003
  -ROI added
  
  Version 0.2 - August 10, 2003
  
  -Display images / blob finding in a window (the 
  evil eye)
  
  -Motor commands are now in steps, not in normalized coordinates (changed
  netisc.h AND isc_protocol.h)
  
  -Take default motor positions in settings file
  
  Version 0.1 - June 13, 2003
  
  server that runs on ISC computer to accept commands from clients 
  on different ports
  
  Please keep Ed Chapin (echapin@inaoep.mx) posted whenever the
  code is updated!
  
--------------------------------------------------------------------------- */

#include "netisc.h"  // globals for camera communication


// --- Read Settings File ----------------------------------------------------
// This will send settings to the camera, and also set values inside the
// frameblob object. Returns 1 on success, 0 on failure (probably 
// settings file doesn't exist).

int read_settings() {
  int i;
  FILE *settingsfile;
  
  char thisline[255];
  
  unsigned long satval;       // pixel saturation value
  double threshold;           // S/N threshold for bright pixels
  unsigned int disttol;       // the source tolerance distance
  unsigned int grid;          // coarse grid cell size
  unsigned int cenbox;        // centroid box size
  unsigned int apbox;         // aperture photometry box size
  
  // --- Try opening the general settings file -------------------------------
  if( (settingsfile = fopen( settingsfilename, "r" )) == NULL )
    return 0;
        
  // Read in the settings
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&NO_CAMERA);
  for(i=0;i<NCLIENTS;i++) { 
    fgets(thisline,80,settingsfile); 
    sscanf(thisline,"%i",&SERVER_PORTS[i]); 
  }
  
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&NO_CALC_POINTING);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&lon);
  lon *= PI/180.;
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&lat);
  lat *= PI/180.;
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&triggertype);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&threshold);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&disttol);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&grid);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&cenbox);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&apbox);

  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&useLost);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&minBlobMatch);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%u",&maxBlobMatch);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&mag_limit);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&rot_tol);
  rot_tol *= DEG2RAD;

  fgets(thisline,80,settingsfile); sscanf(thisline,"%4s",comport);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&motorSpeed);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&pause);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&focusOffset);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%s",catpath);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%s",catalogname);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%s",katalogname);
  
  printf("Path to star catalogue: *** %s ***\n",catpath);
  printf("Pyramid catalogue:\n  %s\n  %s\n", catalogname, katalogname );

  // Close the file
  fclose(settingsfile);
  
  // --- Try opening the camera specific settings file -----------------------
  
  if( I_AM_ISC ) {
    if( (settingsfile = fopen( iscsettingsfilename, "r" )) == NULL )
      return 0;
  } else {
    if( (settingsfile = fopen( oscsettingsfilename, "r" )) == NULL )
      return 0;
  }

  // Read in the settings
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lu",&ccd_exposure); 
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&rel_gain);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&rel_offset);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&noiseGain);        
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&noiseOffset);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%li",&satval);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&platescale);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lf",&ccdRotation);
  
  ccdRotation = ccdRotation * DEG2RAD;
  
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&tempControl);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&tempSleeptime);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%f",&tempSetLimit); 
  fgets(thisline,80,settingsfile); sscanf(thisline,"%f",&tempOffset);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%f",&tempPressuregain);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%f",&tempPressureoffset);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lu",&coolerActive);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lu",&highSensMode);
  fgets(thisline,80,settingsfile); sscanf(thisline,"%lu",&blackoutMode);
  
  fclose(settingsfile);
  // -------------------------------------------------------------------------
  
  printf("nc:%i  t:%i  e:%i  rg:%f  ro:%i  ng:%lf no:%lf sv:%i th:%f  dt:%u \n g:%u cb:%u ab:%u c:%s ms:%i ps:%f p:%i lf:%i \n\n", 
         NO_CALC_POINTING, triggertype, ccd_exposure, 
         rel_gain, rel_offset, noiseGain, noiseOffset, satval, 
         threshold, disttol, 
         grid, cenbox, apbox, comport, motorSpeed, 
         platescale,pause, focusOffset);
  
  // Send settings to camera/frameblob
  QCam_SetParam( &settings, qprmExposure, (unsigned long)ccd_exposure );
  QCam_SetParam( &settings, qprmNormalizedGain, // use normalized gain/offset
                 (unsigned long)( ((double)default_gain)*rel_gain ) ); 
  QCam_SetParamS32( &settings, qprmS32AbsoluteOffset, 
                    (signed long)(default_offset+rel_offset) );
  //QCam_SetParam( &settings, qprmGain,(unsigned long)gain );
  //QCam_SetParam( &settings, qprmOffset, (unsigned long)offset );
  QCam_SetParam( &settings, qprmCoolerActive, coolerActive );
  QCam_SetParam( &settings, qprmHighSensitivityMode, highSensMode );
  QCam_SetParam( &settings, qprmBlackoutMode, blackoutMode );
    
  switch(triggertype) {
  case 0: 
    QCam_SetParam( &settings, qprmTriggerType, qcTriggerFreerun ); 
    break;
    
  case 1: 
    QCam_SetParam( &settings, qprmTriggerType, qcTriggerEdgeHi ); 
    break;
    
  case 2: QCam_SetParam( &settings, qprmTriggerType, qcTriggerPulseHi ); 
    break;
    
  case 3: QCam_SetParam( &settings, qprmTriggerType, qcTriggerPulseLow ); 
    break;
  }
  
  newSettings = 1;
  //QCam_SendSettingsToCam( camhandle, &settings );
  
  Frameblob.set_satval((MAPTYPE)satval);
  Frameblob.set_threshold(threshold);
  Frameblob.set_disttol(disttol);
  Frameblob.set_grid(grid);
  
  printf("cenbox: %u apbox: %u comport: %s\n",cenbox,apbox,comport);

  return 1;
}

// Write a log entry to the server log file
// mode=2 for autofocus entry, mode=1 for startup entry, 
// mode=0 for regular entry, 3=camera error entry
// 4=camera cycle entry 5=ISC shutdown 6=ISC reboot

void server_log( int mode ) {
  FILE *logfile;
  int i;
  time_t now;
  time( &now );
  char *timestr;
  double diskspace;
  
  // String containing current time/date
  timestr = ctime(&now);
  timestr[strlen(timestr)-1] = 0;
  
  // check amount of space remaining on disk
  unsigned error = _getdiskfree(_getdrive(),&driveinfo);
  if( error == 0 ) {
    diskspace = (double) driveinfo.avail_clusters * 
      driveinfo.bytes_per_sector*driveinfo.sectors_per_cluster;
    diskspace = diskspace / 1024. / 1024.;  // in megabytes
  } else diskspace = 0;
  
  // Update diskspace in server_data
  server_data.diskspace = diskspace;

  if( diskspace < MIN_LOG_DISKSPACE ) {
    printf("Error: disk full (%lfMb remain)\n",diskspace);
  }
  // try to open the log file
  else if( (logfile = fopen(serverlogname,"a")) == NULL ) {
    printf("Error opening server log file: %s\n",serverlogname);
  } else {
    
    // ISC shutdown
    if( mode == 5 ) {
      fprintf(logfile,"%s: ISC shutdown\n",timestr);
    }
    
    // ISC reboot
    if( mode == 6 ) {
      fprintf(logfile,"%s: ISC reboot\n",timestr);
    }

    // camera power cycle
    if( mode == 4 ) {
      fprintf(logfile,"%s: camera power cycle\n",timestr);
    }

    // camera error
    if( mode == 3 ) {
      fprintf(logfile,"%s: camera error %i\n",timestr,camErr);
    }

    // autofocus message
    if( mode == 2 ) {
      fprintf(logfile,"%s: autofoc pos=%i frame=%i %ld ",timestr, 
              current_focus, frameNum, frame_fname);
      
      if( server_data.n_blobs > 0 )
        fprintf(logfile,"x=%8.3lf y=%8.3lf flux=%i\n",server_data.blob_x[0],
                server_data.blob_y[0],server_data.blob_flux[0]);
      else fprintf(logfile,"x=%8.3lf y=%8.3lf flux=%i\n",0.,0.,0);
    }
    
    // server up message
    if( mode == 1 ) {
      timestr = ctime(&server_start);
      timestr[strlen(timestr)-1] = 0;
      fprintf(logfile,
              "*** ISC Server up on %s *********************************\n",
              timestr);
    }
    // regular server message
    if( mode == 0 ) {
      // Server state messages
      fprintf(logfile,"%s: p=%i s=%i foc=%i ap=%i abt=%i MCPFr=%i afoc=%i\n",
              timestr, pause, saveFrameMode, current_focus, aperturePosition, 
              abortFlag, MCPFrameNum, mode);
      fprintf(logfile,"    az=%6.4lf el=%6.4lf lat=%6.4lf lst=%5.4lf afocpos=%i exp=%lu\n", 
              az*180/PI, el*180/PI, lat*180/PI, lst*180/PI/15, 
              (int)focusOffset,ccd_exposure);
      
      
      fprintf(logfile,"    thresh=%6.2lf grid=%i mdist=%i mean=%lf\n",
              Frameblob.get_threshold(), Frameblob.get_grid(), 
              (int) Frameblob.get_disttol(), 
              Frameblob.get_mapmean() ); //added (float) jk
      
      fprintf(logfile,"    cenbox=%i apbox=%i minblob=%i maxblob=%i\n",
              cenbox, apbox, minBlobMatch, maxBlobMatch );

      fprintf(logfile,"    mag=%4.1lf rad=%6.2lf tol=%6.2lf mtol=%3.1lf qtol=%3.1lf rtol=%6.2lf\n",
              mag_limit, search_radius*180/PI, tolerance*3600*180/PI, 
              match_tol, quit_tol, rot_tol*180/PI);


      fprintf(logfile,"    T/P: %s\n",tempstring);
      
      // New pointing information if available
      if( newpointinglog ) {
        fprintf(logfile,
                "    frame=%i %ld ra=%lf dec=%lf +/- %lf rot=%8.3lf qual=%i\n",
                frameNum, frame_fname, ra_0*180/PI/15, dec_0*180/PI, 
                sqrt(point_var)*3600*180/PI, server_data.rot*180/PI,  
                pointing_quality);
        
        for(i=0;i<(int)server_data.n_blobs;i++)
          fprintf(logfile,"    %8.3lf %8.3lf %i %lf\n",
                  server_data.blob_x[i],server_data.blob_y[i], 
                  server_data.blob_flux[i],server_data.blob_sn[i]);
        newpointinglog = 0;
      }
    }
        
    if( fclose(logfile) == EOF ) 
      printf("Couldn't close server log file: %s\n",serverlogname);
  }
}

// Get a current time stamp and create a time string
void time_stamp( char *buf, int bufsize ) {
  SYSTEMTIME t;
  GetSystemTime(&t);
  char tempbuf[255];
  
  //printf("%7.3lf Start point\n",(double)f1.wSecond + 
  // ((double)f1.wMilliseconds)/1000. );

  GetTimeFormat( LOCALE_SYSTEM_DEFAULT, TIME_NOSECONDS, &t, "HH':'mm':'", 
                 tempbuf, bufsize );
  sprintf(buf,"--- %s%05.2lf",tempbuf,(double)t.wSecond + 
          ((double)t.wMilliseconds)/1000.);
}

// --- Camera commands -------------------------------------------------------

// initialize the camera. -1 on failure, 1 on success
int init_camera( void ) {
  unsigned long tempCCD_X_PIXELS, tempCCD_Y_PIXELS;
  
     // Power cycle the camera
  printf("Power cycling the camera...\n");
  printf("Lowering pins on the parallel port... status=");
  printf("%i\n",SetPortVal(PARALLEL_BASE,0,1));
  Sleep(500);
  printf("Raising Data 1 on the parallel port... status=");
  printf("%i\n",SetPortVal(PARALLEL_BASE,2,1));
  Sleep(500);
  printf("Raising Data 5 on the parallel port... status=");//temp stuff
  printf("%i\n",SetPortVal(PARALLEL_BASE,32,1));
  Sleep(500);
  printf("Lowering pins on the parallel port... status=");
  printf("%i\n",SetPortVal(PARALLEL_BASE,0,1));
  Sleep(3000);

  // Initialize the camera driver
  printf("Inside init_camera driver...\n");
  if( QCam_LoadDriver() != qerrSuccess) { // Fire up the QCam driver
    printf("Unable to load QCam Driver\n");
    return -1;
  }
  
  // Check the list of cameras found after the power cycle
  if( QCam_ListCameras( list, &listlen ) != qerrSuccess) {
    QCam_ReleaseDriver();
    printf("Couldn't open a camera... exiting.\n\n");
    return -1;
  }
  
  // Kludge to deal with the on/off switch of the camera.
  // If the camera was on in the first place, the power cycle above will shut it down.
  // So we just power cycle it again, also to make sure that any previos settings are reset.
  if (listlen == 0) {
    QCam_ReleaseDriver();
    printf("Power cycling the camera, again...\n");
    printf("Lowering pins on the parallel port... status=");
    printf("%i\n",SetPortVal(PARALLEL_BASE,0,1));
    Sleep(500);
    printf("Raising Data 1 on the parallel port... status=");
    printf("%i\n",SetPortVal(PARALLEL_BASE,2,1));
    Sleep(500);
    printf("Raising Data 5 on the parallel port... status=");//temp stuff
    printf("%i\n",SetPortVal(PARALLEL_BASE,32,1));
    Sleep(500);
    printf("Lowering pins on the parallel port... status=");
    printf("%i\n",SetPortVal(PARALLEL_BASE,0,1));
    Sleep(3000);
    listlen = sizeof(list)/sizeof(list[0]); // Re-inizializing listlen is needed
                                            // otherwise QCam_ListCameras below will take its current
                                            // value (0) as input and won't find any cameras at all.

    if( QCam_LoadDriver() != qerrSuccess) { // Fire up the QCam driver, again. This is needed after 
    printf("Unable to load QCam Driver\n"); // the second power cycle, otherwise QCam will give 
    return -1;                              // camErr = 16, i.e. Driver Fault.
    }
    if( QCam_ListCameras( list, &listlen ) != qerrSuccess) {
      QCam_ReleaseDriver();
      printf("Couldn't open a camera... exiting.\n\n");
    return -1;
    }
  }
 
  if( (listlen > 0) && (list[0].isOpen == false) ) {
    QCam_OpenCamera( list[0].cameraId, &camhandle );
    printf("Camera opened...\n");
    
    settings.size = sizeof(settings); 
    QCam_ReadDefaultSettings( camhandle, &settings );        
    QCam_SetParam( &settings, qprmImageFormat, qfmtMono16); 
    QCam_SendSettingsToCam( camhandle, &settings );

    // Query the camera for the CCD dimenions to figure out if we're ISC or OSC
    QCam_GetInfo( camhandle, qinfCcdWidth,  &tempCCD_X_PIXELS );
    QCam_GetInfo( camhandle, qinfCcdHeight, &tempCCD_Y_PIXELS );
                
    CCD_X_PIXELS = (long) tempCCD_X_PIXELS;
    CCD_Y_PIXELS = (long) tempCCD_Y_PIXELS;

    printf("CCD dimensions: %i %i\n",CCD_X_PIXELS,CCD_Y_PIXELS);

    return 1;
  } else { // If no camera, exit
    QCam_ReleaseDriver();
    
    printf("Couldn't open a camera... exiting.\n\n");
    return -1;
  }
}

// Write frame to a file
// if 0, don't write. 1 write to file
void framewrite( int mode ) {        
  FILE *flogfile;
  double diskspace;
  
  // int arrays to hold blob position;
  int xblob[MAX_ISC_BLOBS];
  int yblob[MAX_ISC_BLOBS];
  int i;
  
  // check remaining disk space
  unsigned error = _getdiskfree(_getdrive(),&driveinfo);
  if( error == 0 ) {
    diskspace = (double) driveinfo.avail_clusters *
      driveinfo.bytes_per_sector *
      driveinfo.sectors_per_cluster;
    diskspace = diskspace / 1024. / 1024.;  // in megabytes
  } else diskspace = 0;

  // Not enough diskspace
  if( diskspace < MIN_IMAGE_DISKSPACE ) {
    printf("Error: disk full (%lfMb remain)\n",diskspace);
    //sprintf(tiffFilename,"NOT SAVED: %6i",frameNum);
    sprintf(tiffFilename,"NOT SAVED: %ld",frame_fname);
  }

  // Save entire image
  else if( mode == 1 ) {
    //sprintf(tiffFilename,"%s%06i.tif",imagePrefix,frameNum);
    sprintf(tiffFilename,"%s%ld.tif",imagePrefix,frame_fname);
                
    CFileTiffWrite tiff_file;
    tiff_file.Open(tiffFilename);
    tiff_file.Write(&QCFrame);
    //printf("Saved frame to %s...\n",tiffFilename);
    
    time( &last_save );
  }
  
  // Save thumbnails
  else {
    if( server_data.n_blobs > 0 ) {

      for( i=0; i<server_data.n_blobs; i++ ) {
        xblob[i] = (int) server_data.blob_x[i];
        yblob[i] = CCD_Y_PIXELS - ((int) server_data.blob_y[i]) - 1;
      }

      sprintf(tiffFilename,"%s%ld.thu",imagePrefix,frame_fname);
                
      thumbnail( (unsigned short *) Frameblob.get_map(), 
                 (int) CCD_X_PIXELS, (int) CCD_Y_PIXELS, 32,
                 server_data.n_blobs, xblob, yblob, tiffFilename );
      
      //sprintf(tiffFilename,"Saved thumbnails: %i",frameNum);
    }
    //else sprintf(tiffFilename,"IMAGE NOT SAVED: %i",frameNum);
  }

  // Log the last completed frame number
  if( (flogfile=fopen(framenumlogname,"w")) == NULL ) {
    printf("Couldn't open %s to log frame number.\n",framenumlogname);
  } else {
    fprintf(flogfile,"%i",frameNum);
    fclose(flogfile);
  }

  // switch the frame buffer used for the next grab
  if( QCFrame.pBuffer == frameBuf1 ) QCFrame.pBuffer = frameBuf2;
  else QCFrame.pBuffer = frameBuf1;

}

// Thread function for grabbing frame with camera - returns upon completion
// Input: Nothing
// Output: Camera error (0=happy, nonzero=ass)
DWORD WINAPI camera_grab( LPVOID parameter ) {
  DWORD temp = *((DWORD *)parameter); // input not used

  //printf(".........Starting a grab to framebuf %i, trig %i\n",
  // (QCFrame.pBuffer == frameBuf2)+1,triggertype);
        
  if( LOUD ) { 
    time_stamp( &timebuf[0], 255 ); 
    printf("%s Start grab to framebuf %i, trig %i\n",
           timebuf,(QCFrame.pBuffer == frameBuf2)+1,triggertype);
  }

  // starting the grab
  grabbingNow = 1; 
  
  // output is the camera error code
  *((DWORD *)parameter) = QCam_GrabFrame(camhandle, &QCFrame);  
  
  // finished
  grabbingNow = 0; 
  
  // Record the instant at which the camera returned the image
  GetSystemTime(&exposureFinished);

  if(BACKGROUND_KLUDGE) {
					unsigned short *ppp = (unsigned short *)QCFrame.pBuffer;
					for(int idx=0; idx < CCD_X_PIXELS*CCD_Y_PIXELS; idx++)
						ppp[idx] += ep_background[idx];
				}
  
#ifdef AUTONOMOUS
  // Update the LST
  lst = get_gst( exposureFinished.wYear,
                 exposureFinished.wMonth, 
                 exposureFinished.wDay,
                 (exposureFinished.wHour*3600 +
                  exposureFinished.wMinute*60 +
                  exposureFinished.wSecond +
                  exposureFinished.wMilliseconds/1000.)/3600./24. ) + lon;
#endif

  // update the time last hardware trigger received
  if( triggertype != 0) time( &lastTrigRec );

  return 1; // return value not used
}


// take a single picture in the current trigger mode
//void expose_frame( void )
DWORD WINAPI expose_frame( LPVOID parameter ) {
  int i, bits;
  //SYSTEMTIME now_fine;

  grabbingNow=2; // flag the thread as preparing to grab

  // increment the frame counter
  frameNum++;
  server_data.framenum = frameNum;

  // If running without a camera, load a test frame
  if( NO_CAMERA > 0 ) {
   
    grabbingNow=1;
    
    CFileTiffRead tiff_file;
    tiff_file.Open(FUDGEFILE);
    if( !tiff_file.Read(&QCFrame) ) printf("TIFF read failed.\n");
    else {
      printf("Read TIFF file.\n");
      
      // make the images LSB aligned
      if( I_AM_ISC ) bits = 2;
      else bits = 2;//**LORENZO** used to be 4, but now OSC is also 14 bits
                        
      for( i=0; i<CCD_X_PIXELS*CCD_Y_PIXELS; i++ ) {
        ((MAPTYPE *) QCFrame.pBuffer)[i] = 
          ((MAPTYPE *) QCFrame.pBuffer)[i] >> bits;
      }
    }
    
    // Record instant when image exposed
    GetSystemTime(&exposureFinished);

#ifdef AUTONOMOUS
    // Update the LST
    lst = get_gst( exposureFinished.wYear,
                   exposureFinished.wMonth, 
                   exposureFinished.wDay,
                   (exposureFinished.wHour*3600 +
                    exposureFinished.wMinute*60 +
                    exposureFinished.wSecond +
                    exposureFinished.wMilliseconds/1000.)/3600./24. ) + lon;
#endif

    grabbingNow=0;

  }
  
  // Otherwise grab a frame from the camera
  else {
    DWORD th_param,thId;
    HANDLE grabHandle;
    grabHandle = CreateThread( NULL,0,camera_grab,&th_param,0,&thId);
    
    // Send the handshake as soon as the grab thread is started
    /*
      if( server_data.flag == 0 ) {
      t7 = clock();
      printf("Elapsed time: %lfs\n",(double)(t7-t6)/CLOCKS_PER_SEC);
      t6 = t7;
      
      printf(" --------> Send handshake\n");
      timeout_send();
      printf(" --------| Sent!\n");
      server_data.flag = 1; // reset the handshake flag
      }
    */

    // Wait maximum of 1 second + integration time for camera to grab a frame
    if( WaitForSingleObject(grabHandle, 
                            TRIGGER_TIMEOUT*1000.+execCmd.exposure/1000) == 
        WAIT_TIMEOUT ) {
      
      printf("Camera exposure timed out... sending ABORT.\n");
      // tell the camera to abort the current frame if timed out
      QCam_Abort(camhandle);  

      // Give camera driver 1 more second to finish exec thread before killing
      if( WaitForSingleObject(grabHandle,1000) == WAIT_TIMEOUT ) {
        printf("ABORT timed out, killing thread...\n");
        TerminateThread(grabHandle,0);
      } else {
        printf("ABORT successful.\n");
        CloseHandle(grabHandle);
      }
      camErr = 100;    // timeout error code
    } else {
      camErr = th_param;
      CloseHandle(grabHandle);
    }
    
    if( (QCam_Err) camErr != qerrSuccess ) {
      printf("$$$$$$$$$$$$$$$$$$$$$$$$\n");
      printf("$$$ Camera error: %i\n", camErr);
      printf("$$$$$$$$$$$$$$$$$$$$$$$$\n");

      server_log( 3 );  // log the camera error
                        // Restart necessary if camera error was fatal
       
      if( (camErr!=23) && (camErr!=25) && (camErr!=100) ) {
        printf("********* Camera error was fatal - re-starting camera !!!!\n");
        // Only tell the camera to reset if the shutdownFlag wasn't already set
        if( shutdownFlag == 0 ) shutdownFlag = 3;
      }
    }
    server_data.cameraerr = camErr;
  }

  // If there were no camera errors, continue
  if( (QCam_Err) camErr == qerrSuccess ) {
    time( &frame_time ); // record the time
    
    // write the frame _AFTER_ analysis
    //framewrite(saveFrameMode);
    
    server_data.framenum = frameNum;
  }
  
  // If there was a camera error
  else {
    // Set the number of blobs in the frame to 0
    server_data.n_blobs = 0;
  }

  // update server_data to reflect the trigger type that is actually being used
  server_data.triggertype = triggertype;
  
  // switch the frame buffer used for the next grab
  //if( QCFrame.pBuffer == frameBuf1 ) QCFrame.pBuffer = frameBuf2;
  //else QCFrame.pBuffer = frameBuf1;
  
  // the filename for the image is also the exposureFinished time:
  
  //frame_fname -= 1000000000; // chop the most-significant part because 
  // only needed for decade
  //frame_fname = frame_fname*10;
  
  frame_fname = ((exposureFinished.wMonth-1L)*31L*24L*3600L + 
                 (exposureFinished.wDay-1L)*24L*3600L + 
                 exposureFinished.wHour*3600L +
                 exposureFinished.wMinute*60L + 
                 exposureFinished.wSecond)*10L + 
   exposureFinished.wMilliseconds/100L;
  
  return 1;
}

// Handles the exposure thread - re-starting it whenever needed

int reExpose( void ) {
  DWORD result;
  
  //printf("expose_frame state: %i...\n",thExpState);
  
  // If it's ready to start exposing again
  if (thExpState == 0 ) {

    // update the settings in the camera if necessary
    if( newSettings == 1 ) {
      //printf("^^^^Sending the updated settings to the camera\n");
      QCam_SendSettingsToCam( camhandle, &settings );
      newSettings = 0;
    }
    
    //printf("^^^^Start expose, trig %i, time %i\n",triggertype,ccd_exposure);
    
    // start taking a picture
    thExp=CreateThread( NULL,0,expose_frame,NULL,0,NULL);
    thExpState = 1; // signal that the thread has been started
    //printf("thExpState = 1\n");

    if(thExp==NULL) {
      thExpState = 2;
      //printf("thExpState = 2\n");
      
      return -1;
    }
  }

  // Check to see if a running thread has finished
  if( thExpState == 1 ) {
    result = WaitForSingleObject(thExp,0);

    if( result != WAIT_TIMEOUT ) {    // thread is finished
      CloseHandle(thExp);
      thExpState = 2; // signal that it has finished and needs to be processed
      //printf("thExpState = 2\n");    
    }
  }
  
  return 1;
}

// Calculte a pointing solution from the current frame
void pointingSolution( void ) {

  if( LOUD ) {
    time_stamp( &timebuf[0], 255 ); 
    printf("%s Start finding blobs in framebuf %i\n",timebuf,
           (Frameblob.get_map() == (MAPTYPE *)frameBuf2)+1);
  }

  // Find blobs
  Frameblob.fix_badpix((MAPTYPE)Frameblob.get_satval()+1); // set the noisies
                                                           // to sat. value
  Frameblob.calc_mapstat();      // calculate mean for all pixels < satval
  server_data.mapMean = Frameblob.get_mapmean();
  Frameblob.fix_badpix((MAPTYPE)server_data.mapMean);      // set the noisies
                                           // to map mean
  Frameblob.calc_searchgrid();             // find sources
  Frameblob.fix_multiple();                // get rid of multiple detections
  Frameblob.sortblobs();                   // sort in descending order of flux
  
  // retrieve blob information from list and store in server status arrays
  bloblist *blobs = Frameblob.getblobs();
  int numextended = 0;   // count extended objects in the frame
  int numpoint = 0;      // count point-like objects in the frame

  int nmatch;
  int nMatchBlobs;

  while( blobs != NULL ) {        
    if( numextended < MAX_ISC_BLOBS ) {
      server_data.blob_x[numextended] = blobs->getx();
      // want y increasing with elevation
      server_data.blob_y[numextended] = CCD_Y_PIXELS - blobs->gety() - 1; 
      server_data.blob_flux[numextended] = blobs->getflux();
      server_data.blob_sn[numextended] = blobs->getsnr();
    }
    numextended++;  
    blobs = blobs->getnextblob();        
  }
                
  if( numextended > MAX_ISC_BLOBS ) server_data.n_blobs = MAX_ISC_BLOBS;
  else server_data.n_blobs = numextended;

  if( LOUD ) {
    time_stamp( &timebuf[0], 255 ); 
    printf("%s Found blobs, start solution calc\n",timebuf);
  }

  // Calculate the new ra/dec guesses from MCP
  calc_ra_dec(az,el,lat,lst,&ra_0_guess,&dec_0_guess);

  lost = 0; // start assuming we're not lost

  // If we've had many bad solutions, and we have enough blobs,
  // try lost in space mode.
  if( (pointing_nbad >= POINT_LOST_NBAD) && (server_data.n_blobs>=4) ) { 
    
    // Before allowing the pyramid solution check useLost
    if( useLost ) {
      lost = 1;
      if( LOUD ) {
        printf("*** NBLOB=%i NBAD=%i: LOST IN SPACE algo\n", 
               server_data.n_blobs, pointing_nbad);
      }
    } else {
      if( LOUD ) {
        printf("*** NBLOB=%i NBAD=%i: CLIENT SOLUTION guess for old algo, lost radius\n", 
               server_data.n_blobs, pointing_nbad);
      }
    }

    search_radius = lost_radius;
  } 
  
  // Otherwise try to use a guess solution from the client
  else {
    if( LOUD ) {
      printf("*** NBLOB=%i NBAD=%i: CLIENT SOLUTION guess for old algo. useLost=%i\n",
             server_data.n_blobs, pointing_nbad, useLost);
    }
    
    search_radius = norm_radius;
  }

  double epoch;
  double rot = ccdRotation;      
  int i;        
  
  double last_a, last_b, last_c, a, b, c;
  double theta, max_theta;
  double thismaglim;

  // Set pointing quality to bad just before we try to do new solution
  pointing_quality = 0;

  if( server_data.n_blobs >= minBlobMatch ) {
    double *x = new double[server_data.n_blobs];
    double *y = new double[server_data.n_blobs];
    double *f = new double[server_data.n_blobs];
    
    // Find the pixel offsets from the map centre increasing to the
    // right and up
    for(i=0;i<(int)server_data.n_blobs;i++) {
      x[i] = (server_data.blob_x[i]-CCD_X_PIXELS/2); 
      y[i] = (server_data.blob_y[i]-CCD_Y_PIXELS/2); 
      f[i] = (double) server_data.blob_flux[i];
      
      //printf("%lf %lf %lf\n",x[i],y[i],f[i]);
    }
    
    if( NO_CALC_POINTING ) nmatch = 0;
    else {        
      if( server_data.n_blobs > maxBlobMatch ) nMatchBlobs = maxBlobMatch;
      else nMatchBlobs = server_data.n_blobs;
      
      epoch = calc_epoch();
      
      //printf("ra_g %lf dec_g %lf lst %lf\n",ra_0_guess,dec_0_guess,lst);
      //printf("search %lf mag %lf tol %lf\n",search_radius,mag_limit,
      //tolerance);
      
      // Tune magnitude limit based on the requested depth and number of blobs
      thismaglim = mag_limit;        
      if( (mag_limit > 8.5) && (nMatchBlobs >= 3) && (nMatchBlobs <= 4) ) 
        thismaglim = 8.5;
      
      if( (nMatchBlobs <= 2) ) thismaglim = 7.5;
    
      nmatch = calc_pointing( ra_0_guess, dec_0_guess, lost,
                              epoch, lat, lst, 
                              x, y, f, nMatchBlobs, 0, 
                              search_radius, thismaglim, tolerance, 
                              540./206265., 0.03*PI/180., 0.5, 1., rot_tol,
                              &ra_0, &dec_0, &point_var, &rot, &platescale,  
                              star_ra, star_dec, star_mag, &abortFlag, 
                              brightStarMode, brightRA, brightDEC );      
    }
    
    // If enough blobs were matched to be considered successful...
    if( (nmatch >= minBlobMatch) && (nmatch > 0) ) {
      
      // If the chi^2 looked bad then flag solution as bad
      if( sqrt(point_var)*3600*180/PI > POINT_MAX_ERR ) {
        pointing_quality = -1;
      }

      // Otherwise the solution appears to be good...
      else {
        
        // if we were lost, solution is definitely good 
        if( lost ) {
          pointing_quality = 2;
          ccdRotation = rot; // update rotation if many blobs matched
        }
        
        // otherwise check for excursions from the previous good solution
        else {
          cel2vec(last_ra_0,last_dec_0,&last_a,&last_b,&last_c);
          cel2vec(ra_0,dec_0,&a,&b,&c);
          theta = acos( last_a*a + last_b*b + last_c*c );
          //max_theta = ((double)frame_time - (double)last_time + 1.) * 
          //  POINT_MAX_SLEW*PI/180.;
          
          max_theta = ((double)frame_time - (double)last_time + 1.) * 
            maxSlew*3.;

          if( (theta < max_theta) || (pointing_nbad >= POINT_EXCUR_NBAD) ) {
            pointing_quality = 1;
          } else {
            pointing_quality = -1;
          }
        }
      }
    } else {
      pointing_quality = 0;
      
      // If we got a bad solution using LOST IN SPACE reset number of bad
      // solutions in a row to try using the old pointing solution again
      
      if( lost ) pointing_nbad = 0;      
    }
    
    delete[] x;
    delete[] y;
    delete[] f;
  }
  
  if( pointing_quality >= 1 ) {    
    // If good pointing reset the number of bad solutions in a row
    pointing_nbad=0;
    server_data.sigma = sqrt(point_var);
    server_data.rot = rot;
    calc_alt_az( ra_0, dec_0, lat, lst, &el, &az );
    
    // Update last_solution / time to current values
    last_time = frame_time;
    last_ra_0 = ra_0;
    last_dec_0 = dec_0;

    
  } else {
    server_data.rot = ccdRotation;
    for( i=0; i<MAX_ISC_BLOBS; i++ ) {
      star_ra[i] = -999;
      star_dec[i] = -999;
      star_mag[i] = -999;
    }
    server_data.sigma = 2*PI;

    // Increase the number of bad solutions with >= POINT_LOST_BLOBS blobs
    if( server_data.n_blobs >= POINT_LOST_BLOBS ) {
      pointing_nbad++;
    }
  }
  
  //printf("PPP nmatch=%i minblobmatch=%i rot=%lf\n", 
  //     nmatch, minBlobMatch, server_data.rot*180/PI);



  // Update server data frame with pointing solution offset correctly for 
  // BDA to give the BDA pointing solution        
  //double az_off = -azBDA;     // az tangent plane offset of BDA from CCD
  //double el_off = -elBDA;     // el   "       "      "    "  "   "    "
  //double cos_theta = cos(-(q+rot)); // negative b/c rotating altaz->radec
  //double sin_theta = sin(-(q+rot));
  //double ra_off = -(az_off*cos_theta + el_off*sin_theta);  
  // RA tangent plane offset
  //double dec_off = -az_off*sin_theta + el_off*cos_theta;         
  // DEC tangent plane offset
  //tan2radec(ra_0,dec_0, &ra_off, &dec_off, &server_data.ra, 
  //&server_data.dec, 1);
  
  server_data.ra = ra_0;
  server_data.dec = dec_0;
  server_data.az = az;
  server_data.el = el;

  if( LOUD ) {
    time_stamp( &timebuf[0], 255 ); printf("%s Solution Finished\n",timebuf);
  }

  // write the frame to a file
  framewrite(saveFrameMode);
}

// Prepare the display by scaling the image buffer into the display buffer
void prep_image( void ) {
  int i;
  
  // scale the frame into the display buffer
  int intmean = (int) floor(Frameblob.get_mapmean());
  int sig = (int) floor(Frameblob.get_sigma());
  
  int clipmin = intmean - 4*sig;
  if( clipmin < 0 ) clipmin = 0;
  int clipmax = intmean + 10*sig; 
  int val;
  
  for( i=0; i<CCD_X_PIXELS*CCD_Y_PIXELS; i++ ) {
    val = ( (Frameblob.get_map())[i] - clipmin)*256/(clipmax-clipmin);
    if( val < 0 ) dispBuffer[i] = 0;
    else if( val > 255 ) dispBuffer[i] = (char) 255;
    else dispBuffer[i] = val;
  }
  
  t5 = clock();
}        

// Drive either stepper motor +ve or -ve # of steps. Return the actual number
// of steps (+ve or -ve). Updates the absolute positions aperturePosition and
// current_focus
int step_motor( int motor, int steps ) {
  FILE *logfile;
  char cmd[80],speedcmd[80];
  char direction;
  int actualSteps; // actual number of motor steps
  
  if( steps > 0 ) direction='P';
  else direction='D';
  
  // Make sure the absolute motor position is valid
  int position; // the new absolute position
  if( motor == AP_MOTOR ) {
    position = aperturePosition + steps;
    if( position < 0 ) position = 0;
    if( position > AP_RANGE ) position = AP_RANGE;
    actualSteps = position - aperturePosition;
    aperturePosition = position;
    printf("New aperture motor position will be: %i\n",aperturePosition);
  } 

  if( motor == FOCUS_MOTOR ) {
    position = current_focus + steps;
    if( position < 0 ) position = 0;
    if( position > FOCUS_RANGE ) position = FOCUS_RANGE;
    current_focus = position;
    server_data.current_focus_pos = current_focus;
    actualSteps = steps;
    printf("New focus motor position will be: %i\n",current_focus);
  }

  // Semafores for updating the windows display
  eyeRefresh=1;
  eyeMotor=motor;
  
  // Create the command strings
  if ((hold_current >= 12) && (hold_current <= 50)) {
	  sprintf(cmd,"h%i%c%i",hold_current,direction,abs(actualSteps));
  }
  else {
	  sprintf(cmd,"h12%c%i",direction,abs(actualSteps));
  }
  sprintf(speedcmd,"V%i",motorSpeed);
  
  if( actualSteps != 0 ) { // prevent motor spinning indefinitely
    
    printf("***step_motor: speed %s move %s steps***\n",speedcmd,cmd);
    printf("Stepping motor %i %i steps\n",motor,actualSteps);
    
    if( NO_CAMERA == 0 ) {
      motorcmd(comport,motor,"z5000");
      motorcmd(comport,motor,speedcmd);
      motorcmd(comport,motor,cmd);
    }

    // Log the stepper positions
    if( (logfile=fopen(stepperlogname,"w")) == NULL ) {
      printf("Couldn't open %s to log stepper positions.\n",stepperlogname);
    } else {
      fprintf(logfile,"%i %i",current_focus,aperturePosition);
      fclose(logfile);
    }
  }
  
  return actualSteps;
}

/*  Subroutine to position focus motor at home position  ....JK 04/05/2004 */

void focus_home(void) {
  char speedcmd[80];
  
  printf("FOCUS GO HOME!!!!!\n");
  
  sprintf(speedcmd,"V%iv%i",motorSpeed,motorSpeed); /* speed cmd V50v50 */
  
  motorcmd(comport,FOCUS_MOTOR,speedcmd);  /* send speed cmd to focus motor */
  motorcmd(comport,FOCUS_MOTOR,"Z10000");  /* send focus to home position */
  motorcmd(comport,FOCUS_MOTOR,"z10000");  /* reset position home to 10,000 */
  motorcmd(comport,FOCUS_MOTOR,speedcmd);  /* reset motor... ??? */

  execCmd.focus_pos = 0;  /* Reset home command to 0 */
	current_focus = 0;
	server_data.current_focus_pos = 0;
  
  if((focusOffset > 0) && (focusOffset < FOCUS_RANGE)) {
	  printf("focusOffset:%i\n",focusOffset);
	  step_motor(FOCUS_MOTOR,focusOffset);
	  //Sleep(abs(focusOffset)*1000/motorSpeed); 
  }
}

// set the focus/aperture to absolute position 0-FOCUS/AP_RANGE 
// (infinity/open) by calling step_motor

void absoluteMotor( int motor, int position ) {
  int delta; //, newpos;
  
  if( motor == AP_MOTOR ) {
    //printf("aperture pos before: %i \n",aperturePosition);
    delta = position - aperturePosition;
  } else {
    //printf("focus pos before: %i \n",current_focus);
    delta = position - current_focus;
  }
  
  // step_motor checks validity of step position
  int truedelta = step_motor( motor, delta ); 
  //printf("dstep: %i\n",truedelta);
}

// Set both motors to their (extreme) calibrated positions
void park_motors( void ) {
  absoluteMotor( FOCUS_MOTOR, FOCUS_RANGE );
  absoluteMotor( AP_MOTOR, AP_RANGE );
}

// Calibrate motors by reading the last position from the stepper log and
// ***then running to both extremes*** this step is now removed
void calibrate_motors( void ) {
  FILE *logfile;
  
  // Assume they were left in the parked position
  current_focus = FOCUS_RANGE;
  aperturePosition = AP_RANGE;
  
  if( (logfile=fopen(stepperlogname,"r")) == NULL ) {
    printf("No stepper log file found, assuming motors in parked position.\n");
  }
  // Get position from the log file
  else {
    int logFocus, logAperture;
    fscanf(logfile,"%i %i",&logFocus,&logAperture);
    fclose(logfile);
    
    // Set current_focus and aperturePosition to represent the logged positions
    // if valid.
    if( (logFocus>=0) && (logFocus<=FOCUS_RANGE) && (logAperture>=0) && 
        (logAperture<=AP_RANGE) ) {
      current_focus = logFocus;
      server_data.current_focus_pos = logFocus;
      aperturePosition = logAperture;
    }
  }
  
  //changed by Enzo & Marie so server will not 
  //reset aperture motor position on startup
  execCmd.ap_pos = aperturePosition;
  //absoluteMotor( FOCUS_MOTOR, 0 );
  //absoluteMotor( FOCUS_MOTOR, FOCUS_RANGE );
  //absoluteMotor( AP_MOTOR, 0 );
  //absoluteMotor( AP_MOTOR, AP_RANGE );
}

// Run the autofocus procedure: Take exposures over a range of focus
// settings recording the flux of the brightest object in the
// frame. Then use the flux at each position to calculate a weighted
// mean for the focus position giving a coarse solution. Re-calibrate
// motors and do a fine-stepping over about the previous solution,
// boxcar smooth and find the maximum.

void doautofocus( void ) {
  int i,j;
  double thisWeight=0.;
  double totalWeight=0.;
  //double thisFocus;
  int bestindex = 0;
  double max=0;
  
  ISCDisplayModeType oldMode;
  //unsigned oldBlob;        
  oldMode = eyeMode;
  
  eyeMode = blob;
  eyeBlobRoi = 0;

  server_data.autofocusOn=1;
  
  printf("Autofocus step =%i\n",AUTOFOCUS_FINE_DELTA);
  
  focus_home();
  //step_motor(FOCUS_MOTOR,-AUTOFOCUS_FINE_DELTA*AUTOFOCUS_FINE_NSAMPLES/2);

  double *fine_fluxes = new double[AUTOFOCUS_FINE_NSAMPLES];
  int goodSolution=0;
  
  for( i=0; i<AUTOFOCUS_FINE_NSAMPLES; i++ ) {

    printf("-----------> autofocus: %i step: %i / %i\n",abortFlag,i,
           AUTOFOCUS_FINE_NSAMPLES);
    
    autoFocusStep = AUTOFOCUS_FINE_NSAMPLES - i - 1;
    
    // grab a frame and find the blobs
    //expose_frame();        
    
    // wait until the expose thread has finished
    //printf("Waiting for expose thread to finish...\n");
    
    if( LOUD ) {
      time_stamp( &timebuf[0], 255 ); 
      printf("%s Start waiting for exposure thread to finish\n",timebuf);
    }
    
    // Sleep to give other processes some time
    while( thExpState != 2 ) Sleep(10);  
    
    //printf("Done waiting for exposure!\n");
    //time_stamp( &timebuf[0], 255 );
    //printf("%s Done waiting for exposure\n",timebuf);
    
    //printf(".........Finding pointing in framebuf %i\n",
    //(Frameblob.get_map() == (MAPTYPE *)frameBuf2)+1);

    // set all the noisey pixels to saturated value
    Frameblob.fix_badpix((MAPTYPE)Frameblob.get_satval()+1); 
    
    // calculate the map mean for all pixels < satval
    Frameblob.calc_mapstat();
    server_data.mapMean = Frameblob.get_mapmean();

    // set all the noisey pixels to the map mean
    Frameblob.fix_badpix((MAPTYPE)server_data.mapMean);
    Frameblob.calc_searchgrid();
    Frameblob.fix_multiple();
    Frameblob.sortblobs();
    
    bloblist *blobs = Frameblob.getblobs();
    int numextended = 0;   // count extended objects in the frame
    int numpoint = 0;      // count point-like objects in the frame
    
    while( blobs != NULL ) {        
      //if( blobs->gettype() == 2 )
      //{
      if( numextended < MAX_ISC_BLOBS ) {
        server_data.blob_x[numextended] = blobs->getx();

        // want y increasing with elevation
        server_data.blob_y[numextended] = CCD_Y_PIXELS - blobs->gety() - 1; 
        server_data.blob_flux[numextended] = blobs->getflux();
        server_data.blob_sn[numextended] = blobs->getsnr();
        
        //printf("%i: %f %f %i, %f\n",numextended,
        //server_data.blob_x[numextended],
        //server_data.blob_y[numextended],server_data.blob_flux[numextended],
        //blobs->getmean());
      }
      numextended++;  
      //}
      //else numpoint++;

      blobs = blobs->getnextblob();        
    }
    
    if( numextended > MAX_ISC_BLOBS ) server_data.n_blobs = MAX_ISC_BLOBS;
    else server_data.n_blobs = numextended;
    
    for( j=0; j<MAX_ISC_BLOBS; j++ ) {
      star_ra[j] = -999;
      star_dec[j] = -999;
      star_mag[j] = -999;
    }

    // write the frame to a file
    framewrite(saveFrameMode);
    
    // prepare display buffer and invalidate the window so that it is re-drawn
    if( eyeOn ) prep_image();

    // Set the Frameblob buffer to the next framebuffer to be filled by a grab
    Frameblob.set_map( (MAPTYPE *) QCFrame.pBuffer );
    
    // signal that it is OK to start another exposure
    thExpState = 0;
    //printf("thExpState = 0\n");
    
    // update the display with the current image
    eyeRefresh=1;
    
    // get the flux of the brightest blob 
    if( server_data.n_blobs > 0 ) {
      goodSolution = 1;
      fine_fluxes[i] = (double)server_data.blob_flux[0];
    } else fine_fluxes[i] = 0;

    // do a focus log entry
    server_log(2);
    
    // move to the new stepper position
    
    //motorcmd(comport,FOCUS_MOTOR,&focusstep);
    step_motor(FOCUS_MOTOR,AUTOFOCUS_FINE_DELTA);
    
    // Exit the loop if the abortFlag was set
    if( abortFlag ) i = AUTOFOCUS_FINE_NSAMPLES;
  }
  autoFocusStep=0;
  
  // Calculate a moving boxcar average for the samples, and step the
  // position to the peak value
  double thisAvg, maxAvg=0.;
  
  if( abortFlag != 1 ) { // only find new autofocus pos if not aborting
    for( i=AUTOFOCUS_BOXCAR/2; i<AUTOFOCUS_FINE_NSAMPLES-AUTOFOCUS_BOXCAR/2; 
         i++ ) {
      thisAvg=0;
      for( j=-AUTOFOCUS_BOXCAR/2; j<(-AUTOFOCUS_BOXCAR/2)+AUTOFOCUS_BOXCAR; 
           j++ )
        thisAvg += fine_fluxes[i+j];
      
      if( thisAvg > maxAvg ) {
        maxAvg = thisAvg;
        bestindex = i; 
      }
    }
    
    if( goodSolution ) { /* Move to best soln, the direction matters! */
      autoFocusStep = AUTOFOCUS_FINE_NSAMPLES - bestindex;
      focus_home();
      //step_motor(FOCUS_MOTOR,-AUTOFOCUS_FINE_DELTA*AUTOFOCUS_FINE_NSAMPLES/2);
      
      //for( i=0; i<=bestindex; i++ )
      //{
      //        step_motor(FOCUS_MOTOR,AUTOFOCUS_FINE_DELTA);
      //}
      
      step_motor(FOCUS_MOTOR,AUTOFOCUS_FINE_DELTA*bestindex);
    }
    else focus_home();
  }
  
  delete[] fine_fluxes;
  
  autoFocusMode = 0; // de-assert because we're done
  server_data.autofocusOn=0;
  eyeMode = oldMode;
  autoFocusStep = 0;
}        

// --- Communications --------------------------------------------------------

// Disconnect a connected socket
void disconnect_client(int client) {
  printf("Disconnecting client on port %i\n",SERVER_PORTS[client]);
  delete connected_sockets[client];    // kill the socket
  thConnectState[client] = 0;          // flag the client as disconnected
}

// Thread function for connecting clients.
// whichclient=index into client list
// parameter will contain 1 for connection success, or 0 for connection errors
DWORD WINAPI connect_client( LPVOID parameter ) {
  DWORD whichclient;
  whichclient = *((DWORD *)parameter);
  printf(": Connect client %i\n",whichclient);
  try {
    connected_sockets[whichclient] = listen_sockets[whichclient]->Accept();
    *((DWORD *)parameter) = 1;
    return 1;
  }
  catch( CSocketException exception ) {
    delete connected_sockets[whichclient];
    *((DWORD *)parameter) = 0;
    return 0;
  }
}

// Update the client connections: 
// mode=0 waits 5s for at least 1 connection b4 returning - only works if
//        no clients are connected
// mode=1 returns immediately
// 
// Return -1 on thread creation error
//         0 no clients are connected
//         1 there is at least one client connected
//
// Note: threads will be left connecting when function returns if no 
// connection has been established
 
int update_connections( int mode ) {
  DWORD result, thId;
  int i, no_connections, retval=0;
  
  // Start threads to wait for clients connecting on either port if
  // not already connected.  If any is already connected, set the
  // return value to 1.
        
  // If thread couldn't be started, return -1.
  for(i=0;i<NCLIENTS;i++) {
    if( thConnectState[i] == 0 ) {
      thConnectParam[i]=i; // parameter to thread function is the client #
      thConnect[i]=CreateThread( NULL,0,connect_client,&thConnectParam[i],0,
                                 &thId);
      if(thConnect[i]==NULL) return -1;
    } else retval = 1;
  }
        
  // Wait 5s for a connection by any client in mode 0
  no_connections = 1; // Make sure no clients already connected before waiting
  for(i=0;i<NCLIENTS;i++) if(thConnectState[i]!=0) no_connections=0;
  
  if( (mode == 0) && no_connections )
    result = WaitForMultipleObjects(NCLIENTS, thConnect, FALSE, 5000); 
  
  // Do WaitForSingleObject with 0 timeout to check completion status of each 
  // connection thread
  for( i=0; i<NCLIENTS; i++ ) {
    if( thConnectState[i] != 1 ) {
      // at this point, thread is running unless already connected
      result = WaitForSingleObject(thConnect[i],0);
      if( result != WAIT_TIMEOUT ) { // thread is finished
        CloseHandle(thConnect[i]);
        if( thConnectParam[i] ) { // if connection good (return connect val)
          retval=1;
          thConnectState[i]=1;
          printf("Client connected on port %i\n",SERVER_PORTS[i]);
        }
        else thConnectState[i]=0;        // connection failed
      }
      else thConnectState[i]=2;                // still connecting
    }
  }
  return retval;
}


// Thread function for receiving frames. Will not return until frame
// received or error.  Input: Client # to try receiving a frame from
// Output: # bytes received for success, 0 for receive error.

DWORD WINAPI receive_frame( LPVOID parameter ) {
  DWORD whichclient;
  int nbytes;
  whichclient = *((DWORD *)parameter);
  
  nbytes = connected_sockets[whichclient]->Read(&client_data[whichclient],
                                                sizeof(ISCStatusStruct));

  if( LOUD ) {
    time_stamp( &timebuf[0], 255 ); 
    printf("%s Receive Frame client %i\n",timebuf,whichclient);
    printf("Trigger type = %i\n",triggertype);
	  printf("Hold current for motors = %i\n",hold_current);
    printf("CCD rotation = %lf deg \n",ccdRotation*180./PI);
    printf("Az: %8.3lf deg\n",az*180./PI);
    printf("El: %8.3lf deg\n",el*180./PI);
  }
  
  // update the timer for the last time a command was received
  time( &lastCmdRec );

  if( nbytes > 0 ) {
    *((DWORD *)parameter) = nbytes;
    return nbytes;
  } else {
    *((DWORD *)parameter) = 0;
    return 0;
  }
}

// Start receive threads / check completion for all connected clients
// Return:
// -1 thread creation error
//  0 no new frames received
//  1 frames received
//  2 an abort command has been received

int update_receive( void ) {
  int i, retval=0;
  DWORD thId, result;
  
  for( i=0; i<NCLIENTS; i++ )

    // Is client connected?
    if( thConnectState[i] == 1 ) {

      // Start new receive thread if not running
      if (thRecvState[i] == 0 ) {
        thRecvParam[i] = i; 
        thRecv[i]=CreateThread( NULL,0,receive_frame,&thRecvParam[i],0,&thId);
        if(thRecv[i]==NULL) return -1;
        thRecvState[i] = 1;
      }
      
      // otherwise check completion of the receive thread
      else {
        result = WaitForSingleObject(thRecv[i],0);
        if( result != WAIT_TIMEOUT ) {        // thread has finished
          thRecvState[i] = 0;
          CloseHandle(thRecv[i]);
          
          // Check that a frame was received 
          if( thRecvParam[i] > 0) {

            // whole frame arrived
            if( thRecvParam[i] == sizeof(ISCStatusStruct) ) 

              //if( thRecvParam[i] == sizeof(clientCmdType) ) 
              // whole frame arrived
              {
                newCmd = i; // set this frame as most recent
                retval = 1;
                time( &clock_newcmd );
                
                //printf("*** %x %x %x %x\n",last_cmd.data[0],
                //last_cmd.data[1],last_cmd.data[2],last_cmd.data[3]);

                //printf("Received frame from client on port %i\n",
                //        SERVER_PORTS[i]);
                
                // Check to see if the command was an abort of
                // the execution thread).
                if( client_data[i].abort == 1 ) abortFlag = 1; 
                if( abortFlag ) retval = 2;
                
                // Check command was shut down / reboot / camera power cycle
                
                if( client_data[i].shutdown > 0 ) 
                  shutdownFlag = client_data[i].shutdown;

                // Change behaviour - actually exit program and re-start
                // Check to see if the command was a camera power cycle
                //if( client_data[i].shutdown==3 ) camcycleFlag=1;
                
                // Set the MCPFramenum here because it is returned in the 
                // acknowledgement before new frame is exposed in command_exec
                
                MCPFrameNum = client_data[i].MCPFrameNum;
                server_data.MCPFrameNum = MCPFrameNum;
                                                        
                //GetSystemTime(&f1);
                //printf("%7.3lf Copied MCP %li LST %lf\n",
                // (double)f1.wSecond + ((double)f1.wMilliseconds)/1000.,
                //                server_data.MCPFrameNum,client_data[i].lst);
                
              } else {
              //printf("Error on read from client on port %i.\n", 
              //SERVER_PORTS[i]);
              
              disconnect_client(i); // disconnect
            }
          } else {
            //printf("Incomplete frame from client on port %i: %i bytes.\n",
            //        SERVER_PORTS[i], thRecvParam[i]);
            disconnect_client(i); // disconnect
          }
        }
      }
    }
  
  return retval;
}

// Thread function for sending frames. Will not return until frame
// sent or error.  Input: Client # to try sending a frame to Output: #
// bytes sent for success, 0 for send error.
DWORD WINAPI send_frame( LPVOID parameter ) {
  DWORD whichclient;
  int nbytes;
  whichclient = *((DWORD *)parameter);
  
  //if( server_data.flag == 1 ) printf(" ISC # %i\n",server_data.framenum);
  
  nbytes = connected_sockets[whichclient]->Write(&server_data,
                                                 sizeof(server_data));

  if( LOUD ) {
    time_stamp( &timebuf[0], 255 ); 
    printf("%s Send Frame %i client %i flag %i\n",timebuf,
           server_data.framenum,whichclient,server_data.flag);
  }        

  //printf("Sent %i / %i bytes to client.\n",nbytes,sizeof(server_data));

  if( nbytes > 0 ) {
    *((DWORD *)parameter) = nbytes;
    return nbytes;
  } else {
    *((DWORD *)parameter) = 0;
    return 0;
  }
}

// Start send threads, wait for completion for all connected clients
// up to 1s before killing them. Don't want to start processing a new
// command until the results of the old one have been broadcasted.
// Return: // -1 thread creation error // 0 no frames sent // >0 #
// clients that received frames

int timeout_send( void ) {
  int i, j, nsending, ncomplete;
  DWORD thId, result;
  HANDLE *thread_list;
  
  //printf("Send... %i\n",server_data.flag);
  
  for( i=0; i<NCLIENTS; i++ )
    // Start new send threads, checking first to see if clients
    // connected / already sending
    if( (thConnectState[i] == 1) && (thSendState[i] != 1) ) {
      thSendParam[i] = i; 
      thSend[i]=CreateThread( NULL,0,send_frame,&thSendParam[i],0,&thId);
      if(thSend[i]==NULL) return -1;
      thSendState[i] = 1;
    }
  
  // Create array of pointers to threads actively sending
  nsending = 0; // number of clients we're sending to
  for( i=0; i<NCLIENTS; i++ ) if( thSendState[i] == 1 ) nsending ++;
  thread_list = new HANDLE[nsending];
  j=0;
  
  for( i=0; i<NCLIENTS; i++ ) 
    if( thSendState[i] == 1 ) {
      thread_list[j] = thSend[i];
      j++;
    }

  // Wait 1000 ms for the completion of all the send threads
  result = WaitForMultipleObjects(nsending, thread_list, TRUE, 1000); 
  
  // waitforsingleobjects to see which of the threads actually completed
  ncomplete = 0;
  for( i=0; i<NCLIENTS; i++ )
    if( thSendState[i] == 1 ) {
      result = WaitForSingleObject(thSend[i],0);
      if( result != WAIT_TIMEOUT ) ncomplete++; // thread finished on time
      //else printf("Client on port %i didn't receive server
      //data.\n",SERVER_PORTS[i]);

      CloseHandle(thSend[i]); // always close the send threads
      thSendState[i] = 0;            // done sending to this client
                }
  
  //thCmdState = 0; // ready to do a new command

  return ncomplete;
}

// Thread function for changing the ISC state to match that of the
// currently executing client frame.  Input: client_frame execCmd
// Output: server_frame is filled appropriately

DWORD WINAPI command_exec( LPVOID parameter ) {
  DWORD par;
  par = *((DWORD *)parameter);
  
  // Error check the incoming frame - set ridiculous values to
  // to the current settings in the execCmd
        
  if( (execCmd.save<0) || (execCmd.save>1) ) execCmd.save = saveFrameMode;

  if( (execCmd.autofocus<0) || (execCmd.autofocus>1) ) execCmd.autofocus = 0;

  if( (execCmd.triggertype<0) || (execCmd.triggertype>3) ) 
    execCmd.triggertype = triggertype;

  if( (execCmd.focusOffset<0) || (execCmd.focusOffset>FOCUS_RANGE) ) 
    execCmd.focusOffset = 0;

  if( (execCmd.ap_pos<0) || (execCmd.ap_pos>AP_RANGE) ) 
    execCmd.ap_pos = aperturePosition;
        
  if( (execCmd.shutdown<0) || (execCmd.shutdown>1) ) execCmd.shutdown = 0;

  if( (execCmd.hold_current<12) || (execCmd.hold_current>50) ) 
    execCmd.hold_current = hold_current;

  if( (execCmd.display_mode!=full) && (execCmd.display_mode!=roi) && 
      (execCmd.display_mode!=blob) )
    execCmd.display_mode=eyeMode;

  if( (execCmd.eyeOn<0) || (execCmd.eyeOn>1) ) execCmd.eyeOn=eyeOn;

  if( (execCmd.exposure<0) || (execCmd.exposure>5000000) ) 
    execCmd.exposure=ccd_exposure;

  if( (execCmd.roi_x<0) || (execCmd.roi_x>CCD_X_PIXELS-1) ) 
    execCmd.roi_x = xRoi;

  if( (execCmd.roi_y<0) || (execCmd.roi_y>CCD_Y_PIXELS-1) ) 
    execCmd.roi_y = xRoi;

  if( (execCmd.blob_num<0) || (execCmd.blob_num>MAX_ISC_BLOBS) ) 
    execCmd.blob_num = 0;
        
  if( (execCmd.brightStarMode<0) || (execCmd.brightStarMode>1) )
    execCmd.brightStarMode = brightStarMode;

  if( execCmd.sn_threshold < 0.1 ) 
    execCmd.sn_threshold = Frameblob.get_threshold();
        
  if( (execCmd.grid<1) || (execCmd.grid>CCD_Y_PIXELS/4) ) 
    execCmd.grid = Frameblob.get_grid();
        
  //if( (execCmd.cenbox<1) || (execCmd.cenbox>CCD_Y_PIXELS/4) )
  //execCmd.cenbox = Frameblob.get_cenbox(); if( (execCmd.apbox<1) ||
  //(execCmd.apbox>CCD_Y_PIXELS/4) ) execCmd.apbox =
  //Frameblob.get_apbox();
        
  if( (execCmd.mult_dist<1) || (execCmd.mult_dist>CCD_Y_PIXELS/4) )
    execCmd.mult_dist = (int) Frameblob.get_disttol();
        
  if( (execCmd.gain<=0) || (execCmd.gain>100) ) execCmd.gain = rel_gain;

  if( (execCmd.useLost<0) || (execCmd.useLost>1) ) execCmd.useLost = useLost;

  if( execCmd.maxSlew*RAD2DEG <= 0.05 ) execCmd.maxSlew = 0.05*DEG2RAD;
  if( execCmd.maxSlew*RAD2DEG > 5) execCmd.maxSlew = maxSlew;

  if( (execCmd.maxBlobMatch<0) || (execCmd.maxBlobMatch>MAX_ISC_BLOBS) )
    execCmd.maxBlobMatch = 7;
        
  if( (execCmd.minBlobMatch<0) || (execCmd.minBlobMatch>MAX_ISC_BLOBS) ) 
    execCmd.minBlobMatch = 3;
        
  if( (execCmd.mag_limit<0) || (execCmd.mag_limit>12) ) 
    execCmd.mag_limit=mag_limit;
        
  if( (execCmd.norm_radius<0.1*PI/180.) || (execCmd.norm_radius>PI) ) 
    execCmd.norm_radius=norm_radius;
        
  if( (execCmd.lost_radius<0.1*PI/180.) || (execCmd.lost_radius>PI) ) 
    execCmd.lost_radius=lost_radius;
        
  if( (execCmd.tolerance<0) || (execCmd.tolerance>PI) ) 
    execCmd.tolerance=tolerance;
        
  if( (execCmd.rot_tol<0) || (execCmd.rot_tol>2*PI) ) 
    execCmd.rot_tol = rot_tol;
        
  if( (execCmd.match_tol<0) || (execCmd.match_tol>1) ) 
    execCmd.match_tol = match_tol;
        
  if( (execCmd.quit_tol<0) || (execCmd.quit_tol>1) ) 
    execCmd.quit_tol = quit_tol;
  
  // Update general flags
  pause = execCmd.pause;
  saveFrameMode = execCmd.save;
  autoFocusMode = execCmd.autofocus;
  eyeOn = execCmd.eyeOn;
  
  // stepper motors
  focusOffset = execCmd.focusOffset;
  focusPosition = execCmd.focus_pos;
  //printf("Focus position received from client: %i\n",focusPosition);
   
  if( (focusPosition != 0) && (focusPosition != -1) && ((current_focus + focusPosition) > 0) && ((current_focus + focusPosition) < FOCUS_RANGE)) 
    step_motor(FOCUS_MOTOR,focusPosition);
 
  if( focusPosition == -1 ) focus_home(); 
      
  if( aperturePosition != execCmd.ap_pos ) 
    printf("Aperture cannot be changed on this camera!\n");
    //**LORENZO** faulty aperture in this camera - avoid any command to be sent to the aperture motor
    //absoluteMotor(AP_MOTOR,execCmd.ap_pos);
        
  if( hold_current != execCmd.hold_current ) {
    char hold_cmd[80];
    hold_current = execCmd.hold_current;
    sprintf(hold_cmd,"h%i",hold_current);
    motorcmd(comport,AP_MOTOR,hold_cmd);
    motorcmd(comport,FOCUS_MOTOR,hold_cmd);
  }

  // display mode variables
  if( (eyeMode != execCmd.display_mode) || (xRoi != execCmd.roi_x ) || 
      (yRoi != execCmd.roi_y) || (eyeBlobRoi != execCmd.blob_num) )
    eyeRefresh=1;
  
  //InvalidateRgn(hwndEye,NULL,(eyeMode!=full)&&(lastMode==full)); 
  // invalidate the display window so that it is re-drawn
        
  eyeMode = execCmd.display_mode;
  xRoi = execCmd.roi_x;
  yRoi = execCmd.roi_y;
  eyeBlobRoi = execCmd.blob_num;
  azBDA = execCmd.azBDA;
  elBDA = execCmd.elBDA;

  // Pointing info
  az = execCmd.az;
  el = execCmd.el;
  lat = execCmd.lat;
  maxSlew = execCmd.maxSlew;

#ifndef AUTONOMOUS
  lst = execCmd.lst;
#endif

  // If a new LST has been sent, derive a new longitude
  if( execCmd.lst != refLST ) {
    refLST = execCmd.lst;
    GetSystemTime(&refSysTime);
    lon = refLST - get_gst( refSysTime.wYear,
                            refSysTime.wMonth, 
                            refSysTime.wDay,
                            (refSysTime.wHour*3600 +
                             refSysTime.wMinute*60 +
                             refSysTime.wSecond +
                             refSysTime.wMilliseconds/1000.)/3600./24. ); 

    lon = fmod( lon, 2*PI );  
    if( lon < -PI ) lon += 2*PI;
    if( lon > PI ) lon -= 2*PI;

  }

  // brightest blob is
  brightStarMode = execCmd.brightStarMode;        
  brightRA = execCmd.brightRA;
  brightDEC = execCmd.brightDEC;
  
  // blob finding
  Frameblob.set_threshold( execCmd.sn_threshold );
  Frameblob.set_grid( execCmd.grid );
  Frameblob.set_disttol( execCmd.mult_dist );

  // CCD settings
  if( ccd_exposure != execCmd.exposure ) {
    ccd_exposure = execCmd.exposure;
    QCam_SetParam( &settings, qprmExposure, (unsigned long)ccd_exposure );
    newSettings = 1;
    //QCam_SendSettingsToCam( camhandle, &settings );
    //printf("*** Set exposure time to %li us.\n",ccd_exposure);
  }

  if( rel_gain != execCmd.gain ) {
    rel_gain = execCmd.gain;
    QCam_SetParam( &settings, qprmNormalizedGain,
                   (unsigned long) ( ((double)default_gain)*rel_gain ) );
    newSettings = 1;
    //QCam_SendSettingsToCam( camhandle, &settings );
    Frameblob.set_gain(noiseGain*rel_gain); // these gains/offsets are
                                            // for the source
                                            // detection NOISE MODEL

    //printf("*** Set CCD gain to %lu us.\n",(unsigned long)(
    //((double)default_gain)*rel_gain ));

  }

  if( rel_offset != execCmd.offset ) {
    rel_offset = execCmd.offset;
    QCam_SetParamS32( &settings, qprmS32AbsoluteOffset, 
                      (signed long)(default_offset+rel_offset) );
    newSettings = 1;
    //QCam_SendSettingsToCam( camhandle, &settings );
    Frameblob.set_readout_offset(noiseOffset+rel_offset);
    Frameblob.set_readout_noise(0);         
    //printf("*** Set CCD offset to %i us.\n",default_offset+rel_offset);
  }

  // frame matching
  mag_limit = execCmd.mag_limit;
  norm_radius = execCmd.norm_radius;
  lost_radius = execCmd.lost_radius;
  tolerance = execCmd.tolerance;
  match_tol = execCmd.match_tol;
  quit_tol = execCmd.quit_tol;
  rot_tol = execCmd.rot_tol;
  minBlobMatch = execCmd.minBlobMatch;
  maxBlobMatch = execCmd.maxBlobMatch;

  // If slewing too fast, disable useLost
  if( maxSlew*RAD2DEG < 1 ) { 
    useLost = execCmd.useLost;
  } else {
    useLost = 0;
  }

  // Clients can only change the trigger mode _TO_ self-triggered
  // External triggers are automatically polled every TRIGGER_RETRY seconds

  if( (triggertype != execCmd.triggertype) && (triggertype != 0) ) {
    triggertype = execCmd.triggertype;
    switch(triggertype) {
    case 0: 
      QCam_SetParam( &settings, qprmTriggerType, qcTriggerFreerun ); 
      break;
      
    case 1: 
      QCam_SetParam( &settings, qprmTriggerType, qcTriggerEdgeHi ); 
      break;
      
    case 2: 
      QCam_SetParam( &settings, qprmTriggerType, qcTriggerPulseHi ); 
      break;
                        
    case 3: 
      QCam_SetParam( &settings, qprmTriggerType, qcTriggerPulseLow ); 
      break;
    }
    newSettings = 1;
    //QCam_SendSettingsToCam( camhandle, &settings );
  }
  
  // poll external trigger if enough time has elapsed
  time_t now;
  time( &now );
  
  if( (triggertype == 0) && ((now - lastTrigRec) > TRIGGER_RETRY) ) {
    triggertype = 3;
    QCam_SetParam( &settings, qprmTriggerType, qcTriggerPulseLow );
    newSettings = 1;
    //QCam_SendSettingsToCam( camhandle, &settings );
    server_data.triggertype = triggertype;
  }
  
  // if we're in autofocus mode
  if( autoFocusMode ) {
    doautofocus();
    //server_data.autoFocusPosition = (int)autoFocusPosition;//this line must be obsolete
  }
  // if we're not paused, calculate a pointing solution
  else if( pause != 1 ) {
    //expose_frame();                        // get a new frame, find blobs
    
    // wait until the expose thread has finished
    //printf("Waiting for expose thread to finish...\n");
    if( LOUD ) {
      time_stamp( &timebuf[0], 255 ); 
      printf("%s ---- Waiting for expose thread to finish...\n",timebuf);
    }
    
    // Sleep to give other processes some time
    while( thExpState != 2 ) Sleep(10);  
    //printf("Done waiting for exposure!\n");
                
    if( LOUD ) {
      time_stamp( &timebuf[0], 255 ); 
      printf("%s Done waiting for exposure\n",timebuf);
    }
    
    //printf("camerr = %i triggertype = %i\n",camErr,triggertype);
    
    if( camErr == 100 ) { // if timeout switch to self-trigger     
      // change to self-triggered mode
      // try taking another exposre

      printf("Switching to self-trigger mode...\n");
      QCam_SetParam( &settings, qprmTriggerType, qcTriggerFreerun );
      newSettings = 1;
      //QCam_SendSettingsToCam( camhandle, &settings );
      triggertype = 0;

      // reset state of the exposure thread
      thExpState = 0;
      //printf("thExpState = 0\n");
                      
      // wait until the expose thread has finished
      printf("Waiting for expose thread to finish...TIMEOUT\n");

      // Sleep to give other processes some time
      while( thExpState != 2 ) Sleep(10);  
      printf("Done waiting for exposure TIMEOUT!\n");

      //expose_frame();

      // reset the time of the last hardware trigger received
      time( &lastTrigRec );
    }

    pointingSolution();       // calculate a pointing solution (+save image)
    
    if( eyeOn ) prep_image(); // prepare the display window buffer
    eyeRefresh=1;

    // Set the Frameblob buffer to the next framebuffer to be filled by a grab
    Frameblob.set_map( (MAPTYPE *) QCFrame.pBuffer );
    
    // signal that it is OK to reset the exposure thread
    thExpState = 3;
    //printf("thExpState = 0\n");
    
    
    //InvalidateRgn(hwndEye,NULL,(eyeMode!=full)&&(lastMode==full)); 
    // invalidate the display window so that it is re-drawn
    
    newpointinglog = 1;  // need to log a new pointing solution result
  }
  //else server_data.n_blobs = 0;

  // write a serverlog entry
  server_log(0);
  
  // Unset abortFlag
  abortFlag = 0;
  
  //saveFrameMode = 0;
  
  // reset the client timeout
  // time( &lastCmdRec );
  
  return par;
}

// Either start the execution of a new ISC command or else check the
// status of one that is currently running.  input: abort. If set,
// kill currently executing command thread, don't start new thread
//
// return:
//   -1: couldn't start new command thread
//    0: busy (command already executing)
//    1: started running a new command
//    2: execution finished
//    3: no new commands to execute, no previous command executing
//    4: command was aborted

int update_command( int abort ) {
  DWORD thId, result;
  int retval=3;  // assume no new commands / nothing executing
  time_t now;    

  // If no command is currently running 
  if( thCmdState == 0 ) {
    time( &now );

    // If a new command is available 
    if( newCmd != -1 ) {
      //printf("Starting command from port %i\n",SERVER_PORTS[newCmd]);
      if( LOUD ) {
        time_stamp( &timebuf[0], 255 ); 
        printf("%s Starting processing frame from client %i\n",
               timebuf,newCmd);
      }

      // frame containing the most recent command for execution
      execCmd = client_data[newCmd];  
      
      newCmd = -1;
      thCmdParam=0;  // this thread parameter is not used for anything
                        
      thCmd=CreateThread( NULL,0,command_exec,&thCmdParam,0,&thId);
      if( thCmd == NULL ) return -1;
      thCmdState = 1; // flag the thread as running
      retval = 1;

      // Setting the flag to 0 signals to expose_frame to handshake
      //server_data.flag = 0;

      // Ensure that the grab has actually started
      if( LOUD ) { 
        time_stamp( &timebuf[0], 255 ); 
        printf("%s Waiting for the camera to be prepared\n",timebuf);
      }

      while( grabbingNow == 2 ) Sleep(10);

      // Should be waiting for a trigger, send handshake now
      server_data.flag = 0;
      t7 = clock();
      printf("****** Elapsed time (between handshakes): %lfs ******\n",
             (double)(t7-t6)/CLOCKS_PER_SEC);
      t6 = t7;
      //printf(" --------> Send handshake\n");
      timeout_send();
      //printf(" --------| Sent!\n");
      server_data.flag = 1; // reset the handshake flag
    }
    // If we're not in paused mode and no commands
    // available/executing, start a new exposure command thread 
    //NEW BEHAVIOUR: added condition - only auto-expose in self
    //trigger mode if 10s have elapsed
        
    else if( (pause != 1) && (thCmdState != 1) && 
             ( (triggertype == 0 ) || ( ((now-lastCmdRec) > CLIENT_TIMEOUT) 
                                        && (triggertype != 0) ) ) ) {
      //printf("*** Client hasn't sent a packet in more than %i
      //seconds, starting exposure...\n",CLIENT_TIMEOUT); printf("P=%i
      //th=%i trig=%i tm=%i\n",pause, thCmdState, triggertype,
      //now-lastCmdRec);

      newCmd = -1;
      thCmdParam=0;  // this thread parameter is not used for anything
      thCmd=CreateThread( NULL,0,command_exec,&thCmdParam,0,&thId);
      if( thCmd == NULL ) return -1;
      thCmdState = 1; // flag the thread as running
      retval = 1;
    }
  }

  // If a command was already executing
  if( thCmdState == 1 ) {
    result = WaitForSingleObject(thCmd,0);
    
    if( result != WAIT_TIMEOUT ) { // command has completed
      thCmdState = 0; // finished execution, result must be sent to clients
      retval = 2;
      CloseHandle(thCmd);
    } else {
      retval = 0;
      // If we are aborting the command
      if( abort ) {
        // *** Don't do anything - semafores signal abort to execution thread

        //server_data.n_blobs=0; 
        // so that we don't try to access blobs later on from aborted frame
        //pointing_quality=0;
        //pointing_nbad++;
        
        // abort camera operations

        //if( NO_CAMERA == 0 )
        //{
        //        QCam_Abort(camhandle);
        //        QCam_SendSettingsToCam( camhandle, &settings );
        //}
        
        // send terminate command strings to the motors
        //printf("abort focus: %i\n",motorabort(comport,FOCUS_MOTOR));
        //printf("abort aperture: %i\n",motorabort(comport,AP_MOTOR));
        
      }
      //else retval = 1; // otherwise command still executing
    }
  }
  
  return retval;
}


// Thread that runs the temperature + pressure + heater routine

DWORD WINAPI th_doTemp( LPVOID parameter ) {
  //int heaterOn;
  char heat[3];
  
  tempDoStuff(&server_data.temp1, &server_data.temp2, &server_data.temp3, 
              &server_data.temp4, &server_data.pressure1, 
              &server_data.heaterOn);        

  // reset the pmd if all temp/pressure values are screwed after power cycling the whole system
  if(server_data.temp1 > 1000. && server_data.temp2 > 1000. && server_data.temp3 > 1000. && server_data.temp4 > 1000. && server_data.pressure1 > 30.) {
    printf("Resetting pmd...");
    printf("%i...",SetPortVal(PARALLEL_BASE,0,1));
    Sleep(500);
    printf("%i...",SetPortVal(PARALLEL_BASE,32,1));
    Sleep(500);
    printf("%i...",SetPortVal(PARALLEL_BASE,0,1));
    Sleep(500);
    printf("done!\n");
  }
        
  if( server_data.heaterOn ) sprintf(heat,"H");
  else sprintf(heat," ");

  sprintf(tempstring,"%0.1f %0.1f %0.1f %0.1f %0.1f %s",
          server_data.temp1,server_data.temp2,server_data.temp3,
          server_data.temp4, server_data.pressure1, heat);

  return 0;
}

DWORD WINAPI debugme( LPVOID parameter ) {
  DWORD result;

  while( hereflag != 2 ) {
    if( hereflag == 1 ) {
      result = WaitForSingleObject(thCmd,0);
      if( result == WAIT_TIMEOUT ) printf("Command thread running\n");
      else printf("Command thread finished\n");
    }
    Sleep(10);
  }

  return 0;
}

// --- Display Window (evil eye) routines ------------------------------------

// Place a formated hr/min/s coordinate string into str
void parse_coordinates( char *str ) {
  double ra = ra_0*180/PI/15.;
  double dec = dec_0*180/PI;
  double h, h_min, h_sec;
  double d, d_min, d_sec;
  char quality[2];
  
  h = floor(ra);
  h_min = floor((ra - h)*60.);
  h_sec = (ra-h-h_min/60.)*3600.;
  
  d = floor(dec);
  d_min = floor((dec - d)*60.);
  d_sec = (dec - d - d_min/60.)*3600.;
  
  switch(pointing_quality) {
  case -1: 
    sprintf(quality,"*"); 
    break;
        
  case 0: 
    sprintf(quality,"B"); 
    break;
        
  case 1: 
    sprintf(quality," "); 
    break;

  case 2: 
    sprintf(quality,"L"); 
    break;
  }

  sprintf(str,"%s%ih%im%4.1lfs %id%i'%i''",quality,
          (int)h,(int)h_min,(double)h_sec,
          (int)d,(int)d_min,(int)d_sec);
}

// Window handling routine - decide what to do based on message received
LRESULT CALLBACK MainWndProc(
    HWND hwnd,        // handle to window
    UINT uMsg,        // message identifier
    WPARAM wParam,    // first message parameter
    LPARAM lParam) {  // second message parameter

  PAINTSTRUCT ps; 
  HDC hdc;
  unsigned int xblob, yblob;
  int i,j,x_roi_start,y_roi_start,mindim,x_text,y_text;
  double pixtick; // numbre of pixels corresponding to TICK_ARCMIN
  int ntick,tickindex,blobMode;
        
  int bda_x_cen, bda_y_cen, bda_x_off, bda_y_off; 
  
  char sourcenum[80];   // string for the source number
  char mag[80];         // magnitude of the star
  char coordstring[80]; // string for the pixel coordinates of ROI
  char fluxstring[80];  // string for the flux of this object
  char radecstring[80]; // current coordinates
  char rotstr[80];      // rotation of the CCD
  //char afocstr1[80];    // autofocus messages
  char afocstr2[80];
  char frameNumStr[80]; // current frame number message
  char maxslewstr[80];  // maxSlew

  char lststr[80];
  char azstr[80];
  char elstr[80];
  char latstr[80];
  char lonstr[80];
  char meanstr[80];
  char stddevstr[80];
  char expstr[80];
  char apstr[80];
  char focstr[80];
  char iscstr[80];
  
  //char newcmdstr[80];
  
  const COLORREF WHITE = RGB(255, 255, 255);
  const COLORREF RED = RGB(255, 0, 0);
  HFONT hfntDefault; //,hfntBold                
  
  bloblist *blobs = Frameblob.getblobs();
  RECT client;                
  unsigned int client_width, client_height;
  
  // Update last display mode
  lastMode=eyeMode;
  eyeRefresh=0;

  switch (uMsg) { 
  case WM_PAINT: // Paint the window with the image
    hdc = BeginPaint(hwnd, &ps); 
    
    // Get the dimensions of the client window
    GetClientRect(hwnd,&client);
    client_width = client.right;
    client_height = client.bottom;
    
    // Set up the font
    SetTextColor(hdc, RED);
    SetBkMode(hdc, TRANSPARENT); 
    
    // Select a white pen
    SelectObject(hdc,GetStockObject(WHITE_PEN));

    // Display small ROI sub-map if eyeMode >= 0 (blob #) or -2
    // (centre over pixel)
    if( (eyeMode==roi) || (eyeMode==blob) ) {
   
      // Update the ROI position if centering over a blob
      if( (eyeMode == blob) && (eyeBlobRoi<(int)server_data.n_blobs) && 
          (eyeBlobRoi>=0)) {
        xRoi = (int) floor(server_data.blob_x[eyeBlobRoi] + 0.5);
        yRoi = (int) floor(server_data.blob_y[eyeBlobRoi] + 0.5); 
        blobMode = 1;
      } else blobMode = 0;

      // error check xRoi and yRoi
      if( (xRoi<0) || (xRoi>=CCD_X_PIXELS) || (yRoi<0) || 
          (yRoi>=CCD_Y_PIXELS) ) {
        xRoi = CCD_X_PIXELS/2;
        yRoi = CCD_Y_PIXELS/2;
      }
      
      x_roi_start =  xRoi - EYE_ROI/2;
      y_roi_start =  (CCD_Y_PIXELS-yRoi-1) - EYE_ROI/2;

      for(i=x_roi_start; i<x_roi_start+EYE_ROI; i++)
        for(j=y_roi_start; j<y_roi_start+EYE_ROI; j++) {
      
          // Create ROI image buffer
          if( (i>=0) && (i<CCD_X_PIXELS) && (j>=0) && (j<CCD_Y_PIXELS) )
            roiBuffer[(j-y_roi_start)*EYE_ROI + i-x_roi_start] = 
              dispBuffer[j*CCD_X_PIXELS+i];
          
          // If outside of the source image, set pixels to grey
          else roiBuffer[(j-y_roi_start)*EYE_ROI + i-x_roi_start] = 80;
        }
      
      // Overplot axes & tick marks
      pixtick = (double)TICK_ARCMIN*60./platescale;
      ntick = (int)floor((double)(EYE_ROI/2)/pixtick);

      for(i=0;i<EYE_ROI/2-(int)pixtick;i++) { // axes
        // x-axis
        roiBuffer[EYE_ROI*EYE_ROI/2 + i] = (char)255;
        roiBuffer[EYE_ROI*EYE_ROI/2 + EYE_ROI-i-1] = (char)255;

        // y-axis
        roiBuffer[i*EYE_ROI + EYE_ROI/2] = (char)255;
        roiBuffer[(EYE_ROI-i-1)*EYE_ROI + EYE_ROI/2] = (char)255;
      }

      
      for( i=(-ntick+1); i<=(ntick-1); i++ ) { // tick marks
        if( i != 0 ) {
          tickindex = (int)floor(i*pixtick+0.5);
                                        
          for( j=0; j<TICK_PIXLEN; j++ ) {

            // horizontal
            roiBuffer[(EYE_ROI/2-tickindex)*EYE_ROI + EYE_ROI/2 + j] = 
              (char)255;

            roiBuffer[(EYE_ROI/2-tickindex)*EYE_ROI + EYE_ROI/2 - j] = 
              (char)255;

            // vertical ticks
            roiBuffer[(EYE_ROI/2-j)*EYE_ROI + EYE_ROI/2-tickindex] = (char)255;
            roiBuffer[(EYE_ROI/2+j)*EYE_ROI + EYE_ROI/2-tickindex] = (char)255;
          }
        }
      }

      BitMapInfo->bmiHeader.biWidth = EYE_ROI; 
      BitMapInfo->bmiHeader.biHeight = DISP_IMAGE_YFLIP*EYE_ROI;
      
      if( client_width > client_height ) mindim=client_height;
      else mindim=client_width;

      StretchDIBits(hdc,0,0,mindim,mindim,0,0,EYE_ROI,EYE_ROI,roiBuffer,
                    BitMapInfo,DIB_RGB_COLORS,SRCCOPY);

      // Plot the pixel coordinates of the ROI centre
      hfntDefault = (HFONT) SelectObject(hdc, hfntBold); 
      sprintf(coordstring,"%i,%i",xRoi,yRoi);
      TextOut(hdc, mindim*3/4, FONT_HEIGHT, coordstring, 
              (int)strlen(coordstring));

      // Plot the blob info if in blob-centering mode
      if( blobMode ) {
        sprintf(fluxstring,"fl: %i",server_data.blob_flux[eyeBlobRoi]);
        TextOut(hdc, mindim*3/4, FONT_HEIGHT*2, fluxstring, 
                (int)strlen(fluxstring));
        sprintf(sourcenum,"blob: %i",eyeBlobRoi);
        TextOut(hdc, FONT_HEIGHT, FONT_HEIGHT*3, sourcenum, 
                (int)strlen(sourcenum));

        if( star_mag[eyeBlobRoi] != -999 ) {
          sprintf(mag,"mag: %3.1lf",star_mag[eyeBlobRoi]);
          TextOut(hdc, mindim*3/4, FONT_HEIGHT*3, mag, (int)strlen(mag));
        }
      }
    }

    // Otherwise do a full screen display
    else {
      BitMapInfo->bmiHeader.biWidth = CCD_X_PIXELS; 
      BitMapInfo->bmiHeader.biHeight = DISP_IMAGE_YFLIP*CCD_Y_PIXELS; 

      // Draw bitmap in the window stretched to EYE_WIDTH x EYE_HEIGHT pixels
      if( dispBuffer != NULL )
        StretchDIBits(hdc,0,0,client_width,client_height,0,0,CCD_X_PIXELS,
                      CCD_Y_PIXELS,dispBuffer,BitMapInfo,DIB_RGB_COLORS,
                      SRCCOPY);
                
      // Overplot rectangular markers where we found sources + number them
      hfntDefault = (HFONT) SelectObject(hdc, hfntBold); 

      for( i=0; i<(int)server_data.n_blobs; i++ ) {
        xblob = (int)floor( server_data.blob_x[i] * client_width/CCD_X_PIXELS 
                            + 0.5 );
                                        
        yblob = (int)floor( (CCD_Y_PIXELS-server_data.blob_y[i]-1) * 
                            client_height/CCD_Y_PIXELS + 0.5 ) - 1;
                                
        // left
        Rectangle(hdc,xblob-MARKER_SIDE,yblob+MARKER_SIDE,
                  xblob-MARKER_SIDE+MARKER_THICK,yblob-MARKER_SIDE);
                        
        // right
        Rectangle(hdc,xblob+MARKER_SIDE-MARKER_THICK,yblob+MARKER_SIDE,
                  xblob+MARKER_SIDE,yblob-MARKER_SIDE);
                        
        // top
        Rectangle(hdc,xblob-MARKER_SIDE,yblob+MARKER_SIDE,xblob+MARKER_SIDE,
                  yblob+MARKER_SIDE-MARKER_THICK);
                        
        // bottom
        Rectangle(hdc,xblob-MARKER_SIDE,yblob-MARKER_SIDE+MARKER_THICK,
                  xblob+MARKER_SIDE,yblob-MARKER_SIDE);

        // Find the best place to put the text relative to the bob centre
        x_text = MARKER_SIDE;
        y_text = 0;
        
        if( xblob > client_width - MARKER_SIDE - 2*FONT_WIDTH) 
          x_text = -MARKER_SIDE-2*FONT_WIDTH;
                
        if( yblob > client_height - MARKER_SIDE - FONT_HEIGHT)
          y_text = -MARKER_SIDE-FONT_HEIGHT;

        if( star_mag[i] != -999 ) {
          sprintf(mag,"%3.1lf",star_mag[i]);
          hfntDefault = (HFONT) SelectObject(hdc, hfntSmall);                 
          TextOut(hdc, xblob-0.7*x_text, yblob-1.2*FONT_HEIGHT, mag, 
                  (int)strlen(mag));
                        
          hfntDefault = (HFONT) SelectObject(hdc, hfntBold);                 
        }
        
        sprintf(sourcenum,"%i",i);
        TextOut(hdc, xblob+x_text, yblob+y_text, sourcenum, 
                (int)strlen(sourcenum));
      }

      // Coordinates at the bottom left of the screen
      //parse_coordinates( radecstring );
      if( pointing_quality >= 1 )
        sprintf(radecstring," %8.5lfh %6.4lfd",
                ra_0*180./PI/15.,dec_0*180./PI);
      else
        sprintf(radecstring,"X %8.5lfh %8.4lfd",ra_0*180./PI/15.,
                dec_0*180./PI);
                
      sprintf(rotstr," R %6.4lfd",server_data.rot*180./PI);

      TextOut(hdc, FONT_HEIGHT, client_height-3*FONT_HEIGHT, radecstring, 
              (int)strlen(radecstring));
      TextOut(hdc, FONT_HEIGHT, client_height-2*FONT_HEIGHT, rotstr, 
              (int)strlen(rotstr));


      // --- Outline of the BDA ---
      bda_x_cen = (int) floor((CCD_X_PIXELS/2 + 
                               azBDA*180./PI*3600./platescale) * 
                              client_width/CCD_X_PIXELS);
                                
      bda_y_cen = (int) floor((CCD_Y_PIXELS/2 - 
                               elBDA*180./PI*3600./platescale) * 
                              client_height/CCD_Y_PIXELS);
                                
      bda_x_off = (int) floor( (13./2.*60./platescale) * 
                               client_width/CCD_X_PIXELS );
                                
      bda_y_off = (int) floor( (6.5/2.*60./platescale) * 
                               client_height/CCD_Y_PIXELS );
                                
      // left
      Rectangle(hdc,bda_x_cen-bda_x_off, bda_y_cen+bda_y_off,
                bda_x_cen-bda_x_off+BDA_THICK, bda_y_cen-bda_y_off);

      // right
      Rectangle(hdc,bda_x_cen+bda_x_off-BDA_THICK,
                bda_y_cen+bda_y_off,
                bda_x_cen+bda_x_off,
                bda_y_cen-bda_y_off);
      
      // top
      Rectangle(hdc,bda_x_cen-bda_x_off,
                bda_y_cen+bda_y_off,
                bda_x_cen+bda_x_off, 
                bda_y_cen+bda_y_off-BDA_THICK);
      
      // bottom
      Rectangle(hdc,bda_x_cen-bda_x_off,                       
                bda_y_cen-bda_y_off+BDA_THICK,
                bda_x_cen+bda_x_off,
                bda_y_cen-bda_y_off);
    }
    
    // Plot the autofocus info if in autofocus mode
    if( autoFocusMode ) {
      //sprintf(afocstr2,"autofoc: %i",(int)autoFocusPosition);
      sprintf(afocstr2,"autofocus: %i",(int)autoFocusStep);
                                
      //TextOut(hdc, client_height/2+FONT_HEIGHT, client_height-3*FONT_HEIGHT, 
      //        afocstr2, (int)strlen(afocstr2));

      TextOut(hdc, FONT_HEIGHT, client_height-3*FONT_HEIGHT, 
              afocstr2, (int)strlen(afocstr2));
    }

    // Plot the motor command if running the motors
    //if( eyeMotor != 0 ) {
    //  if( eyeMotor == FOCUS_MOTOR ) 
    //    sprintf(afocstr1,"set focus: %i",(int)focusPosition);
                                
    //  if( eyeMotor == AP_MOTOR ) 
    //    sprintf(afocstr1,"set aperture: %i",
    //            (int)aperturePosition);
      
      //TextOut(hdc, client_height/2+FONT_HEIGHT, 
      //        client_height-2*FONT_HEIGHT, afocstr1, (int)strlen(afocstr1));

      //TextOut(hdc, FONT_HEIGHT, 
      //        client_height-2*FONT_HEIGHT, afocstr1, (int)strlen(afocstr1));

     // eyeMotor=0;
    //}
    
    // Plot the attitude information
    sprintf(lststr,"LST: %8.4lfh",lst*180./PI/15.);
    sprintf(azstr, "Az: %8.3lfd",az*180./PI);
    sprintf(elstr, "El: %8.3lfd",el*180./PI);
    sprintf(latstr,"Lat: %8.3lfd",lat*180./PI);
    sprintf(lonstr,"Lon: %8.3lfd",lon*180./PI);

    TextOut(hdc, client_width*5/8., client_height-FONT_HEIGHT*6., lststr, 
            (int)strlen(lststr));
                
    TextOut(hdc, client_width*5/8., client_height-FONT_HEIGHT*5., azstr, 
            (int)strlen(azstr));
                
    TextOut(hdc, client_width*5/8., client_height-FONT_HEIGHT*4., elstr, 
            (int)strlen(elstr));
    
    TextOut(hdc, client_width*5/8., client_height-FONT_HEIGHT*3., latstr, 
            (int)strlen(latstr));
    
    TextOut(hdc, client_width*5/8., client_height-FONT_HEIGHT*2., lonstr, 
            (int)strlen(lonstr));
    
    // Mean value of current frame
    sprintf(meanstr,"Mean: %i",(int)Frameblob.get_mapmean());
    TextOut(hdc, FONT_HEIGHT, client_height/2.+FONT_HEIGHT*2., meanstr, 
            (int)strlen(meanstr));     

    // Standard deviation of current frame
    sprintf(stddevstr,"Sigma: %i",(int)Frameblob.get_sigma());
    TextOut(hdc, FONT_HEIGHT, client_height/2.+FONT_HEIGHT*3., stddevstr, 
            (int)strlen(stddevstr));     

    // I am ISC
    sprintf(iscstr,"ISC");
    TextOut(hdc, client_width*3/4.+6, FONT_HEIGHT/2-10, iscstr, 
            (int)strlen(iscstr));
    
    // Exposure time 
    sprintf(expstr,"E: %i",(int)ccd_exposure/1000);
    TextOut(hdc, client_width*3/4., FONT_HEIGHT, expstr, 
            (int)strlen(expstr));     

    // Aperture position
    if(aperturePosition == AP_RANGE) sprintf(apstr,"iris fully open");
    else sprintf(apstr,"ap: %i",aperturePosition);
    
    TextOut(hdc, client_width*3/4., FONT_HEIGHT*2., apstr, 
            (int)strlen(apstr));   

    // Focus position 
    sprintf(focstr,"focus: %i",current_focus);
    TextOut(hdc, client_width*3/4., FONT_HEIGHT*3., focstr, 
            (int)strlen(focstr));   

    // maxslew 
    sprintf(maxslewstr,"%5.2lfd/s",maxSlew*RAD2DEG);
    TextOut(hdc, FONT_HEIGHT, FONT_HEIGHT*3., maxslewstr, 
            (int)strlen(maxslewstr));

    // Plot the current temperature / pressure
    TextOut(hdc, FONT_HEIGHT, FONT_HEIGHT*2., tempstring, 
            (int)strlen(tempstring));
                        
    time_t now;
    
    time( &now );
    
    // Plot the frame number of the current image
    if( saveFrameMode ) sprintf(frameNumStr,"SAVED: %i",frameNum);
    else sprintf(frameNumStr,"NOT SAVED: %i",frameNum);

    TextOut(hdc, FONT_HEIGHT, FONT_HEIGHT, frameNumStr, 
            (int)strlen(frameNumStr));
    EndPaint(hwnd, &ps); 
    return 0; 
    break;
    
  // Process other messages handled by default proc
  default: return DefWindowProc(hwnd, uMsg, wParam, lParam); break;
  }                 
                  
  return 0; 
}

// Register Eye window class and create it (but don't display)
// return 1 for success, 0 on failed register, -1 on failed create

int initEyeWin() {
  WNDCLASS wc; 
  HINSTANCE hinstance=GetModuleHandle(NULL);
  
  wc.style = CS_HREDRAW | CS_VREDRAW; 
  wc.lpfnWndProc = (WNDPROC) MainWndProc; 
  wc.cbClsExtra = 0; 
  wc.cbWndExtra = 0; 
  wc.hInstance = hinstance; 
  wc.hIcon = LoadIcon(NULL, IDI_APPLICATION); 
  wc.hCursor = LoadCursor(NULL, IDC_ARROW); 
  wc.hbrBackground = (HBRUSH) GetStockObject(BLACK_BRUSH); 
  wc.lpszMenuName =  NULL;  
  wc.lpszClassName = "MainWinClass"; 
  
  if (!RegisterClass(&wc)) {
    printf("Couldn't register window class.\n");
    return 0;
  }

  // Create the window (the handle is hwndEye)
  
  hwndEye = CreateWindow( 
                         "MainWinClass",          // class name 
                         "--=eViL eYe v2.1b=--",  // window title
                          WS_OVERLAPPEDWINDOW,    // overlapped window
                         EYE_LEFT,                // default hor. pos.  
                         EYE_TOP,                 // default vert pos.    
                         EYE_WIDTH,               // default width
                         EYE_HEIGHT,              // default height
                         NULL,                    // no parent/owner window    
                         NULL,                    // class menu used
                         hinstance,               // instance handle
                         NULL);

  if ( !hwndEye) {
    printf("Couldn't create the window.\n");
    return -1;
  }
        
  memset(dispBuffer,0,CCD_X_PIXELS*CCD_Y_PIXELS); // Init the image buffer
  
  return 1;
}


// --- Entry Point -----------------------------------------------------------

int main( int argc, char **argv ) {
  DWORD thId,result;
  FILE *fnumlog;
  FILE *settingsfile;
  char thisline[255];        
  int i;

  // **LORENZO** for ground testing purposes: loads and plugs into the frames
    // additional white noise read from "background.dat"
	if(BACKGROUND_KLUDGE) {
		FILE *ep_fs;
		char ep_line[256];
		int ep_idx = 0;
		ep_fs = fopen(backgroundfilename, "r");
		if(ep_fs == NULL) {
			printf("Unable to open file background.dat\n");
			exit(0);
		}
		while(fgets(ep_line, 80, ep_fs)!= NULL) {
			ep_background[ep_idx++] = (unsigned short)atof(ep_line);
		}
	}
  
  // a thread for debugging
  
  // DWORD thId2, junk;
  //thTemp=CreateThread( NULL,0,debugme,&junk,0,&thId2);
  
  // KLUDGE: Although we want to read the settings file _after_ trying to
  // init the camera (so that we know which CCD we're using), the first
  // parameters in settings.cam needs to be read to determine if a camera is
  // being used at all

  if( (settingsfile = fopen( settingsfilename, "r" )) == NULL )
    return 0;
        
  fgets(thisline,80,settingsfile); sscanf(thisline,"%i",&NO_CAMERA);
  fclose(settingsfile);        

  // If NO_CAMERA, kludge server settings. Else determined after
  // camera init
        
  if( NO_CAMERA == 1 ) {
    I_AM_ISC = 1;
    CCD_X_PIXELS = ISC_CCD_X_PIXELS;
    CCD_Y_PIXELS = ISC_CCD_Y_PIXELS;
  }
  
  if( NO_CAMERA == 2 ) {
    I_AM_ISC = 0;
    CCD_X_PIXELS = OSC_CCD_X_PIXELS;
    CCD_Y_PIXELS = OSC_CCD_Y_PIXELS;
  }

  // Server defaults
  pointing_quality = 0;
  pointing_nbad = 0;

  // Get the start time of the server
  time( &server_start );
  last_time = server_start-1000;   // initial value for the last_time
  last_save = server_start;
  lastCmdRec = server_start;
  lastTrigRec = server_start;
  
  // Initialize the exposure timer to 0
  exposureFinished.wDay=0;
  exposureFinished.wDayOfWeek=0;
  exposureFinished.wHour=0;
  exposureFinished.wMilliseconds=0;
  exposureFinished.wMinute=0;
  exposureFinished.wMonth=0;
  exposureFinished.wSecond=0;
  exposureFinished.wYear=0;
  
  // Start entry in the server log
  server_log(1);
  
  // initialize WinIO
  if( InitializeWinIo() != 1 ) {
    printf("WinIO didn't initialize.\n");
    return 31;
  }

  // Cycle the camera / thermometer power
  // Initialize the camera driver
  // Initialize the camera
  if( NO_CAMERA == 0 ) {
    if( init_camera() == -1 ) return -1;
    
    if( CCD_X_PIXELS == ISC_CCD_X_PIXELS ) I_AM_ISC = 1;
    else I_AM_ISC = 0;
  }

  printf("I_AM_ISC = %i\n",I_AM_ISC);

  unsigned long xpix=CCD_X_PIXELS, ypix=CCD_Y_PIXELS;
  FrameSize=xpix*ypix*2;
        
  // Allocate memory for the frame buffers 
  frameBuf1 = new unsigned char[FrameSize];
  frameBuf2 = new unsigned char[FrameSize];
  
  QCFrame.pBuffer = frameBuf1; //new unsigned char[FrameSize];
  QCFrame.bufferSize = FrameSize;
  QCFrame.width=xpix;
  QCFrame.height=ypix;
  
  printf("Allocated image buffer: %i x %i, %i bytes\n",
         CCD_X_PIXELS, CCD_Y_PIXELS, FrameSize );
  
  //if( I_AM_ISC ) QCFrame.bits=14;
  //else QCFrame.bits=12;
  
  // Get the default gain and offset settings
  QCam_GetParam( &settings, qprmNormalizedGain,&default_gain );
  QCam_GetInfo( camhandle, qinfNormGainSigFigs,&gain_res );
  QCam_GetParamS32( &settings, qprmS32AbsoluteOffset,&default_offset );
  
  printf("Default GAIN=%li GAINRES=%li OFFSET=%li\n",default_gain,gain_res,
         default_offset);

  // Initialize frame blob (shares image buffer with QCFrame)
  // Do this AFTER initializing the camera
  Frameblob.commonconstructor( (MAPTYPE *) QCFrame.pBuffer, xpix, ypix, 14, 
                               (double)0.00194 );
        
  // Read in the settings file
  if( !read_settings() ) {
    printf("Couldn't read in settings\n\n");
    return -2;
  }

  // Set the execCmd client frame to the server defaults
  execCmd.exposure=ccd_exposure;
  execCmd.focusOffset = focusOffset;
  execCmd.eyeOn = eyeOn;

  execCmd.sn_threshold = sn_threshold;
  execCmd.grid = grid;
  execCmd.mult_dist = (int) Frameblob.get_disttol();

  execCmd.gain = rel_gain;
  execCmd.offset = rel_offset;

  execCmd.useLost = useLost;
  execCmd.minBlobMatch = minBlobMatch;
  execCmd.maxBlobMatch = maxBlobMatch;
  execCmd.mag_limit = mag_limit;
  execCmd.norm_radius = norm_radius;
  execCmd.lost_radius = lost_radius;
  execCmd.tolerance = tolerance;
  execCmd.match_tol = match_tol;
  execCmd.quit_tol = quit_tol;
  execCmd.rot_tol = rot_tol;  

  execCmd.lat = lat;

  // Intialize the star catalogue
  
  printf( "Attempting to use star catalogue: %s\n", catpath );
  astro_init_catalogue(catpath, catalogname, katalogname);

  // Initialize the temp./pressure/heater routines
  
  if( tempSetup(tempControl,tempSleeptime,tempSetLimit,tempOffset, 
                tempPressuregain,tempPressureoffset) == 0 )  {
    printf("Temperature stuff couldn't open config file...\n");
  } else printf("Temperature stuff initialized AOK..\n");
  tempstring[0] = NULL;

  
  // Initialize the current frame number
  if( (fnumlog = fopen(framenumlogname,"r")) == NULL )
    frameNum = -1;
  else {        
    fscanf(fnumlog,"%i",&frameNum);
    fclose(fnumlog);
    if( frameNum < 0 ) frameNum = 0;
  }

  // Set the gain/offset for the blob finding AFTER getting the
  // default values from the camera init, and reading the relative
  // gain/offset from the settings file

  // these gains/offsets are for the source detection NOISE MODEL
  Frameblob.set_gain(noiseGain*rel_gain); 
  Frameblob.set_readout_offset(noiseOffset+rel_offset);
  Frameblob.set_readout_noise(0); // superfluous - constant included in offset

  if( I_AM_ISC ) Frameblob.load_badpix(iscbadpixfilename);
  else Frameblob.load_badpix(oscbadpixfilename);

  Frameblob.set_maxblobs(1000); // set this higher than in frames b/c
                                // the algorithm also finds spurious
                                // detections before being culled to
                                // make the final linked list of
                                // extended sources


  // Initialize stepper motors, assuming they are in the position
  // indicated in the stepper log (otherwise assumed to be at the
  // maximum (parked) positions.

  printf("Calibrating stepper motors...\n");
  printf("Aborting any previous motor commands...\n");
  motorabort(comport,AP_MOTOR);
  motorabort(comport,FOCUS_MOTOR);
  calibrate_motors();
  
  // We tried to increase the current/acceleration and decrease the velocity of the focus stepper
  // motor because it was unable to drive the focus mechanism. Apparently, this does not help.
  //motorcmd(comport,FOCUS_MOTOR,"m80");
  //motorcmd(comport,FOCUS_MOTOR,"l80");
  //motorcmd(comport,FOCUS_MOTOR,"L20");
  //motorcmd(comport,FOCUS_MOTOR,"V5");
  //motorcmd(comport,FOCUS_MOTOR,"v50");

  // Initialize the server_data frame
  server_data.n_blobs=0; // start with 0 blobs
  server_data.temp1=0;
  server_data.temp2=0;
  server_data.temp3=0;
  server_data.temp4=0;
  server_data.pressure1=0;
  server_data.diskspace=-1;
  server_data.autofocusOn=0;

  // Initialize sockets and thread variables
  for(i=0;i<NCLIENTS;i++) {
    connected_sockets[i]=NULL; 
    thConnect[i]=NULL; thConnectParam[i]=0; thConnectState[i]=0;
    thRecv[i]=NULL; thRecvParam[i]=0; thRecvState[i]=0;
    thCmd=NULL; thCmdParam=0; thCmdState=0;
    thSend[i]=NULL; thSendParam[i]=0; thSendState[i]=0;        
    
    try {
      listen_sockets[i] = new CSocket((int) SERVER_PORTS[i]);
      //printf("Started listening on port %i\n",SERVER_PORTS[i]);
    }
        
    catch( CSocketException exception ) {
      //printf("Couldn't listen on port %i\n",SERVER_PORTS[i]);
      return -3;
    }
  }
        
  //--- Setup for the evil eye window -----------------------------------------
  dispBuffer = new char[FrameSize];
  
  BitMapInfo = (BITMAPINFO *)malloc(sizeof(BITMAPINFOHEADER) + 
                                    (256 * sizeof(RGBQUAD)));
  BitMapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER); 
  BitMapInfo->bmiHeader.biPlanes = 1; 
  BitMapInfo->bmiHeader.biBitCount = 8; 
  BitMapInfo->bmiHeader.biCompression = BI_RGB; 
  BitMapInfo->bmiHeader.biSizeImage  = 0; 
  BitMapInfo->bmiHeader.biXPelsPerMeter = 0; 
  BitMapInfo->bmiHeader.biYPelsPerMeter = 0; 
  BitMapInfo->bmiHeader.biClrUsed = 0;
  BitMapInfo->bmiHeader.biClrImportant = 0;
  
  // Load a greyscale palette
  for ( i=0; i < 256; ++i) {
    BitMapInfo->bmiColors[i].rgbRed = i; 
    BitMapInfo->bmiColors[i].rgbGreen = i; 
    BitMapInfo->bmiColors[i].rgbBlue = i;
    BitMapInfo->bmiColors[i].rgbReserved=0;
  }

  // Set up the fonts
  hfntBold = CreateFont( FONT_HEIGHT,  // height of font
                         FONT_WIDTH,   // average character width
                         0,            // angle of escapement
                         0,            // base-line orientation angle
                         FW_NORMAL,    // font weight
                         FALSE,        // italic attribute option
                         FALSE,        // underline attribute option
                         FALSE,        // strikeout attribute option
                         DEFAULT_CHARSET,   // character set identifier
                         OUT_DEFAULT_PRECIS,  // output precision
                         CLIP_DEFAULT_PRECIS, // clipping precision
                         DEFAULT_QUALITY,     // output quality
                         DEFAULT_PITCH | FF_DONTCARE, // pitch and family
                         NULL          // typeface name
                         );

  hfntSmall = CreateFont( FONT_HEIGHT*3./4.,// height of font
                          FONT_WIDTH*3./4., // average character width
                          0,                // angle of escapement
                          0,                // base-line orientation angle
                          FW_NORMAL,        // font weight
                          FALSE,            // italic attribute option
                          FALSE,            // underline attribute option
                          FALSE,            // strikeout attribute option
                          DEFAULT_CHARSET,  // character set identifier
                          OUT_DEFAULT_PRECIS, // output precision
                          CLIP_DEFAULT_PRECIS,// clipping precision
                          DEFAULT_QUALITY,  // output quality
                          DEFAULT_PITCH | FF_DONTCARE, // pitch and family
                          NULL             // typeface name
                          );


  xRoi = CCD_X_PIXELS/2; // Set region of interest to centre of CCD by default
  yRoi = CCD_Y_PIXELS/2;
        
  if( initEyeWin() != 1 ) {
    return 0;
  }
  MSG winMsg;   // window message handling
        
  ShowWindow(hwndEye, SW_SHOWDEFAULT); 
  UpdateWindow(hwndEye);
  
  // Wait for a client to connect on any port
  //printf("Camera running... waiting for a client to connect.\n");
  
  // --- Main command loop --------------------------------------
  
  // Run the main loop until quit is set
  
  time_t now;
  while( !quitflag ) {
    // Update the status of the connected sockets
    update_connections(1);
    
    // Check for new frames from the clients
    update_receive();
    
    if( shutdownFlag != 0 ) quitflag = 1;

    // Update the ISC command execution thread. If a command just finished,
    // call timeout_send to broadcast the result to all connected clients.
    if( update_command(abortFlag) == 2 ) {
      timeout_send();

      // Once the frame is sent, OK to start exposing a new frame
      if( thExpState == 3 ) thExpState = 0;
    }

    // Handle exposures
    reExpose();

    // Handle window events
    if( eyeOn ) {
      
      if( eyeRefresh) 
        InvalidateRgn(hwndEye,NULL,(eyeMode!=full)&&(lastMode==full));
      
      if( PeekMessage(&winMsg, (HWND) hwndEye, 0, 0, PM_REMOVE) != 0) {
        TranslateMessage(&winMsg); 
        DispatchMessage(&winMsg); 
      }
    }
    
    // check for keystroke to quit
    if( _kbhit() != 0 ) {
      printf("Exiting due to keystroke...\n");
      _getch(); // to empty the keystroke buffer                
      quitflag = 1;
    }
    
    // Kludge for memory leak - automatically quit when
    // ISC_SERVER_RESTART seconds have elapsed since server_start
    // MEMEORY LEAKS FIXED - turned off now

    time( &now );
    //if( (now - server_start) > ISC_SERVER_RESTART ) quitflag = 1;

    // Manage the temperature / pressure sensor + heater thread
    if( (thTempState == 0) ) {
      thTemp=CreateThread( NULL,0,th_doTemp,&thTempParam,0,&thId);
      thTempState = 1;
    } else {
      result = WaitForSingleObject(thTemp,0);
      if( result != WAIT_TIMEOUT ) {       // thread is finished
        CloseHandle(thTemp);
        thTempState = 0;
      }
    }

    //if( thExpState == 2 ) printf(" 2! ");
    
    Sleep(10);  // Sleep to give other processes some time

  }

  // --- Clean Up -----------------------------------------------

  // Close all the sockets and running threads
  for(i=0;i<NCLIENTS;i++) {

    // Kill the listening sockets
    delete listen_sockets[i];

    // If active connection sockets, kill them
    if( thConnectState[i] == 1 ) delete connected_sockets[i];
                
    // If threads running, wait two seconds for them to exit normally,
    // otherwise terminate them    

    if( thConnectState[i] == 2 ) {
      if( WaitForSingleObject(thConnect[i],2000) == WAIT_TIMEOUT ) 
        TerminateThread(thConnect[i],0);
      else CloseHandle(thConnect[i]);
    }

    if( thRecvState[i] == 1 ) {
      if( WaitForSingleObject(thRecv[i],2000) == WAIT_TIMEOUT ) 
        TerminateThread(thRecv[i],0);
      else CloseHandle(thRecv[i]);
    }
                
    if( thSendState[i] == 1 ) {
      if( WaitForSingleObject(thSend[i],2000) == WAIT_TIMEOUT ) 
        TerminateThread(thSend[i],0);
      else CloseHandle(thSend[i]);
    }
  }

  // stop the exposure thread
  if( thExpState != 0 ) {
    thExpState = 2;
    abortFlag = 1;            // request pointing solutions/autofocus to abort
    QCam_Abort(camhandle);    // tell camera to stop taking current exposure
    
    // Give thread 20 seconds to exit before termination 
    if( WaitForSingleObject(thExp,20000) == WAIT_TIMEOUT ) 
      TerminateThread(thExp,0);
    else CloseHandle(thExp);
  }

  if( thCmdState == 1 ) {
    // Give thread 20 seconds to exit before termination 
    if( WaitForSingleObject(thCmd,20000) == WAIT_TIMEOUT ) 
      TerminateThread(thCmd,0);
    else CloseHandle(thCmd);
  }

  // Stop the temp sensor thread
  if( thTempState == 1 ) {
    if( WaitForSingleObject(thTemp,2000) == WAIT_TIMEOUT ) 
      TerminateThread(thTemp,0);
    else CloseHandle(thTemp);        
  }

  // Turn off the heater relay
  tempShutdown();

  // Shut down the camera, free image buffers
  if( NO_CAMERA == 0 ) {
    QCam_CloseCamera(camhandle); 
    QCam_ReleaseDriver();
  }

  // Close the star catalogue
  astro_close_catalogue();

  // kill the debug thread
  //TerminateThread(thDebugme,0);
  
  //delete [] QCFrame.pBuffer;
  delete [] frameBuf1;
  delete [] frameBuf2;
  delete [] dispBuffer;
  
  // Unallocate other resources
  free(BitMapInfo);
  
  // Shut down WinIO
  ShutdownWinIo();
  
  // Shut down the computer if requested --------------------------------------
  if( shutdownFlag != 0 ) {

    // log what we are doing
    if( shutdownFlag == 1 ) server_log( 5 );  // shutdown
    if( shutdownFlag == 2 ) server_log( 6 );  // reboot
    if( shutdownFlag == 3 ) server_log( 4 );  // power cycle the camer
    
    HANDLE token;
    TOKEN_PRIVILEGES privileges;
    
    // Get the current process token handle...
    if( !OpenProcessToken( GetCurrentProcess(), 
                           TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &token ))
      printf( "ERROR: Unable to open process token.\n" );

    // Get the LUID for shutdown privilege...
    LookupPrivilegeValue( NULL, SE_SHUTDOWN_NAME, &privileges.
                          Privileges[ 0 ].Luid );

    // Set parameters for AdjustTokenPrivileges...
    privileges.PrivilegeCount = 1;
    privileges.Privileges[ 0 ].Attributes = SE_PRIVILEGE_ENABLED;
    
    // Enable shutdown privilege...
    AdjustTokenPrivileges( token, FALSE, &privileges, 0, 
                           (PTOKEN_PRIVILEGES)NULL, 0 );

    if( GetLastError() != ERROR_SUCCESS ) 
      printf("ERROR: Unable to adjust token privileges.\n" );
    
    if( shutdownFlag == 1 ) 
      ExitWindowsEx(EWX_SHUTDOWN | EWX_FORCE,0);

    if( shutdownFlag == 2 ) 
      ExitWindowsEx(EWX_REBOOT | EWX_FORCE,0);
  }
  
  return 0;
}
