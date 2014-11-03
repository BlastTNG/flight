/*
iscclient: 

		Version 0.5 - New commanding scheme

		Version 0.4 - Kept up to date with netisc V 0.4

		Version 0.2 - August 10, 2003

			-Motor commands are now in steps, not in normalized coordinates (changed
			netisc.h AND isc_protocol.h)

			-client exits if incomplete frame received (server disconnected)	

		Version 0.1 - June 13, 2003
  
		  test client for the ISC server

		  Please keep Ed Chapin (echapin@inaoep.mx) posted whenever the
		  code is updated!

*/

// remember to link wsock32.lib

#include <windows.h>
#include <stdio.h>
#include <time.h>
#include "isc_protocol.h"
#include "csocket.h"

#define PI 3.14159265358979

char *SERVER_IP;
int SERVER_PORT;
int quitflag;

//FILE *logfile;
//char *logfilename;

ISCStatusStruct client_data;			    // data we get back from the ISC
ISCSolutionStruct server_data;				// data (commands) we send to the ISC
CSocket *isc_socket;						// socket for TCP/IP

HANDLE thRecv;
DWORD thRecvParam;
int thRecvState=0;                          // 0=not running, 1=receiving

HANDLE thInput;
DWORD thInputParam, thInputId;
int thInputState=0;							// 0=not running, 1=running

// Thread function for receiving frames. Will not return until frame received or error.
// Input: Nothing
// Output: # bytes received for success, 0 for receive error.
DWORD WINAPI receive_frame( LPVOID parameter )
{
	DWORD par;
	int nbytes;
	par = *((DWORD *)parameter);
	
	printf(": Receive Frame\n> ");
	
	nbytes = isc_socket->Read(&server_data,sizeof(server_data));
	printf("Received %i / %i bytes from server.\n",nbytes,sizeof(server_data));

	if( nbytes > 0 )
	{
		*((DWORD *)parameter) = nbytes;
		return nbytes;
	}
	else
	{
		*((DWORD *)parameter) = 0;
		return 0;
	}
}

// Start receive thread / check completion
// Return:
//         -2 no data received in last frame (client probably disconnected)
//		   -1 thread creation error
//			0 no new frames received
//			1 frames received

int update_receive( void )
{
	int retval=0;
	DWORD thId, result;

	// Start new receive thread if not running
	if (thRecvState == 0 ) 
	{
		thRecvParam = 0; 
		thRecv=CreateThread( NULL,0,receive_frame,&thRecvParam,0,&thId);
		if(thRecv==NULL) return -1;
		thRecvState = 1;
	}
			
	// otherwise check completion of the receive thread
	else
	{
		result = WaitForSingleObject(thRecv,0);
		if( result != WAIT_TIMEOUT )	// thread has finished
		{
			thRecvState = 0;
			CloseHandle(thRecv);
				
			// Check that a frame was received 
			if( thRecvParam > 0) 
			{
				if( thRecvParam == sizeof(server_data) ) // whole frame arrived
				{
					retval = 1;
					printf("Received frame from server.\n> ");
				}
			}
			else 
			{
				retval = -2;
				printf("Incomplete frame from server on port: %i bytes.\n> ",thRecvParam);
			}
		}
	}

	return retval;
}

// Input thread ---------------------------------------------------

// Input: Nothing
// Output: client_frame is filled with new command.
// par = 1 if frame should be sent to the server
// par = 0 if quit
DWORD WINAPI user_input( LPVOID parameter )
{
	DWORD par;
	par = *((DWORD *)parameter);
	int cmd;
	double par1=0,par2=0,par3=0,par4=0;
	char thisline[255];

	printf("(0) Input Az, El (deg)\n");
	printf("(1) Input LST (hr)\n");
	printf("(2) Input LAT (deg)\n");
	printf("(3) Toggle picture taking\n");
	printf("(4) Toggle save images\n");
	printf("(5) Set Focus to Home position (-1) or step from current position +/- steps\n");
	printf("(6) Set Absolute Aperture (0-%i)\n",AP_RANGE); 
	printf("(7) Set Mag Limit\n");
	printf("(8) Set star association tolerance (arcsec) \n");
	printf("(9) Set normal and lost search radii (degrees)\n");
  printf("(10) Set S/N threshold \n");
	printf("(11) Set display to ROI mode (blob #)\n");
	printf("(12) Set display to ROI mode about pixel (x, y)\n");
	printf("(13) Set display to full screen mode\n");
	printf("(14) Toggle abort state\n");
	printf("(15) Toggle autofocus state\n");
	printf("(16) Set max blobs in a search\n");
	printf("(17) Toggle bright star at RA (hr) DEC (deg)\n");
	printf("(18) Set CCD offset to az, el (deg)\n");
	printf("(19) Set exposure time (us)\n");
	printf("(20) Set relative CCD gain (>0)\n");
	printf("(21) Set relative CCD offset (us)\n");
	printf("(22) Set relative focus offset to home position (usually 0)\n");
	printf("(23) Set trigger mode (0=self,1=edge,2=+pulse,3=-pulse)\n");
	printf("(24) Eye update (0=off,1=on)\n");
	printf("(40) Command ISC to: 1=shutdown 2=reboot\n");
  printf("(41) Set Hold Current for stepper motors (0-50)\n"); 
	printf("(50) Quit (disconnect client)\n> ");
	gets(thisline);
	sscanf(thisline,"%i %lf %lf %lf %lf",&cmd,&par1,&par2,&par3,&par4);

	switch(cmd)
	{
		case 0: client_data.az=par1*PI/180.; client_data.el=par2*PI/180.; break; 
		case 1:	client_data.lst=par1*15*PI/180.; break;
		case 2: client_data.lat=par1*PI/180.; break;
		case 3: client_data.pause=!client_data.pause; break;
		case 4: client_data.save=!client_data.save; break;
		case 5: client_data.focus_pos=(int)par1; break;
		case 6: client_data.ap_pos=(int)par1; break;	
		case 7: client_data.mag_limit=par1; break;
		case 8: client_data.tolerance=par1/3600.*PI/180.; break;
		case 9: client_data.norm_radius=par1*PI/180.; client_data.lost_radius=par2*PI/180.; break;
		case 10: client_data.sn_threshold=par1; break;
		case 11: client_data.display_mode=blob; client_data.blob_num=(int)par1; break;
		case 12: client_data.display_mode=roi; client_data.roi_x=(int)par1; client_data.roi_y=(int)par2; break;
		case 13: client_data.display_mode=full; break;
		case 14: client_data.abort=!client_data.abort; break;
		case 15: client_data.autofocus=!client_data.autofocus; break;
		case 16: client_data.maxBlobMatch=(int)par1; break;
		case 17: client_data.brightStarMode=!client_data.brightStarMode; client_data.brightRA=par1*15.*PI/180.; 
				 client_data.brightDEC=par2*PI/180.; break;
		case 18: client_data.azBDA=par1*PI/180.; client_data.elBDA=par2*PI/180.; break;
		case 19: client_data.exposure=(int)par1; break;
		case 20: client_data.gain=par1; break;
		case 21: client_data.offset=(int)par1; break;
		case 22: client_data.focusOffset=(int)par1; break;
		case 23: client_data.triggertype=(int)par1; break;
		case 24: client_data.eyeOn=(int)par1; break;
		case 40: client_data.shutdown=(int)par1; break;
    case 41: client_data.hold_current=(int)par1; break;	
		case 50: quitflag=1; break;
	}

	//printf("\n\nInput data: %f %f %f\n",par1,par2,par3);
	//printf("Frame: %i %i %i\n",client_data.par1,client_data.par2,client_data.par3);

	if( (cmd>=0) && (cmd<=24) )
	{
		*((DWORD *)parameter) = 1;
		return 1;
	}
	else
	{
		*((DWORD *)parameter) = 0;
		return 0;
	}
}


//--------------------------------------------------------------------------------------------------------------

void print_settings( ISCStatusStruct *data )
{
	printf("Server State:\n");
	printf("p=%i s=%i foc=%i ap=%i abt=%i MCPFr=%i autofoc=%i\n",
			data->pause,data->save,data->focus_pos,data->ap_pos,data->abort,data->MCPFrameNum,data->autofocus);
	printf("az=%6.2lf el=%6.2lf lat=%6.2lf lst=%5.2lf\n",
			data->az*180/PI, data->el*180/PI, data->lat*180/PI, data->lst*180/PI/15);
	printf("thresh=%6.2lf grid=%i mdist=%i\n",
			data->sn_threshold, data->grid, data->mult_dist );
	printf("mag=%6.2lf nrad=%6.2lf lrad=%6.2lf tol=%6.2lf mtol=%3.1lf qtol=%3.1lf rtol=%6.2lf\n",
			data->mag_limit, data->norm_radius*180/PI, data->lost_radius*180/PI, data->tolerance*3600*180/PI, 
			data->match_tol, data->quit_tol, data->rot_tol*180/PI);
}

void print_pointing( ISCSolutionStruct *data )
{
	int i, maxblob;

	maxblob = data->n_blobs;
	if( maxblob > MAX_ISC_BLOBS ) maxblob=MAX_ISC_BLOBS;
	
	double t;

	printf("Server Response:\n");
	for( i=0; i<maxblob;i++ )
	{
		t = (double)clock()/(double)CLOCKS_PER_SEC;	
		printf( "x: %8.2lf y: %8.2lf flux: %i s/N: %lf\n",data->blob_x[i],data->blob_y[i],data->blob_flux[i],data->blob_sn[i] );
	}

	printf("fr=%i mcpfr=%i mean=%8.2lf blobs=%i ra=%6.2lf dec=%6.2lf +/- %lf\n",data->framenum, data->MCPFrameNum, 
		data->mapMean, maxblob, data->ra*180/PI/15, data->dec*180/PI, data->sigma*3600*180/PI);
	//printf("%f %f %f %f %f\n",data->temp1,data->temp2,data->temp3,data->temp4,data->pressure1);


	printf("> ");
}

int main( int argc, char **argv )
{
	DWORD result;
	int responseMessage;
	int nbytes;

	// default settings
	client_data.abort = 0;
  client_data.pause = 0;
	client_data.save = 0;
	client_data.autofocus = 0;
	client_data.focus_pos = 0; 
	client_data.ap_pos = 495;
	client_data.MCPFrameNum = 5;
	client_data.shutdown = 0;
	client_data.hold_current = 0;
  client_data.exposure = 200000l;
	client_data.focusOffset = 0;
  client_data.triggertype = 0;
	client_data.eyeOn = 1;
  client_data.useLost = 1;

  client_data.display_mode = full;
  client_data.roi_x = 512;
  client_data.roi_y = 512;
  client_data.blob_num = 0;
  client_data.azBDA = 0.;
	client_data.elBDA = 0.;

	client_data.az = 0;
	client_data.el = 0;
	client_data.lst = 0;
  client_data.lat = 51.478; 
	
  client_data.brightStarMode = 0;
  client_data.brightRA = 0;
  client_data.brightDEC = 0;

  client_data.sn_threshold = 4.5;
	client_data.grid = 38;       
	client_data.mult_dist = 30;  
	
	client_data.gain = 1.;
	client_data.offset = 0;

	client_data.minBlobMatch = 3;
  client_data.maxBlobMatch = 7;
  client_data.mag_limit = 9.;
	client_data.norm_radius = 2 * PI/180;
	client_data.lost_radius = 5 * PI/180; 
	client_data.tolerance = 20. / 3600.*PI/180.;
	client_data.match_tol = 0.5;
	client_data.quit_tol = 1.0;
  client_data.rot_tol = 5. * PI/180;

	// parse command line
	if( argc != 3 )
	{
		printf("isc client usage:\n\n iscclient <ip address> <port>\n");
		return -10;
	}
	else
	{
		SERVER_IP=argv[1];
		sscanf(argv[2],"%i",&SERVER_PORT);
	}

	// Connect
	printf("Connecting to %s:%i...\n",SERVER_IP,SERVER_PORT);

	try
	{
		isc_socket=new CSocket(SERVER_IP,SERVER_PORT);
		printf("Connected to server.\n");			
		
	}
	catch( CSocketException se)
	{
		printf(" : Exception => %s\n",se.getText()); 
		return -1;
	}

	// Start command thread
	thInput = CreateThread( NULL,0,user_input,&thInputParam,0,&thInputId);

	// Main while loop
	quitflag = 0;
	while( !quitflag )
	{
		// See if new command for sending
		Sleep(10);
	
		result = WaitForSingleObject(thInput,0);
		if( result != WAIT_TIMEOUT )	// thread has finished
		{
			CloseHandle(thInput);

			// send to server
			if( thInputParam == 1 )
			{
				nbytes = isc_socket->Write(&client_data,sizeof(client_data));
				printf("Sent %i / %i bytes to server.\n",nbytes,sizeof(client_data));
			}
			//reset focus_pos
			client_data.focus_pos = 0;

			// re-start input thread
			thInput = CreateThread( NULL,0,user_input,&thInputParam,0,&thInputId);
      
			// display the server settings
			//print_settings(&client_data);
      printf("Done\n");
		}
		
		// get the response
		//responseMessage = 0;
    responseMessage = update_receive();
		if( (responseMessage == 1) ) 
		{
			print_pointing(&server_data);
		}
		if( responseMessage == -2 ) 
		{
			printf("Server probably disconnected.\n");
			quitflag=1;
		}	
	}

	// Clean up
	isc_socket->Close();
	delete isc_socket;
}