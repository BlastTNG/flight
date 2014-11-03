

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <qapplication.h>
#include <pthread.h>
#include "mycam.h"
#include "blobimage.h"
#include "sbigudrv.h"
#include "csbigimg.h"
#include "bloblist.h"
#include "clensadapter.h"
#include "frameblob.h"
#include "pyramid.h"
#include "imageviewer.h"

#define TEST_MODE 3
//TEST_MODES: 1 = noise calculation, 2 = adapter/focus test, 3 = blob finder test / box drawer
//            4 = use "testing" main function (takes picture, processes, etc.)
//            5 = FIFO test

using namespace std;

#if TEST_MODE == 1 || TEST_MODE == 2 || TEST_MODE == 3
///////////////////////////////////////////////////////////////////////////////////////////////////
//  Old test program                                                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////

#if TEST_MODE == 3
//function for calling image viewer in a thread
void* viewerExec(void* arg)
{
	QApplication* a = (QApplication*)arg;
	static int res = a->exec();
	return (void *)&res;
}
#endif

int main(int argc, char *argv[])
{
	ifstream sbigfile;
#if TEST_MODE == 2
	MyCam cam;
	LENS_ERROR err = LE_NO_ERROR;
	PAR_ERROR cerr = CE_NO_ERROR;
	CLensAdapter* lens = cam.getLensAdapter();
	string lens_return_str;
#elif TEST_MODE == 3
	ImageViewer *iv;
	QApplication a(argc, argv);
	int firstTime = 1;
	pthread_t app_thread;
	bloblist *blobs;
#endif
	cout << "Initializing BlobImage object (loading catalog takes a while)..." << endl;
	BlobImage img;
	frameblob* fblob = img.getFrameBlob();
		
	do { // allow break out
		
#if TEST_MODE == 1
		string start_str = "/data/rawdir/";
		string end_str = ".sbig";
		string times[] = {"0.04", "0.08", "0.15", "0.5", "0.75", "1", "2", "4", "8", 
			"15", "30", "60"};
		int num_times = 12;
		cout << "time\tmean\tsigma2" << endl;
		for (int i=0; i<num_times; i++)
		{
			string filename = start_str + times[i] + end_str;
			if (img.OpenImage(filename.c_str()) != SBFE_NO_ERROR)
			{
				cout << "An error occured while opening the file: " << times[i] << end_str << endl;
				break;
			}
			count_lock
			fblob->calc_stddev();
			cout << times[i] << "\t" << fblob->get_mapmean() << "\t" << fblob->get_stddev() << endl;
		}
		
#elif TEST_MODE == 2
 		cout << "Opening Camera Driver..." << endl;
		if ((cerr = cam.OpenDriver()) != CE_NO_ERROR)
			break;
		cout << "Opening Camera Device..." << endl;
		if ((cerr = cam.OpenUSBDevice(0)) != CE_NO_ERROR)
			break;
		cam.SetExposureTime(0.1);
		cout << "Establishing a Link to the USB Camera...(4 tries)" << endl;
		for (int i=0; i<4; i++) {
			cout << "...attempt " << i + 1 << endl;
			if ((cerr = cam.EstablishLink()) == CE_NO_ERROR) break;
		}
		if (cerr != CE_NO_ERROR) break;

		int position = 0;
// 		cout << "Opening a connection to the lens adapter..." << endl;
// 		if ((err = lens->openConnection("/dev/ttyACM0")) != LE_NO_ERROR)
// 			break;
// 		
// 		cout << "Finding focal range..." << endl;
// 		if ((err = lens->findFocalRange()) != LE_NO_ERROR) 
// 			break;
// 		position = lens->getFocalRange();
// 		cout << "...Focal Range is: " << position << endl;
		
// 		int remaining;
// 		cout << "Testing forced move..." << endl;
// 		lens->preciseMove(-1500, remaining, 1);
// 		cout << "...after move of -1500 there are " << remaining << " counts left" << endl;
// 		lens->preciseMove(-1000, remaining, 1);
// 		cout << "...after move of -1000 there are " << remaining << " counts left" << endl;
// 		
// 		return 0;
// 		
		int numPoints = 20;
		int step = position/300;
		int remaining = 0;
		int flux;
		lens->setFocusTol(1);
		
		cout << "Finding the brightest blob intensity at " << numPoints << " points..." << endl;
		cout << "\nPosition\tFlux" << endl;
		cout << "========================" << endl;
		for (int i=0; i<numPoints; i++) {
			if ( (cerr = cam.GrabImage(&img, SBDF_LIGHT_ONLY)) != CE_NO_ERROR )
				break;
			img.findBlobs();
			if (fblob->get_numblobs())
				flux = fblob->getblobs()->getflux();     //flux of brightest blob
			else
				flux = -1;
			cout << position << "\t\t" << flux << endl;
			
			lens->preciseMove(-step+remaining, remaining);
			position -= step + remaining;
		}
		if (cerr != CE_NO_ERROR) break;
		
#elif TEST_MODE == 3
		string path = "/data/rawdir/03-01/";
/*		string names[] = {"1153prog.sbig", "1140ops.sbig", "1143prog.sbig", "1144prog.sbig",
			"1150prog.sbig", "1155prog.sbig", "1159prog.sbig", "1159bprog.sbig", "1200prog.sbig",
			"1200bprog.sbig", "1211ops.sbig", "1213prog.sbig", "1213bprog.sbig", "1214prog.sbig",
			"1214bprog.sbig", "1215prog.sbig", "1216prog.sbig", "1217prog.sbig", "1218prog.sbig",
			"1219prog.sbig"};
*/
		int num_names = 3;
		string names[num_names];
		int read_int=0;
		sbigfile.open("0301b.txt");
		while (sbigfile >> names[read_int]){
			++read_int;
		}
		for (int i=0; i<num_names; i++) {
			string filename = path + names[i];
			cout << "\n\n\n\nOpening the saved image: " << filename << endl;
			if (img.OpenImage(filename.c_str()) != SBFE_NO_ERROR)
			{
				cout << "An error occured while opening: " << filename << endl;
				break;
			}
/*FIXME	
			//initialize the image viewer window
			if (firstTime) {                 //only do this on first loop iteration
				firstTime = 0;
//				cout << "Starting Image viewer application" << endl;
  				ImageViewer iv(640, 480, img.GetWidth(), img.GetHeight(), 10, 0, "viewer");
				//FIXMEiv = new ImageViewer(img.GetWidth(), img.GetHeight(), 1000, 0, "viewer");
				a.setMainWidget(iv);
				iv->show();
				pthread_create(&app_thread, NULL, &viewerExec, (void*)&a);
			}
*/			
//			cout << "Finding blobs" << endl;
			img.findBlobs();
			if(fblob->get_numblobs() > 7) { 
				cout << "\nFound " << fblob->get_numblobs() << " blobs, map mean = "
				<< fblob->get_mapmean() << " sigma = " << fblob->get_sigma() << endl;
			}
			int num = 1;
			if (fblob->get_numblobs()) {
				if(fblob->get_numblobs() > 7) cout <<"Their locations, flux and snr  (x y f s) are " << endl;
				blobs = fblob->getblobs();
				while (blobs != NULL) {
					double x = blobs->getx();
					double y = blobs->gety();
					double f = blobs->getflux();
					double s = blobs->getsnr();
					//cout << "\t" << x << " " << y << " " << f << " " << s << endl;
					if (fblob->get_numblobs() > 7) cout << "\t" << x << " " << y << endl;
					img.drawBox(blobs->getx(), blobs->gety(), 40, num, true);
					blobs = blobs->getnextblob();
					num++;
				}
			}
/*FIXME			
			iv->load(&img, TRUE);
*/
			//wait for a bit and observe image
//			sleep(5);
			
// 			filename = path + "boxes/" + names[i];
// 			cout << "Saving image with boxes: " << filename << endl;
// 			if (  img.SaveImage(filename.c_str(), SBIF_COMPRESSED) != SBFE_NO_ERROR ) {
// 				cout << "error saving altered file" << endl;
// 				break;
// 			}
			
// 			cout << "\nPerforming Star matching..." << endl;
// 			solution_t* sol;
// 			int nsol = img.matchStars(&sol);
// 			cout << "\nStar matching found: " << nsol << " solutions";
// 			if (nsol > 0) {
// 				cout << " with:" << endl;
// 				for (int j=0; j<((nsol<15)?nsol:15); j++) {
// 					cout << "  " << j << ":\tn=" << sol[j].n << ", brightest star has: ra=" 
// 						 << sol[j].C[0]->ra << " dec=" << sol[j].C[0]->dec << " mag=" 
// 						 << sol[j].C[0]->mag << endl;
// 					if (nsol > 15 && j == 14)
// 						cout << "  *\tList truncated after this point..." << endl;
// 				}
// 			}
// 			else cout << endl;
			
 		}


#endif
	} while (0);
#if TEST_MODE == 2
	if ( err != LE_NO_ERROR)
 		cout << "A Lens Error Occured: " << lens->getErrorString(err) << endl;
 	if (cerr != CE_NO_ERROR)
		cout << "A Camera Error Occured: " << cam.GetErrorString(cerr) << endl;
#endif
#if TEST_MODE == 3
	pthread_join(app_thread, NULL);
#endif

	return EXIT_SUCCESS;
}
#endif //if TEST_MODE == 1 || == 2 || == 3





#if TEST_MODE == 4
///////////////////////////////////////////////////////////////////////////////////////////////////
//  Old main program (for testing)                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////


#define OPENING_METHOD 3

int main(int argc, char *argv[])
{
#if OPENING_METHOD == 1
	cout << "Creating the MyCam Object on USB and Opening Driver, Device..." << endl;
	MyCam pCam(DEV_USB);
#else
	cout << "Creating the MyCam Object on USB..." << endl;
	MyCam pCam;
#endif
	BlobImage img;
	PAR_ERROR err = CE_NO_ERROR;
	SBIG_FILE_ERROR ferr = SBFE_NO_ERROR;
#if OPENING_METHOD == 3
	QueryUSBResults qur;
#endif
	bloblist *blobs;
	string lens_return_str;
	CLensAdapter* lens = pCam.getLensAdapter();
	frameblob* fblob;


	do { // allow break out
		
//there are three methods being used here to open the camera device; only one is needed
#if OPENING_METHOD==1
//original method: only works for a single camera
		if ( (err = pCam.GetError()) != CE_NO_ERROR )
			break;
		
#elif OPENING_METHOD==2
//manual one-at-a-time method. could be used to implement two cameras
		cout << "Opening Driver..." << endl;
		if ((err = pCam.OpenDriver()) != CE_NO_ERROR)
			break;
		cout << "Opening Device..." << endl;
		if ((err = pCam.OpenUSBDevice(0)) != CE_NO_ERROR)
			break;

#elif OPENING_METHOD==3
//automatic one-at-a-time-method. better way to implement multiple cameras
		if ((err = pCam.OpenDriver()) != CE_NO_ERROR)
			break;
		cout << "Querying USB...(4 tries)" << endl;
		for (int i=0; i<4; i++) {
			cout << "Attempt " << i+1 << endl;
			if ((err = pCam.QueryUSB(qur)) == CE_NO_ERROR && qur.camerasFound)
				break;
		}
		if (err != CE_NO_ERROR) 
			break;
		if (qur.camerasFound) {
			cout << "There are " << qur.camerasFound << " USB Cameras." << endl;
			for (int i=0; i<4; i++) {
				if (qur.usbInfo[i].cameraFound) {
					cout << "Using Cam " <<i<< ": Name=" << qur.usbInfo[i].name 
							<< ", Serial Number=" << qur.usbInfo[i].serialNumber << endl;
					cout << "Opening the device..." << endl;
					err = pCam.OpenUSBDevice(i);
					break;
				}
			}
			if (pCam.GetError() != CE_NO_ERROR) //opening device failed
				break;
		} else {
			cout << "No cameras were found on USB..." << endl;
			err = CE_CAMERA_NOT_FOUND;
			break;
		}
#endif

			
//establish a link and do everything else
		pCam.SetExposureTime(0.1);
		cout << "Establishing a Link to the USB Camera...(4 tries)" << endl;
		for (int i=0; i<4; i++) {
			cout << "Attempt " << i + 1 << endl;
			if ((err = pCam.EstablishLink()) == CE_NO_ERROR) break;
		}
		if (err != CE_NO_ERROR)
			break;
		cout << "Link Established to Camera Type: " << pCam.GetCameraTypeString() << endl;

		// Take a full frame image and save compressed
		cout << "Taking full-frame light image on USB..." << endl;
		if ( (err=pCam.GrabImage(&img, SBDF_LIGHT_ONLY)) != CE_NO_ERROR )
			break;
/*
		//move the lens focus to test lens adapter interfaceBlobImage
		cout << "Opening a connection to the lens adapter..." << endl;
		if (lens->openConnection("/dev/ttyS0") != LE_NO_ERROR) {
		cout << "Error establishing link to lens adapter" << endl;
		break;
	}
		cout << "Finding length of lens focal axis..." << endl;
		if (lens->findFocalRange() != LE_NO_ERROR) {
		cout << "Error moving lens" << endl;
		break;
	}
		cout << "  the focal axis is " << lens->getFocalRange() << " counts long." << endl;
		int move_by = -400;
		cout << "Moving the lens focus by " << move_by << " counts..." << endl;
		int remaining;
		if (lens->preciseMove(move_by, remaining) != LE_NO_ERROR) {
		cout << "Error moving lens" << endl;
		break;
	}
		cout << "  there are " << remaining << " counts to move (tolerance=1)" << endl;

		//execute blob finding operations and display results
		cout << "Processing the image (blob finding)..." << endl;
		img.findBlobs();
		fblob = img.getFrameBlob();
		cout << "\nFound " << fblob->get_numblobs() << " blobs." << endl;
		if (fblob->get_numblobs()) {
		cout <<"Their locations (x,y) are " << endl;
		blobs = fblob->getblobs();
		while (blobs != NULL) {
		if (blobs->gettype() == 2)         //display only extended blobs
		cout << "(" << blobs->getx() << "," << blobs->gety() << ")" << endl;
		blobs = blobs->getnextblob();
	}
	}
		blobs = NULL;*/

		img.AutoBackgroundAndRange();            //needed to view images on computer
		cout << "Saving compressed full-frame image..." << endl;
		if ( (ferr = img.SaveImage("temp.sbig", SBIF_COMPRESSED)) != SBFE_NO_ERROR )
			break;
		
// shut down cameras
		cout << "Closing Devices..." << endl;
		if ( (err = pCam.CloseDevice()) != CE_NO_ERROR )
			break;
		cout << "Closing Drivers..." << endl;
		if ( (err = pCam.CloseDriver()) != CE_NO_ERROR )
			break;		
	} while (0);
	if ( err != CE_NO_ERROR )
		cout << "Camera Error: " << pCam.GetErrorString(err) << endl;
	else if ( ferr != SBFE_NO_ERROR )
		cout << "File Error: " << ferr << endl;
	else
		cout << "SUCCESS" << endl;
	return EXIT_SUCCESS;
}
#endif       //if TEST_MODE == 4

#if TEST_MODE == 5
///////////////////////////////////////////////////////////////////////////////////////////////////
// FIFO tester
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	FIFOCommunicator fifoc("/home/steve/starcam/readfifo", "/home/steve/starcam/writefifo");
	if (fifoc.getErrorFlag() == -1) 
		cout << "An error occured while opening the fifos." << endl;
	else fifoc.readLoop();
	cout << "Read loop returned...an error has occured" << endl;
	return 0;
}

#endif       //TEST_MODE == 5
