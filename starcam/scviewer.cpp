/* viewer.cpp: simple app for displaying star camera images
 * Updating requires the pointed-to file to a a symlink to an image
 * Polls the file and updates image as file changes.
 * Mostly used to view current images from star camera
 */

#include <fstream>
#include <iostream>
#include <qapplication.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include "blobimage.h"
#include "csbigimgdefs.h"
#include "imageviewer.h"

#define DEFAULT_FILE "/data/etc/current_bad.sbig"

char* filename;
BlobImage* img[2];   //images. Using only 1 caused segfaults...
int imgindex = 0;

void failure()
{
  delete img[0];
  delete img[1];
  exit(1);
}

void* updateImage(void* arg)
{
  ImageViewer* iv = (ImageViewer*)arg;
  struct stat stats;
  time_t time, oldtime;
  int retries = 0;
  int nextindex;

  if (stat(filename, &stats)) {
    cerr << "Problem getting file stats..." << endl;
    failure();
  }
  oldtime = stats.st_mtime;

  while (1) {
    if (stat(filename, &stats)) {
      cerr << "Problem getting file stats..." << endl;
      failure();
    }
    time = stats.st_mtime;
    if (time > oldtime) {  //the image file has been modified
      //usleep(5000); //give starcam a chance to save the file
      nextindex = (imgindex == 0) ? 1 : 0;
      oldtime = time;
      cout << "Image updated, refreshing it" << endl;
      while (img[nextindex]->OpenImage(filename) != SBFE_NO_ERROR) {
        retries++;
        if (retries > 1) cout << "...failed, retrying" << endl;
        if (retries >= 5) {
          cerr << "Oops, something's broken" << endl;
          failure();
        }
        usleep(100);
      }
      retries = 0;
      iv->load(img[nextindex], TRUE);
      imgindex = nextindex;
    }
    usleep(10000);  //wait 10ms before trying again
  }

  return NULL;
}

int main(int argc, char* argv[])
{
  //open the image
  filename = DEFAULT_FILE;
  if (argc == 2)
    filename = argv[1];
  else if (argc > 2) {
    cerr << "You're not using me right" << endl;
    return 1;
  }
  img[0] = new BlobImage();
  img[1] = new BlobImage();
  if (img[0]->OpenImage(filename) != SBFE_NO_ERROR) {
    cerr << "Error opening image: " << filename << endl;
    return 1;
  }
  else cout << "Opened image: " << filename << endl;

  //set up the viewer
  QApplication a(argc, argv);
  ImageViewer iv(640, 480, img[0]->GetWidth(), img[0]->GetHeight(), 10, 0, "viewer");
  iv.setGeometry(0,0,640,480);
  a.setMainWidget(&iv);
  iv.show();
  iv.load(img[0], TRUE);

  //start a thread to update the image
  pthread_t update_id;
  pthread_create(&update_id, NULL, &updateImage, (void*)&iv);

  //start the viewer
  cout << "Starting the viewer" << endl;
  a.exec();

  pthread_cancel(update_id);

  delete img[0];
  delete img[1];
  return 0;
}
