/* findbadpixel.cpp
 *
 * utility program to analyze bad pixel dark exposures of different integration times
 * and construct a bad pixel text file, for use by starcam.cpp
 *
 */
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include "blobimage.h"
#include "camstruct.h"

using namespace std;

const unsigned int box_size = 15;
const unsigned int n_boxes = 3;
const double expose_min = 0.1;
const char* avgfilename = "/tmp/average.sbig";
const double snr_threshold = 7.0;

int main(int argc, char* argv[])
{
  BlobImage bimg(CAM_HEIGHT, CAM_WIDTH);
  unsigned int* sum = new unsigned int[(int)CAM_HEIGHT*(int)CAM_WIDTH];
  unsigned short* bimg_p;

  if (argc < 2) {
    cerr << "Must specify a list of image files to process" << endl;
    return 1;
  }

  for (int i=0; i<CAM_WIDTH*CAM_HEIGHT; i++) sum[i] = 0;

  for (int i=1; i<argc; i++) {
    cout << "Opening Image File " << argv[i];
    bimg.OpenImage(argv[i]);
    cout << " with integration time " << bimg.GetExposureTime() << "s" << endl;
    bimg.setBadpixFilename("/tmp/IdontExist"); //clear badpix file name
    bimg.FixBadpix("/tmp/badpix.txt");	  //don't test already detected blobs
    bimg.highPassFilter(box_size, n_boxes);
    bimg_p = bimg.GetImagePointer();
    double normalization = bimg.GetExposureTime()/expose_min;
    for (int i=0; i<CAM_WIDTH*CAM_HEIGHT; i++)
      sum[i] += (unsigned int)round((double)bimg_p[i]/normalization);
  }

  BlobImage avgimg(CAM_HEIGHT, CAM_WIDTH);
  unsigned short* avgimg_p = avgimg.GetImagePointer();
  for (int i=0; i<CAM_WIDTH*CAM_HEIGHT; i++)
    avgimg_p[i] = (unsigned short)round((double)sum[i]/(double)(argc-1));
  double avgmean = avgimg.getFrameBlob()->get_mapmean();
  double avgsigma = avgimg.getFrameBlob()->calc_stddev();
  cout << "Mean: " << avgmean << " Sigma: " << avgsigma << endl;
  int badcount = 0;
  ofstream fout("/tmp/badpix.txt");
  BlobImage avgimg_copy(CAM_HEIGHT, CAM_WIDTH);
  avgimg_copy.copyImageFrom(avgimg.GetImagePointer());
  for (int x=0; x<(int)CAM_WIDTH; x++) {
    for (int y=0; y<(int)CAM_HEIGHT; y++) {
      if ((int)avgimg_p[x+y*(int)CAM_WIDTH] - avgmean > snr_threshold*avgsigma) {
	cout << "Bad pixel at " << x <<"," << y << endl;
	fout << x << " " << y << endl;
  avgimg_copy.drawBox(x, y, 40, 0);
	badcount++;
      }
    }
  }
  cout << "Found " << badcount << " bad pixels with SNR above " << snr_threshold << endl;

  cout << "Saving averaged image to " << avgfilename << endl;
  avgimg_copy.SaveImage(avgfilename);


  delete [] sum;
  return 0;
}
