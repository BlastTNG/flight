/* fixbadpixel.cpp
 *
 * utility program to apply a bad pixel file to an image
 * also high-pass filters
 *
 */
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include "blobimage.h"
#include "camstruct.h"

using namespace std;

const char* badpixfilename = "/tmp/badpix.txt";
const char* outfilename = "/tmp/fixed.sbig";
const unsigned int box_size = 15;
const unsigned int n_boxes = 3;

int main(int argc, char* argv[])
{
  if (argc != 2) {
    cerr << "Must specify an image file as parameter" << endl;
  }
  cout << "Opening Image File " << argv[1] << endl;
  BlobImage bimg(CAM_HEIGHT, CAM_WIDTH);
  bimg.OpenImage(argv[1]);
  bimg.FixBadpix("/tmp/badpix.txt");	  //don't test already detected blobs
  bimg.highPassFilter(box_size, n_boxes);

  cout << "Saving corrected image to " << outfilename << endl;
  bimg.SaveImage(outfilename);

  return 0;
}
