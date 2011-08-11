#include <string>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <climits>
#include <sys/time.h>
#include <sys/stat.h>
#include "blobimage.h"
#include "csbigimg.h"
#include "frameblob.h"
#include "bloblist.h"
#include "pyramid.h"

#define BLOB_IMAGE_DEBUG 0
#if BLOB_IMAGE_DEBUG
#include <iostream>
#endif

using namespace std;

/*

 BlobImage:
 
 Default constructor: sets all parameters to a default value.
 
*/
BlobImage::BlobImage(BlobImageConfigParams params /*=defaultImageParams*/)
//	: m_cPyramid(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname)
{
	Init(params);
}

/*

 BlobImage:
 
 Alternate constructor: sets all parameters to a default value and allocated image buffer
 
*/
BlobImage::BlobImage(int height, int width, BlobImageConfigParams params /*=defaultImageParams*/)
//	: m_cPyramid(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname)
{
	Init(params);
	AllocateImageBuffer(height, width);
}

/*

 ~BlobImage:
 
 Destructor: clears image data
 
*/
BlobImage::~BlobImage()
{
	DeleteImageData();
}

/*

 Init:
 
 Sets all inherited and member parameters to a default value.
 
*/
void BlobImage::Init(BlobImageConfigParams params/*=defaultImageParams*/)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in Init method..." << endl;
#endif
	CSBIGImg::Init(params.SBIGParams);
	m_cBlob.commonconstructor(params.blobParams);
	m_sBadpixFilename = params.badpixFilename;
	m_nTimeError = params.timeError;
#if USE_PYRAMID
	m_cPyramid.Init(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname);
//	cout << "erase me: left Init for Pyramid" << endl;
	m_dMatchTol = params.matchTol;
	m_dPlatescale = params.platescale;
#endif
}
/* original version has been replaced
void BlobImage::Init(void)
{
	CSBIGImg::Init();             //perform all initialization of inherited members
	m_sBadpixFilename = "";
}
*/

/*

 InitFrameblob:
 
 Sets all frameblob parameters to a default value to enable blob finding
 
*/
/* this function is no longer needed
void BlobImage::InitFrameblob()
{
	m_cBlob.commonconstructor((MAPTYPE*)CSBIGImg::GetImagePointer(), CSBIGImg::GetWidth(),
							   CSBIGImg::GetHeight(), 16, (double)9.3/60/60);
	m_cBlob.set_gain(1.0/1.48);                 //in units/e-
	m_cBlob.set_readout_offset(100);            //in units/pixel
	m_cBlob.set_readout_noise(13.8 / 1.48);     //this is in units/pixel
	m_cBlob.set_satval((MAPTYPE)65534);
	m_cBlob.set_threshold(5);
	m_cBlob.set_disttol(30*30);
	m_cBlob.set_grid(30);
	m_cBlob.set_maxblobs(99);       //set mostly to save space in the image note
}
*/

/*

 AllocateImageBuffer:
 
 Overide method in CSBIGImg to keep frameblob information current
 
*/
MY_LOGICAL BlobImage::AllocateImageBuffer(int height, int width)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in AllocateImageBuffer method with " 
		 << width << "x" << height << endl;
#endif
	if (!CSBIGImg::AllocateImageBuffer(height, width))
		return FALSE;
	m_cBlob.set_map(CSBIGImg::GetImagePointer(), height, width);	
	m_bIsChanged = true;
	return TRUE;
}

/*

 DeleteImageData:
 
 Overide method in CSBIGImg to keep frameblob information current
 
*/
void BlobImage::DeleteImageData()
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in DeleteImageData method..." << endl;
#endif
	CSBIGImg::DeleteImageData();
	m_cBlob.set_map(NULL, 0, 0);
}

/*
    OpenImage:

    Overide parent method to use correct buffer allocation

*/
SBIG_FILE_ERROR BlobImage::OpenImage(const char *pFullPath)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	char			header[HEADER_LEN];
	FILE			*fh;
	SBIG_FILE_ERROR res;
	MY_LOGICAL		isCompressed;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in OpenImage method..." << endl;
#endif

	if ((fh = fopen(pFullPath, "rb")) == NULL)
		return SBFE_OPEN_ERROR;
	else {
		do {	// allow break out
			// read and pars header
			res = SBFE_FORMAT_ERROR;
			if (fread(header, 1, HEADER_LEN, fh) != HEADER_LEN)
				break;
			if (!CSBIGImg::ParseHeader(header, isCompressed))
				break;

			// allocate image buffer
			res = SBFE_MEMORY_ERROR;
			if (!AllocateImageBuffer(CSBIGImg::GetHeight(), CSBIGImg::GetWidth()))
				break;

			if (isCompressed)
				res = CSBIGImg::ReadCompressedImage(fh);
			else
				res = CSBIGImg::ReadUncompressedImage(fh);

			if (res != SBFE_NO_ERROR) {
				CSBIGImg::DeleteImageData();
			}
		} while (FALSE);
	}

	fclose(fh);
	if (res == SBFE_NO_ERROR) {
		CSBIGImg::SetImageModified(FALSE);
		CSBIGImg::SetDefaultImageForamt(isCompressed ? SBIF_COMPRESSED : SBIF_UNCOMPRESSED);
	}

	m_bIsChanged = true;
	return res;
}

/*

 SetImageStartTime:
 
 Overrides parent method to supply more precision
 Sets the image start time to the current system time

*/
void BlobImage::SetImageStartTime(void)
{
	gettimeofday(&m_sImageStartTime, NULL);
	CSBIGImg::SetImageStartTime(m_sImageStartTime.tv_sec);   //set less precise time stored in parent
}

/*

 SetImageStartTime:
 
 same as above method, but also uses ref (found before exposure) to estimate time error
 of course, ref should refer to a time before the call to this function

*/
void BlobImage::SetImageStartTime(timeval* ref)
{
	gettimeofday(&m_sImageStartTime, NULL);
	CSBIGImg::SetImageStartTime(m_sImageStartTime.tv_sec);   //set less precise time stored in parent
	m_nTimeError = m_sImageStartTime.tv_sec - ref->tv_sec;
	m_nTimeError *= 1000000;
	m_nTimeError += m_sImageStartTime.tv_usec - ref->tv_usec;
	
	//set the image note to indicate time of exposure
	ostringstream sout;          //will contain the image note
	sout << "UNIX time: " << m_sImageStartTime.tv_sec << "s " << m_sImageStartTime.tv_usec
		 << "us" << " err <= " << m_nTimeError << "us";
	CSBIGImg::SetImageNote(sout.str());
}


/*

 copyImageFrom:
 
 copies image (it better be the right size) from data array
 should be stored as single row-by-row array
 
*/
void BlobImage::copyImageFrom(const unsigned short* data) {
  unsigned short* img = this->GetImagePointer();
  for (int i=0; i<this->GetHeight()*this->GetWidth(); i++) {
    img[i] = data[i];
  }
}

/*

 setBadpixFilename:
 
 sets bad pixel filename based on C-style string
 
*/
void BlobImage::setBadpixFilename(const char* name)
{
	string name_str = name;
	setBadpixFilename(name_str);
}

/*

 FixBadpix:
 
 Loads file of badpixels and sets their value to the map mean.
 to disable bad pixel checking, send filename ""
 Returns an error when loading the file fails.
 If a file of the same name is already loaded, will not load it again
*/
SBIG_FILE_ERROR BlobImage::FixBadpix(string filename)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in FixBadPix with: \"" << filename << "\"" 
	     << " old name \"" << m_sBadpixFilename << "\"" << endl;
#endif
	if (filename == "") {
		m_sBadpixFilename = filename;
		return SBFE_NO_ERROR;
	}
	if (m_sBadpixFilename != filename) {         //don't load file if it's same as before
		if (m_cBlob.load_badpix(filename.c_str()) < 0) {
			return SBFE_OPEN_ERROR;
		}
		m_sBadpixFilename = filename;
	}
	m_cBlob.calc_mapstat();
	m_cBlob.fix_badpix((MAPTYPE)m_cBlob.get_mapmean());
	return SBFE_NO_ERROR;
}

/*

 findBlobs:
 
 uses frameblob to find the blobs in the image
 sets the image note to summarize results
 returns number of blobs found or -1 on error
 
*/
int BlobImage::findBlobs()
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in findBlobs method..." << endl;
#endif
	string note = "";
	ostringstream sout;
	int numblobs;
	bloblist *blobs;
	
	//find the blobs
	if (FixBadpix(m_sBadpixFilename) == SBFE_OPEN_ERROR)
		return -1;
	highPassFilter(15,3);        
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: running calc_mapstat method..." << endl;
#endif
	m_cBlob.calc_mapstat();
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: running calc_searchgrid method..." << endl;
#endif
	m_cBlob.calc_searchgrid();
	m_cBlob.fix_multiple();
	m_cBlob.sortblobs();
	
	numblobs = m_cBlob.get_numblobs();
	blobs = m_cBlob.getblobs();
	
/*  not using this any more, the image note will now provide UNIX timestamp
	
	//set the image note to provide rough information about the blobs found
	int x, y;
	char buf[40];
	sout << numblobs << " blobs";
	if (numblobs) sout << " at:";
	while (blobs != NULL) {
//		if (blobs->gettype() == 2)      //only include extended blobs
		{
			x = (int) floor(blobs->getx() + 0.5);     //round location to save space
			y = (int) floor(blobs->gety() + 0.5);
			sout << " (" << x << "," << y << ")";
			if (strlen(buf) + note.length() > 67) {   //note field has maximum length 70
				note += "...";
				break;
			}
		}
		blobs = blobs->getnextblob();
	}
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: The camera note will be set to: " << sout.str() << endl;
#endif
	CSBIGImg::SetImageNote(sout.str());
*/
	
	return numblobs;
}

/*

 highPassFilter:

 simple (and hopefully quick, good enough) high pass fitler
 operates on sqaure regions box_size on side
 averages a region n_boxes boxes on a side, and subtracts mean from central box
 probably best if box_size evenly divides image dimensions, and n_boxes is odd

*/
#include <iostream>
using namespace std;
int BlobImage::highPassFilter(int box_size, int n_boxes)
{
  unsigned short *img = this->GetImagePointer();
  unsigned int height = this->GetHeight();
  unsigned int width = this->GetWidth();

  //coarse image, binned in box_size chunks
  unsigned short* c_img;
  unsigned int c_height, c_width;
  c_height = (unsigned int)ceil((double)height/(double)box_size);
  c_width = (unsigned int)ceil((double)width/(double)box_size);
  c_img = new unsigned short[c_height*c_width];

  //create the coarse image
  for (unsigned int c_x=0; c_x<c_width; c_x++) {
    for (unsigned int c_y=0; c_y<c_height; c_y++) {
      unsigned int sum = 0;
      unsigned int npix = 0;
      for (unsigned int x=c_x*box_size; x<(c_x+1)*(box_size); x++) {
	for (unsigned int y=c_y*box_size; y<(c_y+1)*(box_size); y++) {
	  if (x < width && y < height) {
	    sum += img[x+y*width];
	    npix++;
	  }
	}
      }
      c_img[c_x+c_y*c_width] = (unsigned short)round((double)sum/(double)npix);
    }
  }
  
  //calculate average in n_boxes sided square around coarse pixel
  unsigned short* to_subtract;
  to_subtract = new unsigned short[c_height*c_width];
  unsigned short to_subtract_max = 0;
  for (unsigned int c_x=0; c_x<c_width; c_x++) {
    for (unsigned int c_y=0; c_y<c_height; c_y++) {
      unsigned int sum = 0;
      unsigned int npix = 0;
      unsigned short max = 0;

      for (int avg_x=(int)c_x-n_boxes/2; avg_x<=(int)c_x+n_boxes/2; avg_x++) {
	for (int avg_y=(int)c_y-n_boxes/2; avg_y<=(int)c_y+n_boxes/2; avg_y++) {
	  if (avg_x >= 0 && avg_x < (int)c_width && avg_y >= 0 && avg_y < (int)c_height) {
	    unsigned short datum = c_img[avg_x + avg_y*c_width];
	    if (datum > max) max = datum;
	    sum += datum;
	    npix++;
	  }
	}
      }
      sum -= max;  //remove brightest pixel from average
      npix--;
      unsigned short mean = (unsigned short)round((double)sum/(double)npix);
      to_subtract[c_x+c_y*c_width] = mean;
      if (mean > to_subtract_max) to_subtract_max = mean;
    }
  }
    
  //subtract the to_subtract means from the full-res image
  //also add to_subtract_max to each pixel to prevent underflow
  for (unsigned int c_x=0; c_x<c_width; c_x++) {
    for (unsigned int c_y=0; c_y<c_height; c_y++) {
      for (unsigned int x=c_x*box_size; x<(c_x+1)*(box_size); x++) {
	for (unsigned int y=c_y*box_size; y<(c_y+1)*(box_size); y++) {
	  if (x < width && y < height) {
	    if (USHRT_MAX - img[x+y*width] < to_subtract_max - to_subtract[c_x+c_y*c_width])
	      img[x+y*width] = USHRT_MAX;
	    else img[x+y*width] += to_subtract_max - to_subtract[c_x+c_y*c_width];
	  }
	}
      }
    }
  }

  delete [] c_img;
  return 1;
}

/*

 drawBox:
 
 draws a white square of dimension side around the point x, y
 when willChange is true, sets the images changed flag
 
*/
void BlobImage::drawBox(double x, double y, double side, bool willChange /*=true*/)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: In drawBox method." << endl;
#endif
	
	MAPTYPE* map = CSBIGImg::GetImagePointer();
	//rather than using saturation, try just background + range
	//MAPTYPE sat = m_cBlob.get_satval();
	this->AutoBackgroundAndRange();
	MAPTYPE sat = this->GetBackground() + this->GetRange();
	int top = (int)(y - side/2), left = (int)(x - side/2), size = (int)side;
	int xdim = CSBIGImg::GetWidth(), ydim = CSBIGImg::GetHeight();
	int xlimit = left + size, ylimit = top + size;
	if (top < 0) top = 0;
	if (left < 0) left = 0;
	if (xlimit > xdim) xlimit = xdim;
	if (ylimit > ydim) ylimit = ydim;
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: entering loop for: " << top << " < y < " << ylimit << " and " 
 		 << left << " < x < " << xlimit << endl;
#endif
	for (int i=top; i<ylimit; i++) {        //row
		for (int j=left; j<xlimit; j++) {   //column
			if (i == top || i == (ylimit-1) || j == left || j == (xlimit-1))   //on edge of box
				map[i*xdim+j] = sat;
		}
	}
	if (willChange) m_bIsChanged = 1;
}

/*

 createFilename:
 
 makes a filename starting at the directory root based on time picture was taken
 if bloxflag is true will add /boxes/ to end of directory (for images with boxes around blobs)
 
*/
string BlobImage::createFilename()
{
	ostringstream sout;
	struct tm timestruct = CSBIGImg::GetImageStartTime();
	
	sout << ((timestruct.tm_hour<10)?"0":"") << timestruct.tm_hour << ":"
		 << ((timestruct.tm_min<10)?"0":"") << timestruct.tm_min << ":"
		 << ((timestruct.tm_sec<10)?"0":"") << timestruct.tm_sec << ".sbig";
	return sout.str();
}

/*

 createDirectory:
 
 creates the directory component of the filename used in createFilename
 
*/
string BlobImage::createDirectory(string root, int boxflag)
{
	ostringstream sout;
	struct tm timestruct = CSBIGImg::GetImageStartTime();
	
	sout << root << ((root[root.length()-1]=='/')?"":"/");
	sout << ((timestruct.tm_mday<10)?"0":"") << timestruct.tm_mday << "-" 
		 << ((timestruct.tm_mon<9)?"0":"") << timestruct.tm_mon + 1 << ((boxflag)?"/boxes/":"/");
	return sout.str();
}

/*

 SaveImageIn:
 
 saves the image to a path based on the image time, root path, and boxes flag
 uses default format
 
*/
SBIG_FILE_ERROR BlobImage::SaveImageIn(string root/*="/data/rawdir"*/, int boxflag/*=0*/)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: In SaveImageIn method." << endl;
#endif
	string filename = createFilename();
	string directory = createDirectory(root, boxflag);
	string total = directory + filename;
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: saving image as: " << total << endl;
#endif
	SBIG_FILE_ERROR err = CSBIGImg::SaveImage(total.c_str(), CSBIGImg::GetDefaultImageFormat());
	if (err == SBFE_OPEN_ERROR) {      //problem opening file...try creating the directory
#if BLOB_IMAGE_DEBUG
		cout << "[Blob image debug]: error saving, trying to create directory" << endl;
#endif
		if (mkdir(directory.c_str(), 0777) < 0) {
#if BLOB_IMAGE_DEBUG
			cout << "[Blob image debug]: error creating directory" << endl;
#endif
			return err;
		}
		else     //try again if creating directory succeeds
			return CSBIGImg::SaveImage(total.c_str(), CSBIGImg::GetDefaultImageFormat());
	}
	else return err;
}

/*

 createReturnStruct:
 
 populates a struct (pointer passed as arg) with return values for flight computer
 
*/
#include <iostream>
StarcamReturn* BlobImage::createReturnStruct(StarcamReturn* arg)
{
	arg->frameNum = m_iFrame;
	arg->mapmean = m_cBlob.get_mapmean();
	arg->sigma = m_cBlob.get_sigma();
	arg->exposuretime = CSBIGImg::GetExposureTime();
	arg->imagestarttime = GetImageStartTime();
	arg->camID = m_sCamID;
	arg->ccdtemperature = CSBIGImg::GetCCDTemperature();
	
	//blob info (on 15 brightest blobs)
	arg->numblobs = m_cBlob.get_numblobs();
	bloblist *blobs = m_cBlob.getblobs();
	for (int i=0; i<15; i++) {
		if (blobs == NULL) break;      //leaves higher indexes uninitialized...be careful
		arg->flux[i] = blobs->getflux();
		arg->mean[i] = blobs->getmean();
		arg->snr[i] = blobs->getsnr();
		arg->x[i] = blobs->getx();
		arg->y[i] = blobs->gety();
		blobs = blobs->getnextblob();
	}
	return arg;
}


#if USE_PYRAMID
/*

 matchStars:
 
 performs star pattern matching, sol is returned as a pointer to the solution array
 returns number of solutions found or -1 on error
 
*/
int BlobImage::matchStars(solution_t **sol)
{
	int nblobs = m_cBlob.get_numblobs();
	double *x, *y;
	x = new double[nblobs];
	y = new double[nblobs];
	bloblist *blobs = m_cBlob.getblobs();
	int i = 0, retval = -1;
	
	while (blobs != NULL) {
		x[i] = blobs->getx()*m_dPlatescale;
		y[i] = blobs->gety()*m_dPlatescale;
		blobs = blobs->getnextblob();
		i++;
	}
	
	if (m_cPyramid.GetSolution(m_dMatchTol, x, y, nblobs, sol, &retval, 0, 0, 0) < 0)
		retval = -1;
	
	delete[] x;
	delete[] y;
	return retval;
}
#endif    //if USE_PYRAMID
