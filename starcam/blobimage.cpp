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

#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>

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
#if USE_PYRAMID	
	: m_cPyramid(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname)
#endif
{
	Init(params);
}

/*

 BlobImage:
 
 Alternate constructor: sets all parameters to a default value and allocated image buffer
 
*/
BlobImage::BlobImage(int height, int width, BlobImageConfigParams params /*=defaultImageParams*/)
#if USE_PYRAMID	
	: m_cPyramid(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname)
#endif
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
	//m_cPyramid.Init(params.pyrParams.fov, params.pyrParams.catalogname, params.pyrParams.katalogname);
	//cout << "erase me: left Init for Pyramid" << endl;
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
	
	//crapifyImage();
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

	return numblobs;
}

/*
 crapifyImage:

 adds another image's value for each pixel to the current image

*/


int BlobImage::crapifyImage()
{
	const char* filename[14];
	filename[0] = "/home/spider/starcam/sbsc/13:01:41.sbig";
	filename[1] = "/home/spider/starcam/sbsc/13:05:53.sbig";
	filename[2] = "/home/spider/starcam/sbsc/13:10:31.sbig";
	filename[3] = "/home/spider/starcam/sbsc/13:14:48.sbig";
	filename[4] = "/home/spider/starcam/sbsc/13:19:20.sbig";
	filename[5] = "/home/spider/starcam/sbsc/13:23:38.sbig";
	filename[6] = "/home/spider/starcam/sbsc/13:27:53.sbig";
	filename[7] = "/home/spider/starcam/sbsc/13:32:19.sbig";
	filename[8] = "/home/spider/starcam/sbsc/13:36:30.sbig";
	filename[9] = "/home/spider/starcam/sbsc/13:40:59.sbig";
	filename[10] = "/home/spider/starcam/sbsc/13:45:25.sbig";
	filename[11] = "/home/spider/starcam/sbsc/13:49:40.sbig";
	filename[12] = "/home/spider/starcam/sbsc/13:54:03.sbig";
	filename[13] = "/home/spider/starcam/sbsc/13:58:29.sbig";
	int randi = rand()%13;
	BlobImage* sbscimg = new BlobImage();
  	if (sbscimg->OpenImage(filename[randi]) != SBFE_NO_ERROR) {
    		cerr << "Error opening image: " << filename[randi] << endl;
    		return 1;
  	} else cout << "Opening image: " << filename[randi] << endl;
  	unsigned short *img = this->GetImagePointer();
  	/*unsigned NNG*/ int height = this->GetHeight();
  	/*unsigned NNG*/ int width = this->GetWidth();
  	unsigned short *crapimg = sbscimg->GetImagePointer();
	for (int x=0; x<width; x++) {
		for (int y=0; y<height; y++) {
			img[x+width*y]+=crapimg[x+width*y];
		}
	}			
	delete sbscimg;
	return 1;
}

/*

 highPassFilter:

 simple (and hopefully quick, good enough) high pass filter
 operates on sqaure regions box_size on side
 averages a region n_boxes boxes on a side, and subtracts mean from central box
 probably best if box_size evenly divides image dimensions, and n_boxes is odd

*/
#include <iostream>
using namespace std;
int BlobImage::highPassFilter(int box_size, int n_boxes)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: in highPassFilter..." << endl;
#endif
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
void BlobImage::drawBox(double x, double y, double side, int bnum, bool willChange /*=true*/)
{
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: In drawBox method." << endl;
#endif
	MAPTYPE* map = CSBIGImg::GetImagePointer();
	//rather than using saturation, try just background + range
	//MAPTYPE sat = m_cBlob.get_satval();
	this->AutoBackgroundAndRange();
	MAPTYPE sat = this->GetBackground() + this->GetRange();
	int i,j;
	int top = (int)(y - side/2), left = (int)(x - side/2), size = (int)side;
	int xdim = CSBIGImg::GetWidth(), ydim = CSBIGImg::GetHeight();
	int xlimit = left + size, ylimit = top + size;
	int first = bnum/10;
	int second = bnum%10;
	if (top < 0) top = 0;
	if (left < 0) left = 0;
	if (xlimit > xdim) xlimit = xdim;
	if (ylimit > ydim) ylimit = ydim;
#if BLOB_IMAGE_DEBUG
	cout << "[Blob image debug]: entering loop for: " << top << " < y < " << ylimit << " and " 
 		 << left << " < x < " << xlimit << endl;
#endif
	for (i=top; i<ylimit; i++) {        //row
		for (j=left; j<xlimit; j++) {   //column
			if (i == top || i == (top+1) || i == (ylimit-2) || i == (ylimit-1) || 
			   j == left || j == (left+1) || j == (xlimit-2) || j == (xlimit-1))   //on edge of box
				map[i*xdim+j] = sat;
		}
	}
	int xnum,ynum;
	//Draw the blob number next to the box-----------
	if ((first == 4) || (first == 5) || (first == 6) || (first == 8) || (first == 9)) {
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+2+j)>xdim) ? xdim : (xlimit+2+j);	
			ynum = ((top+i) > ydim) ? ydim : (top+i);
			map[ynum*xdim+xnum] = sat;  //left vertical top
		}
	  }
	}
	if ((first == 2) || (first == 6) || (first == 8)) {
	  for (i=12; i<29; i++) { 
		for (j=0; j<6; j++) {
			xnum = ((xlimit+2+j)>xdim) ? xdim : (xlimit+2+j);	
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //left vertical bottom
		}
	  }
	}
	if ((first == 1) || (first == 2) || (first == 3) || (first == 4) || (first == 7) || (first == 8) || (first == 9)) {
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+14+j)>xdim) ? xdim : (xlimit+14+j);	
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //right vertical top
		}
	  }
	}
	if ((first == 1) || (first == 3) || (first == 4) || (first == 5) || (first == 6) || (first == 7) || (first == 8) || (first == 9)) {
	  for (i=12; i<29; i++) { 
		for (j=0; j<6; j++) {
			xnum = ((xlimit+14+j)>xdim) ? xdim : (xlimit+14+j);	
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //right vertical bottom
		}
	  }
	}
	if ((first == 2) || (first == 3) || (first == 5) || (first == 7) || (first == 8) || (first == 9)) {
	  for (i=0; i<18; i++) { 
		for (j=0; j<6; j++) {
			xnum = ((xlimit+2+i) > xdim) ? xdim : (xlimit+2+i);
			ynum = ((top+j)>ydim) ? ydim : (top+j);
			map[ynum*xdim+xnum] = sat;          //top horizontal
		}
	  }
	}
	if ((first == 2) || (first == 3) || (first == 4) || (first == 5) || (first == 6) || (first == 8) || (first == 9)) {	
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+2+i) > xdim) ? xdim : (xlimit+2+i);
			ynum = ((top+12+j)>ydim) ? ydim : (top+12+j);
			map[ynum*xdim+xnum] = sat;      //middle horizontal
		}
	  }
	}
	if ((first == 2) || (first == 3) || (first == 5) || (first == 6) || (first == 8)) {
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+2+i) > xdim) ? xdim : (xlimit+2+i);
			ynum = ((top+24+j) > ydim) ? ydim : (top+24+j);
			map[ynum*xdim+xnum] = sat;      //bottom horizontal
		}
	  }
	}
        if ((second == 0) || (second == 4) || (second == 5) || (second == 6) || (second == 8) || (second == 9)) {
          for (i=0; i<18; i++) { 
		for (j=0; j<6; j++) {
			xnum = ((xlimit+26+j)>xdim) ? xdim : (xlimit+26+j);
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //left vertical top
		}
	  }
        }
	if ((second == 0) || (second == 2) || (second == 6) || (second == 8)) {
	  for (i=12; i<29; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+26+j)>xdim) ? xdim : (xlimit+26+j);
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //left vertical bottom
		}
	  }
	}
	if ((second == 0) || (second == 1) || (second == 2) || (second == 3) || (second == 4) || (second == 7) || (second == 8) || (second == 9)) {
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+38+j)>xdim) ? xdim : (xlimit+38+j);
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //right vertical top
		}
	  }
	}
	if ((second == 0) || (second == 1) || (second == 3) || (second == 4) || (second == 5) || (second == 6) || (second == 7) || (second == 8) || (second == 9)) {
	  for (i=12; i<29; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+38+j)>xdim) ? xdim : (xlimit+38+j);
			ynum = ((top+i) > ydim) ? ydim : (top+i);	
			map[ynum*xdim+xnum] = sat;  //right vertical bottom
		}
	  }
	}
	if ((second == 0) || (second == 2) || (second == 3) || (second == 5) || (second == 7) || (second == 8) || (second == 9)) {
	  for (i=0; i<18; i++) { 
		for (j=0; j<6; j++) {
			xnum = ((xlimit+26+i) > xdim) ? xdim : (xlimit+26+i);
			ynum = ((top+j)>ydim) ? ydim : (top+j);
			map[ynum*xdim+xnum] = sat;          //top horizontal
		}
	  }
	}
	if ((second == 2) || (second == 3) || (second == 4) || (second == 5) || (second == 6) || (second == 8) || (second == 9)) {	
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+26+i) > xdim) ? xdim : (xlimit+26+i);
			ynum = ((top+12+j)>ydim) ? ydim : (top+12+j);
			map[ynum*xdim+xnum] = sat;      //middle horizontal
		}
	  }
	}
	if ((second == 0) || (second == 2) || (second == 3) || (second == 5) || (second == 6) || (second == 8)) {
	  for (i=0; i<18; i++) {
		for (j=0; j<6; j++) {
			xnum = ((xlimit+26+i) > xdim) ? xdim : (xlimit+26+i);
			ynum = ((top+24+j)>ydim) ? ydim : (top+24+j);
			map[ynum*xdim+xnum] = sat;      //bottom horizontal
		}
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
	arg->focusposition = CSBIGImg::GetApertureArea();
	
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
	int nblobs = (m_cBlob.get_numblobs() > 7) ? 7 : m_cBlob.get_numblobs();
//	int nblobs = 5; //NNG for testing
	double *x, *y;
	x = new double[nblobs];
	y = new double[nblobs];
	double *x_p = new double [nblobs];
  	double *y_p = new double [nblobs];
	memset(x_p, 0, sizeof(x_p) );
	memset(y_p, 0, sizeof(y_p) );
  	double XC = 1530/2;
  	double YC = 1020/2;
	bloblist *blobs = m_cBlob.getblobs();
	int i = 0, retval = -1;
  	double ra0, dec0, r0;
	
	for (i=0; i< nblobs; i++) {
		x[i] = blobs->getx();
		y[i] = blobs->gety();
		blobs = blobs->getnextblob();
		i++;
	}

	//AND change platescale to 9.44	
/*	x[0]=930.157011384096;
	y[0]=167.158449582342;
	x[1]=1472.58714161560;
	y[1]=922.480301367182;
	x[2]=908.983169656947;
	y[2]=323.056623586626;
	x[3]=1129.46618620267;
	y[3]=677.975355808673;
	x[4]=330.391564235287;
	y[4]=825.453326589156;

	for (i = 0; i < nblobs; i++) {
		x_p[i] = (x[i] - XC)*m_dPlatescale;
		y_p[i] = (y[i] - YC)*m_dPlatescale;
	}
*/
	if (m_cPyramid.GetSolution(m_dMatchTol, x_p, y_p, nblobs, sol, &retval, &ra0, &dec0, &r0) < 0)
		retval = -1;

	delete[] x;
	delete[] y;
	return retval;
}
#endif    //if USE_PYRAMID
