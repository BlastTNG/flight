#include "imageviewer.h"
#include <qapplication.h>
#include <qimage.h>
#include <qlabel.h>
#include <qpainter.h>
#include <qtimer.h>
#include "blobimage.h"

#define VIEWER_DEBUG 1
#if VIEWER_DEBUG
#include <iostream>
#endif

/*

 Image Viewer:
 
 constructor, creates a widget for displaying a BlobImage of width w and height h
 image will be set to refresh after a time of msec milliseconds
 parent widget is for Qt purposese, and name used internally only
 
*/
ImageViewer::ImageViewer(int w, int h, int msec/*=10*/, QWidget* parent/*=0*/, const char * name/*=0*/)
	: QWidget(parent, name)
{
#if VIEWER_DEBUG
	cerr << "[Viewer debug]: constructing with w=" << w << " h=" << h << " msec=" << msec << endl;
#endif
	this->setFixedSize(w, h);
	this->setPalette(QPalette(QColor(0, 0, 0)));
	
	qimg = new QImage(w, h, 8, 256);
	for (int i=0; i<256; i++)                  //make image greyscale
		qimg->setColor(i, qRgb(i, i, i));
	
	bimg = NULL;                   //no image has been loaded
	needsRepaint = false;          //no need until first load
	autoBR = FALSE;                //remembers last used value
	loading = false;               //not in the process of loading an image
	
	refreshTimer = new QTimer(this, "refresh timer");
	connect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
	refreshTimer->start(msec);
	
}

/*

 ~ImageViewer:
 
 destructor, cleans up image data (label is taken care of by Qt)
 
*/
ImageViewer::~ImageViewer()
{
#if VIEWER_DEBUG
	cerr << "[Viewer debug]: in destructor" << endl;
#endif
	delete qimg;
}

/*

 load:
 
 loads a BlobImage into the window, converts from 16bpp to 8bp, repaints
 if autoBR is TRUE, will find a recommended background and range value for the image
 reasonable values for background and range are needed for good converion
 
*/
void ImageViewer::load(BlobImage* img, bool in_autoBR/*=FALSE*/)
{
#if VIEWER_DEBUG
	cerr << "[Viewer debug]: loading an image" << endl;
#endif
	loading = true;
	bimg = img;
	autoBR = in_autoBR;
	if (autoBR) img->AutoBackgroundAndRange();
	unsigned short background = img->GetBackground();
	unsigned short range = img->GetRange();
	unsigned short *data16 = img->GetImagePointer();
	
	for (int i=0; i<height(); i++) {
		unsigned char *qline = qimg->scanLine(i);
		unsigned short *bline = data16 + i*width();
		for (int j=0; j<width(); j++)
			*(qline+j) = conv16to8(*(bline+j),background, range);
	}
	bimg->setChanged(false);
	needsRepaint = true;
	loading = false;
	
}

/*

 conv16to8:
 
 converts a 16bpp pixel value to an 8bpp one for given image background and range
 
*/
unsigned char ImageViewer::conv16to8(unsigned short val16, unsigned short back, 
							   unsigned short range)
{
	unsigned char result;
	if (val16 <= back) result = (unsigned char)0;
	else if (val16 >= back + range) result = (unsigned char)255;
	else result = (unsigned char)((val16 - back) * 255 / range);
	
	return result;
}

/*

 setRefreshTime:
 
 sets the refresh interval to msec milliseconds
 
*/
void ImageViewer::setRefreshTime(int msec)
{
	refreshTimer->changeInterval(msec);
}

/*

 refresh:
 
 public slot for refreshing the image
 
*/
void ImageViewer::refresh()
{
#if VIEWER_DEBUG
	static int count = 1;
#endif
	if (loading) return;  //do nothing while an image is loading

	if (bimg && bimg->isChanged()) //check if changed without load
		load(bimg, autoBR);
	if (needsRepaint && bimg) {
#if VIEWER_DEBUG
	  cerr << "[Viewer debug]: " << count++ << " repainting the image. " << endl;
#endif
		repaint();
		needsRepaint = 0;
	}
}

/*

 paintEvent:
 
 unoptimized paint function that redraws entire image on every repaint
 
*/
void ImageViewer::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.drawImage(this->rect(), *qimg);
}


