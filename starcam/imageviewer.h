#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <qwidget.h>

/**
	@author Steve Benton <steve.benton@utoronto.ca>
	Qt widget for displaying BlobImage objects
*/

class BlobImage;
class QTimer;

class ImageViewer : public QWidget{
	Q_OBJECT
public:
    ImageViewer(int width, int height, int msec = 1000, QWidget *parent = 0, const char * name = 0);
	void load(BlobImage *img, bool autoBR = FALSE);
	static unsigned char conv16to8(unsigned short val16, unsigned short back, 
								   unsigned short range);
	int getRefreshTime() { return refreshTime; }
	void setRefreshTime(int msec);
	void setLoadOnRefresh(bool flag) { loadOnRefresh = flag; }

    ~ImageViewer();
	
public slots:
	void refresh();
	
protected:
	void paintEvent(QPaintEvent*);
	
private:
	QImage *qimg;       //copy of BlobImage in QImage format
	QTimer *refreshTimer;
	int refreshTime;    //time (ms) between image refreshes
	
	BlobImage *bimg;    //pointer to current blob image
	bool loadOnRefresh; //flag that can trigger reloading of image data on refresh
	bool autoBR;
};

#endif
