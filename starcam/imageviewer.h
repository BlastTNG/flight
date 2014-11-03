#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <qwidget.h>

/**
	@author Steve Benton <steve.benton@utoronto.ca>
	Qt widget for displaying BlobImage objects
*/

class BlobImage;
class QTimer;

class ImageViewer : public QWidget
{
	Q_OBJECT
public:
    ImageViewer(int width, int height, int img_width, int img_height, int msec = 10, QWidget *parent = 0, const char * name = 0);
    void load(BlobImage *img, bool autoBR = FALSE);
    static unsigned char conv16to8(unsigned short val16, unsigned short back, 
		    unsigned short range);
    int getRefreshTime() { return refreshTime; }
    void setRefreshTime(int msec);
    void manualRepaint() { needsRepaint = true; }

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
	bool autoBR;
	bool needsRepaint;
	bool loading;
};

#endif
