/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

namespace Tools
{

template <typename T>
void correlate2d(T data[], double psf[], double outdata[],
        int halfwidth, int halfheight, int width, int height)
{
	int i, j, u, v, iu, jv;
	int psfwidth, psfheight;
	double result;
	psfwidth = 2*halfwidth + 1;
	psfheight = 2*halfheight + 1;

	for(j=0; j<height; j++){
        for(i=0; i<width; i++){
            result = 0;
            if( (i >= halfwidth) && (i < (width-halfwidth)) &&
            (j >= halfheight) && (j < (height-halfheight)) ){
                for(v=0; v<psfheight; v++){
                    for(u=0; u<psfwidth; u++){
                        iu = i - halfwidth + u;
                        jv = j - halfheight + v;
                        result += (psf[v*psfwidth+u] * data[jv*width+iu]);
                    }
                }
            } else{
                // this looks like mirroring
                for(v=0; v<psfheight; v++){
                    for(u=0; u<psfwidth; u++){
                        iu = i - halfwidth + u;
                        jv = j - halfheight + v;
                        if(iu < 0)
                            iu = -iu;
                        else if(iu >= width)
                            iu = (width-1) - (iu - (width-1));
                        if(jv < 0)
                            jv = -jv;
                        else if(jv >= height)
                            jv = (height-1) - (jv - (height-1));
                        result += (psf[v*psfwidth+u] * data[jv*width+iu]);
                    }
                }
            }
            outdata[j*width+i] = result;
        }
	}
}

void add_single_pixel_correlation(unsigned short indata[], double outdata[], int width, int height,
    int pixel_x, int pixel_y, double pixel_value);

}

