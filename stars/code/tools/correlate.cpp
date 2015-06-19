/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "correlate.h"
#include <algorithm>

using std::min;
using std::max;

void Tools::add_single_pixel_correlation(unsigned short indata[], double outdata[],
    int width, int height, int pixel_x, int pixel_y, double pixel_value)
{
    int u_offset = -pixel_x;
    int v_offset = -pixel_y;

    // at umin, u = umin and u_out = umin + u_offset
    // -> if the minimum were when u = 0, then umin = 0
    // -> if the minimum were when u_out = 0, then umin = 0 - u_offset
    int umin = max(0, 0 - u_offset);
    int vmin = max(0, 0 - v_offset);

    // at umax, u = umax and u_out = umax + u_offset
    // -> if the maximum were when u = width, then umax = width
    // -> if the maximum were when u_out = width, then umax = width - u_offset
    int umax = min(width, width - u_offset);
    int vmax = min(height, height - v_offset);

    int u_out = 0;
    int v_out = 0;
    for(int v=vmin; v<vmax; v++) {
        for(int u=umin; u<umax; u++) {
            u_out = u + u_offset;
            v_out = v + v_offset;
            outdata[v_out*width+u_out] += double(indata[v*width+u])*pixel_value;
        }
    }
}

