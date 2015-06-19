/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "base_set.h"
#include "blob.h"

using namespace Displaying;
using std::vector;

BaseSet::BaseSet(Solving::BaseSet base_set_): base_set(base_set_)
{
    timer.start();
    aged = false;
}

void BaseSet::draw_lines(vector<Blob>& display_blobs, double progress, bool done, double global_brightness, int num_blobs)
{
    double padding = 10.0;
    if (progress == 1.0 || done) {
        double brightness = 0.2;
        if (!aged) {
            double age = timer.time()+0.25;
            brightness = std::max(brightness, exp(-age/1.5));
            if (age > 3.0) {
                aged = true;
            }
        }
        glColor4d(0.0, 1.0, 0.5, brightness*global_brightness);
    }
    else {
		glColor4d(0.0, 1.0, 0.5, 1.0*global_brightness);
        timer.start();
    }
    glBegin(GL_LINES);
    double x0, y0, dx, dy, length, progress_short;
    int i, j;
    for (i=0; i<(num_blobs*(num_blobs-1)/2); i++) {
        j = (i+1)%num_blobs;
        x0 = display_blobs[base_set.ids[i]].x;
        y0 = display_blobs[base_set.ids[i]].y;
        dx = display_blobs[base_set.ids[j]].x - x0;
        dy = display_blobs[base_set.ids[j]].y - y0;

        length = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
        progress_short = progress * (1.0 - 2.0*padding/length);
        x0 += dx/length*padding;
        y0 += dy/length*padding;

        glVertex2d(x0, y0);
        glVertex2d(x0+dx*progress_short, y0+dy*progress_short);

    }
    glEnd();
}

void BaseSet::draw(vector<Blob>& display_blobs, double progress, bool done, double global_brightness)
{
    int num_blobs = 0;
    if (base_set.type == Solving::BaseSet::triplet) {
        num_blobs = 3;
    }
    if (base_set.type == Solving::BaseSet::pair) {
        num_blobs = 2;
    }
    draw_lines(display_blobs, progress, done, global_brightness, num_blobs);

}

