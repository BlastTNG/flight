/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "housekeeping.h"
#if defined(_MSC_VER)
    #define NOMINMAX
    #include <windows.h>
#endif
#include <string>
#include <math.h>
#include <limits>
#include "GL/gl.h"
#include "GL/glu.h"
#include "../shared/housekeeping/housekeeper.h"
#include "../tools/timing.h"

using namespace Displaying;
using std::min;
using std::max;
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_camera.w))

Housekeeping::Housekeeping()
{
    change_size(140, 2*padding+1*text_height);
}

void Housekeeping::update()
{
    Color measured_color = Color(0.5, 0.5, 1.0, 1.0);
    unsigned int num_measurements = shared_housekeeper.measurements.size();
    if (measurements.size() != num_measurements) {
        measurements.clear();
        for (unsigned int i=0; i<num_measurements; i++) {
            DynamicValue measurement;
            measurement.init(shared_housekeeper.measurements[i].name);
            measurement.color = measured_color;
            measurements.push_back(measurement);
        }
        Size new_size;
        new_size.w = width();
        new_size.h = min((unsigned int) 2, num_measurements+1)*padding + (num_measurements+1-1)*text_height;
        change_size(new_size, true);
    }
    for (unsigned int i=0; i<num_measurements; i++) {
        if (shared_housekeeper.measurements[i].valid) {
            if (shared_housekeeper.measurements[i].name == "disktime") {
                measurements[i].set_value(Time::time_string(shared_housekeeper.measurements[i].value));
            }
            else {
                int precision = 1;
                if (shared_housekeeper.measurements[i].units == " a") {
                    precision = 2;
                }
                measurements[i].set_value(shared_housekeeper.measurements[i].value,
                    shared_housekeeper.measurements[i].units, precision);
            }
        }
    }
    update_size(true);
}

void Housekeeping::draw(Position &position)
{
    begin_draw(position);
    draw_border();
    draw_title();

    if (size_update_finished()) {
        Position pos = {padding+padding, height() - padding - text_height};
        if (measurements.size() >= 2) {
            if (disk_timer.time() > 4.0) {
                disk_timer.start();
            }
            if (measurements[1].known && disk_timer.time() > 2.0) {
                draw_value(measurements[1], pos);
            } else {
                draw_value(measurements[0], pos);
            }
            pos.y -= text_height;
        }
        for (unsigned int i=2; i<measurements.size(); i++) {
            draw_value(measurements[i], pos);
            pos.y -= text_height;
        }
    }

    end_draw();
}

