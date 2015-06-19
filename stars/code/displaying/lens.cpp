/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "lens.h"
#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include <cstdio>
#include <string>
#include <math.h>
#include <boost/format.hpp>
#include "GL/gl.h"
#include "GL/glu.h"
#include "../shared/lens/requests.h"
#include "../shared/lens/results.h"

using namespace Displaying;
using std::string;
#define shared_fcp_requests (*(Shared::Lens::fcp_requests_main_to_camera.w))
#define shared_fcp_results (*(Shared::Lens::fcp_results_camera_to_main.r))
#define shared_stars_requests (*(Shared::Lens::stars_requests_main_to_camera.w))
#define shared_stars_results (*(Shared::Lens::stars_results_camera_to_main.r))

Lens::Lens()
{
    string title;
    title = "Focus";
    focus.init(title);
    title = "Apert";
    aperture.init(title);
    Color measured_color = Color(0.5, 0.5, 1.0, 1.0);
    focus.color = measured_color;
    aperture.color = measured_color;

    change_size(140, 2*padding+3*text_height);
}

void Lens::update()
{
    if (shared_fcp_results.device_found) {
        set_title((boost::format("Lens (%s)")%shared_fcp_results.device_name).str());
    } else {
        set_title(string("Lens"));
    }

    focus.set_value(shared_fcp_results.focus_value, 0);
    focus.known = false;
    aperture.set_value(shared_fcp_results.aperture_value, 0);
    aperture.known = false;

    if (shared_fcp_results.is_focus_valid(shared_fcp_requests) &&
        shared_stars_results.is_focus_valid(shared_stars_requests))
    {
        focus.known = true;
    }
    if (shared_fcp_results.is_aperture_valid(shared_fcp_requests) &&
        shared_stars_results.is_aperture_valid(shared_stars_requests))
    {
        aperture.known = true;
    }
}

void Lens::draw(Position &position)
{
    update();

    glColor3f(1.0, 1.0, 1.0);
    begin_draw(position);

    draw_border();
    draw_title();

    Position pos = {padding+padding, height() - padding - text_height};
    draw_value(focus, pos);
    pos.y -= text_height;
    draw_value(aperture, pos);


    end_draw();
}

