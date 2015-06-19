/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "camera.h"
#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include "GL/gl.h"
#include "GL/glu.h"
#include "block.h"
#include "dynamic_value.h"
#include "../shared/camera/requests.h"
#include "../shared/camera/results.h"

using namespace Displaying;

#define shared_requests (*(Shared::Camera::requests_for_camera.w))
#define shared_results (*(Shared::Camera::results_for_main.r))

Camera::Camera()
{
    gain.init("Gain");
    Color measured_color = Color(0.5, 0.5, 1.0, 1.0);
    gain.color = measured_color;
    change_size(140, 2*padding+2*text_height);
}

void Camera::update()
{
    if (shared_results.connected) {
        set_title("Camera (conn)");
    } else {
        set_title("Camera");
    }
    gain.set_value(shared_results.get_gain.value, 2);
    gain.known = false;
    if (shared_results.is_gain_valid(shared_requests)) {
        gain.known = true;
    }
}

void Camera::draw(Position &position)
{
    update();
    glColor3f(1.0, 1.0, 1.0);
    begin_draw(position);
    draw_border();
    draw_title();

    Position pos = {padding+padding, height() - padding - text_height};
    draw_value(gain, pos);

    end_draw();
}

