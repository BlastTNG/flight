/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#define NOMINMAX
#include "name.h"
#include "../tools/math.h"
#include "../shared/network/packets.h"
#include "../shared/general/network_reset.h"
#include "../tools/quick_cout.h"

using namespace Displaying;

#define shared_packets_from_fc1 (*(Shared::Network::packets_from_fc1.r))
#define shared_packets_from_fc2 (*(Shared::Network::packets_from_fc2.r))
#define shared_network_reset_status (*(Shared::General::network_reset_status_for_main.r))

Name::Name()
{
    change_size(140, 2*padding+1*text_height);
    timer.start();
}

void Name::draw_packets()
{
    Position pos = {width()-padding-55, height() - padding};
    double dim_rate = 2.0;

    Color gray(0.8, 0.8, 0.8, 1.0);
    Color green(0.5, 1.0, 0.5, 1.0);

    {
        double scale = std::max(1.0 - shared_packets_from_fc1.time_since_sent.time()*dim_rate, 0.0);
        set_color(gray*scale, true);
        draw_text("s", pos);
        pos.x += 11;
        scale = std::max(1.0 - shared_packets_from_fc1.time_since_received.time()*dim_rate, 0.0);
        if (shared_packets_from_fc1.in_charge) {
            set_color(green*scale, true);
            draw_text("R", pos);
        } else {
            set_color(gray*scale, true);
            draw_text("r", pos);
        }
    }

    pos.x += 20;
    {
        double scale = std::max(1.0 - shared_packets_from_fc2.time_since_sent.time()*dim_rate, 0.0);
        set_color(gray*scale, true);
        draw_text("s", pos);
        pos.x += 11;
        scale = std::max(1.0 - shared_packets_from_fc2.time_since_received.time()*dim_rate, 0.0);
        if (shared_packets_from_fc2.in_charge) {
            set_color(green*scale, true);
            draw_text("R", pos);
        } else {
            set_color(gray*scale, true);
            draw_text("r", pos);
        }
    }
}

void Name::draw_network_resetting()
{
    double center_x = 105.0;
    double center_y = 15.0;
    double angle = timer.time()*2*M_PI*1.5;
    double phase = M_PI/2.0;
    double x1_offset = cos(angle)*20.0;
    double y1_offset = sin(angle)*3.0;
    double x2_offset = cos(angle+phase)*20.0;
    double y2_offset = sin(angle+phase)*5.0;

    if (timer.time() > 10000.0) {
        timer.start();
    }

    glPushMatrix();
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    glColor4f(1.0, 0.5, 0.5, 1.0);
    glVertex2d((GLfloat) (center_x+x1_offset), (GLfloat) (center_y+y1_offset));
    glColor4f(0.0, 1.0, 0.5, 1.0);
    glVertex2d((GLfloat) (center_x+x2_offset), (GLfloat) (center_y+y2_offset));
    glEnd();
    glPointSize(1.0f);
    glPopMatrix();
}

void Name::draw(Position &position)
{
    update_size();
    begin_draw(position);
    draw_border();
    draw_title();
    if (shared_network_reset_status.resetting) {
        draw_network_resetting();
    } else {
        draw_packets();
    }
    end_draw();
}

