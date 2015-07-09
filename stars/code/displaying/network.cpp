/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#define NOMINMAX
#include "network.h"
#include "../shared/network/packets.h"
#include "../shared/network/client.h"
#include "../shared/camera/results.h"

using namespace Displaying;
using std::string;

#define shared_packets_from_fc1 (*(Shared::Network::packets_from_fc1.r))
#define shared_client (*(Shared::Network::client_for_main.r))
#define shared_camera_results (*(Shared::Camera::results_for_main.r))

Network::Network()
{
    string name;
    name = "ctr_f";
    counter_fcp.init(name);
    name = "ctr_s";
    counter_stars.init(name);

    //change_size(140, 2*padding+3*text_height);
    change_size(140, 2*padding+1*text_height);

}

void Network::draw_packets()
{
    Position pos = {padding+72, height() - padding};
    double scale = 0.0;
    double dim_rate = 2.0;

    scale = std::max(1.0 - shared_packets_from_fc1.time_since_sent.time()*dim_rate, 0.0);
    Color color_sent(1.0*scale, 0.7*scale, 0.7*scale, 1.0);
    scale = std::max(1.0 - shared_packets_from_fc1.time_since_received.time()*dim_rate, 0.0);
    Color color_received(0.7*scale, 1.0*scale, 0.7*scale, 1.0);

    set_color(color_sent, true);
    draw_text("s", pos);
    pos.x += 11;
    set_color(color_received, true);
    draw_text("r", pos);
}

void Network::draw(Position &position)
{
    update_size();

    counter_fcp.set_value(shared_client.counter_fcp, 0);
    counter_stars.set_value(shared_camera_results.counter_stars, 0);

    begin_draw(position);
    draw_border();
    draw_title();
    draw_packets();
    //Position pos = {padding+padding, height() - padding - text_height};
    //draw_value(counter_fcp, pos);
    //pos.y -= text_height;
    //draw_value(counter_stars, pos);
    end_draw();
}
