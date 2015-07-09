/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "solving_filters.h"
#include "../tools/angles.h"
#include "../shared/solving/filters.h"

using namespace Displaying;

#define shared_filters (*(Shared::Solving::filters_net_to_main.r))

SolvingFilters::SolvingFilters()
{
    change_size(630, 140);

    Color color = Color(0.0, 1.0, 0.0, 1.0);

    horizontal_location_enabled.init("ho");
    horizontal_location_radius.init("ho_rad");
    horizontal_location_az.init("ho_az");
    horizontal_location_el.init("ho_el");
    horizontal_roll_limit_enabled.init("hroll");
    horizontal_roll_limit_min.init("hroll_min");
    horizontal_roll_limit_max.init("hroll_max");
    horizontal_el_limit_enabled.init("el");
    horizontal_el_limit_min.init("el_min");
    horizontal_el_limit_max.init("el_max");
    equatorial_location_enabled.init("eq");
    equatorial_location_radius.init("eq_rad");
    equatorial_location_ra.init("eq_ra");
    equatorial_location_dec.init("eq_dec");

    horizontal_location_enabled.color = color;
    horizontal_location_radius.color = color;
    horizontal_location_az.color = color;
    horizontal_location_el.color = color;
    equatorial_location_enabled.color = color;
    equatorial_location_radius.color = color;
    equatorial_location_ra.color = color;
    equatorial_location_dec.color = color;
    horizontal_roll_limit_enabled.color = color;
    horizontal_roll_limit_min.color = color;
    horizontal_roll_limit_max.color = color;
    horizontal_el_limit_enabled.color = color;
    horizontal_el_limit_min.color = color;
    horizontal_el_limit_max.color = color;
}

void SolvingFilters::update()
{
    horizontal_location_enabled.known = false;
    horizontal_location_radius.known = false;
    horizontal_location_az.known = false;
    horizontal_location_el.known = false;
    if (shared_filters.horizontal_location.enabled) {
        horizontal_location_enabled.set_value("enabled");
        horizontal_location_enabled.known = true;
        horizontal_location_radius.set_value(to_degrees(shared_filters.horizontal_location.radius), "*", 1);
        horizontal_location_radius.known = true;
        horizontal_location_az.set_value(to_degrees(shared_filters.horizontal_location.az), "*", 1);
        horizontal_location_az.known = true;
        horizontal_location_el.set_value(to_degrees(shared_filters.horizontal_location.el), "*", 1);
        horizontal_location_el.known = true;
    }
    horizontal_roll_limit_enabled.known = false;
    horizontal_roll_limit_min.known = false;
    horizontal_roll_limit_max.known = false;
    if (shared_filters.horizontal_roll_limit.enabled) {
        horizontal_roll_limit_enabled.set_value("enabled");
        horizontal_roll_limit_enabled.known = true;
        horizontal_roll_limit_min.set_value(to_degrees(shared_filters.horizontal_roll_limit.min_roll), "*", 1);
        horizontal_roll_limit_min.known = true;
        horizontal_roll_limit_max.set_value(to_degrees(shared_filters.horizontal_roll_limit.max_roll), "*", 1);
        horizontal_roll_limit_max.known = true;
    }
    horizontal_el_limit_enabled.known = false;
    horizontal_el_limit_min.known = false;
    horizontal_el_limit_max.known = false;
    if (shared_filters.horizontal_elevation_limit.enabled) {
        horizontal_el_limit_enabled.set_value("enabled");
        horizontal_el_limit_enabled.known = true;
        horizontal_el_limit_min.set_value(to_degrees(shared_filters.horizontal_elevation_limit.min_el), "*", 1);
        horizontal_el_limit_min.known = true;
        horizontal_el_limit_max.set_value(to_degrees(shared_filters.horizontal_elevation_limit.max_el), "*", 1);
        horizontal_el_limit_max.known = true;
    }
    equatorial_location_enabled.known = false;
    equatorial_location_radius.known = false;
    equatorial_location_ra.known = false;
    equatorial_location_dec.known = false;
    if (shared_filters.equatorial_location.enabled) {
        equatorial_location_enabled.set_value("enabled");
        equatorial_location_enabled.known = true;
        equatorial_location_radius.set_value(to_degrees(shared_filters.equatorial_location.radius), "*", 1);
        equatorial_location_radius.known = true;
        equatorial_location_ra.set_value(to_degrees(shared_filters.equatorial_location.ra), "*", 1);
        equatorial_location_ra.known = true;
        equatorial_location_dec.set_value(to_degrees(shared_filters.equatorial_location.dec), "*", 1);
        equatorial_location_dec.known = true;
    }
}

void SolvingFilters::draw(Position& position)
{
    update();

    begin_draw(position);
    draw_border();
    draw_title();

    Position pos = {padding + padding, height() - padding - text_height - 4};
    draw_value(horizontal_location_enabled, pos);
    pos.y -= text_height;
    draw_value(horizontal_location_radius, pos);
    pos.y -= text_height;
    draw_value(horizontal_location_az, pos);
    pos.y -= text_height;
    draw_value(horizontal_location_el, pos);

    pos.x += 160;
    pos.y = height() - padding - text_height - 4;
    draw_value(horizontal_roll_limit_enabled, pos);
    pos.y -= text_height;
    draw_value(horizontal_roll_limit_min, pos);
    pos.y -= text_height;
    draw_value(horizontal_roll_limit_max, pos);

    pos.x += 160;
    pos.y = height() - padding - text_height - 4;
    draw_value(horizontal_el_limit_enabled, pos);
    pos.y -= text_height;
    draw_value(horizontal_el_limit_min, pos);
    pos.y -= text_height;
    draw_value(horizontal_el_limit_max, pos);

    pos.x += 160;
    pos.y = height() - padding - text_height - 4;
    draw_value(equatorial_location_enabled, pos);
    pos.y -= text_height;
    draw_value(equatorial_location_radius, pos);
    pos.y -= text_height;
    draw_value(equatorial_location_ra, pos);
    pos.y -= text_height;
    draw_value(equatorial_location_dec, pos);

    end_draw();
}

