/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "image.h"
#include <string>
#include <boost/format.hpp>
#include "../tools/angles.h"
#include "../shared/image/blobs.h"
#include "../shared/image/status.h"
#include "../shared/image/stats.h"
#include "../shared/image/solution_summary.h"

using namespace Displaying;

#define shared_blobs (*(Shared::Image::blobs_solver_for_main.r))
#define shared_status (*(Shared::Image::status_solver_for_main.r))
#define shared_stats (*(Shared::Image::stats_solver_for_main.r))
#define shared_solution (*(Shared::Image::solution_summary_for_main.r))

Image::Image()
{
    change_size(630, 140);

    Color stored_color = Color(0.0, 1.0, 0.0, 1.0);
    Color measured_color = Color(0.5, 0.5, 1.0, 1.0);
    Color solution_color = Color(1.0, 1.0, 0.5, 1.0);

    age.init("age");
    counter_stars.init("ctr_s");
    counter_fcp.init("ctr_f");
    lat.init("lat");
    lst.init("lst");
    mean.init("mean");
    noise.init("noise");
    gain.init("gain");
    num_pixels_saturated.init("num_sat");
    stage.init("");
    ra.init("ra");
    dec.init("dec");
    az.init("az");
    el.init("el");
    hroll.init("hroll");
    matched.init("match");
    measured_exposure.init("mexp");
    pointing_error.init("pt error");
    fit_error.init("fit error");
    platescale.init("plate");

    age.color = stored_color;
    counter_stars.color = stored_color;
    counter_fcp.color = stored_color;
    lat.color = stored_color;
    lst.color = stored_color;
    mean.color = measured_color;
    noise.color = measured_color;
    gain.color = measured_color;
    num_pixels_saturated.color = measured_color;
    stage.color = solution_color;
    ra.color = solution_color;
    dec.color = solution_color;
    az.color = solution_color;
    el.color = solution_color;
    hroll.color = solution_color;
    matched.color = solution_color;
    measured_exposure.color = solution_color;
    pointing_error.color = solution_color;
    fit_error.color = solution_color;
    platescale.color = solution_color;
}

void Image::update()
{
    using std::string;

    set_title("Emptiness");
    age.known = false;
    counter_stars.known = false;
    counter_fcp.known = false;
    lat.known = false;
    lst.known = false;
    stage.known = false;
    if (shared_status.age.started()) {
        if (shared_status.num_exposures > 1) {
            set_title((boost::format("%s (%i)") % shared_status.filename % shared_status.num_exposures).str());
        } else {
            set_title((boost::format("%s") % shared_status.filename).str());
        }
        age.set_value(shared_status.age.time(), " s", 0);
        age.known = true;
        if (shared_status.from_camera) {
            counter_stars.set_value(shared_status.counter_stars, "", 0);
        } else {
            counter_stars.set_value(shared_status.counter_stars, " [nc]", 0);
        }
        counter_stars.known = true;
        counter_fcp.set_value(shared_status.counter_fcp, "", 0);
        counter_fcp.known = true;
        if (shared_status.horizontal_known) {
            lat.set_value(to_degrees(shared_status.lat), "*", 2);
            lat.known = true;
            lst.set_value(to_hours(shared_status.lst), " hrs", 2);
            lst.known = true;
        }
        switch (shared_status.stage) {
            using namespace Shared::Image;
            case Status::empty:
                stage.set_value(""); break;
            case Status::doing_statistics:
                stage.set_value("getting stats..."); break;
            case Status::blob_finding:
                stage.set_value("blob finding..."); break;
            case Status::pattern_matching:
                stage.set_value("pattern matching..."); break;
            case Status::refitting:
                stage.set_value("refitting..."); break;
            case Status::getting_names:
                stage.set_value("getting names..."); break;
            case Status::done:
            {
                string reason_string = "";
                switch(shared_status.reason_for_being_done) {
                    case Status::no_reason:
                        reason_string = "no reason specified"; break;
                    case Status::solved:
                        reason_string = "solved"; break;
                    case Status::not_solving:
                        reason_string = "not solving"; break;
                    case Status::saturated:
                        reason_string = "too saturated"; break;
                    case Status::tried_all_patterns:
                        reason_string = "tried_all_patterns"; break;
                    case Status::timed_out:
                        reason_string = "timed out"; break;
                    case Status::aborted:
                        reason_string = "aborted"; break;
                    default:
                        break;
                }
                string solving_time_string = "";
                if (shared_solution.counter_stars == shared_status.counter_stars) {
                    if (shared_solution.equatorial.valid) {
                        solving_time_string = (boost::format(" (%.1f s)") % shared_solution.solving_time).str();
                    }
                }
                stage.set_value((boost::format("done, %s%s") % reason_string % solving_time_string).str());
                break;
            }
            default:
                stage.set_value("");
                stage.known = false;
        }
    }
    if (shared_blobs.motion_psf_used) {
        string motion_psf_symbol = "~ ";
        stage.value = motion_psf_symbol.append(stage.value);
    }

    mean.known = false;
    noise.known = false;
    gain.known = false;
    num_pixels_saturated.known = false;
    if (shared_stats.counter_stars == shared_status.counter_stars) {
        if (shared_stats.mean_known) {
            mean.set_value(shared_stats.mean, "", 1);
            if (shared_stats.fraction_pixels_saturated > 0.001) {
                num_pixels_saturated.set_value(shared_stats.fraction_pixels_saturated*100.0, "%", 1);
            } else {
                num_pixels_saturated.set_value(shared_stats.num_pixels_saturated, "", 0);
            }
        }
        if (shared_stats.noise_known) {
            noise.set_value(shared_stats.noise, "", 1);
        }
        if (shared_stats.gain_known) {
            gain.set_value(shared_stats.gaindb, " dB", 1);
        }
    }

    ra.known = false;
    dec.known = false;
    az.known = false;
    el.known = false;
    hroll.known = false;
    matched.known = false;
    measured_exposure.known = false;
    pointing_error.known = false;
    fit_error.known = false;
    platescale.known = false;
    if (shared_solution.counter_stars == shared_status.counter_stars) {
        if (shared_solution.equatorial.valid) {
            ra.set_value(to_degrees(shared_solution.equatorial.ra), "*", 2);
            dec.set_value(to_degrees(shared_solution.equatorial.dec), "*", 2);
            matched.set_value((boost::format("%s of %s") % shared_solution.num_blobs_matched % shared_solution.num_blobs_total).str());
            measured_exposure.set_value(shared_solution.measured_exposure*1000.0, " ms", 0);
            pointing_error.set_value(to_arcsec(shared_solution.equatorial.sigma_pointing), "\"", 2);
            fit_error.set_value(shared_solution.equatorial.fit_error, " px", 2);
            platescale.set_value(to_arcsec(shared_solution.equatorial.iplatescale), "\"", 3);
        }
        if (shared_solution.horizontal.valid) {
            az.set_value(to_degrees(shared_solution.horizontal.az), "*", 2);
            el.set_value(to_degrees(shared_solution.horizontal.el), "*", 2);
            hroll.set_value(to_degrees(shared_solution.horizontal.roll), "*", 2);
        }
    }
}

void Image::draw(Position& position)
{
    update();

    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 0
    begin_draw(position);
    draw_border();
    draw_title();

    Position pos = {padding + padding, height() - padding - text_height - 4};
    draw_value(age, pos);
    pos.y -= text_height;
    draw_value(counter_stars, pos);
    pos.y -= text_height;
    draw_value(counter_fcp, pos);
    pos.y -= text_height;
    draw_value(lat, pos);
    pos.y -= text_height;
    draw_value(lst, pos);

    pos.x += 160;
    pos.y = height() - padding - text_height - 4;
    draw_value(mean, pos);
    pos.y -= text_height;
    draw_value(noise, pos);
    pos.y -= text_height;
    draw_value(num_pixels_saturated, pos);
    pos.y -= text_height;
    draw_value(gain, pos);

    pos.y = height() - padding;
    pos.x += 160;
    draw_value(stage, pos);
    pos.y -= 4;
    pos.y -= text_height;
    draw_value(ra, pos);
    pos.y -= text_height;
    draw_value(dec, pos);
    pos.y -= text_height;
    draw_value(az, pos);
    pos.y -= text_height;
    draw_value(el, pos);
    pos.y -= text_height;
    draw_value(hroll, pos);

    pos.y = height() - padding - text_height - 4;
    pos.x += 160;
    draw_value(matched, pos);
    pos.y -= text_height;
    draw_value(pointing_error, pos);
    pos.y -= text_height;
    draw_value(fit_error, pos);
    pos.y -= text_height;
    draw_value(platescale, pos);
    pos.y -= text_height;
    draw_value(measured_exposure, pos);

    end_draw();
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 0
}

