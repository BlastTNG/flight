/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "autofocus.h"
#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include <boost/format.hpp>
#include <limits>
#include "GL/gl.h"
#include "GL/glu.h"
#include "glhelper.h"
#include "block.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "../shared/autofocus/results.h"
#include "../shared/autofocus/requests.h"
#include "../shared/autofocus/datapoints.h"
#include "../tools/quick_cout.h"

using namespace Displaying;
using namespace Shared::Autofocus;
using std::min;
#define shared_results (*(results_solver_to_main.r))
#define shared_requests (*(requests_network_to_main.r))
#define shared_datapoints (*(datapoints_lens_to_main.r))

Autofocus::Autofocus()
{
    change_size(200, 100);
}

bool Autofocus::display_enabled()
{
    if (shared_requests.display_mode == xC_autofocus_display_mode_auto) {
        if (shared_results.mode != Results::mode_not_running) {
            return true;
        }
    }
    if (shared_requests.display_mode == xC_autofocus_display_mode_on) {
        return true;
    }
    return false;
}

void Autofocus::get_plot_ranges(double& value_min, double& value_max, double& focus_min,
    double& focus_max, datapoints_t metric_type, bool any_metric_except_sobel)
{
    value_min = std::numeric_limits<double>::infinity();
    value_max = 0.0;
    focus_min = shared_requests.focus_search_min;
    focus_max = shared_requests.focus_search_max;
    for (unsigned int i=0; i<shared_datapoints.curves.size(); i++) {
        if (shared_datapoints.curves[i].type == metric_type || (any_metric_except_sobel && shared_datapoints.curves[i].type != metric_sobel)) {
            for (unsigned int j=0; j<shared_datapoints.curves[i].points.size(); j++) {
                if (shared_datapoints.curves[i].points[j].value > value_max) {
                    value_max = shared_datapoints.curves[i].points[j].value;
                }
                if (shared_datapoints.curves[i].points[j].value < value_min) {
                    value_min = shared_datapoints.curves[i].points[j].value;
                }
                if (shared_datapoints.curves[i].points[j].focus > focus_max) {
                    focus_max = shared_datapoints.curves[i].points[j].focus;
                }
                if (shared_datapoints.curves[i].points[j].focus < focus_min) {
                    focus_min = shared_datapoints.curves[i].points[j].focus;
                }
            }
        }
    }
}

void Autofocus::set_curve_color_and_width(datapoints_t metric_type)
{
    switch (metric_type) {
        case metric_brightest_blob_flux:
            GL::Color4f(0.33, 0.33, 0.75, 1.0);
            glLineWidth(2.0);
            break;
        case metric_star_flux:
            GL::Color4f(0.5, 0.0, 0.15, 1.0);
            glLineWidth(1.0);
            break;
        case metric_sobel:
            GL::Color4f(0.0, 0.5, 0.1, 1.0);
            glLineWidth(2.0);
            break;
        default:
            GL::Color4f(0.2, 0.2, 0.2, 1.0);
            glLineWidth(1.0);
            break;
    }
}

void Autofocus::plot_curves(double value_min, double value_max, double focus_min, double focus_max,
    double plot_offset, datapoints_t metric_type, double plot_width, double plot_height)
{
    for (unsigned int i=0; i<shared_datapoints.curves.size(); i++) {
        if (shared_datapoints.curves[i].type == metric_type) {
            set_curve_color_and_width(shared_datapoints.curves[i].type);
            glBegin(GL_LINE_STRIP);

            double curve_progress = 2.0*shared_datapoints.curves[i].age_since_last_datapoint_added.time();
            curve_progress *= plot_width;
            bool last_point_was_displayable = false;
            double last_x = 0.0;
            double last_y = 0.0;
            for (unsigned int j=0; j<shared_datapoints.curves[i].points.size(); j++) {
                double focus = shared_datapoints.curves[i].points[j].focus;
                double value = shared_datapoints.curves[i].points[j].value;
                double x = (focus-focus_min) / (focus_max-focus_min);
                double y = (value-value_min) / (value_max-value_min) + plot_offset;
                x *= plot_width;
                y *= plot_height;
                if (x <= curve_progress) {
                    glVertex2d(x, y);
                    last_point_was_displayable = true;
                    last_x = x;
                    last_y = y;
                }
                if (x > curve_progress && last_point_was_displayable) {
                    last_point_was_displayable = false;
                    double cropped_x = min(x, curve_progress);
                    double fraction = (cropped_x - last_x) / (x - last_x);
                    double cropped_y = (y-last_y)*fraction + last_y;
                    glVertex2d(cropped_x, cropped_y);
                }
            }
            glEnd();
            glLineWidth(2.0);
        }
    }
    double point_halfsize = 1.25;
    for (unsigned int i=0; i<shared_datapoints.curves.size(); i++) {
        if (shared_datapoints.curves[i].type == metric_type) {
            set_curve_color_and_width(shared_datapoints.curves[i].type);

            double curve_progress = 2.0*shared_datapoints.curves[i].age_since_last_datapoint_added.time();
            curve_progress *= plot_width;
            for (unsigned int j=0; j<shared_datapoints.curves[i].points.size(); j++) {
                double focus = shared_datapoints.curves[i].points[j].focus;
                double value = shared_datapoints.curves[i].points[j].value;
                double x = (focus-focus_min) / (focus_max-focus_min);
                double y = (value-value_min) / (value_max-value_min) + plot_offset;
                x *= plot_width;
                y *= plot_height;
                if (x <= curve_progress) {
                    glBegin(GL_POLYGON);
                        glVertex2f((GLfloat) (x - point_halfsize), (GLfloat) (y - point_halfsize));
                        glVertex2f((GLfloat) (x - point_halfsize), (GLfloat) (y + point_halfsize));
                        glVertex2f((GLfloat) (x + point_halfsize), (GLfloat) (y + point_halfsize));
                        glVertex2f((GLfloat) (x + point_halfsize), (GLfloat) (y - point_halfsize));
                    glEnd();
                }
            }
        }
    }
}

void Autofocus::draw(Position &position)
{
    if (shared_results.mode == Results::mode_running) {
        set_title("Autofocus (running)");
    } else if (shared_results.mode == Results::mode_finished_and_gracing) {
        set_title("Autofocus (finished)");
    }

    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 0
    GL::Color4f(1.0, 1.0, 1.0, 1.0);
    begin_draw(position);
    draw_border();
    draw_title();

    double plot_padding = 10.0;
    double plot_width = width() - 2.0*plot_padding;
    double plot_height = height() - 2.0*plot_padding - 2.0*text_height;
    double plot_offset = 2.0 / plot_height;

	glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    glTranslatef((GLfloat) plot_padding, (GLfloat) (text_height+plot_padding), 0.0f);
    GL::Color4f(1.0, 1.0, 1.0, 1.0);
    glBegin(GL_LINES);
        glVertex2d(0.0, 0.0);
        glVertex2d(0.0, (GLfloat) plot_height);
        glVertex2d(0.0, 0.0);
        glVertex2d((GLfloat) plot_width, 0.0);
    glEnd();

    double value_min, value_max, focus_min, focus_max;
    double focus_min_all, focus_max_all;
    get_plot_ranges(value_min, value_max, focus_min_all, focus_max_all, metric_brightest_blob_flux, true);

    get_plot_ranges(value_min, value_max, focus_min, focus_max, metric_star_flux);
    plot_curves(0.0, value_max*1.1, focus_min_all, focus_max_all, plot_offset,
        metric_star_flux, plot_width, plot_height);

    get_plot_ranges(value_min, value_max, focus_min, focus_max, metric_brightest_blob_flux);
    plot_curves(0.0, value_max*1.1, focus_min_all, focus_max_all, plot_offset,
        metric_brightest_blob_flux, plot_width, plot_height);

    if (shared_results.best_focus_known) {
        GL::Color4f(0.95, 0.09, 0.87, 1.0);
        double focus_x = (shared_results.best_focus - focus_min_all) / (focus_max_all - focus_min_all);
        focus_x *= plot_width;
        double arrow_y = -21.0;
        glBegin(GL_LINES);
            glVertex2d((GLfloat) focus_x,     arrow_y);
            glVertex2d((GLfloat) focus_x,     arrow_y+17.0);
            glVertex2d((GLfloat) focus_x-5.0, arrow_y+11.0);
            glVertex2d((GLfloat) focus_x,     arrow_y+17.0);
            glVertex2d((GLfloat) focus_x+5.0, arrow_y+11.0);
            glVertex2d((GLfloat) focus_x,     arrow_y+17.0);
        glEnd();
    }

    glPopMatrix();
	glEnable(GL_TEXTURE_2D);

    Position text_pos;
    std::string focus_str;
    double focus_str_width = 1.0;
    double focus_str_heigth = 1.0;

    focus_str = (boost::format("%i")%focus_min).str();
    text_pos.x = plot_padding;
    text_pos.y = plot_padding + text_height;
    draw_text(focus_str, text_pos);
    focus_str = (boost::format("%i")%focus_max).str();
    get_text_size(focus_str, focus_str_width, focus_str_heigth);
    text_pos.x = plot_padding + plot_width - focus_str_width;
    text_pos.y = plot_padding + text_height;
    draw_text(focus_str, text_pos);

    end_draw();
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 0
}

