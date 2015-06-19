/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "visual.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/filesystem/operations.hpp>
#include "../parameters/manager.h"
#include "../shared/general/main_settings.h"
#include "../shared/image/blobs.h"
#include "../shared/image/leveled.h"
#include "../shared/image/matching.h"
#include "../shared/image/matching_progress.h"
#include "../shared/image/status.h"
#include "../shared/image/solution_summary.h"
#include "../shared/solving/mask.h"
#include "../logger_main.h"

using namespace Displaying;
using std::max;
using std::min;
using std::string;
using Main::logger;
#define shared_blobs (*(Shared::Image::blobs_solver_for_main.r))
#define shared_matching (*(Shared::Image::matching.r))
#define shared_match_progress (*(Shared::Image::matching_progress.r))
#define shared_leveled (*(Shared::Image::leveled.r))
#define shared_progress (*(Shared::Image::matching_progress.r))
#define shared_status (*(Shared::Image::status_solver_for_main.r))
#define shared_solution (*(Shared::Image::solution_summary_for_main.r))
#define shared_mask (*(Shared::Solving::mask_solver_for_main.r))
#define shared_main_settings (*(Shared::General::main_settings_net_for_main.r))

// Disable warnings about conversions between GLfloat and double
#ifdef _MSC_VER
    #pragma warning(disable:4244)
    #pragma warning(disable:4305)
#endif

Visual::Visual(Parameters::Manager& params)
{
    display_enabled = params.general.try_get("main.display_enabled", true);
    stars_dir = params.stars_dir;
    change_size(1024, 1024);
    main_window_height_ = 50;
    some_height_ = 50;
    base_blobs[0] = -1;
    base_blobs[1] = -1;
    base_blobs[2] = -1;
    base_progress = 0.0;
    display_blobs_.clear();
    frame_number = -1;
    display_base_sets.clear();
    texture_status = no_image;
    image_size.w = 600;
    image_size.h = 400;
    current_id = -1;
    last_blobs_counter_stars = -1;
    counter_stars_for_last_blob_names = -1;

    loading_screen_begun = false;
}

Visual::~Visual()
{
    loading_screen.destroy(display_enabled);
    flash.destroy(display_enabled);
    mask.destroy(display_enabled);
    white.destroy(display_enabled);
    rendered_image.destroy(display_enabled);
    canvas.destroy(display_enabled);
}

void Visual::init(std::string title, Textures& textures_)
{
    using namespace boost::filesystem;
    Block::init(title, textures_);
    loading_screen.init(display_enabled);
    loading_screen.load_tiff(system_complete(stars_dir + "code/resources/edge2.tiff").string());
    flash.init(display_enabled);
    flash.load_tiff(system_complete(stars_dir + "code/resources/flash.tiff").string());
    mask.init(display_enabled);
    mask.load_tiff(system_complete(stars_dir + "code/resources/mask.tiff").string());
    white.init(display_enabled);
    white.load_tiff(system_complete(stars_dir + "code/resources/white.tiff").string());
    rendered_image.init(display_enabled);
    canvas.init(display_enabled);
}

void Visual::clear()
{
    display_blobs_.clear();
    base_blobs[0] = -1;
    base_blobs[1] = -1;
    base_blobs[2] = -1;
    base_progress = 0.0;
    display_base_sets.clear();
    image_age_timer.start();
    rendered_image_timer.start();
    texture_status = no_image;
    frame_number = -1;
}


void Visual::update_size(Size max_size)
{
    Size new_size;
    new_size.w = min(max_size.w, max_size.h * image_size.w/image_size.h);
    new_size.h = min(max_size.h, max_size.w * image_size.h/image_size.w);
    Block::change_size(new_size);
}

void Visual::update()
{
    using namespace Shared::Image;

    if (current_id != shared_status.counter_stars) {
        clear();
        current_id = shared_status.counter_stars;
        texture_status = image_not_rendered;
        image_age_timer = shared_status.age;
        image_size.w = shared_status.width;
        image_size.h = shared_status.height;
        display_blobs_.clear();
    }

    if (texture_status == image_not_rendered && shared_leveled.valid) {
        if (shared_leveled.counter_stars == shared_status.counter_stars) {
            rendered_image.bind_data(shared_leveled.pixels,
                shared_status.width, shared_status.height);
            rendered_image_timer.start();
            display_base_sets.clear();
            texture_status = image_rendered;
        }
    }
    if (texture_status == image_rendered) {
        if (shared_blobs.counter_stars != last_blobs_counter_stars) {
            display_blobs_.clear();
            for (unsigned int i=0; i<shared_blobs.blobs.size(); i++) {
                display_blobs_.push_back(Blob(shared_blobs.blobs[i].u, shared_blobs.blobs[i].v, i));
            }
            blobs_timer.start();
            last_blobs_counter_stars = shared_blobs.counter_stars;
        }
        if (display_blobs_.size() == shared_blobs.blobs.size()) {
            if (shared_matching.counter_stars == shared_status.counter_stars) {
                if (display_base_sets.size() != shared_matching.base_sets.size()) {
                    for (unsigned int i=display_base_sets.size(); i<shared_matching.base_sets.size(); i++) {
                        display_base_sets.push_back(BaseSet(shared_matching.base_sets[i]));
                        base_progress = 0.0;
                    }
                }
                if (shared_match_progress.counter_stars == shared_status.counter_stars &&
                        shared_match_progress.triplet_counter == shared_matching.triplet_counter) {
                    base_progress = shared_match_progress.progress;
                }
            }
        }
        if (shared_solution.counter_stars == shared_blobs.counter_stars && shared_solution.blob_names.size() > 0
                && shared_solution.counter_stars != counter_stars_for_last_blob_names) {
            counter_stars_for_last_blob_names = shared_solution.counter_stars;
            rendered_solution_timer.start();
            for (unsigned int i=0; i<shared_solution.blob_names.size(); i++) {
                for (unsigned int j=0; j<display_blobs_.size(); j++) {
                    if (shared_solution.blob_names[i].blob_id == display_blobs_[j].blob_num) {
                        display_blobs_[j].name = shared_solution.blob_names[i].name;
                    }
                }
            }
        }
    }
}


void Visual::draw3d(Position& pos, Size& scaling)
{
    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::drawing_3d)); // Bind 0

    glPushMatrix();
    glViewport(pos.x*scaling.w, pos.y*scaling.h,
        width()*scaling.w, height()*scaling.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, width()/(height()-some_height_), 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 1.5, -3.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    static double angle = 0.0;
    angle += 3.0;
    glRotatef(angle,0.0,1.0,0.0);
    glBegin(GL_TRIANGLES);
        glColor3f(1.0f,0.0f,0.0f);
        glVertex3f( 0.0f, 1.0f, 0.0f);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(-1.0f,-1.0f, 0.0f);
        glColor3f(0.0f,0.0f,1.0f);
        glVertex3f( 1.0f,-1.0f, 0.0f);
    glEnd();

    glPopMatrix();
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 0

}

void Visual::draw_flash(double age)
{
    double opacity = 0.75*max(0.0, 1.0 - 2.0*age);
    opacity = 1.0;
    flash.draw(size(), opacity);
}

void Visual::draw_mask()
{
    glPushMatrix();
    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 1
    glScalef(width()/image_size.w, height()/image_size.h, 1);

    Size block_dimensions;
    block_dimensions.w = shared_mask.block_size;
    block_dimensions.h = shared_mask.block_size;
    for (int j=0; j<shared_mask.num_blocks_y; j++) {
        for (int i=0; i<shared_mask.num_blocks_x; i++) {
            if (shared_mask.block_masked(i, j)) {
                glPushMatrix();
                glTranslatef(shared_mask.block_size*i, shared_mask.block_size*j, 0);
                mask.draw(block_dimensions, 0.50);
                glPopMatrix();
            }
        }
    }

    glBindTexture(GL_TEXTURE_2D, 0); // unBind 1
    glPopMatrix();
}

void Visual::draw_loading_screen()
{
    if (!loading_screen_timer.started()) {
        loading_screen_timer.start();
    }
    loading_screen.draw(size());

    double age = loading_screen_timer.time() - 0.5;
    double speed = 0.2;
    Position pos;
    string words = "";

    words = "Star Tracking";
    pos.x = 70.0 + 30.0*(1.0 - exp(-age*speed));
    pos.y = 242.0;
    if (age <= 0.8) {
        pos.x -= exp( -20.0*(age - 0.8) ) - 1.0;
    }
    set_color(Color(1.0, 1.0, 1.0, 1.0));
    set_text_scaling(0.75,  true);
    draw_text(words, pos);

    pos.x = 155.0 - 15.0*(1.0 - exp(-age*speed));
    pos.y = 210.0;
    words = "Attitude Reconstruction Software";
    if (age <= 1.0) {
        pos.x += exp( -20.0*(age - 1.0) ) - 1.0;
    }
    set_color(Color(1.0, 1.0, 1.0, 1.0));
    set_text_scaling(0.65,  true);
    draw_text(words, pos);
}

void Visual::draw_blob_stripe(double intercept)
{
    if (intercept < shared_status.width + shared_status.height) {
        glLineWidth(8.0);
        glPushMatrix();
        glColor4f(0.3, 0.0, 1.0, 0.7);
        glBegin(GL_LINES);
        {
            double x0, y0, x1, y1;
            if (intercept < shared_status.height) {
                x0 = 0.0;
                y0 = intercept;
            } else {
                x0 = intercept - shared_status.height;
                y0 = shared_status.height;
            }
            if (intercept < shared_status.width) {
                x1 = intercept;
                y1 = 0.0;
            } else {
                x1 = shared_status.width;
                y1 = intercept - shared_status.width;
            }
            glVertex2d(x0, y0);
            glVertex2d(x1, y1);
        }
        glEnd();
        glPopMatrix();
        glLineWidth(2.0);
    }
}

void Visual::draw_blobs(double age, double stripe_progress)
{
    glPushMatrix();
    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 2
    glScalef(width()/image_size.w, height()/image_size.h, 1);

    Position pos;
    double width = 0.0;
    double height = 050;

    Size my_size = size();
    double intercept = stripe_progress * (shared_status.width + shared_status.height);
    draw_blob_stripe(intercept);
    for (unsigned int i=0; i<display_blobs_.size(); i++) {
        bool named = false;
        if (display_blobs_[i].name.size() > 0 && display_blobs_[i].born) {
            named = true;
        }
        display_blobs_[i].draw(my_size, age, intercept, shared_blobs.blobs[i].matched, named);
    }
    set_text_scaling(2.0);
    //double color_scaling = pow(sin(rendered_solution_timer.time()*1.5), 2.0);
    double color_scaling = sin(rendered_solution_timer.time()*3.0-M_PI/2.0)/2.0 + 0.5;
    set_color(Color(1.0, 1.0, 0, color_scaling));
    for (unsigned int i=0; i<display_blobs_.size(); i++) {
        if (display_blobs_[i].name.size() > 0 && display_blobs_[i].born) {
            get_text_size(display_blobs_[i].name.c_str(), width, height);
            if (display_blobs_[i].x + width > image_size.w) {
                pos.x = display_blobs_[i].x - width - 10.0;
            } else {
                pos.x = display_blobs_[i].x + 10.0;
            }
            if (display_blobs_[i].y - height < 0) {
                pos.y = display_blobs_[i].y + 25.0 + height;
            } else {
                pos.y = display_blobs_[i].y + -25.0;
            }
            draw_text(display_blobs_[i].name, pos);
        }
    }
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 2
    glPopMatrix();
}

void Visual::draw_base_sets(double stripe_progress)
{
    using namespace Shared::Image;
    glPushMatrix();
    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 3
    glScalef(width()/image_size.w, height()/image_size.h, 1);
    unsigned int num_sets = display_base_sets.size();
    double global_brightness = max(min(stripe_progress*0.5-0.3, 1.0), 0.0);
    if (num_sets > 0) {
        for (unsigned int i=0; i<num_sets-1; i++) {
            display_base_sets[i].draw(display_blobs_, 1.0, true, global_brightness);
        }
        display_base_sets[num_sets-1].draw(display_blobs_, base_progress, shared_status.stage == Status::done, global_brightness);
    }
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 3
    glPopMatrix();
}

void Visual::draw(Position &pos, double flip_scaling)
{
    glColor4f(1.0, 1.0, 1.0, 1.0);
    begin_draw(pos, flip_scaling);
    draw_border();

    if (texture_status == no_image) {
        draw_loading_screen();
    }
    else if (texture_status == image_rendered) {
        rendered_image.draw(size());

        if (!shared_main_settings.display_image_only) {
            draw_mask();

            if (shared_blobs.counter_stars == shared_status.counter_stars) {
                double blobs_age = blobs_timer.time();
                double stripe_progress = blobs_age*1.5;
                draw_blobs(blobs_age, stripe_progress);
                draw_base_sets(stripe_progress);
            }
        }

        double rendered_image_age = rendered_image_timer.time();
        if (rendered_image_age <= 0.5) {
            //draw_flash(rendered_image_age);
        }
    }
    else {
        white.draw(size(), 0.75);
        rendered_image_timer.start();
    }

    end_draw();
}

