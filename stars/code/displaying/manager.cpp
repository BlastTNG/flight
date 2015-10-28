/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "manager.h"
#include "../parameters/manager.h"
#include "../shared/general/main_settings.h"
#include "../shared/general/quit.h"
#include "../logger_main.h"
#include "../preprocessor.h"		
#include "../tools/quick_cout.h"


using namespace Displaying;
using Main::logger;
using std::max;
using std::min;

#define shared_main_settings (*(Shared::General::main_settings_net_for_main.r))

DisplayManager::DisplayManager(Parameters::Manager& params):
    textures(params), visual(params)
{
    if (PREPROCESSOR_USING_CAMERA > 0) {
        using_camera = true;
    } else {
        using_camera = false;
    }
    screen = NULL;
    enabled = params.general.try_get("main.display_enabled", true);
    fullscreen = shared_main_settings.display_fullscreen;
    shared_fullscreen_counter = shared_main_settings.display_fullscreen_counter;

    normal_screen_size.w = 800;
    normal_screen_size.h = 600;

    full_screen_size.w = 1024;
    full_screen_size.h = 768;

    if (fullscreen) {
        current_screen_size = full_screen_size;
    } else {
        current_screen_size = normal_screen_size;
    }

    resolution.w = 640;
    resolution.h = 480;

    current_zoom_x = 320;
    current_zoom_y = 240;
    current_zoom_magnitude = 1.0;

    if ( SDL_Init(SDL_INIT_EVERYTHING) != 0 ) {
        logger.log(format("Unable to initialize SDL: %s") % SDL_GetError());
    }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) ;
    //SDL_SetWindowTitle("STARS", "STARS");
	SDL_WM_SetCaption("STARS", "STARS");

    set_video_mode();
    textures.init();
    init();
    name.set_title(params.general.try_get("main.which_sensor", std::string("(sensor name)")));
}

DisplayManager::~DisplayManager()
{
}

void DisplayManager::init()
{
    if (!enabled) return;
    glEnable(GL_TEXTURE_2D);
    glLineWidth(2.0);

    housekeeping.init("Housekeeping", textures);
    camera_block.init("Camera", textures);
    lens.init("Lens", textures);
    image.init("Working Block", textures);
    visual.init("Visual", textures);
    autofocus.init("Autofocus", textures);
    solving_filters.init("Solving Filters", textures);
    quit_box.init("QUIT", textures);
    name.init("(sensor name)", textures);

    autofocus.change_size(image.width(), image.height());
    solving_filters.change_size(image.width(), image.height());
}

void DisplayManager::set_video_mode()
{
    Uint32 flags = SDL_OPENGL | SDL_RESIZABLE;
    if (fullscreen) {
        flags |= SDL_FULLSCREEN;
        current_screen_size = full_screen_size;
    } else {
        current_screen_size = normal_screen_size;
    }
    SDL_FreeSurface(screen);
    if (enabled) screen = SDL_SetVideoMode(int(current_screen_size.w), int(current_screen_size.h), 32, flags);
    if (!screen) {
        logger.log("after call to SDL_SetVideoMode, screen is NULL");
        logger.log(format("    error is: %s") % SDL_GetError());
    }
    reshape();
}
//Changed to work with SDL2
void DisplayManager::reshape()
{
    if (!enabled) return;
    glViewport(0, 0, int(current_screen_size.w), int(current_screen_size.h));
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, resolution.w, 0.0, resolution.h);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


void DisplayManager::process_events()
{
    if (!enabled) return;
    static SDL_Event event;
    while ( SDL_PollEvent(&event) ) {
        switch ( event.type ) {
            case SDL_VIDEORESIZE:
                normal_screen_size.w = (double)event.resize.w;
                normal_screen_size.h = (double)event.resize.h;
                set_video_mode();
                break;
            case SDL_KEYDOWN:
                switch ( event.key.keysym.sym ) {
                    case SDLK_ESCAPE:
                        Shared::General::quit = true;
                        break;
                    case SDLK_q:
                        Shared::General::quit = true;
                        break;
                    case SDLK_f:
                        break;
                    case SDLK_n:
                        break;
                    default:
                        break;
                }
                break;
            case SDL_QUIT:
                Shared::General::quit = true;
                break;
            default:
                break;
        }
    }
}

void DisplayManager::print_available_chars()
{
    logger.log("display reporting available characters:");
    for (int i=textures.font.GetStartChar(); i<=textures.font.GetEndChar(); i++) {
        logger.log(format("%d %c") % i % char(i));
    }
}

void DisplayManager::update_zoom(double delta_t)
{
    if (delta_t > 0.2) {
        delta_t = 0.2;
    }
    current_zoom_x += 5.0*((double) shared_main_settings.display_zoom_x-current_zoom_x)*delta_t;
    current_zoom_y += 5.0*((double) shared_main_settings.display_zoom_y-current_zoom_y)*delta_t;
    current_zoom_magnitude += 5.0*((double) shared_main_settings.display_zoom_magnitude-current_zoom_magnitude)*delta_t;

}

void DisplayManager::draw()
{
    if (!enabled) return;

    if (shared_fullscreen_counter != shared_main_settings.display_fullscreen_counter) {
        shared_fullscreen_counter = shared_main_settings.display_fullscreen_counter;
    }

    housekeeping.update();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPushMatrix();
    glTranslated(resolution.w/2.0, resolution.h/2.0, 0.0);
    glScaled(current_zoom_magnitude, current_zoom_magnitude, 1.0);
    glTranslated(-current_zoom_x, -current_zoom_y, 0.0);

    if (shared_main_settings.display_image_only) {
        double padding = 2.0;

        Size max_size;
        max_size.w = resolution.w - 2.0*padding;
        max_size.h = resolution.h - 2.0*padding;
        visual.update_size(max_size);

        Position pos;
        pos.x = padding;
        pos.y = resolution.h - visual.height() - 1.0*padding;
        visual.update();
        visual.draw(pos);
    }
    else {

        double padding = 5.0;
        Size max_size;
        max_size.w = resolution.w - name.width() - 3*padding;
        max_size.h = resolution.h - image.height() - 3*padding;
        visual.update_size(max_size);

        glColor4f(1.0, 1.0, 1.0, 1.0);

        Position pos;
        pos.x = padding;
        pos.y = resolution.h - visual.height() - image.height() - 2*padding;

        if (autofocus.display_enabled()) {
            autofocus.draw(pos);
        } else {
            if (shared_main_settings.display_solving_filters) {
                solving_filters.draw(pos);
            } else {
                image.draw(pos);
            }
        }
        glColor4f(1.0, 1.0, 1.0, 1.0);

        pos.x = visual.width() + 2.0*padding;
        pos.y = resolution.h - name.height() - padding;
        name.draw(pos);
        pos.y -= (housekeeping.height() + padding);
        housekeeping.draw(pos);
        if (using_camera) {
            pos.y -= (camera_block.height() + padding);
            camera_block.draw(pos);
        }
        pos.y -= (lens.height() + padding);
        lens.draw(pos);
        pos.x = padding;
        pos.y = resolution.h - visual.height() - padding;

        visual.update();
        visual.draw(pos);

        if (Shared::General::quit) {
            pos.x += visual.width()/2 - quit_box.width()/2;
            pos.y += visual.height()/2 - quit_box.height()/2;
            quit_box.draw(pos);
        }
    }

    glPopMatrix();
    SDL_GL_SwapBuffers();
}

void DisplayManager::wait_for_quit(){
    if (!enabled) return;
    SDL_Quit();
}

