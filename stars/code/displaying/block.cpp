/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "block.h"

#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include <boost/lexical_cast.hpp>
#ifndef _MSC_VER
    #include "GL/glext.h"
#endif
#include "color.h"
#include "glhelper.h"

using namespace Displaying;
using std::string;

Block::Block()
{
    current_size.w = 200;
    current_size.h = 200;
    requested_size = current_size;
    padding = 5.0;
    text_height = 20.0;

    color_permanent = Color(1.0, 1.0, 1.0, 1.0);
    color_temporary = Color(1.0, 1.0, 1.0, 1.0);
    color_use_temporary = false;

    text_scaling_permanent = 0.55;
    text_scaling_temporary = 0.55;
    text_scaling_use_temporary = false;

    rjust_permanent = false;
    rjust_width_permanent = 50.0;
    rjust_temporary = false;
    rjust_width_temporary = 50.0;
    rjust_use_temporary = false;

    change_size(140, 2*padding+1*text_height);

}

Block::~Block()
{
}

void Block::init(string title_, Textures& textures_)
{
    title = title_;
    textures = &textures_;
}

void Block::set_title(string title_)
{
    title = title_;
}

void Block::draw_text(string text, Position& pos, bool hold_temporary)
{
    Color color = color_permanent;
    double text_scaling = text_scaling_permanent;
    bool rjust = rjust_permanent;
    double rjust_width = rjust_width_permanent;
    if (color_use_temporary) {
        color = color_temporary;
    }
    if (text_scaling_use_temporary) {
        text_scaling = text_scaling_temporary;
    }
    if (rjust_use_temporary) {
        rjust = rjust_temporary;
        rjust_width = rjust_width_temporary;
    }
    if (!hold_temporary)
    {
        color_use_temporary = false;
        text_scaling_use_temporary = false;
        rjust_use_temporary = false;
    }
    double xshift = 0.0;
    if (rjust) {
        std::pair<int, int> text_size;
        textures->font.GetStringSize(text.c_str(), &text_size);
        xshift = (rjust_width - (text_size.first * text_scaling));
    }
    textures->font.Begin(); // Bind 0
    GL::Color4f(color.values[0], color.values[1], color.values[2], color.values[3]);
    textures->font.DrawString(text, (float) text_scaling, (float) (pos.x + xshift), (float) pos.y);
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 0
}

void Block::get_text_size(string text, double& width, double& height)
{
    double text_scaling = text_scaling_permanent;
    if (text_scaling_use_temporary) {
        text_scaling = text_scaling_temporary;
    }
    std::pair<int, int> text_size;
    textures->font.GetStringSize(text.c_str(), &text_size);
    width = text_size.first * text_scaling;
    height = text_size.second * text_scaling;
}

void Block::begin_draw(Position& pos, double flip_scaling)
{
    glPushMatrix();
    glTranslatef((GLfloat) pos.x, (GLfloat) pos.y, (GLfloat) 0.0);
    glTranslatef((GLfloat) 1.0, (GLfloat) ((1.0-flip_scaling)/2.0*height()), 0.0f);
    glScalef(1.0f, (GLfloat) flip_scaling, 1.0f);
}

void Block::end_draw()
{
    glPopMatrix();
}

void Block::draw_border()
{
    glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    GL::Color4f(0.3, 0.3, 0.6, 1);
    glBegin(GL_LINE_LOOP);
        glVertex2f(0.0, 0.0);
        glVertex2f(0.0, (GLfloat) current_size.h);
        glVertex2f((GLfloat) current_size.w, (GLfloat) current_size.h);
        glVertex2f((GLfloat) current_size.w, 0.0);
    glEnd();
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
}

void Block::draw_value(DynamicValue& value, Position pos)
{
    int width = 60;
    string str;
    if (value.known) {
        str = value.name + ": ";
        set_color(value.color, true);
        set_rjust(width, true);
        draw_text(str, pos);
        pos.x += width;
        str = value.value;
        set_color(value.color, true);
        draw_text(str, pos);
    } else {
        str = value.name + ": ";
        set_color(value.color*0.5, true);
        set_rjust(width, true);
        draw_text(str, pos);
    }

}

void Block::draw_title()
{
    Position pos = {padding, current_size.h-padding};
    draw_text(title, pos);
}

void Block::set_color(Color color, bool just_once)
{
    if (just_once) {
        color_temporary = color;
        color_use_temporary = true;
    } else {
        color_permanent = color;
        color_use_temporary = false;
    }
}

void Block::set_text_scaling(double scaling, bool just_once)
{
    if (just_once) {
        text_scaling_temporary = scaling;
        text_scaling_use_temporary = true;
    } else {
        text_scaling_permanent = scaling;
        text_scaling_use_temporary = false;
    }
}

void Block::set_rjust(double width, bool just_once)
{
    if (just_once) {
        rjust_temporary = true;
        rjust_width_temporary = width;
        rjust_use_temporary = true;
    } else {
        rjust_permanent = true;
        rjust_width_permanent = width;
        rjust_use_temporary = false;
    }
}

void Block::scale_color(double scaling)
{
    if (color_use_temporary) {
        color_temporary.values[0] *= scaling;
        color_temporary.values[1] *= scaling;
        color_temporary.values[2] *= scaling;
    } else {
        color_permanent.values[0] *= scaling;
        color_permanent.values[1] *= scaling;
        color_permanent.values[2] *= scaling;
    }
}


void Block::apply_flip_scaling(Position& position, double flip_scaling)
{
    glScalef(1.0f, (GLfloat) flip_scaling, 1.0f);
    //position.y += height()*(1.0 - flip_scaling)/2.0;
    position.y += 100;
}

void Block::change_size(Size new_size, bool slow)
{
    change_size(new_size.w, new_size.h, slow);
}

void Block::change_size(double width, double height, bool slow)
{
    requested_size.w = width;
    requested_size.h = height;
    if (!slow) {
        previous_size = requested_size;
        current_size = requested_size;
        sizing_finished = true;
    } else {
        previous_size = current_size;
        sizing_finished = false;
        sizing_timer.start();
        update_size(slow);
    }
}

void Block::update_size(bool slow)
{
    if (!slow) {
        previous_size = requested_size;
        current_size = requested_size;
        sizing_finished = true;
    } else {
        double period = 0.5;
        if (sizing_timer.time() > period) {
            previous_size = requested_size;
            current_size = requested_size;
            sizing_finished = true;
        } else {
            double fraction = 1.0 - exp(-3.0*sizing_timer.time()/period) + exp(-3.0);
            current_size.w = previous_size.w + (requested_size.w-previous_size.w)*fraction;
            current_size.h = previous_size.h + (requested_size.h-previous_size.h)*fraction;
        }
    }
}

bool Block::size_update_finished()
{
    if (sizing_finished || sizing_timer.time() > 3) {
        return true;
    }
    return false;
}

double Block::width()
{
    return current_size.w;
}

double Block::height()
{
    return current_size.h;
}

Size Block::size()
{
    return current_size;
}

void Block::draw(Position& position, double flip_scaling)
{
    glBindTexture(GL_TEXTURE_2D, textures->get(Textures::basic_drawing)); // Bind 1
    begin_draw(position);
    draw_border();
    draw_title();
    end_draw();
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 1
}

