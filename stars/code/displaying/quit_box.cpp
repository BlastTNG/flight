/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "quit_box.h"
#include "glhelper.h"

using namespace Displaying;

QuitBox::QuitBox()
{
    change_size(51, 29);
}

void QuitBox::draw(Position &position)
{
    begin_draw(position);
    draw_border();

    glPushMatrix();
    GL::Color4f(0.0, 0.0, 0.0, 1.0);
    glBegin(GL_POLYGON);
        glVertex2d(1, 0);
        glVertex2d(width(), 0);
        glVertex2d(width(), height());
        glVertex2d(1, height());
    glEnd();
    glPopMatrix();

    draw_title();
    end_draw();
}

