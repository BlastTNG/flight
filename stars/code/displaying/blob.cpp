/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "blob.h"
#include <cstdlib>
#include <string>
#include <time.h>
#include "glhelper.h"
#include <math.h>

using namespace Displaying;

double random_number()
{
    return double(rand()/double(RAND_MAX));
}

Blob::Blob(double x_, double y_, int blob_num_)
{
    x = x_;
    y = y_;
    halfsize = 26.0;
    birth_angle = 0.0;
    blob_num = blob_num_;
    rotation_speed = 80.0;
    birth_time = (x + y)/4000.0;
    born = false;
    timer.start();
    name = "";
}

void Blob::draw(Size& size_, double global_age, double intercept, bool matched, bool named)
{
    if (!born) {
        if (intercept > x+y) {
            born = true;
            timer.start();
        }
    }
    if (born) {
        double age = timer.time();
        double scaling = 1.2;
        double angle = 0;

        if (matched) {
            if (named) {
                GL::Color4f(1.0, 0.8, 0.3, 1.0);
            } else {
                GL::Color4f(1.0, 0.0, 0.3, 1.0);
            }
        } else {
            scaling = age*30.0 * exp(-age*11.0) + 1.0;
            angle = birth_angle + rotation_speed*global_age;
            angle = fmod(angle, 360.);
            GL::Color4f(0.75, 0.75, 1.0, 1.0);
        }

        glPushMatrix();
        glTranslatef((GLfloat) x, (GLfloat) y, 0);
        glRotatef((GLfloat) angle, 0, 0, 1);
        glScalef((GLfloat) scaling, (GLfloat) scaling, 1);
        glBegin(GL_LINE_LOOP);
            glVertex2d(-halfsize, -halfsize);
            glVertex2d(-halfsize, halfsize);
            glVertex2d(halfsize, halfsize);
            glVertex2d(halfsize, -halfsize);
        glEnd();
        glPopMatrix();
    }
}

