/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "leveler.h"
#include "../../shared/image/raw.h"
#include "../../tools/math.h"
#include "../../parameters/manager.h"
#include "../logger.h"

using namespace Solving::Finding;

Leveler::Leveler(Parameters::Manager& params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);
    int max_coarse4_width = int(floor(image_width/4.0));
    int max_coarse4_height = int(floor(image_height/4.0));
    coarse4 = new unsigned short[max_coarse4_width*max_coarse4_height];
}

Leveler::~Leveler()
{
    delete [] coarse4;
}

bool Leveler::is_inbounds(int& i, int& j, int& width, int& height)
{
    if ((i<0) || (i>=width))
        return false;
    if ((j<0) || (j>=height))
        return false;
    return true;
}

void Leveler::level(Shared::Image::Raw& image, unsigned short leveled[])
{
    double value = 0.0;
    double counter = 0.0;
    int subj, subi = 0;

    for (int j=0; j<image_height; j++) {
        for (int i=0; i<image_width; i++) {
            leveled[j*image_width+i] = image.pixels[j*image_width+i];
        }
    }

    int coarse4_width = int(floor(image_width/4.0));
    int coarse4_height = int(floor(image_height/4.0));

    for (int j=0; j<coarse4_height; j++) {
        for (int i=0; i<coarse4_width; i++) {
            value = 0;
            for (subj=0; subj<4; subj++) {
                for (subi=0; subi<4; subi++) {
                    value += image.pixels[(4*j+subj)*image_width + (4*i+subi)];
                }
            }
            value = round(value/16.0);
            coarse4[j*coarse4_width + i] = (unsigned short) value;
        }
    }
    for (int j=0; j<coarse4_height; j++) {
        for (int i=0; i<coarse4_width; i++) {
            value = 0;
            counter = 0;
            for (subj=j-4; subj<=j+4; subj+=8) {
                for (subi=i-3; subi<=i+3; subi++) {
                    if (is_inbounds(subi, subj, coarse4_width, coarse4_height)) {
                        value += coarse4[subj*coarse4_width + subi];
                        counter++;
                    }
                }
            }
            for (subi=i-4; subi<=i+4; subi+=8) {
                for (subj=j-3; subj<=j+3; subj++) {
                    if (is_inbounds(subi, subj, coarse4_width, coarse4_height)) {
                        value += coarse4[subj*coarse4_width + subi];
                        counter++;
                    }
                }
            }
            if (counter > 0) {
                value = round(value/counter);
            }
            for (subj=0; subj<4; subj++) {
                for (subi=0; subi<4; subi++) {
                    leveled[(4*j+subj)*image_width + (4*i+subi)] = (unsigned short) value;
                }
            }
        }
    }
    if ((coarse4_height*4 != image_height) ||
        (coarse4_width*4 != image_width)) {
        logger.log("finder: leveler: warning: image dimensions not a multiple of 4");
    }

}


