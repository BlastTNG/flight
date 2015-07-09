/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "refitter.h"
#include "../solution.h"
#include "../../shared/image/raw.h"

#include "../../tools/quick_cout.h"

using namespace Solving;
using namespace Solving::Refitting;

void Refitter::fit(Solution& solution, Shared::Image::Raw& image)
{

    // scratch
    /*
    for (unsigned int i=0; i<solution.matched_blobs.size(); i++) {
        cout << "matched_blobs[" << i << "] has ";
        cout << solution.matched_blobs[i].x << " ";
        cout << solution.matched_blobs[i].y << " ";
        cout << solution.matched_blobs[i].sigma_x << endl;
    }
    */


    // sort matched blobs

    // get blob shapes from brightest N
    //shape_fitter.fit_gaussian(image, solution.matched_blobs);

    // fit blobs with set shape

    // fit solution with set blobs
}

