/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__AUTOFOCUS_HELPER_H
#define SOLVING__AUTOFOCUS_HELPER_H

#include <vector>
#include "../shared/autofocus/datapoints.h"

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    class AutofocusDatapoint;
    class AutofocusHelper;
    class Blob;
    class Solution;
}

class Solving::AutofocusDatapoint
{
  public:
    AutofocusDatapoint();
    AutofocusDatapoint& operator=(const AutofocusDatapoint& rhs);
    static bool sort_by_focus(AutofocusDatapoint first, AutofocusDatapoint second)
    {
        return first.focus < second.focus;
    }

    Shared::Autofocus::datapoints_t type;
    int star_id;
    double focus;
    double value;
    Tools::Timer age;
};


class Solving::AutofocusHelper
{
  public:
    AutofocusHelper(Parameters::Manager& params);
    ~AutofocusHelper();
    void update_and_share();
    void extract_sobel_datapoint(Shared::Image::Raw& image);
    void extract_brightest_blob_datapoint(Shared::Image::Raw& image,
        std::vector<Solving::Blob>& blobs);
    void extract_star_datapoints(Shared::Image::Raw& image, Solution& solution);
    void set_fully_solved(int counter_stars);

    std::vector<AutofocusDatapoint> datapoints;
    int image_width;
    int image_height;
    double* scratch;
};

#endif
