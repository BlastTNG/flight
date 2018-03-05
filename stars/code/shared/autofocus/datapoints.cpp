/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "datapoints.h"
#include <limits>
#include "../../logging/logger.h"

using namespace Shared::Autofocus;

Curve::Curve()
{
    type = metric_brightest_blob_flux;
    points.clear();
}

bool Curve::get_peak(double& focus, double& value)
{
	if (points.size() < 5) {
		focus = 0.0;
		value = 0.0;
		return false;
	}

	unsigned int best_index = 0;
	double best_focus = points[0].focus;
	double best_value = points[0].value;
	for (unsigned int i = 0; i<points.size(); i++) {
		if (points[i].value > best_value)
		{
			best_value = points[i].value;
			best_focus = points[i].focus;
			best_index = i;
		}
	}

	focus = best_focus;
	value = best_value;
	return true;

	if (false) {
		// Decided that it was better to end up at something that is a max rather than at the last focus tried
	    // check that the point is not on the edge in order to check that it's a local maximum
    	if (best_index > 0 && best_index < points.size() - 1) {
    		focus = best_focus;
	    	value = best_value;
	    	return true;
    	}
	    focus = 0.0;
	    value = 0.0;
	    return false;
	}
}

void Curve::log(Logging::Logger& logger)
{
    switch (type) {
        case metric_brightest_blob_flux:
            logger.log("autofocus: metric for brightest blob");
            break;
        case metric_star_flux:
            logger.log(format("autofocus: metric for star %i") % star_id);
            break;
        case metric_sobel:
            logger.log("autofocus: sobel metric");
            break;
    }
    for (unsigned int i=0; i<points.size(); i++) {
        logger.log(format("autofocus: datapoint: focus %f value %f") % points[i].focus % points[i].value);
    }
}

Curve& Curve::operator=(const Curve &rhs)
{
    if (this != &rhs) {
        type = rhs.type;
        points = rhs.points;
        age_since_last_datapoint_added = rhs.age_since_last_datapoint_added;
    }
    return *this;
}

Datapoints::Datapoints()
{
    curves.clear();
    last_fully_solved_counter_stars = -1;
}

Datapoints& Datapoints::operator=(const Datapoints &rhs)
{
    if (this != &rhs) {
        curves = rhs.curves;
        last_fully_solved_counter_stars = rhs.last_fully_solved_counter_stars;
    }
    return *this;
}

