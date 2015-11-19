/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "matcher.h"
#include "../update.h"
#include "../../tools/angles.h"
#include "../../tools/ccd_projection.h"
#include "../../shared/general/quit.h"
#include "../../shared/image/blobs.h"
#include "../../shared/image/matching.h"
#include "../../shared/image/matching_progress.h"
#include "../../shared/image/raw.h"
#include "../../shared/image/solution_summary.h"
#include "../../shared/image/status.h"
#include "../../shared/solving/settings.h"
#include "../../parameters/manager.h"
#include "../logger.h"

using namespace Solving;
using namespace Solving::Matching;
using std::vector;
using std::numeric_limits;

#define shared_blobs (*(Shared::Image::blobs_solver_for_main.w))
#define shared_matching (*(Shared::Image::matching.w))
#define shared_progress (*(Shared::Image::matching_progress.w))
#define shared_solution_for_main (*(Shared::Image::solution_summary_for_main.w))
#define shared_status (*(Shared::Image::status_solver_for_main.w))
#define shared_settings (*(Shared::Solving::settings.r))

Matcher::Matcher(Parameters::Manager& params):
    catalog_manager(params, shared_settings.enabled, shared_settings.catalog)
{
    image_width = params.general.image_width;
    image_height = params.general.image_height;
    result_update_success_counter = 0;
    result_update_failure_counter = 0;
    base_set_counter = 0;
}

void Matcher::init()
{
    catalog_manager.init();
}

bool is_inbounds_centered(double& u, double& v, double& width, double& height)
{
    // this should be put in tools, as should angdist func
    if ((u > width/2.0) || (u < -width/2.0)) {
        return false;
    }
    if ((v > height/2.0) || (v < -height/2.0)) {
        return false;
    }
    return true;
}

void rotate(double& x, double& y, double& cos_theta, double& sin_theta)
{
    double temp_x, temp_y;
    temp_x = x*cos_theta - y*sin_theta;
    temp_y = x*sin_theta + y*cos_theta;
    x = temp_x;
    y = temp_y;
}

bool good_for_debug(Solution& solution)
{
    return true;

    vector<int> ids;
    ids.push_back(132);
    ids.push_back(364);

    unsigned int num_matching_ids = 0;
    for (unsigned int i=0; i<3; i++) {
        for (unsigned int j=0; j<ids.size(); j++) {
            if (solution.base_stars[i].id == ids[j]) {
                num_matching_ids++;
            }
        }
    }
    if (num_matching_ids == ids.size()) {
        return true;
    }
    else {
        return false;
    }

}

vector<Star> Matcher::match_stars_to_blob(Solution& solution, Blob& blob, vector<Star>& stars,
    Shared::Image::Raw& image)
{
    vector<Star> matched_stars;
    double u = 0.0;
    double v = 0.0;
    double star_ra = 0.0;
    double star_dec = 0.0;
    double cos_roll = cos(solution.equatorial.roll);
    double sin_roll = sin(solution.equatorial.roll);
    double distance = 0.0;
    double width = image_width;
    double height = image_height;
	double hyp = sqrt(width*width + height*height) * solution.equatorial.iplatescale;
    double tolerance_squared = pow(shared_settings.match_tolerance_px, 2.0);
    double distance_squared = numeric_limits<double>::infinity();
    double best_distance_squared = numeric_limits<double>::infinity();
    unsigned int best_i = -1;
    for (unsigned int i=0; i<stars.size(); i++) {
        distance = great_circle(solution.equatorial.ra, solution.equatorial.dec, stars[i].ra, stars[i].dec);
		
        if (distance < from_arcsec(hyp)) {
            Tools::get_refraction_corrected_equatorial(stars[i].ra, stars[i].dec,
                shared_settings.refraction, image.filters, star_ra, star_dec);
            Tools::ccd_projection(star_ra, star_dec, solution.equatorial.ra, solution.equatorial.dec,
                1.0/solution.equatorial.iplatescale, cos_roll, sin_roll, u, v, true);
            if (is_inbounds_centered(u, v, width, height)) {
                distance_squared = pow(blob.x-u, 2) + pow(blob.y-v, 2);
                if (distance_squared < best_distance_squared) {
                    best_distance_squared = distance_squared;
                    best_i = i;
                }
            }
        }
    }
    if (best_distance_squared < tolerance_squared) {
        matched_stars.push_back(stars[best_i]);
    }
    return matched_stars;
}

void Matcher::match_blobs_to_stars(vector<Blob>& blobs, Solution& solution, vector<Star>& stars,
    Shared::Image::Raw& image)
{
    vector<Star> temp_matched_stars;
    for (unsigned int i=0; i<blobs.size(); i++) {
        temp_matched_stars = match_stars_to_blob(solution, blobs[i], stars, image);
        if (temp_matched_stars.size() > 0) {
            solution.matched_blobs.push_back(blobs[i]);
            solution.matched_stars.push_back(temp_matched_stars[0]);
        }
    }
}

void Matcher::perform_matches(vector<Blob>& blobs, Solution& solution, Shared::Image::Raw& image)
{
    //vector<Star> stars;
    //stars = catalog_manager.get_stars_near(solution.ra, solution.dec);
    //sort(stars.begin(), stars.end(), Star::sort_by_mag);
    solution.num_blobs_total = blobs.size();
    match_blobs_to_stars(blobs, solution, solution.stars_in_fov, image);
}

void Matcher::print_solutions(vector<Solution>& solutions)
{
    logger.log(format("there are %d solutions") % solutions.size());
    for (unsigned int i=0; i<5 && i<solutions.size(); i++) {
        solutions[i].print(logger, true);
    }
}

/*
void Matcher::calculate_pvalue(Solution& solution)
{
    int N = solution.matched_blobs.size();
    double dof = 2*N-3;  // if platescale varies its not 3

    if ((solution.chi_squared > 0) && (dof >= 1)) {
    }
    else {
        solution.pvalue = 0.0;
        return;
    }

    boost::math::chi_squared chi_squared_distribution(dof);
    solution.pvalue = 1 - boost::math::cdf(chi_squared_distribution, solution.chi_squared);
    solution.c1value = dof / sqrt(solution.chi_squared/dof);
}
*/

bool Matcher::get_next_base_set(BaseSet& base_set, int num_blobs, bool reset)
{
    if (reset) {
        base_set_counter = 0;
        shared_matching.counter_stars = shared_status.counter_stars;
        shared_matching.base_sets.clear();
    }

    base_set.ids[0] = 0;
    base_set.ids[1] = 1;
    base_set.ids[2] = 2;
    base_set.type = BaseSet::triplet;

    int counter = 0;
    int n = std::min(7, num_blobs);

    unsigned int j, k;
    for (int i = 0; i < n - 2; i++) {
        for (int j = i + 1; j < n - 1; j++) {
			for (int k = j + 1; k < n; k++) {
                if (counter == base_set_counter) {
                    base_set.ids[0] = i;
                    base_set.ids[1] = j;
                    base_set.ids[2] = k;
                    base_set.type = BaseSet::triplet;
                    base_set_counter++;
                    return false;
                }
                counter++;
            }
        }
    }

    n = num_blobs;
	for (int i = 0; i < n - 1; i++) {
		for (int j = i + 1; j < n; j++) {
            if (counter == base_set_counter) {
                base_set.ids[0] = i;
                base_set.ids[1] = j;
                base_set.ids[2] = 0;
                base_set.type = BaseSet::pair;
                base_set_counter++;
                return false;
            }
            counter++;
        }
    }

    return true;
}

void Matcher::inform_blobs_of_matches(Solution& solution)
{
    for (unsigned int i=0; i<shared_blobs.blobs.size(); i++) {
        for (unsigned int j=0; j<solution.matched_blobs.size() && !shared_blobs.blobs[i].matched; j++) {
            if (solution.matched_blobs[j].id == shared_blobs.blobs[i].id) {
                shared_blobs.blobs[i].matched = true;
            }
        }
    }
}

void Matcher::get_blob_names(Solution& solution)
{
    if (solution.matched_stars.size() > 0) {
        int i = 0;
        std::string name = "";
        double mag = 0.0;
        logger.log("getting brightnes star name");
        catalog_manager.get_star_name(solution.matched_stars[i].id, name, mag);
        logger.log(format("got brightnes star name: %s") % name);
        if (name.size() > 0) {
            Shared::Image::BlobName blob_name;
            blob_name.blob_id = solution.matched_blobs[i].id;
            blob_name.name = name;
            blob_name.mag = mag;
            shared_solution_for_main.blob_names.push_back(blob_name);
        }
    }
}

void Matcher::fill_solutions_for_next_base_set(vector<Solution>& solutions,
    Shared::Image::Raw& image, BaseSet& base_set, vector<Blob>& blobs)
{
    solutions.clear();
    vector<Blob> base_blobs;
    if (base_set.type == BaseSet::pair && blobs.size() > base_set.ids[1]) {
        base_blobs.clear();
        base_blobs.push_back(blobs[base_set.ids[0]]);
        base_blobs.push_back(blobs[base_set.ids[1]]);

        shared_matching.counter_stars = shared_status.counter_stars;
        shared_matching.triplet_counter = base_set_counter;
        shared_matching.base_sets.push_back(base_set);
        Shared::Image::matching.share();

        shared_progress.triplet_counter = base_set_counter;
        shared_progress.progress = 0.0;
        Shared::Image::matching_progress.share();

        solutions = catalog_manager.load_solutions_from_pair(base_blobs, image.filters);

    } else if (base_set.type == BaseSet::triplet && blobs.size() > base_set.ids[2]) {
        base_blobs.clear();
        base_blobs.push_back(blobs[base_set.ids[0]]);
        base_blobs.push_back(blobs[base_set.ids[1]]);
        base_blobs.push_back(blobs[base_set.ids[2]]);

        shared_matching.counter_stars = shared_status.counter_stars;
        shared_matching.triplet_counter = base_set_counter;
        shared_matching.base_sets.push_back(base_set);
        Shared::Image::matching.share();

        shared_progress.triplet_counter = base_set_counter;
        shared_progress.progress = 0.0;
        Shared::Image::matching_progress.share();

        solutions = catalog_manager.load_solutions_from_triplet(base_blobs, image.filters);
    }
}

bool Matcher::match(vector<Blob>& blobs, Shared::Image::Raw& image, Solution& solution)
{
    bool solution_found = false;

    bool out_of_base_sets = false;
    vector<Solution> solutions;
    BaseSet base_set;
    out_of_base_sets = get_next_base_set(base_set, blobs.size(), true);
    do {
        fill_solutions_for_next_base_set(solutions, image, base_set, blobs);
        for (unsigned int i=0; i<solutions.size() && !done() && !solution_found; i++) {
            shared_progress.counter_stars = shared_matching.counter_stars;
            shared_progress.progress = double(i)/double(solutions.size());
            Shared::Image::matching_progress.share();

            solution_fitter.fit(solutions[i].base_blobs, solutions[i].base_stars,
                solutions[i], image);

            catalog_manager.fill_stars_in_fov(solutions[i], image, shared_settings.refraction);
            perform_matches(blobs, solutions[i], image);

            if (solutions[i].matched_blobs.size() >= 2) {
                solution_fitter.fit(solutions[i].matched_blobs, solutions[i].matched_stars,
                    solutions[i], image);
                if (image.filters.check_solution(solutions[i], logger, true)) {
                    logger.log("solution found");
                    solution_found = true;
                    solution = solutions[i];
                    solution_fitter.fit(solution.matched_blobs, solution.matched_stars, solution, image, true);
                    fluxer.fit(solution, solution.matched_blobs, solution.matched_stars);
                }
            }
        }
        out_of_base_sets = get_next_base_set(base_set, blobs.size());
    } while (!out_of_base_sets && !done() && !solution_found);
    //} while (false);

    if (solution_found) {
        shared_status.reason_for_being_done = Shared::Image::Status::solved;
    } else if (out_of_base_sets) {
        shared_status.reason_for_being_done = Shared::Image::Status::tried_all_patterns;
    }
    Shared::Image::status_solver_for_main.share();

    return solution_found;
}


