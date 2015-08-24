/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "abstract_catalog_manager.h"
#include <cstdio>
#include <string>
#include <cstdlib>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>					/*added by KNS*/
#include <boost/filesystem/path.hpp>			/*added by KNS*/
#include <boost/typeof/std/iterator.hpp>		/*added by KNS*/
#include "../tools/slalib.h"
#include "../tools/angles.h"
#include "../tools/ccd_projection.h"
#include "../shared/solving/settings.h"
#include "../parameters/manager.h"
#include "../shared/general/quit.h"
#include "../shared/image/raw.h"
#include "../logging/logger.h"

using namespace Cataloging;
using std::vector;
using std::string;
using std::ifstream;
using std::ios;
using std::numeric_limits;
using std::system;
using Solving::Star;
using Solving::Blob;
using Solving::Solution;
#define shared_settings (*(Shared::Solving::settings.r)) // this assumes called from solver thread

typedef float catfloat;

struct RawTriplet
{
    float dist1;
    float dist2;
    float dist3;
    int   s1_id;
    float s1_ra;
    float s1_dec;
    int   s2_id;
    float s2_ra;
    float s2_dec;
    int   s3_id;
    float s3_ra;
    float s3_dec;
};

struct RawPair
{
    float dist;
    int   s1_id;
    float s1_ra;
    float s1_dec;
    int   s2_id;
    float s2_ra;
    float s2_dec;
};

struct RawStar
{
    int   id;
    float ra;
    float dec;
    float mag;
};

Region::Region()
{
}

bool mark_as_special(vector<Star>& stars)
{
    vector<int> sids;
    sids.push_back(43635);
    sids.push_back(43627);
    sids.push_back(46351);

    int num_matches = 0;
    for (unsigned int i=0; i<stars.size(); i++) {
        for (unsigned int j=0; j<sids.size(); j++) {
            if (sids[j] == stars[i].id) {
                num_matches++;
            }
        }
    }
    if (num_matches == 3) {
        return true;
    }
    else {
        return false;
    }
}

AbstractCatalogManager::AbstractCatalogManager(Parameters::Manager& params, bool enabled,
    string catalog_path_, Logging::Logger& logger_)
{ 
    image_width = params.general.image_width;
    image_height = params.general.image_height;
    mag_limit = numeric_limits<double>::infinity();
    catalog_path = fs::path(catalog_path_);
    logger = &logger_;
}

AbstractCatalogManager::~AbstractCatalogManager() {}

void AbstractCatalogManager::init()
{
    Tools::Timer timer;
    timer.start();
    if (shared_settings.enabled) { 
        logger->log(format("loading catalog %s")%catalog_path);
        regions.reserve(422); // 422 is for combo_9p0_8p0 and combo_top3_top10
        if (exists(catalog_path)) {
            load_regions();
        }
        else {
            logger->log(format("catalog %s does not exist")%catalog_path);
        }
        logger->log(format("loading catalog took %.3f s")%timer.time());
    }
}

Region AbstractCatalogManager::load_region(fs::path filename)
{
    catfloat temp = 0.0;
    Region region;
    region.stars.reserve(1451); // 1451 is the most in any region in combo_top3_top10
    ifstream infile(filename.string().c_str(), ios::in | ios::binary);		//file_string
    if (infile.is_open()) {
        infile.read(reinterpret_cast <char *> (&temp), sizeof(catfloat));
        region.ra = double(temp);
        infile.read(reinterpret_cast <char *> (&temp), sizeof(catfloat));
        region.dec = double(temp);

        RawStar raw_star;
        while (infile.read((char *) &raw_star, sizeof(raw_star))) {
            if (raw_star.mag <= mag_limit) {
                Star star;
                star.id = raw_star.id;
                star.ra = double(raw_star.ra);
                star.dec = double(raw_star.dec);
                star.mag = double(raw_star.mag);
                region.stars.push_back(star);
            }
        }


        infile.close();
    }
    return region;
}

void AbstractCatalogManager::load_regions()
{
    fs::path dir = catalog_path / "regions";
    if (exists(dir)) {
        fs::directory_iterator end;
        for (fs::directory_iterator iter(dir); iter!= end && !Shared::General::quit; ++iter) {
            if (!fs::is_directory(*iter)) {
                Region region = load_region(iter->path());
                regions.push_back(region);
            }
        }
    }

}

bool AbstractCatalogManager::in_range(double value, double min, double max)
{
    return ((value >= min) && (value <= max));
}

vector<Solution> AbstractCatalogManager::load_triplets_from_file(vector<Blob>& base_blobs,
        int leg0, int leg1,
        double leg0min, double leg0max, double leg1min,
        double leg1max, double leg2min, double leg2max, Shared::Solving::Filters& filters)
{
    fs::path filename = catalog_path / "triplets";
    filename /= boost::lexical_cast<string> (leg0);
    filename /= boost::lexical_cast<string> (leg1);

    vector<Solution> solutions;
    double epoch = get_epoch();

    RawTriplet triplet;

    ifstream infile(filename.string().c_str(), ios::in | ios::binary);	//changed from file_string KNS
    if (infile.is_open()) {
        while (infile.read((char*) &triplet, sizeof(triplet))) {

            if (!in_range(triplet.dist1, leg0min, leg0max) ||
                !in_range(triplet.dist2, leg1min, leg1max) ||
                !in_range(triplet.dist3, leg2min, leg2max)) {
                continue;
            }

            precess_if_desired(epoch, triplet.s1_ra, triplet.s1_dec);
            precess_if_desired(epoch, triplet.s2_ra, triplet.s2_dec);
            precess_if_desired(epoch, triplet.s3_ra, triplet.s3_dec);

            if (!filters.check_field_star(triplet.s1_ra, triplet.s1_dec, shared_settings.iplatescale_max)) {
                continue;
            }
            if (!filters.check_field_star(triplet.s2_ra, triplet.s2_dec, shared_settings.iplatescale_max)) {
                continue;
            }
            if (!filters.check_field_star(triplet.s3_ra, triplet.s3_dec, shared_settings.iplatescale_max)) {
                continue;
            }


            Solution solution;
            solution.base_distances.push_back(triplet.dist1);
            solution.base_distances.push_back(triplet.dist2);
            solution.base_distances.push_back(triplet.dist2);
            Star star;
            star.id = triplet.s1_id; star.ra = triplet.s1_ra; star.dec = triplet.s1_dec; star.mag = -1234;
            solution.base_stars.push_back(star);
            star.id = triplet.s2_id; star.ra = triplet.s2_ra; star.dec = triplet.s2_dec; star.mag = -1234;
            solution.base_stars.push_back(star);
            star.id = triplet.s3_id; star.ra = triplet.s3_ra; star.dec = triplet.s3_dec; star.mag = -1234;
            solution.base_stars.push_back(star);
            solution.base_blobs = base_blobs;
            solutions.push_back(solution);

        }

        infile.close();
    }
    else {
    }

    return solutions;
}

void merge_solutions(vector<Solution>& solutions, vector<Solution>& new_solutions)
{
    solutions.insert(solutions.end(), new_solutions.begin(), new_solutions.end());
}

vector<Solution> AbstractCatalogManager::load_solutions_from_triplet(vector<Blob>& blobs, Shared::Solving::Filters& filters)
{
    Tools::Timer timer;
    timer.start();

    // this is a funny thing; already skipped the beginning of load solutions
    // assume blobs has size 3 and that's the triangle we're supposed to load

    double radians_to_filename_units = 180./M_PI*3600./60.; // filenames correspond to arcsec*60
    double double_correction = 0.00000001;

    double legs_min[3];
    double legs_max[3];
    double leg_tolerance = 40.0 / 3600.0 * M_PI/180.;

    double leg0min, leg0max, leg1min, leg1max, leg2min, leg2max;
    int leg0int, leg1int, leg0min_int, leg0max_int, leg1min_int, leg1max_int;

    int i, j, k, m, n;

    vector<Solution> solutions, temp_solutions;
    vector<Blob> base_blobs;

    for (i=0; i<3; i++) {
        j = (i+1)%3;
        legs_min[i] = tanplane_distance(blobs[i].x, blobs[i].y, blobs[j].x, blobs[j].y, shared_settings.iplatescale_min);
        legs_max[i] = tanplane_distance(blobs[i].x, blobs[i].y, blobs[j].x, blobs[j].y, shared_settings.iplatescale_max);
    }

    for (m=0; m<3; m++) {
        for (n=0; n<2; n++) {

            i = m;
            j = (m+(n+1))%3;
            k = (m+(1-n+1))%3;

            base_blobs.clear();
            base_blobs.push_back(blobs[i]);
            base_blobs.push_back(blobs[j]);
            base_blobs.push_back(blobs[k]);
            if (n==0) {
                leg0min = legs_min[i] - leg_tolerance;
                leg0max = legs_max[i] + leg_tolerance;
                leg1min = legs_min[j] - leg_tolerance;
                leg1max = legs_max[j] + leg_tolerance;
                leg2min = legs_min[k] - leg_tolerance;
                leg2max = legs_max[k] + leg_tolerance;
            }
            else {
                leg0min = legs_min[j] - leg_tolerance;
                leg0max = legs_max[j] + leg_tolerance;
                leg1min = legs_min[k] - leg_tolerance;
                leg1max = legs_max[k] + leg_tolerance;
                leg2min = legs_min[i] - leg_tolerance;
                leg2max = legs_max[i] + leg_tolerance;
            }
            if (leg1min <= leg0max && leg2min <= leg1max) {

                leg0min_int = std::max(0, int(floor(leg0min*radians_to_filename_units+double_correction)));
                leg0max_int = int(floor(leg0max*radians_to_filename_units+double_correction));
                leg1min_int = std::max(0, int(floor(leg1min*radians_to_filename_units+double_correction)));
                leg1max_int = int(floor(leg1max*radians_to_filename_units+double_correction));

                for (leg0int = leg0min_int; leg0int <= leg0max_int; leg0int++) {
                    for (leg1int = leg1min_int; leg1int <= leg1max_int; leg1int++) {
                        temp_solutions = load_triplets_from_file(base_blobs,
                             leg0int, leg1int, leg0min, leg0max, leg1min, leg1max, leg2min, leg2max, filters);
                        for (unsigned int snum = 0; snum < temp_solutions.size(); snum++) {
                            if (true) {
                                solutions.push_back(temp_solutions[snum]);
                            }
                        }
                    }
                }

            }
        }
    }

    logger->log(format("cat: loading %d triplets took %f s")%solutions.size()%timer.time());

    return solutions;
}

vector<Solution> AbstractCatalogManager::load_pairs_from_file(vector<Blob>& base_blobs,
        int leg, double leg_min, double leg_max, Shared::Solving::Filters& filters)
{
    fs::path filename = catalog_path / "pairs";
    filename /= boost::lexical_cast<string> (leg);

    vector<Solution> solutions;

    double epoch = get_epoch();

    ifstream infile(filename.string().c_str(), ios::in | ios::binary);	//changed from file_string KNS
    if (infile.is_open()) {

        RawPair pair;
        while (infile.read((char *) &pair, sizeof(pair))) {
            if (!in_range(pair.dist, leg_min, leg_max)) {
                continue;
            }

            precess_if_desired(epoch, pair.s1_ra, pair.s1_dec);
            precess_if_desired(epoch, pair.s2_ra, pair.s2_dec);

            if (!filters.check_field_star(pair.s1_ra, pair.s1_dec, shared_settings.iplatescale_max)) {
                continue;
            }
            if (!filters.check_field_star(pair.s2_ra, pair.s2_dec, shared_settings.iplatescale_max)) {
                continue;
            }

            Solution solution1;
            solution1.base_distances.push_back(pair.dist);
            Star star1;
            star1.id = pair.s1_id; star1.ra = pair.s1_ra; star1.dec = pair.s1_dec; star1.mag = -1234;
            solution1.base_stars.push_back(star1);
            star1.id = pair.s2_id; star1.ra = pair.s2_ra; star1.dec = pair.s2_dec; star1.mag = -1234;
            solution1.base_stars.push_back(star1);
            solution1.base_blobs = base_blobs;
            solutions.push_back(solution1);

            Solution solution2;
            solution2.base_distances.push_back(pair.dist);
            Star star2;
            star2.id = pair.s2_id; star2.ra = pair.s2_ra; star2.dec = pair.s2_dec; star2.mag = -1234;
            solution2.base_stars.push_back(star2);
            star2.id = pair.s1_id; star2.ra = pair.s1_ra; star2.dec = pair.s1_dec; star2.mag = -1234;
            solution2.base_stars.push_back(star2);
            solution2.base_blobs = base_blobs;
            solutions.push_back(solution2);
        }

        infile.close();
    }
    else {
    }

    return solutions;
}

vector<Solution> AbstractCatalogManager::load_solutions_from_pair(vector<Blob>& blobs, Shared::Solving::Filters& filters)
{
    Tools::Timer timer;
    timer.start();

    vector<Solution> solutions, temp_solutions;

    double radians_to_filename_units = 180./M_PI*3600./60.; // filenames correspond to arcsec*60
    double double_correction = 0.00000001;
    double leg_tolerance = 40.0 / 3600.0 * M_PI/180.;

    double leg_min = tanplane_distance(blobs[0].x, blobs[0].y, blobs[1].x, blobs[1].y, shared_settings.iplatescale_min) - leg_tolerance;
    double leg_max = tanplane_distance(blobs[0].x, blobs[0].y, blobs[1].x, blobs[1].y, shared_settings.iplatescale_max) + leg_tolerance;

    vector<Blob> base_blobs;
    for (unsigned int i=0; i<2; i++) {
        unsigned int j = 1-i;
        base_blobs.clear();
        base_blobs.push_back(blobs[i]);
        base_blobs.push_back(blobs[j]);

        int leg_min_int = std::max(0, int(floor(leg_min*radians_to_filename_units+double_correction)));
        int leg_max_int = int(floor(leg_max*radians_to_filename_units+double_correction));
        for (int leg_int = leg_min_int; leg_int <= leg_max_int; leg_int++) {
            temp_solutions = load_pairs_from_file(base_blobs, leg_int, leg_min, leg_max, filters);
            for (unsigned int snum = 0; snum < temp_solutions.size(); snum++) {
                if (true) {
                    solutions.push_back(temp_solutions[snum]);
                }
            }
        }
    }
    logger->log(format("cat: loading %d pairs took %f s")%solutions.size()%timer.time());
    return solutions;
}


vector<Star> AbstractCatalogManager::get_stars_near(double ra, double dec)
{
    unsigned int i, best_i;
    double distance, best_distance;
    best_distance = numeric_limits<double>::infinity();
    best_i = 0;
    for (i=0; i<regions.size(); i++) {
        distance = great_circle(ra, dec, regions[i].ra, regions[i].dec);
        if (distance < best_distance) {
            best_distance = distance;
            best_i = i;
        }
    }
    return regions[best_i].stars;
}

bool cm_is_inbounds_centered(double& u, double& v, double& width, double& height)
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

double AbstractCatalogManager::get_epoch()
{
    double epoch = 2000.0;
    if (shared_settings.precession_epoch_source == Shared::Solving::Settings::precession_system_time) {
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        int year = now.date().year();
        int month = now.date().month();
        int day = now.date().day();
        double mjd;
        int flag;
        Slalib::slaCaldj(year, month, day, &mjd, &flag);
        epoch = Slalib::slaEpj(mjd);
    } else if (shared_settings.precession_epoch_source == Shared::Solving::Settings::precession_manual) {
        epoch = shared_settings.precession_manual_epoch;
    }
    return epoch;
}

void AbstractCatalogManager::precess_if_desired(double epoch, float& ra, float& dec)
{
    double ra_d = (double) ra;
    double dec_d = (double) dec;
    precess_if_desired(epoch, ra_d, dec_d);
    ra = (float) ra_d;
    dec = (float) dec_d;
}

void AbstractCatalogManager::precess_if_desired(double epoch, double& ra, double& dec)
{
    if (shared_settings.precession_epoch_source == Shared::Solving::Settings::precession_system_time ||
        shared_settings.precession_epoch_source == Shared::Solving::Settings::precession_manual)
    {
        Slalib::slaPreces((char*) "FK5", 2000, epoch, &ra, &dec);
    }
}

void AbstractCatalogManager::fill_stars_in_fov(Solution& solution, Shared::Image::Raw& image,
    Shared::Solving::Refraction& refraction_settings)
{

    vector<Star> nearby_stars;
    double u = 0.0;
    double v = 0.0;
    double cos_roll = cos(solution.equatorial.roll);
    double sin_roll = sin(solution.equatorial.roll);
    double distance = 0.0;
    double width = image_width;
    double height = image_height;

    nearby_stars = get_stars_near(solution.equatorial.ra, solution.equatorial.dec);

    double epoch = get_epoch();
    for (unsigned int i=0; i<nearby_stars.size(); i++) {
        precess_if_desired(epoch, nearby_stars[i].ra, nearby_stars[i].dec);
    }

    double star_ra = 0.0;
    double star_dec = 0.0;
    for (unsigned int i=0; i<nearby_stars.size(); i++) {
        distance  = great_circle(solution.equatorial.ra, solution.equatorial.dec, nearby_stars[i].ra, nearby_stars[i].dec);
        if (distance < from_degrees(2.5)) {
            Tools::get_refraction_corrected_equatorial(nearby_stars[i].ra, nearby_stars[i].dec,
                shared_settings.refraction, image.filters, star_ra, star_dec);
            Tools::ccd_projection(star_ra, star_dec, solution.equatorial.ra, solution.equatorial.dec,
                1.0/solution.equatorial.iplatescale, cos_roll, sin_roll, u, v, true);

            if (cm_is_inbounds_centered(u, v, width, height)) {
                solution.stars_in_fov.push_back(nearby_stars[i]);
            }
        }
    }
}

void AbstractCatalogManager::get_star_name(int id, string& name, double& mag)
{
    Tools::Timer timer;
    timer.start();

    string line;
    int max_line_reads = 200000;
    int counter = 0;
    int in0 = 0;
    double in1, in2, in3;
    string in4;
    unsigned int pos_of_first_space = 0;
    bool aborting_loop = false;
    unsigned int aborting_loop_counter = 0;

    fs::path filename = catalog_path / "lists" / "stars.txt";
    ifstream infile(filename.string().c_str());			//changed from file_string to string KNS
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    if (infile.is_open()) {
        while (infile.good() && counter<max_line_reads && !aborting_loop) {
            if (aborting_loop_counter == 0) {
                aborting_loop_counter = 1000;
                if (abort()) {
                    aborting_loop = true;
                }
            }
            aborting_loop_counter--;

            getline(infile, line);
            pos_of_first_space = line.find(' ');
            if (pos_of_first_space != string::npos) {
                // find + atoi are much faster than stringstream
                in0 = atoi( (line.substr(0, pos_of_first_space)).c_str() );
                if (in0 == id) {
                    ss.clear();
                    ss.str("");
                    ss << line;
                    ss >> in0 >> in1 >> in2 >> in3 >> in4;
                    if (!ss.fail()) {
                        name = in4.substr(0, 16);
                        mag = in3;
                        counter = max_line_reads;
                    }
                }
            }
        }
        infile.close();
    }
    logger->log(format("cat: loading star name took %f s") % timer.time());
}

bool AbstractCatalogManager::abort()
{
    return false;
}

