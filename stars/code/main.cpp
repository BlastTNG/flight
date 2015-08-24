/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include <string.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#ifndef _MSC_VER
    #include <signal.h>
#endif
#include "star_camera.h"
#include "parameters/manager.h"
#include "shared/shared_list.h"
#include "tools/timing.h"
#include "logging/logger.h"
#include "solving/logger.h"
#include "housekeeping/logger.h"
#include "imaging/logger_lens.h"
#include "imaging/logger_camera.h"
#include "networking/logger.h"
#include "logger_main.h"

#define shared_shutdown (*(Shared::General::shutdown_for_main.r))
#define CBuf  CircularBuffer
#define CBufP CircularBufferPass

// Every logger's update function should be regularly called by its owner (thread)
Logging::Logger Solving::logger("solving", true);
Logging::Logger Housekeeping::logger("hk", false);
Logging::Logger Imaging::Lensing::logger("lens", true);
Logging::Logger Imaging::Cameraing::logger("cam", true);
Logging::Logger Main::logger("main", true);
Logging::Logger Networking::logger("network", true);
Logging::Logger NetworkingImageClient::logger1("nic1", true);
Logging::Logger NetworkingImageClient::logger2("nic2", true);

namespace Shared
{
using namespace ThreadNames;
CBufP  <Autofocus::Datapoints>         Autofocus::datapoints_solver_to_lens    (solver,      lens);
CBuf   <Autofocus::Datapoints>         Autofocus::datapoints_lens_to_main      (lens,        main);
CBuf   <Autofocus::LatestImage>        Autofocus::latest_image                 (camera,      lens);
CBufP  <Autofocus::Requests>           Autofocus::requests_network_to_main     (net_client,  main);
CBuf   <Autofocus::Requests>           Autofocus::requests_main_to_lens        (main,        lens);
CBufP  <Autofocus::Results>            Autofocus::results_lens_to_solver       (lens,        solver);
CBufP  <Autofocus::Results>            Autofocus::results_solver_to_main       (solver,      main);
CBuf   <Autofocus::Results>            Autofocus::results_main_to_network      (main,        net_server);
CBufP  <Camera::Requests>              Camera::requests_for_main               (net_client,  main);
CBuf   <Camera::Requests>              Camera::requests_for_camera             (main,        camera);
CBufP  <Camera::Results>               Camera::results_for_main                (camera,      main);
CBuf   <Camera::Results>               Camera::results_for_network             (main,        net_server);
CBufP  <General::MainSettings>         General::main_settings_net_for_main     (net_client,  main);
CBuf   <General::MainSettings>         General::main_settings_main_for_solver  (main,        solver);
CBuf   <General::Shutdown>             General::shutdown_for_main              (net_client,  main);
CBuf   <General::NetworkReset>         General::network_reset_for_net_reset    (net_client,  net_reset);
CBuf   <General::NetworkResetStatus>   General::network_reset_status_for_main  (net_reset,   main);
CBufP  <Housekeeping::Housekeeper>     Housekeeping::housekeeper_for_camera    (main,        camera);
CBuf   <Housekeeping::Housekeeper>     Housekeeping::housekeeper_for_network   (camera,      net_server);
CBufP  <Image::Blobs>                  Image::blobs_solver_for_main            (solver,      main);
CBuf   <Image::Blobs>                  Image::blobs_main_for_net               (main,        net_server);
CBuf   <Image::Leveled>                Image::leveled                          (solver,      main);
CBuf   <Image::Matching>               Image::matching                         (solver,      main);
CBuf   <Image::MatchingProgress>       Image::matching_progress                (solver,      main);
CBuf   <Image::Raw>                    Image::raw_from_camera                  (camera,      solver);
CBuf   <Image::Raw>                    Image::raw_for_image_client1     (camera,      net_image_client1);
CBuf   <Image::Raw>                    Image::raw_for_image_client2     (camera,      net_image_client2);
CBufP  <Image::SolutionSummary>        Image::solution_summary_for_main        (solver,      main);
CBuf   <Image::SolutionSummary>        Image::solution_summary_main_for_net    (main,        net_server);
CBufP  <Image::Stats>                  Image::stats_solver_for_main            (solver,      main);
CBuf   <Image::Stats>                  Image::stats_main_for_net               (main,        net_server);
CBufP  <Image::Status>                 Image::status_solver_for_main           (solver,      main);
CBufP  <Image::Status>                 Image::status_main_for_camera           (main,        camera);
CBuf   <Image::Status>                 Image::status_camera_for_network        (camera,      net_server);
CBufP  <Lens::Requests>                Lens::fcp_requests_network_to_main      (net_client,  main);
CBufP  <Lens::Requests>                Lens::fcp_requests_main_to_camera       (main,        camera);
CBuf   <Lens::Requests>                Lens::fcp_requests_camera_to_lens       (camera,      lens);
CBufP  <Lens::Requests>                Lens::stars_requests_lens_to_main       (lens,        main);
CBufP  <Lens::Requests>                Lens::stars_requests_main_to_camera     (main,        camera);
CBuf   <Lens::Requests>                Lens::stars_requests_camera_to_lens     (camera,      lens);
CBufP  <Lens::Results>                 Lens::fcp_results_lens_to_camera        (lens,        camera);
CBufP  <Lens::Results>                 Lens::fcp_results_camera_to_main        (camera,      main);
CBuf   <Lens::Results>                 Lens::fcp_results_main_to_network       (main,        net_server);
CBufP  <Lens::Results>                 Lens::stars_results_lens_to_camera      (lens,        camera);
CBuf   <Lens::Results>                 Lens::stars_results_camera_to_main      (camera,      main);
CBufP  <Network::Client>               Network::client_for_main                (net_client,  main);
CBuf   <Network::Client>               Network::client_for_camera              (main,        camera);
CBuf   <Network::Packets>              Network::packets_from_fc1               (net_client,  main);
CBuf   <Network::Packets>              Network::packets_from_fc2               (net_client,  main);
CBuf   <Network::ImageClientSettings>  Network::image_client_settings1         (net_client,  net_image_client1);
CBuf   <Network::ImageClientSettings>  Network::image_client_settings2         (net_client,  net_image_client2);
CBuf   <Simulations::Brightness>       Simulations::brightness                 (net_client,  camera);
CBufP  <Solving::Filters>              Solving::filters_net_to_main            (net_client,  main);
CBuf   <Solving::Filters>              Solving::filters_main_to_camera         (main,        camera);
CBufP  <Solving::Mask>                 Solving::mask_network_for_solver        (net_client,  solver);
CBuf   <Solving::Mask>                 Solving::mask_solver_for_main           (solver,      main);
CBuf   <Solving::MotionPSF>            Solving::motion_psf_network_for_solver  (net_client,  solver);
CBuf   <Solving::Settings>             Solving::settings                       (net_client,  solver);
bool General::quit;
}

void init_loggers(Parameters::Manager& params, boost::posix_time::ptime& birthtime, Tools::Timer& age)
{
    Solving::logger.init(params, birthtime, age);
    Housekeeping::logger.init(params, birthtime, age);
    Imaging::Lensing::logger.init(params, birthtime, age);
    Imaging::Cameraing::logger.init(params, birthtime, age);
    Main::logger.init(params, birthtime, age);
    Networking::logger.init(params, birthtime, age);
    NetworkingImageClient::logger1.init(params, birthtime, age);
    NetworkingImageClient::logger2.init(params, birthtime, age);
}

void init_shared_objects(Parameters::Manager& params)
{
    Shared::Solving::settings.init(params);
    Shared::Solving::filters_net_to_main.init(params);
    Shared::Solving::filters_main_to_camera.init(params);
    Shared::Solving::mask_network_for_solver.init(params);
    Shared::Solving::mask_solver_for_main.init(params);
    Shared::Solving::motion_psf_network_for_solver.init(params);
    Shared::Simulations::brightness.init(params);
    Shared::Image::raw_from_camera.init(params);
    Shared::Image::raw_for_image_client1.init(params);
    Shared::Image::raw_for_image_client2.init(params);
    Shared::Image::leveled.init(params);
    Shared::Autofocus::requests_network_to_main.init(params);
    Shared::Autofocus::requests_main_to_lens.init(params);
    Shared::Network::image_client_settings1.init(params);
    Shared::Network::image_client_settings2.init(params);
    Shared::General::main_settings_net_for_main.init(params);
    Shared::General::main_settings_main_for_solver.init(params);
    Shared::General::network_reset_for_net_reset.init(params);
    Shared::General::quit = false;
}

void kill_handler(int signal)
{
    Main::logger.log(format("caught signal %d") % signal);
    Shared::General::quit = true;
}

void register_kill_handler()
{
	#ifndef _MSC_VER
		struct sigaction handler;
		handler.sa_handler = kill_handler;
		sigemptyset(&handler.sa_mask);
		handler.sa_flags = 0;
		sigaction(SIGINT, &handler, NULL);
		sigaction(SIGTERM, &handler, NULL);
	#endif
}

void shutdown_if_requested()
{
	#ifdef _MSC_VER
    if (shared_shutdown.shutdown_now) {
        HANDLE hToken;
        TOKEN_PRIVILEGES tkp;
        if (!OpenProcessToken(GetCurrentProcess(), TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &hToken)) {
            return;
        }
        LookupPrivilegeValue(NULL, SE_SHUTDOWN_NAME, &tkp.Privileges[0].Luid);
        tkp.PrivilegeCount = 1;
        tkp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
        AdjustTokenPrivileges(hToken, FALSE, &tkp, 0, (PTOKEN_PRIVILEGES)NULL, 0);
        if (GetLastError() != ERROR_SUCCESS) {
            return;
        }
        UINT restart_flag = EWX_POWEROFF;
        if (shared_shutdown.include_restart) {
            Main::logger.log("STARS is finishing, requesting shutdown with restart");
            restart_flag = EWX_REBOOT;
        } else {
            Main::logger.log("STARS is finishing, requesting shutdown without restart");
            restart_flag = EWX_POWEROFF;
        }
        if(!ExitWindowsEx(restart_flag | EWX_FORCEIFHUNG, SHTDN_REASON_MAJOR_OTHER)) {
            Main::logger.log("ExitWindows call was unsuccessful");
        } else {
            Main::logger.log("ExitWindows call was successful");
        }
    }
    #endif
}

int main(int argc, char* argv[]){
    Tools::Timer age;
    age.start();
    boost::posix_time::ptime birthtime = boost::posix_time::microsec_clock::local_time();
    //int width = 1536;		//dimensions of practice images
    //int height = 1024;
    //int depth = 4096;
	int width = 1392;		//new dimensions for QCam
	int height = 1040;
	int depth = 16384;
	
	#ifdef _MSC_VER
        std::string stars_absolute_dir = "C:/stars_data/";
    #else
        std::string stars_absolute_dir = "../../";
    #endif
    Parameters::Manager params(width, height, depth, stars_absolute_dir);
    params.load(argc, argv);
    init_loggers(params, birthtime, age);
    params.load(argc, argv);
    Main::logger.log("STARS is running");
    register_kill_handler();
    init_shared_objects(params);
    Shared::Image::Raw solvers_working_image;
    solvers_working_image.init(params);
    StarCamera star_camera(params, solvers_working_image);
    star_camera.run(params);
    shutdown_if_requested();
    Main::logger.log("STARS is finished");
    return 0;
}

