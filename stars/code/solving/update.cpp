/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "update.h"
#include "../shared/update.h"
#include "../shared/image/status.h"
#include "../shared/solving/settings.h"
#include "../shared/general/quit.h"
#include "logger.h"

#define shared_status (*(Shared::Image::status_solver_for_main.w))
#define shared_settings (*(Shared::Solving::settings.r))

void Solving::update_shared()
{
    Shared::update(Shared::ThreadNames::solver);
    logger.update();
}

bool Solving::done()
{
    using namespace Shared::Image;

    if (Shared::General::quit) {
        return true;
    }

    if (shared_status.stage == Status::done) {
        return true;
    }

    update_shared();

    if (shared_status.abort_counter != shared_settings.abort_counter) {
        shared_status.abort_counter = shared_settings.abort_counter;
        shared_status.stage = Status::done;
        shared_status.reason_for_being_done = Status::aborted;
        Shared::Image::status_solver_for_main.share();
        return true;
    }

    if (shared_status.age.time() > shared_settings.timeout) {
        shared_status.stage = Status::done;
        shared_status.reason_for_being_done = Status::timed_out;
        Shared::Image::status_solver_for_main.share();
        return true;
    }
    if (!shared_settings.enabled) {
        shared_status.stage = Status::done;
        shared_status.reason_for_being_done = Status::not_solving;
        Shared::Image::status_solver_for_main.share();
        return true;
    }
    return false;
}

