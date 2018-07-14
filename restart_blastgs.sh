#! /bin/bash

REPO_DIR="~/flight_master"
GROUNDHOG_BUILD_DIR="$REPO_DIR/groundhog/build"

GIT_PULL_CMD="cd $REPO_DIR; git pull origin master"
GROUNDHOG_BUILD_CMD="cd $GROUNDHOG_BUILD_DIR; cmake ../; make clean all; sudo make install"
GROUNDHOG_RESTART_CMD="sudo pkill groundhog"

ssh -t blast@$1 "$GIT_PULL_CMD; $GROUNDHOG_BUILD_CMD; $GROUNDHOG_RESTART_CMD"
