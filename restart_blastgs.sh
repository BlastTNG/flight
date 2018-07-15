#! /bin/bash
REPO_DIR="~/flight_master"
GROUNDHOG_BUILD_DIR="$REPO_DIR/groundhog/build"
BLASTCMD_BUILD_DIR="$REPO_DIR/blastcmd"

if [ -z "$2" ]
then
  GIT_BRANCH="master"
else
  GIT_BRANCH="$2"
fi

GIT_PULL_CMD="cd $REPO_DIR; git pull; git checkout $GIT_BRANCH; git pull origin $GIT_BRANCH"

GROUNDHOG_BUILD_CMD="cd $GROUNDHOG_BUILD_DIR; cmake ../; make clean all; sudo make install"
GROUNDHOG_RESTART_CMD="sudo pkill groundhog"

BLASTCMD_BUILD_CMD="cd $BLASTCMD_BUILD_DIR; make; sudo make install"
BLASTCMD_RESTART_CMD="sudo pkill blastcmd"

LINKLIST_UPDATE_CMD="cd $REPO_DIR; ./upload_linklists.h"

ssh -t blast@$1 "$GIT_PULL_CMD; $LINKLIST_UPDATE_CMD; $GROUNDHOG_BUILD_CMD; $GROUNDHOG_RESTART_CMD; $BLASTCMD_BUILD_CMD; $BLASTCMD_RESTART_CMD"
