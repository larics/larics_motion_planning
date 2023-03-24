#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

# additional setup
SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=additional_setup.sh


# start tmuxinator
tmuxinator hawk_asap_sim setup_name=$SETUP_NAME
