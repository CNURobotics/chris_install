#!/bin/bash

#cd $WORKSPACE_ROOT

if [ -z $WORKSPACE_ROOT ]; then
  echo "Variable WORKSPACE_ROOT not set, make sure the workspace is set up properly!"
  exit 1
fi

echo "Installing CHRISLab software setup ..."

cd $WORKSPACE_ROOT

# List the rosinstall files containing any packages we wish to install here
wstool merge rosinstall/optional/flexible_navigation.rosinstall
if [ $? -eq 0 ]
then
  echo "Successfully merged FlexBE + Flexible Navigation ROS install files, now merge standard third party repos for base ..."
  wstool merge rosinstall/optional/chris_third_party.rosinstall
else
  echo "Failed to merge flexible navigation repos !"
  exit 1
fi

if [ $? -eq 0 ]
then
  echo "Successfully merged ROS install files, now update and build ..."
  sh rosinstall/install_scripts/update_and_build.sh $1
else
  echo "Could not merge flexible navigation repos " >&2
fi
