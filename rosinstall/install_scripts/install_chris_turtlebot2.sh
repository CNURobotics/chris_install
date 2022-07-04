#!/bin/bash

#cd $WORKSPACE_ROOT

if [ -z $WORKSPACE_ROOT ]; then
  echo "Variable WORKSPACE_ROOT not set, make sure the workspace is set up properly!"
  exit 1
fi

echo "Installing CHRISLab Turtlebot software setup ..."

cd $WORKSPACE_ROOT

# List the rosinstall files containing any packages we wish to install here
wstool merge rosinstall/optional/flexible_navigation.rosinstall
if [ $? -eq 0 ]
then
  echo "Successfully merged FlexBE + Flexible Navigation ROS install files, now merge third party repos ..."
  wstool merge rosinstall/optional/chris_third_party.rosinstall
else
  echo "Failed to merge flexible navigation repos !"
  exit 1
fi

if [ $? -eq 0 ]
then
  echo "Successfully merged third party ROS install files, now merge CHRISLab TurtleBot 2 repos ..."
  wstool merge rosinstall/optional/chris_turtlebot2.rosinstall
else
  echo "Failed to merge third party files!"
  exit 1
fi

if [ $? -eq 0 ]
then
  echo "Successfully merged required ROS install files, now update and build ..."
  sh rosinstall/install_scripts/update_and_build.sh $1
else
  echo "Could not merge turtlebot demo repos " >&2
fi
