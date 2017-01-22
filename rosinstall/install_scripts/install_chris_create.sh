#!/bin/bash

#cd $WORKSPACE_ROOT

if [ -z $WORKSPACE_ROOT ]; then
  echo "Variable WORKSPACE_ROOT not set, make sure the workspace is set up properly!"
else
  echo "Installing CHRISLab software setup ..."

  cd $WORKSPACE_ROOT

  # List the rosinstall files containing any packages we wish to install here
  wstool merge rosinstall/optional/chris_flexible_navigation.rosinstall
  wstool merge rosinstall/optional/chris_third_party.rosinstall
  wstool merge rosinstall/optional/chris_create.rosinstall
  wstool merge rosinstall/optional/chris_create_behaviors.rosinstall
  wstool merge rosinstall/optional/chris_create_navigation.rosinstall



  #--------------------- common code below here ----------------------------
  # Optionally check if update is requested. Not doing update saves some
  # time when called from other scripts
  while [ -n "$1" ]; do
    case $1 in
    --no_ws_update)
        UPDATE_WORKSPACE=1
        ;;
    esac

    shift
  done

  if [ -n "$UPDATE_WORKSPACE" ]; then
    echo "Not updating workspace as --no_ws_update was set"
  else
    wstool update
    rosdep update
    rosdep install -r --from-path . --ignore-src
  fi

fi
