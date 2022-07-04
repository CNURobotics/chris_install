#!/bin/bash

if [ -z $WORKSPACE_ROOT ]; then
  echo "Variable WORKSPACE_ROOT not set, make sure the workspace is set up properly!"
  exit 1
fi


cd $WORKSPACE_ROOT


#--------------------- common code below here ----------------------------
# Optionally check if update and build are to be ignored.
# Not doing update and build saves some time when called from other scripts
while [ -n "$1" ]; do
  case $1 in
  --no_ws_update)
      UPDATE_WORKSPACE=1
      ;;
  --no_build)
      BUILD_WORKSPACE=1
      ;;
  esac

  shift
done

if [ -n "$UPDATE_WORKSPACE" ]; then
  echo "Not updating workspace as --no_ws_update was set"
else
  echo "Updating workspace ..."
  wstool update
  echo "Updating dependencies ..."
  rosdep update
  rosdep install -r --from-path . --ignore-src
fi

# invoke build after install
if [ -n "$BUILD_WORKSPACE" ]; then
  echo "Not building workspace as --no_build was set"
else
  echo "Building ROS workspace ... "
  colcon build
  echo
  echo "Do not forget to update the workspace by invoking setup.bash after build!"

fi
echo "Done!"
