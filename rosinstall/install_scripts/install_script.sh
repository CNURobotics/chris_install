#!/bin/bash

#cd $WORKSPACE_ROOT

# parse parameters
while [ -n "$1" ]; do
    case $1 in
    --no_sim)
        WORKSPACE_CHRIS_NO_SIM=1
        ;;
    esac

    shift
done

if [ -z "$WORKSPACE_ROOT" ]; then
    WORKSPACE_ROOT=$(cd `dirname $0`; pwd)
fi
echo "Using workspace root of $WORKSPACE_ROOT"

# exit if one of the commands fail
#set -e

# install

# delete old files
echo Cleaning up old workspace files...
for f in .rosinstall* devel build; do
    [ -f $f ] && echo "rm -iv $f" && rm -i $f
    [ -d $f ] && echo "rm -Irv $f" && rm -Ir $f
done
echo

# find an installation of ROS
if [ -z "$ROS_DISTRO" ]; then
    _ROS_DISTROS="melodic"

    # use basename of the current folder as default ROS distro
    ROS_DISTRO=$(basename $(cd `dirname $0`; pwd))
    for _distro in $_ROS_DISTROS ask; do
        if [ "$_distro" = "$ROS_DISTRO" ]; then break; fi
    done

    if [ "$_distro" = "ask" ]; then
        echo -n "Which ROS distro you want to setup this workspace for ($_ROS_DISTROS)? "
        read ROS_DISTRO
    fi

fi

# Check to see if this distro actually exists
if [ ! -r /opt/ros/$ROS_DISTRO/setup.sh ]; then
    echo "Directory /opt/ros/$ROS_DISTRO does not exists!"
    exit 1
fi

echo " Source the base $ROS_DISTRO setup for clean install ..."
source /opt/ros/$ROS_DISTRO/setup.sh
echo

# make sure package dependencies are installed
source $WORKSPACE_ROOT/rosinstall/install_scripts/install_package_dependencies.sh

# initialize workspace
if [ ! -f ".rosinstall" ]; then
    wstool init .
fi

# merge rosinstall files from rosinstall/*.rosinstall
for file in $WORKSPACE_ROOT/rosinstall/*.rosinstall; do
    filename=$(basename ${file%.*})
    if [ -n "$WORKSPACE_CHRIS_NO_SIM" ] && [ $filename == "chris_simulation" ]; then
        continue;
    else
        echo "Merging to workspace: '$filename'.rosinstall"
        wstool merge $file -y
    fi
done
echo

# update workspace
wstool update
echo

# install dependencies
rosdep update
rosdep install -r --from-path . --ignore-src

# --- The symlink is no longer added, so removing the copy -----
# remove symlink to write protected CMakefile and use a copy instead
#rm $WORKSPACE_ROOT/src/CMakeLists.txt
#cp /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake $WORKSPACE_ROOT/src/CMakeLists.txt

echo

# generate top-level setup.bash
cat >setup.bash <<EOF
#!/bin/bash
# automated generated file
export ROS_HOSTNAME=$HOSTNAME
. $WORKSPACE_ROOT/devel/setup.bash
echo "Set up ROS workspace for \$WORKSPACE_ROOT@\$ROS_HOSTNAME"
EOF

. $WORKSPACE_ROOT/setup.bash

# invoke make for the initial setup
catkin build
echo

echo "Create a folder to hold log files ..."
mkdir -p $WORKSPACE_ROOT/logs

# Initialization successful. Print message and exit.
cat <<EOF

===================================================================

Workspace initialization completed.
You can setup your current shell's environment by entering

    source $WORKSPACE_ROOT/setup.bash

or by executing the below command to add the workspace setup to your
.bashrc file for automatic setup on each invocation of an interactive shell:

    echo "source $WORKSPACE_ROOT/setup.bash" >> ~/.bashrc

ROS Networking Note:
This setup.bash script sets the ROS_HOSTNAME of this machine to the HOSTNAME of this machine.
For single machine testing, you might want to change this to "localhost" by editing
$WORKSPACE_ROOT/setup.bash

You can modify your workspace config (e.g. for adding additional repositories or
packages) by using the wstool command (http://wiki.ros.org/wstool).
See the system install scripts in $WORKSPACE_ROOT/rosinstall/install_scripts and
example rosinstall files in $WORKSPACE_ROOT/rosinstall/optional for example usage.

===================================================================

EOF
