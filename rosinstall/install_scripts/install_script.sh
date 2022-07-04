#!/bin/bash

if [ -z "$WORKSPACE_ROOT" ]; then
    WORKSPACE_ROOT=$(cd `dirname $0`; pwd)
    echo "Setting workspace root of $WORKSPACE_ROOT"
else
    echo "Using workspace root of $WORKSPACE_ROOT"
fi

# find an installation of ROS
if [ -z "$ROS_DISTRO" ]; then
    # No distribution set up, so either base off of workspace name, or
    # ask user to specify
    _ROS_DISTROS="humble"

    # use basename of the current folder as default ROS distro
    ROS_DISTRO=$(basename $(cd `dirname $0`; pwd))

    # Check if base WORKSPACE folder name matches a ROS distribution name
    # (For mutliple version setups)
    for _distro in $_ROS_DISTROS ask; do
        if [ "$_distro" = "$ROS_DISTRO" ]; then
           echo "Using $_distro as desired ROS distribution "
           break;
        fi
    done

    if [ "$_distro" = "ask" ]; then
        echo -n "Which ROS distro you want to setup this workspace for (e.g. $_ROS_DISTROS)? "
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

echo "Define standard src folder ..."
mkdir -p src

# make sure package dependencies are installed
echo " Install any standard package dependencies ..."
source $WORKSPACE_ROOT/rosinstall/install_scripts/install_package_dependencies.sh

# initialize workspace
if [ ! -f ".rosinstall" ]; then
    echo "Calling wstool init ..."
    wstool init .
fi

# merge any rosinstall files from rosinstall/*.rosinstall
for file in $WORKSPACE_ROOT/rosinstall/*.rosinstall; do
    filename=$(basename ${file%.*})
    if [ -n "$WORKSPACE_CHRIS_NO_SIM" ] && [ $filename == "chris_simulation" ]; then
        echo "Skipping wstool merge ..."
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

echo

# generate top-level setup.bash
cat >setup.bash <<EOF
#!/bin/bash
# automated generated file
export ROS_DOMAIN_ID=113
export WORKSPACE_ROOT=$WORKSPACE_ROOT
export TURTLEBOT3_MODEL=burger


if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ];
then
  echo "Setting up ROS workspace ..."
  . ${WORKSPACE_ROOT}/install/setup.bash
else
  echo "Setting up clean ${ROS_DISTRO} workspace ..."
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi

echo "Set up ROS 2 \${ROS_DISTRO} workspace for \${WORKSPACE_ROOT}@DDS:\$ROS_DOMAIN_ID"


# Define relevant environment variables
# The following are used by some launch scripts

# Presuming simulation for now
export USE_SIM_TIME=true
echo "Use sime time =\${USE_SIM_TIME}"

export FLEX_NAV_SETUP=flex # (e.g. flex, flex_multi_level)
echo "FLEX_NAV setup=\${FLEX_NAV_SETUP} ..."

export LOCALIZATION=slam # (e.g. slam, amcl, or cartographer)
echo "Localization setup uses \${LOCALIZATION}"

export WORLD_MODEL=gazebo_creech_world
echo "Simulation world model setup uses \${WORLD_MODEL}"


CHRIS_SCRIPTS="\${WORKSPACE_ROOT}/install/chris_scripts/lib/chris_scripts/"
if [ -d "\${CHRIS_SCRIPTS}" ] && [[ ! \$PATH =~ (^|:)\${CHRIS_SCRIPTS}(:|\$) ]]; then
    PATH+=":\${CHRIS_SCRIPTS}"
    echo "Added chris_scripts to system path ..."
fi


# Define a bash shortcut called "rws" since roscd does not exist in ROS 2 Humble
# ROS Workspace cd (rws)
rws() {
  cd "\$WORKSPACE_ROOT"
}

# Define aliases for safe build by first changing to workspace
ccb() {
  cd "\$WORKSPACE_ROOT"
  pwd
  colcon build
}

ccbs() {
  cd "\$WORKSPACE_ROOT"
  pwd
  colcon build --packages-select "\$1"
}

ccbu() {
  cd "\$WORKSPACE_ROOT"
  pwd
  colcon build --packages-up-to "\$1"
}



EOF

. $WORKSPACE_ROOT/setup.bash

# invoke make for the initial setup
colcon build
echo

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
This setup.bash script sets the ROS_DOMAIN_ID of this machine based
on last two digits of HOSTNAME, which assumes single computer usage.

For multiple machine use, you may need to change the ROS_DOMAIN_ID by
editing $WORKSPACE_ROOT/setup.bash

You can modify your workspace config (e.g. for adding additional repositories or
packages) by using the wstool command (http://wiki.ros.org/wstool).
See the system install scripts in $WORKSPACE_ROOT/rosinstall/install_scripts and
example rosinstall files in $WORKSPACE_ROOT/rosinstall/optional for example usage.

===================================================================

EOF
