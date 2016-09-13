echo "No additional package dependencies for now!"

# This should be extended to first check if everything is installed and only do the sudo requiring call when there's anything missing.
#echo "Installing needed packages (both ROS package and system dependency .deb packages) ..."
#
#PACKAGES_TO_INSTALL="\
#mercurial \
#git \
#python-catkin-tools \
#protobuf-compiler \
#libargtable2-dev \
#libcoin80-dev \
#libglew-dev \
#libgsl0-dev \
#liblapack-dev \
#libsnappy-dev \
#libsoqt4-dev \
#libtinyxml-dev \
#libunittest++-dev \
#python-rosinstall \
#libsdl-image1.2-dev \
#libvtk-java \
#python-pymodbus \
#ros-$ROS_DISTRO-spacenav-node \
#ros-$ROS_DISTRO-bfl \
#ros-$ROS_DISTRO-cmake-modules \
#ros-$ROS_DISTRO-desktop \
#ros-$ROS_DISTRO-eigen-stl-containers \
#ros-$ROS_DISTRO-map-server \
#ros-$ROS_DISTRO-laser-assembler"
#
#
#sudo apt-get -y install $PACKAGES_TO_INSTALL
