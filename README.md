# CNU Robotics CHRISLab ROS Workspace Installation

Install and setup files for the basic setup of our software.

Computer Setup
--------------

 * Follow the [ROS Installation] instructions for setting up your ROS system on Ubuntu.
  * This system has been tested on ROS Kinetic Release on `Ubuntu 16.04`.
  * We recommend `ros-kinetic-desktop-full` as the base for this system; it includes most of the required packages, including the Gazebo simulator for testing.
 * Install the [catkin_tools] package.
  * `sudo apt-get install python-catkin-tools`
  * This is required by many of our scripts, and is generally preferred over the standard catkin_build system
 * Install ROS stand alone tools
  * `sudo apt-get install python-rosinstall`
  * Our install script (below) uses rosinstall tools
 * Install your favorite editor or IDE. (We mostly use QtCreator and atom.)


CHRISLab Software Setup
-----------------------

#### *If installing a new workspace, remove existing workspace setup from .bashrc, and reopen terminal sourcing only the /opt/ros/kinetic/setup.bash prior to running this script.*

1. Create workspace root folder (e.g. ~/CHRISLab)  and change to that directory
<pre>
mkdir CHRISLab
cd CHRISLab
</pre>

2. Clone the install setup  (*_Note: the dot at end is critical!_* )
 * `git clone https://github.com/CNURobotics/chris_install.git .`
 * We are choosing to clone within our workspace root folder, as opposed to creating the folder during the clone.  The cloned folder should not be named chris_install.

3. Change to correct branch
 `git checkout kinetic_devel`
 * This version has the CHRISLab iRobot Create and Kobuki-based Turtlebot setup, including the complete `flexible_navigation` demonstration.

4. Run the install script
 `./install.sh`

5. Follow on-screen instructions to add the new setup to bashrc and re-source the terminal

6. Test the setup
  `roscd`
  * you should be in the workspace root /src folder  (e.g. ~/CHRISLab/src )

7. Go back to the original directory
 `cd $WORKSPACE_ROOT`
  * This should put you back where you started (e.g. ~/CHRISLab)

8. Install the CHRISLab specific code
 * The most complete demonstration instructions can be found at https://github.com/CNURobotics/chris_turtlebot_flexible_navigation
 * `./rosinstall/install_scripts/install_chrislab.sh` for just base software, or
 * `./rosinstall/install_scripts/install_chris_turtlebot.sh` for base + Turtlebot specific, or
 * `./rosinstall/install_scripts/install_chris_create.sh` for base + Create specific
 * *NOTE:* Create and Turtlebot set ups are compatible with one another in the same workspace.

9. Build and install any external libraries installed in the `$WORKSPACE_ROOT/external` folder
  * For example:
<pre>
cd $WORKSPACE_ROOT/external/sbpl
mkdir build
cd build
cmake ..
make
sudo make install
</pre>

10. Build our system
<pre>
catkin build
. setup.bash
</pre>

11. Enjoy!

[ROS Installation]: http://wiki.ros.org/ROS/Installation/
[catkin_tools]: https://catkin-tools.readthedocs.io/en/latest/installing.html
