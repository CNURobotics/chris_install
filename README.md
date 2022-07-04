# CNU Robotics CHRISLab ROS Workspace Installation

Install and setup files for the basic setup of our software.

Computer Setup
--------------

 * Follow the [ROS Installation] instructions for setting up your ROS system on Ubuntu.
  * This system has been tested on ROS 2 Humble Release on `Ubuntu 22.04`.
  * We recommend `ros-humble-desktop` as the base for this system; it includes most of the required packages, including the Gazebo simulator for testing.
 * Install ROS stand alone tools
  * `sudo apt-get install python3-rosinstall`
  * `sudo apt-get install python3-rosdep`
  * `sudo apt-get install python3-wstool`
  * Our install script (below) uses rosinstall tools
 * Install your favorite editor or IDE. (e.g. atom, QtCreator, CLion, ...)




CNURobotics Software Setup
-----------------------

#### *If installing a new workspace, remove existing workspace setup from .bashrc, and
#### reopen terminal sourcing only the /opt/ros/humble/setup.bash prior to running this script.*

1. Create workspace root folder (e.g. ~/CHRISLab)  and change to that directory
<pre>
cd ~
mkdir CHRISLab
cd CHRISLab
</pre>

2. Clone the install setup
 * **Note: This is for the public CNURobotics GitHub Server for development**
 * `git clone https://github.com/CNURobotics/chris_install.git .`
    * *_Note: the dot at end of below command is critical!_*
      * We are choosing to clone within our workspace root folder, as opposed to creating the folder during the clone.  
        * The cloned folder should *NOT* be named chris_install.
      * Alternately, you could replaced dot ('.') with desired name of the workspace and clone from home directory (skipping step 1 above)

3. Change to correct branch
 * e.g., `git checkout humble-devel`
 * **Note: This is for the public CNURobotics GitHub Server for development**
 * This version has options for selectively installing the standard ROS 2 Turtlebot3 and CHRISLab Kobuki-based Turtlebot2 demonstrations for
 `flexible_navigation` and `flexible_behavior_trees` demonstrations.

4. Run the install script
 `./install.sh`

5. Follow on-screen instructions to add the new setup to bashrc and re-source the terminal
  * You can open a new terminal, or just `. ~/.bashrc`
6. Test the setup
  `rws`
  * you should be in the workspace root  (e.g. ~/CHRISLab )

7. Install the CHRISLab specific code by choosing one of the following
 * `./rosinstall/install_scripts/install_flex_base.sh` for just base software without demonstrations, or
 * `./rosinstall/install_scripts/install_flex_turtlebot3.sh` for base + Turtlebot 3 Simulation demonstration setup
 * `./rosinstall/install_scripts/install_chris_turtlebot2.sh` for base + CHRISLab specific Turtlebot 2 simulation and hardware setup

  For basic demonstration with minimal dependencies, choose the `install_flex_turtlebot3.sh` installation.

  The installations can be run separated with the required packages merged in the rosinstall files.

8. Build and install any external libraries installed in the `$WORKSPACE_ROOT/external` folder
  * For example:
<pre>
cd $WORKSPACE_ROOT/external/sbpl
mkdir build
cd build
cmake ..
make
sudo make install
</pre>

  Currently we do *NOT* require any external builds for these demonstrations.

9. Build our system
<pre>
ccb
. setup.bash
</pre>

> NOTE: `ccb` is using a safe alias for `colcon build` defined in the setup script created by the install script.
> It changes to the ROS workspace first to avoid accidentally building in a source folder.
> There are variants for colcon build --packages-select (ccbs) and --packages-up-to (ccbu)

> NOTE: Be sure to re-run the setup.bash script whenever a new package is added to the workspace.

10. Enjoy!

[ROS Installation]: https://docs.ros.org
