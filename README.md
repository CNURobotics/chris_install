# Install and setup files for the basic setup of CNU Robotics CHRISLab ROS Workspace software


### *If installing a new workspace, remove existing workspace setup from .bashrc, and reopen terminal sourcing only the /opt/ros/kinetic/setup.bash.*

1. Create workspace root folder (e.g. ~/CHRISLab)  and change to that directory
<pre>
mkdir CHRISLab
cd CHRISLab
</pre>

2. Clone the install setup  (*_Note: the dot at end is critical!_* )
<pre>
git clone https://github.com/CNURobotics/chris_install.git .
</pre>

3. Change to current branch
<pre>
git checkout kinetic_devel
</pre>

4. Run the install script
<pre>
./install.sh
</pre>

5. Follow on-screen instructions to add setup to bashrc and re-source the terminal

6. Test the setup
<pre>
roscd
</pre>
  * you should be in the workspace root /src folder  (e.g. ~/CHRISLab/src )

7. Go back to the original directory
<pre>
cd $WORKSPACE_ROOT
</pre>
  * This should put you back where you started (e.g. ~/CHRISLab)

8. Install the CHRISLab specific code
<pre>
./rosinstall/install_scripts/install_chrislab.sh
</pre>

9. Build the external libraries
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
