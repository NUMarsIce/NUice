# ROS Install

This project uses ROS melodic on Ubuntu 18.04. I would suggest running [Ubuntu 18](https://releases.ubuntu.com/18.04.5/) on a Virtual Machine such as [VirtualBox](https://www.virtualbox.org/wiki/Downloads) if you do not want to dual boot. If you are unfamilliar with Virtual Machines, [this is a good tutorial](https://linuxhint.com/install_ubuntu_18-04_virtualbox/) to get one up and running.

In terminal, run these commands in succession (we are following [this](http://wiki.ros.org/melodic/Installation) tutorial)
```bash
#Install sources and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Install ROS
sudo apt update
sudo apt install -y ros-melodic-desktop-full

#Setup the environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Install build tools
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

Congrats! ROS Melodic should be installed. To test, run roscore:
```bash
roscore
```
This should run and not show any errors. To close roscore do a classic `Ctrl+C`.

# ROS Tutorials
If you are not already familliar with ROS, running through the tutorials will bring you up to speed. [ROS's tutorials](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) are great, but understand that you don't have to memorise everything they tell you. Also note that we will be using `catkin` to build and using the `ROS melodic` distro.

I suggest at least working up to the `Writing a Simple Service and Client` tutorial as most of the important stuff will be covered there. Otherwise, please feel free to go as far as you desire as the tutorials go on further to teach you about building programs and using more fancy services.

# Repo Setup
If you feel comfortable with ROS, go ahead and get this repo set up on your local machine.

```bash
#Clone the Repo
cd ~/
git clone https://github.com/SpyGuyIan/NUice.git

#Build the workspace
cd NUice/catkin_ws/
catkin_make

#Automatically source our workspace so every new terminal knows about it
echo "source ~/NUice/catkin_ws/devel/setup.bash" >> ~/.bashrc

```

A good place to start exploring is `NUice/catkin_ws/src/nuice_simulations/src/estop_sim/estop_sim_node.py`
