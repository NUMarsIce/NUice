# ROS Install

This project uses ROS melodic on Ubuntu 18.04. I would suggest running [Ubuntu 18](https://releases.ubuntu.com/18.04.5/) on a Virtual Machine such as [VirtualBox](https://www.virtualbox.org/wiki/Downloads) if you do not want to dual boot. 

In terminal, run these commands in succession (following this tutorial)
```bash
#Install sources and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Install ROS
sudo apt update
sudo apt install ros-melodic-desktop-full

#Setup the environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Install build tools
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

Congrats! ROS Melodic should be installed. To test, run roscore:

```roscore```

This should run and not show any errors. To close roscore do a classic `Ctrl+C`.

# ROS Tutorials
If you are not already familliar with ROS, running through the tutorials will bring you up to speed. [ROS's tutorials](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) are great, but understand that you don't have to memorise everything they tell you. Also note that we will be using `catkin` to build and using the `ROS melodic` distro.

I suggest at least working up to the `Writing a Simple Service and Client` tutorial as most of the important stuff will be covered there. Otherwise, please feel free to go as far as you desire as the tutorials go on further to teach you about building programs and using more fancy services.

# Repo Setup
Hahaha, we have not gotten this far yet. Setting up all the ROS workspace stuff isn't a simple task.
