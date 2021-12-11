#!/usr/bin/env bash

# Bash Strict Mode
set -euo pipefail
IFS=$'\n\t'

################################################

# set up workspace
setup_workspace () {
  # TODO: check if dir exist
  #       if it does, do something
  #       else setup_ws
  # if can authenticate ssh connection
  #   then set up the repo
  # else
  #   try through https
  # if both fail the nprompt user to log in / set up SSH
}


# set up visualizer

################################################

# Start

install_ros_melodic () {
  # setup sources.list
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  # setup keys
  sudo apt -y install curl
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  # install
  sudo apt update
  sudo apt -y install ros-melodic-desktop-full

  # env setup
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc

  # install depends for build pkgs
  sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update
}


# TODO: before executing check if it is already installed 
install_ros_melodic



