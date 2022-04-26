#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'


# TODO
  # once script is "done", go through and add comments to anything unclear

# some variables to make output pretty(er)
RED_FG=$(tput setaf 1)
GREEN_FG=$(tput setaf 2)
YELLOW_FG=$(tput setaf 3)
RESET_FG=$(tput sgr0)

# the arduino libraries used by the RoboSub (may not be all)
declare -a ARDUINO_LIBRARIES=(
  "Adafruit BNO055"
  "Adafruit BusIO"
  "Adafruit Circuit Playground"
  "Adafruit Unified Sensor"
  "BlueRobotics MS5837 Library"
  "Keyboard"                    
  "PID"
)

# the catkin workspace for RoboSub
WORKSPACE_DIRECTORY=~/RoboSub_WS

download_arduino_cli () {
  # check if arduino-cli program already exists
  if arduino-cli version &> /dev/null; then
    echo "arduino-cli is already installed"
    return 0
  fi

  sudo apt update

  # check for dependencies
  if ! curl -V &> /dev/null; then
    sudo apt -y install curl
  fi

  # eval is used to handle relative paths
  eval ARDUINO_BINDIR="~/local/bin"   # install directory
  ADD_TO_PATH=true                    # false -> arduino-cli only enabled in install directory

  # ensure install directory exists
  if [[ ! -d $ARDUINO_BINDIR ]]; then 
    mkdir -p "$ARDUINO_BINDIR"
  fi

  # add arduino-cli bin directory to PATH
  # PATH tells bash where to look for programs
  if [[ $ADD_TO_PATH = true ]] && [[ ! $PATH =~ $ARDUINO_BINDIR ]]; then
    if ! grep "PATH=\"\$PATH:$ARDUINO_BINDIR\"" ~/.bashrc > /dev/null; then
      echo "PATH=\"\$PATH:$ARDUINO_BINDIR\"" >> ~/.bashrc
    fi
  fi

  # check https://arduino.github.io/arduino-cli if this url doesn't work
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR=$ARDUINO_BINDIR sh

  # command line tab completion
  if cd "$ARDUINO_BINDIR"; then 
    ./arduino-cli completion bash > arduino-cli.sh
    yes | sudo mv arduino-cli.sh /etc/bash_completion.d
  else
    echo "arduino-cli tab completion was not configured"
  fi


  printf "\nRestart your terminal or source the bashrc file by running: source ~/.bashrc"
}

setup_microprocessor () {
  true
}

setup_arduino_libraries () {
  if ! arduino-cli version; then
    echo "arduino-cli is not installed"
    return 1
  fi

  # grab a list of installed libraries
  # awk and sed used for formatting
  INSTALLED_ARDUINO_LIBRARIES=$(arduino-cli lib list \
    | awk '{if (NR!=1) {print $1}}' \
    | sed 's/_/ /g'
  )

  # check if library name is a substring in installed library list
  # if it isn't, attempt to download and install the library
  # TODO: Better way of checking if library is installed
    # could be a library name is a substring of another library name but not the
    # same library (e.g. PIDv_2 =~ PID returns true)
  for LIBRARY in "${ARDUINO_LIBRARIES[@]}"; do
    if [[ ! $INSTALLED_ARDUINO_LIBRARIES =~ $LIBRARY ]]; then
      echo "${YELLOW_FG}Attempting to install $LIBRARY${RESET_FG}"
      if arduino-cli lib download "$LIBRARY" \
          && arduino-cli lib install "$LIBRARY"; then
        echo "${GREEN_FG}$LIBRARY was installed successfully.${RESET_FG}"
      else
        echo "${RED_FG}$LIBRARY was not installed.${RESET_FG}"
      fi
    else
      echo "${GREEN_FG}$LIBRARY is installed!${RESET_FG}"
    fi
  done
}

setup_ros_library () {
  # check if ROS is installed
  # TODO
    # redirect output to /dev/null
  if ! dpkg -l | grep ros-melodic &> /dev/null; then
    echo "Please download ROS to setup the roslib arduino library."
    return 1
  fi

  if [[ ! -d $WORKSPACE_DIRECTORY/src ]]; then
    mkdir -p "$WORKSPACE_DIRECTORY/src"
  fi

  # TODO
    # check if there is a git repo with this name already
  git clone https://github.com/ros-drivers/rosserial.git \
    --branch melodic-devel \
    --single-branch \
    "$WORKSPACE_DIRECTORY/src/rosserial"

  # TODO
    # check if catkin is installed
  cd "$WORKSPACE_DIRECTORY"
  catkin_make
  catkin_make install

  # TODO
    # check if arduino is installed ?
    # check the directory of arduino library environment variable
    # check if roslib is already a arduino library
  if [[ ! -d ~/Arduino/libraries ]]; then
    mkdir -p ~/Arduino/libraries
  fi

  # TODO
    # make less dynamic
  cd ~/Arduino/libraries
  rosrun rosserial_arduino make_libraries.py .
}


# function calls
download_arduino_cli
setup_microprocessor
setup_arduino_libraries
setup_ros_library
