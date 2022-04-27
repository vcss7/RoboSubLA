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

  # save a list of installed libraries to a tmp file
  # awk and sed are used for formatting
  arduino-cli lib list \
  | awk '{if (NR!=1) {print $1}}' \
  | sed 's/_/ /g' \
  > $$tmp

  # iterate through array of arduino libraries
  for LIBRARY in "${ARDUINO_LIBRARIES[@]}"; do
    ALREADY_INSTALLED="false"

    # take tmp file as input for the while loop and read a line
    while read -r LINE; do
      # check if library is exactly line (string comparison)
      if [[ "$LIBRARY" == "$LINE" ]]; then
        echo "${GREEN_FG}$LIBRARY is installed!${RESET_FG}"
        ALREADY_INSTALLED="true"
        # break from while loop if library name found
        break
      else
        ALREADY_INSTALLED="false"
      fi
    done < $$tmp

    # try to install library if not found in list of installed libraries
    if [[ $ALREADY_INSTALLED == "false" ]]; then
      echo "${YELLOW_FG}Attempting to install $LIBRARY${RESET_FG}"
      if arduino-cli lib download "$LIBRARY" \
          && arduino-cli lib install "$LIBRARY"; then
        echo "${GREEN_FG}$LIBRARY was installed successfully.${RESET_FG}"
      else
        echo "${RED_FG}$LIBRARY was not installed.${RESET_FG}"
      fi
    fi
  done

  # clean up tmp file
  rm -f $$tmp
}

setup_ros_library () {
  # check if ROS is installed
  if ! dpkg -l | grep ros-melodic- &> /dev/null; then
    printf "%sROS Melodic is not installed and is required for " "${RED_FG}"
    printf "ros_lib.%s\n" "${RESET_FG}"
    exit
  fi

  # check if there is a workspace src directory
  if [[ ! -d $WORKSPACE_DIRECTORY/src ]]; then
    mkdir -pv "$WORKSPACE_DIRECTORY/src"
  fi

  # check if there is a rosserial git repo in workspace src directory
  # if there is, prompt user for overwrite
  if [[ -d $WORKSPACE_DIRECTORY/src/rosserial ]]; then
    printf "There is a directory at %s/src/rosserial " $WORKSPACE_DIRECTORY 
    printf "and it is not empty.\n"
    read -rp "Do you want to overwrite it with git clone? [Y/n] " yn
    case $yn in
      [Nn][Oo] | [Nn] )
        ;;
      * )
        rm -rf $WORKSPACE_DIRECTORY/src/rosserial
        git clone https://github.com/ros-drivers/rosserial.git \
          --branch melodic-devel \
          --single-branch \
          "$WORKSPACE_DIRECTORY/src/rosserial"
        ;;
    esac
  else
    git clone https://github.com/ros-drivers/rosserial.git \
      --branch melodic-devel \
      --single-branch \
      "$WORKSPACE_DIRECTORY/src/rosserial"
  fi

  # TODO: before building rosserial package, add teensy 4.1 Microcontroller Unit
  #       name to the ArduinoHardware.h file in ros_lib package. (with patch 
  #       file)

  # check if catkin is installed
  # if it isn't, return control to outside function
  if ! dpkg -l | grep ros-melodic-catkin &> /dev/null; then
    printf "%sCatkin is not installed and is required to build " "${RED_FG}"
    printf "ros_lib.%s\n" "${RESET_FG}"
    return 1
  fi

  # TODO: check cd was successful
  cd "$WORKSPACE_DIRECTORY"

  # build packages in workspace (effectively rosserial)
  # if build fail, return control to outside function
  if catkin_make && catkin_make install; then
    printf "%srosserial successfully built.%s\n" "${GREEN_FG}" "${RESET_FG}"
  else
    printf "%sCould not build ros_lib.%s\n" "${RED_FG}" "${RESET_FG}"
    return 1
  fi

  # TODO: check if arduino is installed ?

  # make sure there is a Arduino libraries directory
  if [[ ! -d ~/Arduino/libraries ]]; then
    mkdir -p ~/Arduino/libraries
  fi

  # check for rosserial_arduino ros package
  # if it isn't there, return control to outside function
  if ! rospack list-names | grep rosserial_arduino &> /dev/null; then
    printf "%sThere was a problem finding the rosserial_arduino package.\n" \
      "${RED_FG}"
    printf "Try sourcing bash again and make sure ROS, "
    printf "catkin and arduino are insalled.%s\n" "${RESET_FG}"
    return 1
  fi

  # check if ros_lib arduino library directory already exists
  # if it does, prompt user for reinstall
  if [[ -d ~/Arduino/libraries/ros_lib ]]; then
    read -rp "ros_lib is already installed. Do you want to reinstall? [Y/n] " yn
    case $yn in
      [Nn][Oo] | [Nn] ) 
        ;;
      * ) 
        rm -rf ~/Arduino/libraries/ros_lib
        ;;
    esac
  fi

  # TODO: maybe redirect output to /dev/null
  if rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries; then
    printf "%sros_lib successfully installed!%s\n" "${GREEN_FG}" "${RESET_FG}"
  fi
}

#
## function calls
#download_arduino_cli
#setup_microprocessor
setup_arduino_libraries
#setup_ros_library
