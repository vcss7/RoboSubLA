#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

sudo apt update

download_arduino_cli () {
  # check for dependencies
  if ! curl -V &> /dev/null; then
    sudo apt -y install curl
  fi

  # check if arduino-cli program already exists
  if arduino-cli version &> /dev/null; then
    echo "arduino-cli is already installed"
    exit 1
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

setup_sensor_libraries () {
  true
}

setup_ros_library () {
  true
}


# function calls
download_arduino_cli
setup_microprocessor
setup_sensor_libraries
setup_ros_libraries
