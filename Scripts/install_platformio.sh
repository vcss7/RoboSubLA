#!/usr/bin/env bash

set -e
set -u
set -o pipefail
IFS=$'\n\t'

if ! curl --version &> /dev/null; then
    tput setaf 1
    printf "curl is not installed"
    tput sgr0
    exit 1
fi

# install platformio
install_platformio () {
    local URL=https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    curl -fsSL $URL -o get-platformio.py
    python3 get-platformio.py

    #TODO: clean up get-platformio.py file
}

add_platformio_to_shell_commands () {
    tput setaf 2
    printf "Adding shell commands...             "
    tput sgr0

    if [[ "$PATH" =~ "$HOME/.local/bin" ]]; then
        printf "$HOME/.local/bin in path"
    else
        printf "export PATH=\$PATH:\$HOME/.local/bin" >> $HOME/.bashrc
    fi

    # add symbolic links
    if [ ! -d $HOME/.local/bin ]; then
        mkdir -p ~/.local/bin/
    fi

    ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
    ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
    ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb

    tput setaf 2
    printf "done!\n"
    tput sgr0

    tput setaf 3
    printf "Log out and log back in for shell commands to become available.\n"
    tput sgr0
}

install_platformio
add_platformio_to_shell_commands

