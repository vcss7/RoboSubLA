#!/usr/bin/env bash

set -e
set -u
set -o pipefail
IFS=$'\n\t'


# check for root privileges
if [[ $(id -u) -ne 0  ]]; then
    tput setaf 1
    printf "This script needs root privileges. Run it with sudo.\n"
    tput sgr0
    exit 1
fi


check_dependencies () {
    declare -a DEPENDENCIES=(
        "curl"
        "udevadm"
    )

    for DEP in "${DEPENDENCIES[@]}"; do
        if ! $DEP --version &> /dev/null; then
            tput setaf 1
            printf "%s is not installed.\n" "$DEP"
            tput sgr0
        fi
    done
}


download_and_write_udev_rules () {
    printf "Downloading platformio-udev rules...     "

    local tmpfile=$(mktemp)
    local URL="https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules"
    curl -fsSL $URL -o "$tmpfile"

    printf "done!\n"

    printf "Writing platformio-udev rules...         "

    cat "$tmpfile" > /etc/udev/rules.d/99-platformio-udev.rules
    rm "$tmpfile"

    printf "done!\n"
}


restart_udev_mgnt_tool () {
    printf "Restarting udev management tool...       "

    udevadm control --reload-rules
    udevadm trigger

    printf "done!\n"
}


add_user_to_groups () {
    printf "Adding user to dialout group...          "

    local KERNAL_RELEASE=$(uname -a)

    if [[ "$KERNAL_RELEASE" =~ "arch" ]]; then
        usermod -a -G uucp "$SUDO_USER"
        usermod -a -G lock "$SUDO_USER"
    elif [[ "$KERNAL_RELEASE" =~ "Ubuntu" ]]; then
        usermod -a -G dialout "$SUDO_USER"
        usermod -a -G plugdev "$SUDO_USER"
    fi

    printf "done!\n"

    tput setaf 3
    printf "Log out and log back in for the user changes to take affect.\n"
    tput sgr0
}

check_dependencies
download_and_write_udev_rules
restart_udev_mgnt_tool
add_user_to_groups

