#! /bin/sh

# This script records video from a given camera using ffmpeg and saves it to a
# directory in Videos.

# The camera is specified by the first argument

# sh strict
set -eu

SAVE_DIR="$HOME/Videos/Recordings"

# Check if the save directory exists, if not, create it
if [ ! -d "$SAVE_DIR" ]; then
    mkdir -p "$SAVE_DIR"
fi

# Check if the first argument is given if not return an error
if [ -z "$1" ]; then
    echo "No camera given"
    exit 1
fi

# Get the current date and time
date=$(date +"%Y-%m-%d_%H-%M-%S")

# Record the video
ffmpeg -f v4l2 -i "$1" -c:v libx264 -preset ultrafast -crf 0 -c:a copy "$SAVE_DIR/$date.mkv"

