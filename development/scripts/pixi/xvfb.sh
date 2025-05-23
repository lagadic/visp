#! /bin/bash

if ! xdpyinfo -display $DISPLAY >/dev/null 2>&1; then
  Xvfb $DISPLAY -screen 0 1280x1024x24 -nolisten tcp &
  echo "Xvfb started on display $DISPLAY"
else
  echo "Xvfb already running on display $DISPLAY"
fi