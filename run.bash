#!/bin/bash

if [ $(id -u) -ne 1000 ]
then
  echo "ERROR: This script must be run with UID and GID of 1000."
  echo "       Current UID: $(id -u), current GID: $(id -g)"
  exit 1
fi

# Make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
fi
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ -n "$xauth_list" ]
then
  echo "$xauth_list" | xauth -f $XAUTH nmerge -
fi
chmod a+r $XAUTH

docker run --rm -it --gpus all \
    --net=host \
    -u 1000 \
    --privileged \
    -e "TERM=xterm-256color" \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name eeyore-docker \
    eeyore-docker \
    bash
