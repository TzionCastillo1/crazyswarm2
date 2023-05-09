#!/bin/bash

XAUTH=/tmp/.docker.xauth
if [ ! -f ${XAUTH} ]; then
    touch ${XAUTH}
    chmod a+r ${XAUTH}

    XAUTH_LIST=$(xauth nlist "${DISPLAY}")
    if [ -n "${XAUTH_LIST}" ]; then
        # shellcheck disable=SC2001
        XAUTH_LIST=$(sed -e 's/^..../ffff/' <<<"${XAUTH_LIST}")
        echo "${XAUTH_LIST}" | xauth -f ${XAUTH} nmerge -
    fi
fi


docker run -it \
    --env="DISPLAY" \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
	--volume ~/ECE495_dev_ws/src:/opt/ros_ws/src \
    --network host \
    midas-sim:latest 
