#!/bin/bash

ABSDIR=$(dirname $(readlink -f $0))

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=`readlink -f ${SCRIPTS_DIR}/../`
DOCKER_MOUNT_ARGS="-v ${REPO_DIR}/:/home/ws/src"

REPOSITORY_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ..; pwd)"
REPOSITORY_FOLDER_NAME=$( basename $REPOSITORY_FOLDER_PATH )
WORKSPACE_SRC_CONTAINER=/home/$(whoami)/ws/src/$REPOSITORY_FOLDER_NAME

xhost +local:root
docker run -it -e DISPLAY -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	${DOCKER_MOUNT_ARGS} \
	-e "TERM=xterm-256color" \
    -v ${REPOSITORY_FOLDER_PATH}:$WORKSPACE_SRC_CONTAINER \
	--net=host \
	--gpus all \
    --name andino_humble_fortress \
	--privileged --rm docker_humble_fortress_andino /bin/bash
