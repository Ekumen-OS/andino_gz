#!/bin/bash

ABSDIR=$(dirname $(readlink -f $0))
SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

docker build -t docker_humble_fortress_andino ${SCRIPTS_DIR}
