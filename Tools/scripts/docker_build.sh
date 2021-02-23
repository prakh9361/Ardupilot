#!/usr/bin/env bash

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "No board supplied, run as ./docker_build.sh <board name> or ./docker_build.sh list"
    exit 1
fi

BOARD=$1

cd "$(git rev-parse --show-toplevel)"
git submodule update --init --recursive

PATH='/home/ardupilot/.local/bin:/usr/lib/ccache:/ardupilot/Tools/autotest:/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'

docker build . -t ardupilot
docker run --rm -it -v "$(pwd)":/ardupilot ardupilot:latest ./waf configure --board="$BOARD"
docker run --rm -it -v "$(pwd)":/ardupilot ardupilot:latest ./waf build
