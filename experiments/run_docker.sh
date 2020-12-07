#!/bin/bash
# To be run from the host computer (outside docker)

BASE_PATH=$(realpath $(dirname "$0"))

docker run -it --rm -v $BASE_PATH:/root/ws/src/plan_library/experiments:rw gerardcanal/ionut_plan_lib "$@"

