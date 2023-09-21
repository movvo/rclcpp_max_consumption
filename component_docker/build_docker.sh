#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Build of component_docker. Building as latest"
    docker build -t component_docker:latest --build-arg NUM_THREADS=8 .
else
    echo "Build of component_docker. Building as $1"
    docker build -t component_docker:$1 --build-arg TAG=$1 --build-arg NUM_THREADS=8 .
fi