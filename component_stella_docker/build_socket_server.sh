#!/bin/bash
docker build -f docker/Dockerfile.socket -t geo_socket_server:latest . --build-arg NUM_THREADS=8 --no-cache
