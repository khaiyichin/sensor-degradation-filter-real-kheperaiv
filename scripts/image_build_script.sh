#!/usr/bin/env bash

# Stop execution after any error
set -e

# Build docker images
docker image build -t nestlab/kheperaiv --file docker/DockerfileBase .

docker image build -t khaiyichin/sensor-degradation-filter-real-kheperaiv --file docker/DockerfileController .