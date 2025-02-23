#!/usr/bin/env bash

# Stop execution after any error
set -e

# Trap errors and log the line number
trap 'echo "${LINENO}: ${BASH_COMMAND}"; exit 1' ERR

# Change to root directory
pushd ..

# Check to see if we're in the project's root directory
required_items=("docker" "src" "README.md")

# Check if all items exist
for item in "${required_items[@]}"; do
    if [ ! -e "$item" ]; then
        echo "Error: Required item '$item' not found! Please run this from the project's docker/ directory." >&2
        exit 1
    fi
done

docker image build -t nestlab/kheperaiv --file docker/DockerfileBase .
