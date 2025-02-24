#!/usr/bin/env bash

# Trap errors and log the line number
trap 'echo "${LINENO}: ${BASH_COMMAND}"; exit 1' ERR

# Check to see if we're in the project's root directory
required_items=("docker" "src" "scripts" "README.md")

# Check if all items exist
for item in "${required_items[@]}"; do
    if [ ! -e "$item" ]; then
        echo "Error: Required item '$item' not found! Please run this from the project's root directory." >&2
        exit 1
    fi
done

# Build docker images that generate the cross-compiled code
./scripts/image_build_script.sh

# Run the docker image once to copy desired contents into a compressed archive
docker container create --name cross_compiler khaiyichin/sensor-degradation-filter-real-kheperaiv
docker container cp cross_compiler:/work/deployment_files/. - > ./deployment_files.tar.gz
docker container rm -v cross_compiler

# Do scp to copy the archive to the Khepera, then untar it into the /opt directory: tar -xf deployment_files.tar.gz -C /opt/sensor_degradation_filter_real_kheperaiv/
echo -e "\n#################### Cross-compilation complete ####################\n"
echo -e "1. Copy 'deployment_files.tar.gz' to the robot: 'scp deployment_files.tar.gz root@KheperaXX:/home/root'"
echo -e "2. SSH onto the robot and extract the archive to the '/opt/sensor_degradation_filter_real_kheperaiv/' directory: 'tar -xf deployment_files.tar.gz -C /opt/sensor_degradation_filter_real_kheperaiv/'"
echo -e "3. Run the installation script (while still logged into the robot): '/opt/sensor_degradation_filter_real_kheperaiv/install.sh'"