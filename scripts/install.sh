#!/usr/bin/env bash

################## WARNING ##################
# THE SCRIPT IS MEANT FOR EXECUTION ON THE ROBOT
# NOT ON THE HOST MACHINE
#############################################

# This file is intended to be used after the `deployment_files.tar.gz` archived is extracted
# Incidently, you would obtain this file from the extracted archive
# Do scp to copy the archive to the Khepera, then untar it into the /opt directory: tar -xf deployment_files.tar.gz -C /opt/sensor_degradation_filter_real_kheperaiv/

set -e

# Relocate to the home directory
cd /home/root

# Set up sourcing of setup script
echo "source /opt/sensor_degradation_filter_real_kheperaiv/setup.sh" >> .bashrc

# First mkdir /etc/ld.so.conf.d/ so that we can create a library config file in there
mkdir -p /etc/ld.so.conf.d/

# Next, tell ld.so.conf to look into /etc/ld.so.conf.d
echo "include /etc/ld.so.conf.d/*.conf" > /etc/ld.so.conf

# Then add every lib path (needed by the robot that belongs to this repo) into the path
find /opt/sensor_degradation_filter_real_kheperaiv/lib -type d | tee /etc/ld.so.conf.d/sensor_degradation_filter_real_kheperaiv.conf
ldconfig

exec bash