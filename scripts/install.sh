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
SETUP_CONFIG_LINE="source /opt/sensor_degradation_filter_real_kheperaiv/setup.sh"
SETUP_CONFIG_FILE="/home/root/.bashrc"
grep -Fxq "${SETUP_CONFIG_LINE}" "${SETUP_CONFIG_FILE}" || echo "${SETUP_CONFIG_LINE}" >>"${SETUP_CONFIG_FILE}" # only append to file if the line doesn't exist already

# First mkdir /etc/ld.so.conf.d/ so that we can create a library config file in there
mkdir -p /etc/ld.so.conf.d/

# Next, tell ld.so.conf to look into /etc/ld.so.conf.d
LD_CONFIG_LINE="include /etc/ld.so.conf.d/*.conf"
LD_CONFIG_FILE="/etc/ld.so.conf"
grep -Fxq "${LD_CONFIG_LINE}" "${LD_CONFIG_FILE}" || echo "${LD_CONFIG_LINE}" >>"${LD_CONFIG_FILE}" # only append to file if the line doesn't exist already

# Then add every lib path (needed by the robot that belongs to this repo) into the path
find /opt/sensor_degradation_filter_real_kheperaiv/lib -type d | tee /etc/ld.so.conf.d/sensor_degradation_filter_real_kheperaiv.conf
ldconfig

exec bash
