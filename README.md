# Sensor Degradation Filter Library for Real Khepera IV Robots
This repository contains code for the libraries and controllers to be cross-compiled for the Khepera IV. The controller is primarily for physical experiments with the Vicon Datastream server, which is the setup that [NEST Lab](https://nestlab.net/) has currently.

Things that are simulated:
- degradation of ground sensor readings (the actual readings are still being read from the physical sensor)

Things that aren't simulated:
- communication, via WiFi
- obstacle avoidance
- led (may consider turning this off to improve robot battery life)



## Instructions
1. Run `cross_compile_binaries.sh`:
    ```
    $ ./scripts/cross_compile_binaries.sh
    ```
    This runs `image_build_script.sh` under the hood to create the cross-compiler images. You should get a compressed archive with the name `deployment_files.tar.gz`, which contains the binaries and libraries compiled for the Khepera IV.

2. Copy the archive to the robot (`Khepera01` in this example):
    ```
    $ scp deployment_files.tar.gz root@Khepera01:/home/root/
    ```

3. SSH into the robot, then extract the archive into the `/opt/sensor_degradation_filter_real_kheperaiv` directory:
    ```
    $ mkdir -p /opt/sensor_degradation_filter_real_kheperaiv
    $ tar -xf deployment_files.tar.gz -C /opt/sensor_degradation_filter_real_kheperaiv/
    ```
    Note that for the Khepera IV robots at the NEST Lab `root` is typically used to SSH into the robots---the scripts may not work as intended if that isn't the case.

4. On the robot, run the installation script:
    ```
    $ ./opt/sensor_degradation_filter_real_kheperaiv/install.sh
    ```

5. Copy the controller configuration file to any desired directory (where you will be executing the controller from):
    ```
    $ mkdir BayesCPF/
    $ cp /opt/sensor_degradation_filter_real_kheperaiv/controller_config.argos /home/root/BayesCPF
    ```
    Here our example work directory is `BayesCPF`.

6. Execute the controller (will require that an active ARGoS server):
    ```
    $ cd BayesCPF/
    $ bayes_cpf_diffusion_controller -c controller_config.argos -i bcdc
    ```
    `bcdc` is the ID given to the controller in the `controller_config.argos` file and is arbitrary.

## Dev notes
For the docker cross-compilation to work, the following is required in the `docker` directory:
- libkhepera-2.1.tar.bz2
- poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh

Both can originally be found on the [K-Team](https://www.k-team.com) site, but as of 02/24/2025 the website is down. Alternatively, copies are backed up on the NEST Lab OneDrive and can be downloaded from there.


- `self_pose_` is automatically updated with new updates thanks to the listener thread

what changed:
- std::accumulate for std::reduce (only supported in c++17)
- CCI_RangeAndBearingSensor::TReadings replaced with std::vector<CCI_KheperaIVWiFiSensor::SMessage> in the controller
- convert double to float
- removed leds (save battery)
- create a new CRandom::CRNG category (because the simulator doesn't create it for us this time)
- add bounds to prevent robots from falling off
- add connection to argos server to get pose information and send recorded data
- add assumed sensor accuracy to controller (since we're simulating degradation anyway)
- storing current time step instead of rng seed