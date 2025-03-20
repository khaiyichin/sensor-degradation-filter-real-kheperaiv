# Sensor Degradation Filter Library for Real Khepera IV Robots
This repository contains code for the libraries and controllers to be cross-compiled for the Khepera IV. The controller is primarily for physical experiments with the Vicon Datastream server, which is the setup that [NEST Lab](https://nestlab.net/) has currently.

Things that are simulated:
- degradation of ground sensor readings (the actual readings are still being read from the physical sensor).

Things that aren't simulated:
- communication, via WiFi,
- obstacle avoidance,
- LED (also turned off to improve robot battery life).

While the controller runs onboard the physical Khepera IV robots, it still requires information from the ARGoS server (which runs the loop functions). This is because the Khepera IV robots rely on WiFi communication, which is centralized in principle. The `RealKheperaIVExperimentLoopFunctions` class&mdash;provided by the [sensor-degradation-filter](https://github.com/khaiyichin/sensor-degradation-filter/blob/3537ee1e2e9f9f47b2c22377e9aa2945d887095c/include/sensor_degradation_filter/loop_functions/RealKheperaIVExperimentLoopFunctions.hpp) repository&mdash;provides localization information, which the robots use to emulate decentralize communication.

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
For the docker cross-compilation to work, the following files are required in the `docker` directory:
- `libkhepera-2.1.tar.bz2`
- `poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh`

Both can originally be found on the [K-Team](https://www.k-team.com) site, but as of 02/24/2025 the company appears to no longer exist. Alternatively, copies are backed up on the NEST Lab OneDrive and can be downloaded from there.

Here are some of the differences between the [`BayesCPFDiffusionController`](./src/controllers/BayesCPFDiffusionController.hpp) class and the [`KheperaIVDiffusionMotion`](https://github.com/khaiyichin/sensor-degradation-filter/blob/3537ee1e2e9f9f47b2c22377e9aa2945d887095c/include/sensor_degradation_filter/controllers/KheperaIVDiffusionMotion.hpp) class (from the [`sensor-degradation-filter repository](https://github.com/khaiyichin/sensor-degradation-filter.git)):
- Replaced `std::reduce` (only supported in C++17) with `std::accumulate`.
- Replaced `CCI_RangeAndBearingSensor::TReadings` with `std::vector<CCI_KheperaIVWiFiSensor::SMessage>`.
- Replaced `double` with `float` for real numbers.
- Removed usage of LEDs (to improve battery life)
- Created a new `CRandom::CRNG` category (because the simulator creates it if run in the ARGoS server).
- Added bounds to prevent robots from falling off.
- Added connection to the ARGoS server to get pose information and send back robot data.
- Added assumed sensor accuracy to controller (since we're simulating degradation)
- Modified data logging to store current time step instead of random number generator seed.