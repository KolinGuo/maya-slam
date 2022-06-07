# maya-slam
Maya Archaeology VR - Project for CSE 237D Course

## D435i IMU Calibration

The steps are mainly summaries taken from
[Intel D435i IMU Calibration Guide](https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf)
and their [official RealSense SDK](https://github.com/IntelRealSense/librealsense).  
If there is anything unclear or any software change occur, refer to them for
any updates.

<details>
<summary><b>Detailed steps to perform IMU calibration of D435i on Ubuntu.</b> </summary>
<p>

Prerequisites: Ubuntu >= 18.04, Python 3 (pip, numpy)

* Step 1: Install Intel RealSense SDK `pyrealsense2` wrapper:  
  ```bash
  sudo pip3 install pyrealsense2
  ```
* Step 2: Clone [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense):  
  ```bash
  git clone https://github.com/IntelRealSense/librealsense.git
  ```
* Step 3: Run `librealsense/tools/rs-imu-calibration/rs-imu-calibration.py` with sudo
  to perform IMU calibration.
  Check [this README.md](https://github.com/IntelRealSense/librealsense/tree/master/tools/rs-imu-calibration)
  for more information.  
  ```bash
  cd librealsense/tools/rs-imu-calibration
  sudo python3 rs-imu-calibration.py
  ```
* Step 4: At the end of the calibration script, select to write the results to D435i's eeprom.

Note that if you are connected to a laptop, your screen might rotate according
to the connected D435i orientation. This is because the OS sees the IMU inside
D435i as any IMU inside a tablet and thus will rotate screen orientation following
it. To turn screen rotation off, see
[this post](https://askubuntu.com/questions/1035209/how-to-turn-off-screen-rotation-in-ubuntu-18-04-lts).

</p>
</details>

# Workflow
The steps to run the entire system are :
1. Open camera using `./realsense_capture/start_camera.sh` from `realsense_capture` container.
2. Start tracking using `./slam_algorithms/maplab/run_maplab.sh <bagname>` from `maplab` container
3. Stop tracking by `ctrl+c` and stop containers.
4. Reconstruct 3D mesh using `scene_recon/open3d_tsdf/build_mesh.sh <bagname> <voxelsize>` inside `maya_recon` container.
5. Split mesh into chunks using `split_mesh_into_json.py <plyname> <chunksize>` within same container.

Done.

<!--
## Dataset folder

Please place all datasets in [datasets/](datasets)

For [TUM_VI](https://vision.in.tum.de/data/datasets/visual-inertial-dataset),
ORBSLAM3 expects the data in raw format (i.e. _Euroc / DSO 512x512 dataset_)
instead of ros bags.

```bash
datasets
├── TUM_VI
│   ├── dataset-corridor1_512_16
│   │   ├── dso
│   │   │   ├── cam0
│   │   │   ├── cam1
│   │   │   ├── camchain.yaml
│   │   │   ├── gt_imu.csv
│   │   │   ├── imu_config.yaml
│   │   │   └── imu.txt
│   │   └── mav0
│   │       ├── cam0
│   │       │   ├── data
│   │       │   └── data.csv
│   │       ├── cam1
│   │       │   ├── data
│   │       │   └── data.csv
│   │       ├── imu0
│   │       │   └── data.csv
│   │       └── mocap0
│   │           └── data.csv
│   └── ...
└── EuRoC
```

## Camera Launch

<details>
<summary><b>openchisel</b> </summary>
<p>

Run [docker_setup_realsense.sh -l](docker_setup_openchisel.sh) build the docker image.

### Steps

#### To compile/build
* `cd realsense_capture/`
* `./build_ros.sh`

#### To run
`./start_camera.sh`
`./start_recording.sh`

</p>
</details>


## SLAM Algorithms

<details>
<summary><b>ORBSLAM3</b> </summary>
<p>

Use [docker_setup.sh](docker_setup.sh) to pull orbslam3 docker images from DockerHub
or build it locally with `./docker_setup.sh -l`.

Use [tum_vi_examples.sh](slam_algorithms/ORB_SLAM3/tum_vi_examples.sh)
to run with TUM_VI dataset.
(Note: the seg fault at the end is not an issue since it only happens during destruction.)

</p>

<summary><b>MAPLAB</b> </summary>
<p>

Use [docker_setup_maplab.sh -l](docker_setup_maplab.sh) to build the maplab docker image

Build using `./build_ros'
and run using `./run_maplab.sh`

This opens up a new rviz with tracking

</p>


</details>

## Reconstruction Algorithms

<details>
<summary><b>openchisel</b> </summary>
<p>

Run [docker_setup_openchisel.sh -l](docker_setup_openchisel.sh) build the docker image.

### Steps

#### To compile/build
* `cd scene_recon/openchisel`
* `./build_ros.sh`

#### To run
`./run_openchisel.sh`

</p>
</details>
-->
# Description of different containers for the system
## realsense_capture
Contains the scripts for launching and configuring the Realsense D435i RGBD camera. The docker container runs on `ros-melodic` and `librealsense`, using the `ros-realsense` package wrapper. The launch file is configured to launch the color (RGB) feed, depth feed (aligned with RGB) and the IMU sensors. 

### Building
1. `./docker_setup.sh realsense_capture -l` to build the container.
2. `cd realsense_capture && ./build_ros.sh` to build the packages.

### Running
Run `./realsense_capture/start_camera.sh` to start the camera.

Go to `realsense_capture/catkin_ws/src/realsense_d435i_capture/launch/realsense_d435i_rviz.launch` to configure more options.

## maplab
Container running the SLAM algorithm. In this work, [maplab](https://github.com/ethz-asl/maplab) is used, which is a Stereo-IMU based SLAM. 

### Building
1. `./docker_setup.sh maplab -l` to build the container.
2. `cd slam_algorithms/maplab && ./build_ros.sh` to build the packages.

### Running
Run `./slam_algorithms/maplab/run_maplab.sh <bagname>` to start the camera tracking. It begins tracking the camera. Displays the trajectory in RVIZ. Also saves the output of depth, color images, IMU and trajectort pose to a rosbag. Pass rosbag name for shell script

To configure options, go to modify the contents, go to `slam_algorithms/maplab/catkin_ws/src/maplab/applications/rovioli/launch`. Edit `realsense.launch` to change the launch file. Edit the `realsense .yaml` files inside `share` to change calibration parameters.

## orbslam
An alternative SLAM algorithm, for feature-full areas. [orbslam3](https://github.com/UZ-SLAMLab/ORB_SLAM3) is the state of the art Visual Inertial SLAM. 

### Building
1. `./docker_setup.sh orbslam3 -l` to build the container.
2. `cd slam_algorithms/maplab && ./build.sh && /build_ros.sh` to build the library.

### Running
Run `roslaunch RGBD-Inertial rgbd_d435i.launch` to launch tracking along with camera. The trajectory is written onto `CameraTrajectory.txt`.

**ORBSLAM3 has not been tested extensively for the project and we recommend using maplab for SLAM**

## maya_recon
Container that performs 3D reconstruction after performing pose estimation. Uses Open3D TSDF reconstruction for a 3D mesh reconstruction that takes in RGBD images along with the camera trajectory.

Also performs mesh splitting.

### Building
1. `./docker_setup.sh maya_recon -l` to build the container.

### Running
1. Call `scene_recon/open3d_tsdf/build_mesh.sh <bagname> <voxelsize>` to perform 3D reconstruction. The output is written in the `output_plys` directory.
2. Call `split_mesh_into_json.py <plyname> <chunksize>` to perform 3D reconstruction.

**We recommend using this on powerful system with >64GB RAM**

