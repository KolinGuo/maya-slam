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
