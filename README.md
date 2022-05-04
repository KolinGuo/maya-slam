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

```bash
datasets
├── TUM_VI
│   ├── dataset-corridor1_512_16.bag
│   └── ...
└── EuRoC
```
