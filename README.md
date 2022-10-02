<upper>*Inspired by [NVIDIA dw_ros](https://github.com/NVIDIA/dw-ros)
& [leo-drive drivers](https://gitlab.com/leo-drive/Drivers/sekonix_camera).* </upper>

# Nvidia Drive GMSL Camera ROS Driver

- *In this documentation and in the source 'interface' designates the HFM connector, and 'link' the number of the FAKRA
  Z.
  See [here](https://docs.nvidia.com/drive/drive_os_5.1.6.1L/nvvib_docs/index.html#page/DRIVE_OS_Linux_SDK_Development_Guide/Camera/camera_xavier.html)*
  .
- In this documentation **host** relates to the Ubuntu 18.04 computer used to build the driver. **Target** refers to the
  Nvidia Drive system.

---

- Compatible with Driveworks 3.5.
- Compatible with ROS Melodic.
- Tested with Sekonix GMSL SF3324 and SF3325 cameras.

## How to build

### Prerequisites on the Host

- Ubuntu 18.04.
- Nvidia GPU drivers:
  ```bash
  sudo apt install nvidia-drivers-470
  ```
- NVIDIA DRIVE™ OS 5.2.0 and DriveWorks 3.5 (Linux):
    - Follow the download [page](https://developer.nvidia.com/drive/downloads) to install NVIDIA DRIVE™ OS 5.2.0 and
      DriveWorks 3.5 (Linux). Follow the 'DriveWorks 3.5 Installation Guide'.

### How to crosscompile on the Host

It is not possible to build directly on the target, we must crosscompile the driver on the host.

#### Prepare system

Export env variables pointing to the PDK and to the built image SYSROOT (These paths might change on your system):

```bash
PDK=$HOME/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DRIVE_AGX_XAVIER/DRIVEOS/drive-t186ref-linux
SYSROOT=$PDK/targetfs
```

#### Prepare the cross-compilation sysroot

```bash
cd $SYSROOT

sudo apt install qemu-user-static
sudo cp /usr/bin/qemu-aarch64-static usr/bin
sudo cp -b /etc/resolv.conf etc
sudo mount -o bind /dev dev
sudo mount -o bind /proc proc
sudo mount -o bind /sys sys

sudo LC_ALL=C chroot .
# apt update
# apt install libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev python-defusedxml python-rospkg python-catkin-pkg python-netifaces liblog4cxx-dev libopenblas-dev libgflags-dev libglew-dev libopencv-dev
# exit

sudo umount sys proc dev 
sudo rm usr/bin/qemu-aarch64-static
sudo mv etc/resolv.conf~ etc/resolv.conf
sudo rm -rf var/lib/apt/lists/*
sudo rm -rf dev/*
sudo rm -rf var/log/*
sudo rm -rf var/tmp/*
sudo rm -rf var/cache/apt/archives/*.deb
sudo rm -rf tmp/*
```

#### Fix broken symlinks

The broken symlinks can be fixed temporarily with overlays, using commands similar to the following:

```bash
sudo mkdir /lib/aarch64-linux-gnu
sudo mkdir /tmp/ros-cc-overlayfs
sudo mount -t overlay -o lowerdir=$SYSROOT/lib/aarch64-linux-gnu,upperdir=/lib/aarch64-linux-gnu,workdir=/tmp/ros-cc-overlayfs overlay /lib/aarch64-linux-gnu
```

#### ROS Prerequisites

- Follow this [page](http://wiki.ros.org/melodic/Installation/Ubuntu) to set up your sources.list and set up your keys.
- Follow this [page](http://wiki.ros.org/melodic/Installation/Source) to install bootstrap dependencies and initialize
  rosdep.
- Run below commands to download source of ROS Melodic Morenia (Ubuntu 18.04 is the target root file system of DRIVE OS
  Linux 5.2.0).
  ```
  mkdir -p ~/ros_catkin_ws/src && cd ~/ros_catkin_ws
  rosinstall_generator ros_comm sensor_msgs camera_info_manager cv_bridge image_transport nodelet roscpp std_msgs --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall
  vcs import src < melodic-ros_comm.rosinstall
  ```

#### Install ROS dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

#### Extract ROS dependencies

```bash
rosdep install -si --reinstall --from-path src
```

Install the displayed dependencies on the emulated sysroot with `apt install`.

#### Build dependencies

```bash
sudo apt install python-numpy libyaml-cpp-dev python-empy
```

#### Crossbuild ROS

```bash
src/catkin/bin/catkin_make_isolated -DCMAKE_BUILD_TYPE=Release \
  -DVIBRANTE_PDK:STRING=$PDK 
  -DTRT_VERSION:STRING=6.3.1.3 \
  -DCMAKE_TOOLCHAIN_FILE=$HOME/ros_catkin_ws/src/nvidia_gmsl_driver_ros/Toolchain-V5L.cmake -DCMAKE_EXE_LINKER_FLAGS="${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/driveworks/targets/aarch64-Linux/lib -Wl,-rpath,/usr/local/driveworks/targets/aarch64-Linux/lib -L$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib -Wl,-rpath,$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib -L$SYSROOT/usr/lib/aarch64-linux-gnu/openblas -Wl,-rpath,$SYSROOT/usr/lib/aarch64-linux-gnu/openblas" \
  --install
```

Replace with the current installation path with the binary installation path so we can run any binary installed packages
on the target.

```bash
sed -i "s#$HOME/ros_catkin_ws/install_isolated#/opt/ros/melodic#g" install_isolated/_setup_util.py
```

## How to run on target

#### Install ROS dependencies and ROS

Follow http://wiki.ros.org/melodic/Installation/Ubuntu to install ROS necessary apt packages via below command

```
sudo apt install ros-melodic-ros-base ros-melodic-image-view
```

Install the same dependancies as we have installed on the emulated sysroot :

```
apt install libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev python-defusedxml python-rospkg python-catkin-pkg python-netifaces liblog4cxx-dev libopenblas-dev libgflags-dev libglew-dev libopencv-dev
```

Transfer the compiled workspace from the host to the target.

Install the dependancies with rosdep :

```bash
rosdep install --from-paths src --ignore-src -r -y
```

#### Remove old library versions

*(The detailed reason is detailed in
this [post](https://forums.developer.nvidia.com/t/libgdal-so-has-undefined-symbol/110239/5)).*

```bash
rm /usr/lib/libxerces-c*
```

#### Set up ROS environment

```
source ~/install_isolated/setup.bash
```

## How to use

### Setup

In case you need to make changes to `ports.yaml` :

- Copy the config file `ports.yaml` to your workspace.
- Edit the config file `ports.yaml` to your need.
- Pass its path to the launchfile into the `config_path` param .

### Calibrating the cameras

This driver **doesn't** use the native nvidia RIG calibration file. It uses the ROS ones.  
To calibrate the cameras :

- Run the driver with an empty calibration dir.
- Use ROS [camera_calibration](http://wiki.ros.org/camera_calibration) to calibrate the cameras.
- Put the camera calibration yaml files in your calib dir. Pass its path to the launchfile into the `calib_dir_path`
  param .

**Note :** In the calibration dir the calibration files **MUST** have this syntax : `interface<a-d>_link<0-3>.yaml`. For
example : `interfacea_link1.yaml`.

### Run the launchfile

```bash
roslaunch nvidia_gmsl_driver_ros nvidia_gmsl_driver_ros.launch
```

##### Launchfile parameters

| Parameter        |                           Default |                                                       Comment |
|------------------|----------------------------------:|--------------------------------------------------------------:|
| `config_path`    | `$(dirname)/../config/ports.yaml` |                                      Path to the config file. |
| `calib_dir_path` |            `$(dirname)/../calib/` |                          Path to the camera calibration file. |
| `framerate`      |                              `30` |                                             Output framerate. |
| `verbose`        |                           `False` |                                       Enables verbose output. | 
| `encoder`        |                             `jpg` |                                    Encoder. (`jpg` or `h264`) | 
| `bitrate`        |                         `8000000` | Output bitrate (Minimum `30000`).<br>Only for `encoder=h264`. | 
| `output_width`   |                            `1920` |                                                 Output width. | 
| `output_height`  |                            `1208` |                                                Output height. | 

### Useful links

- [DriveWorks SDK Reference Documentation](https://docs.nvidia.com/drive/driveworks-3.5/index.html)