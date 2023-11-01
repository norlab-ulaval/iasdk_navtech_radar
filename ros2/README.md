# Navtech ROS2 Driver

The Navtech ROS2 Driver repository contains examples of ROS2 publishers and subscribers, which interface with RTSP cameras and Navtech Radar.
Both basic and complete examples are provided which allow simple publishing of data, simple subscribing to data, and also immediate viewing
of radar/camera data, using the ROS2 visulaisation tool, RVIZ.

Note - The Navtech ROS2 driver is dependent on the Navtech SDK.

Please see lower level README.md files, for more specific information on the ROS2 project folders.

## Navtech SDK Requirements

### C++ 17

* C++17 Compiler
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt-get update

sudo apt install gcc-9 gcc-9-base gcc-9-doc g++-9

sudo apt install libstdc++-9-dev libstdc++-9-doc
```

* GCC 9.x and above
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt update

sudo apt install gcc-9
```

* Clang 10 and above
Install with the following command:

```bash
sudo apt-get install clang-10
```

### Microsoft .NET
* .NET 4.8 and above

## Linux Requirements
To use the shell scripts provided in the 'utility scripts' folder we require bash on Ubuntu. First you must execute:
```bash
sudo dpkg-reconfigure -p critical dash
```

## ROS2 Requirements
As above and:
ROS2 Galactic Geoclone - CPP and Python bindings
Installation instructions can be found here: https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Development-Setup.html

## Python Requirements
Python3.0+

```bash
sudo apt install python3.8
```

Numpy 1.17.4
Can be installed with the following command:

```bash
sudo apt install python3-numpy
```

## OpenCV Requirements
OpenCV 4.5.3 - CPP and Python bindings
Installation instructions here: https://www.linuxfromscratch.org/blfs/view/svn/general/opencv.html
Note - this link has been updated, subsitute version number with 4.5.3

## FFMPEG Requirements
ffmpeg version 4.2.4-1ubuntu0.1
Can be installed with the following command:

```bash
sudo apt install ffmpeg
```

## License

The ROS2 driver which is released under The MIT License (MIT).
See file LICENSE.txt or go to <https://opensource.org/licenses/MIT> for full license details.

## ROS2 Packages

To use ROS commands, ROS2 must first be sourced using: source /opt/ros/galactic/setup.bash
Note - this must be done in every new terminal
Alternatively, make this permanent by adding to your bash file: echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
This will source ROS for every newly opened terminal window

All ROS2 packages must be built and installed before being run
Build with:
```bash
colcon build
```

Install with:
```bash
. install/setup.bash
```

Packages can be run like so:

```bash
ros2 run <package_name> <executable_name>
```

For example:

```bash
ros2 run nav_ros colossus_publisher
```

Packages can be run with their corresponding paramater files like so:

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <params_file_path>
```

For example:

```bash
ros2 run nav_ros colossus_publisher --ros-args --params-file ./src/nav_ros/config/colossus_publisher.yaml
```

## camera_ros

This folder contains examples based on a connection to an RTSP camera stream. Examples include conencting to and reading
camera images, publishing camera images, subscribing to camera images, and saving camera iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

**See the README.md under 'camera_ros', for more detailed instructions**

## launch_ros

This folder contains launch files which are intended to receive and display data with ease. The launch files will launch the
required ros nodes, transform(s) and UI (RVIZ) to display data reived from a RTSP camera or Navtech Radar. The
Launch files also use configuration files which contain the default settings for the examples.

**See the README.md under 'launch_ros', for more detailed instructions**

## messages

Contains the custom message types used within the ROS2 Navtech driver - this includes camera messages and Navtech radar messages.

**See the README.md under 'messages', for more detailed instructions**

## nav_ros

This folder contains examples based on a connection to a Navtech radar. Examples include conencting to and reading
radar data, publishing radar data and images, subscribing to radar data and images, and saving radar iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

**See the README.md under 'nav_ros', for more detailed instructions**

## rviz_views

Contains rviz view configuration files used to quickly visualise data output from the above packages. These view configuration
files are used by the launch files.

**See the README.md under 'rviz_views', for more detailed instructions**



# ROS2 Example install procedure

## Update Ubuntu 20.04

```bash
sudo apt update

sudo apt upgrade
```
	
## Install IASDK prerequisites

```bash
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```

The SDK also needs the header only Boost ASIO library, this can be found here (current newest version is 1.24.0):
https://sourceforge.net/projects/asio/files/asio/

It is recommended that the library be installed where your compiler toolchain system path is located; for example /usr/include

To build using cmake, navigate to the root of the iasdk folder and run the following commands

```bash
./bootstrap_cpp17.sh

cd cpp_17/

cmake .

make
```

If you are running in Linux, you are likely to get an error with running the ./bootstrap_cpp17.sh script. This is
due to the script having dos line endings, and can be fixed using the tool 'dos2unix'

```bash
sudo apt-get install dos2unix

dos2unix bootstrap_cpp17.sh
```

Full install instructions, including building using VS Code can be found here:
https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2328199577/C+17
	
## Install ROS2 prerequisites

```bash
sudo apt install -y build-essential libssl-dev libffi-dev python3-dev python3 python3-pip software-properties-common
```

## Install ROS2

```bash
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu 

$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt install ros-galactic-desktop

source /opt/ros/galactic/setup.bash

ros2 run demo_nodes_cpp talker
```
	
## Check install by running examples

```bash
source /opt/ros/galactic/setup.bash

ros2 run demo_nodes_py listener
```

Check that the talker and listener are connected, and messages are being exchanged

## Install OpenCV

```bash
sudo apt install git

git clone https://github.com/opencv/opencv.git

git clone https://github.com/opencv/opencv_contrib.git

sudo apt install build-essential cmake git pkg-config libpng-dev libtiff-dev gfortran openexr libgtk-3-dev libavcodec-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev

cd opencv

mkdir build

cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE    -D CMAKE_INSTALL_PREFIX=/usr/local          -D WITH_CUDA=OFF        -D INSTALL_PYTHON_EXAMPLES=ON          -D OPENCV_GENERATE_PKGCONFIG=ON         -D  OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules  -D OPENCV_ENABLE_NONFREE=ON         -D BUILD_EXAMPLES=ON ..

make -j8  // (replace the 8 by the number of usable cores on your machine)

sudo make install

sudo ldconfig
```

## Check OpenCV is installed

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

check that version >=4.5.3 is reported
	
```bash
pkg-config --modversion opencv4
```
	
check that version >=4.5.3 is reported
	
## Install the Navtech Radar IASDK

```bash
git clone https://bitbucket.org/navtechradar/iasdk-public.git
```

## Install Colcon (the ROS2 build tool)

```bash
sudo apt install python3-colcon-common-extensions

cd ~/iasdk/cpp_17

colcon build
```

Check the above command does not produce any errors


## Build the ROS2 IASDK pacakges

```bash
cd ~/iasdk/ros2/

source /opt/ros/galactic/setup.bash

colcon build

. install/setup.bash
```

## Common errors and solutions

# Ros executables cannot be found after build and execution attempt

Run the following to source the newly build executables

```bash
source /opt/ros/galactic/setup.bash

. install/setup.bash
```

# Unable to launch RVIZ due to missing library (libQt5Core.so.5)

Run the following to source the newly build executables

```bash
rviz2: error while loading shared libraries: libQt5Core.so.5: cannot open shared object file: No such file or directory
```

In short - the above command removes a piece of code which is looking for something (renameat2 system call) that does not exist in kernels < 3.15