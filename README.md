# Cardiff Autonomous Racing

Cardiff Autonomous Racing is a team that develop an autonomous system to compete in the [FS-AI competition](https://www.imeche.org/events/formula-student/team-information/fs-ai) at Silverstone.

## Table of Contents

1. [Ways to Run](#ways-to-run)
2. [Docker Installation](#docker)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Contributing](#contributing)

## Ways to Run

There are 2 ways to run this system, natively, or via docker.

||Natively|Docker|
|---|---|---|
|Performance|Can have significantly faster performance (over 10x in some cases)|Slower|
|OS|Only (officially) runs on Ubuntu 22.04 (and distros based on it)|Runs on all operating systems supported by docker|
|Ease of install|Can be complicated to get system working correctly|Easy to get working|
|Build time|Quick|Can take up to an hour - especially on first build|

## Docker

A guide to installing via docker is available [in the docker folder](/docker/DOCKER_README.md).

## Installation

### Installation Table of Contents

### Prerequisites

You will need the following:

1. **Operating System**: [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/) (or a distro based on this version)
2. **ROS 2**: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

A guide to dual-booting alongside windows can be found [online](https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/). When partitioning, try to set aside at least 100GB, not just 20GB.

### Clone the repo

Once you have installed both Ubuntu and ROS, you are ready to get the code from this repo. Navigate to your desired file location using the terminal, then clone the repo using:

```bash
git clone https://git.cardiff.ac.uk/c23015833/cardiff-autonomous-racing.git
cd cardiff-autonomous-racing
git submodules init
git submodules update
```


### Dependencies

Once you have installed both Ubuntu and ROS, you should install all the dependencies this project uses.


```bash
sudo apt install -y \
    git \
    wget \
    cmake \
    build-essential \
    iproute2 \
    can-utils \
    kmod \
    gazebo \
    gfortran \
    pkg-config \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy \
    python3-venv \
    python3-scipy \
    python3-matplotlib \
    python3-ament-package \
    python3-pandas \
    ros-humble-ackermann-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rqt* \
    ros-humble-std-srvs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-visualization-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-libraries \
    ros-humble-ament-cmake-core \
    ros-humble-ament-package \
    ros-humble-rosidl-typesupport-c
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-plugins \
    ros-humble-message-filters \
    ros-humble-vision-opencv \
    ros-dev-tools \
    libeigen3-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libatlas-base-dev \
    libgoogle-glog-dev \
    libopenblas-dev \
    liblapack-dev
```

And then some important python libraries.

```bash
pip install -r requirements.txt
```

### Perception

## Usage

## Contributing
