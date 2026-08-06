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

### Dependencies

Once you have installed both Ubuntu and ROS, you should install some basic system dependencies.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-tk \
    tk-dev \
    libeigen3-dev \
    libpangolin-dev \
    libopencv-dev
```

### Extra ROS tools

Next install some extra ROS packages required by our system, as well as the tools needed to compile ROS nodes:

```bash
sudo apt install -y \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rqt* \
    python3-colcon-common-extensions \
    ros-dev-tools
```

### Perception

## Usage

## Contributing
