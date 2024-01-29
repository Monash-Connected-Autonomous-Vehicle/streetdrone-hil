# streetdrone-hil
A collection of scripts to run hardware-in-the-loop (HIL) demos with the Streetdrone Renault Twizy or Nissan ENV200.

# CIV4100 Demo
A demo with basic car following functionality prepared in CARLA for the [CIV4100](https://handbook.monash.edu/2024/units/CIV4100?year=2024) unit.

<!-- A screenshot or Gif of the working project -->
![](images/ex.png)

<!-- All requirements of the project should be links to where we can install them -->
<!-- pip dependencies must be added into a requirements.txt file -->
## Requirements
- [Ubuntu 20.04/22.04](https://ubuntu.com/download/desktop)
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [docker-compose](https://docs.docker.com/compose/install/linux/#install-using-the-repository)
- [CARLA Requirements](https://carla.readthedocs.io/en/latest/start_quickstart/#before-you-begin)

## Installation
- Go to the `src` directory: `cd ~/colcon_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/colcon_ws`
- Install ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Build: `colcon build --symlink-install`

<!-- Directory structure gives a brief on what folders contain what. -->
This should result in a directory structure similar to the following:
```
ws/                                                     
├── build                                                                                                               
└── src
    └── mcav_autonomy
        ├── folder                  # what's in there
        ├── folder                  # what's in there
        └── folder                  # what's in there
```

## Usage
<!-- 
 Usage instructions must be concise. Any export statements must be added to .bashrc (add steps in either requirements of installation).

It should follow the structure mentioned below:
Terminal # (What are we doing):
- `shell code`
-->

### CARLA example 
Terminal 1 (Launch CARLA server):
- `/opt/carla-simulator/CarlaUE4.sh`

Terminal 2 (Launch ...):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch ...`

Terminal 3 (Launch ....):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch ...`

### StreetDrone example
...

## Tests

Located under `tests`. Enter `pytest` in the terminal to run all tests.

## ROS Parameters and Topics
Please see the [`ROSINFO.md`](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav-GitHub-documentation-standard/blob/main/ROSINFO.md) file for more info.
