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

This should result in a directory structure similar to the following:
```
ros2_ws/                                                     
├── build                                                                                                               
├── install 
├── log
└── src
    └── civ4100
        ├── demo # demo package
    └── external #all external repos
        ├── SD-VehicleInterface #sd_vehicle_interface ros2 package
        ├── carla-ros-bridge 
        └── hardware_in_loop # carla_odom_to_twist ros2 package

```

## Usage

### CARLA example 
Launch CARLA server:
- `docker compose --profile carla up`

Launch Docker container with carla-ros-bridge dependencies
- `docker compose --profile carla-ros-bridge up`

Attach to this container 
By default, `<container-name>` is `streetdrone-hil-carla-ros-bridge`. If it doesn't work, see the actual container name using `docker container ls`.
- `docker exec -it <container-name> /bin/bash`

Run custom scripts
- `ros2 launch demo demo.launch.py` - this runs carla-ros-bridge and spawns all relevant objects.
- `ros2 run demo ego_vehicle_control` - this is the script that contains the lead/ego vehicle car logic.

Convert CARLA ego_vehicle odometry to Twist messages (so that sd_vehicle_interface can understand)
- `ros2 run hardware_in_loop carla_odom_to_twist`

Run sd_vehicle_interface
- `cd ~/ros2_ws/src/external/SD-VehicleInterface` `./can_setup.sh`
- `ros2 launch sd_vehicle_interface sd_vehicle_interface.launch.xml`

Put the Streetdrone into autonomous mode and you should be good to go.
