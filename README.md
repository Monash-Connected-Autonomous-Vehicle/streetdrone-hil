### Prerequisites
- Ubuntu 22.04 (amd64)
- ROS Humble
- Python 3.10

## Setting up Carla X Autoware
- Note: if you encounter any strange errors, consult *Common Bugs* listed at the end of this README. Alternatively, check out [this resolved issue](https://github.com/hatem-darweesh/op_bridge/issues/27) or create a new issue for this repo
- If you have issues with Autoware, ensure you can at least install & build the latest Autoware repo without any issues 
- This is currently set up in *the beast* (28 fps on rviz) under `~/dbat` directory, *hive-3* (20 fps) and SD Alienware Laptop (8 fps) under `~/carlaxautoware` 

### Set up Autoware
1. Create directory called `carlaxautoware` in Home directory:
```bash
mkdir ~/carlaxautoware
cd ~/carlaxautoware
```

2. Clone Autoware.universe and checkout to `release/2023.10`
```sh
cd ~/carlaxautoware
git clone https://github.com/autowarefoundation/autoware.git 
cd autoware
```

```sh
git checkout release/2023.10
./setup-dev-env.sh
```

3. Clone `autoware.repos`:
```sh
# In ~/carlaxautoware/autoware
mkdir src 
vcs import src < autoware.repos
```

4. Install dependent ROS packages 
```sh
source /opt/ros/humble/setup.bash rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

5. Clone OpenPlanner and LIDAR driver in the *universe* folder (/src/universe/external)
```sh
cd ~/carlaxautoware/autoware/src/universe/external
git clone https://github.com/ZATiTech/open_planner.git -b humble
```

```sh
cd ~/carlaxautoware/autoware/src/universe/external
git clone https://github.com/autowarefoundation/awf_velodyne
```

6. Setup `yaml` files
```sh
cd ~/carlaxautoware/autoware/src/param/autoware_individual_params/individual_params/config/default/
mkdir carla_sensor_kit
cd carla_sensor_kit
wget https://raw.githubusercontent.com/ZATiTech/open_planner/humble/op_carla_bridge/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml
wget https://raw.githubusercontent.com/ZATiTech/open_planner/humble/op_carla_bridge/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensors_calibration.yaml
```

7. Build Autoware
```sh
cd ~/carlaxautoware/autoware
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Set up Carla + additional directories
1. Create sub directory for Carla-related files
```sh
mkdir ~/carlaxautoware/carla-0.9.15
cd ~/carlaxautoware/carla-0.9.15
```

2. Download [Carla 0.9.15](https://github.com/carla-simulator/carla/releases) and extract the contents into the directory above
3. Clone these additional repos in `~/carlaxautoware/autoware/carla-0.9.15`:
- `op_agent`
```sh
git clone https://github.com/hatem-darweesh/op_agent.git -b ros2-humble
```
  
- `scenario_runner`
```sh
git clone https://github.com/hatem-darweesh/scenario_runner.git
```

- This repo (which is a modified version of `op_bridge`) fork and install its python dependencies
```sh
git clone https://github.com/Monash-Connected-Autonomous-Vehicle/streetdrone-hil.git -b carla_autoware
cd streetdrone-hil
pip install -r requirements.txt
```
- **Note**: if you encounter errors when install from `requirements.txt`, you'll need to install each python library manually

4. Download relevant Carla`.egg` file
```sh
cd ~/carlaxautoware/carla-0.9.15/PythonAPI/carla/dist
wget https://github.com/Monash-Connected-Autonomous-Vehicle/streetdrone-hil/blob/carla_autoware/carla-0.9.15-py3.10-linux-x86_64.egg
```

5. Download `pcd` map from this [website](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/point_cloud_maps/Town01.pcd) into the Downloads folder.

6. Then move it to correct location and rename it
```sh
cd ~/carlaxautoware/carla-0.9.15/op_agent/autoware-contents/maps/Town01/
mv ~/Downloads/Town01.pcd pointcloud_map.pcd
```

## Running Carla X Autoware
First Terminal:
```sh
cd ~/carlaxautoware/carla-0.9.15
./CarlaUE4.sh -RenderOffScreen
```

Second Terminal:
```sh
export OP_BRIDGE_ROOT=~/carlaxautoware/carla-0.9.15/streetdrone-hil
export OP_AGENT_ROOT=~/carlaxautoware/carla-0.9.15/op_agent
export OP_BRIDGE_ROOT=~/carlaxautoware/carla-0.9.15/streetdrone-hil
export AUTOWARE_ROOT=~/carlaxautoware/autoware
export CARLA_ROOT=~/carlaxautoware/carla-0.9.15
export SCENARIO_RUNNER_ROOT=~/carlaxautoware/carla-0.9.15/scenario_runner
export LEADERBOARD_ROOT=~/carlaxautoware/carla-0.9.15/streetdrone-hil/leaderboard
export TEAM_CODE_ROOT=~/carlaxautoware/carla-0.9.15/op_agent
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg
```

```sh
cd ~/carlaxautoware/carla-0.9.15/streetdrone-hil/op_scripts
./run_exploration_mode_ros2.sh
```

### Broadcasting ROS publishes to LAN (Optional)
Do this before running `/run_exploration_mode_ros2.sh`:

```sh
export ROS_LOCALHOST_ONLY=2
export ROS_DOMAIN_ID=2
```

### Common Bugs
*ImportError: cannot import name '...' from 'collections' using Python 3.10*
- Refer to [this stackoverflow answer](https://stackoverflow.com/questions/69381312/importerror-cannot-import-name-from-collections-using-python-3-10)


*'carla' has not attribute 'Client'*
```
pip3 install -Iv setuptools==47.3.1
pip install carla
```


*Unable to create glx context*   
```
sudo apt-get remove --purge xserver-xorg
sudo apt-get install xserver-xorg
sudo dpkg-reconfigure xserver-xorg
sudo reboot
```


*ERROR:colcon.colcon_core.extension_point:Exception loading extension 'colcon_core.verb.version-check': The 'typing>=3.7.4' distribution was not found and is required by typing-extensions*
```
pip install typing
```


*ModuleNotFoundError: No module named 'distutils.command.bdist_wininst'*
```
`pip install --upgrade setuptools`
```

*ImportError: cannot import name 'gcd' from 'fractions' (/usr/lib/python3.10/fractions.py)*
- Refer to [this stackoverflow answer](https://stackoverflow.com/questions/66174862/import-error-cant-import-name-gcd-from-fractions)
