# Model Predictive Control for Bipedal Robot TRON 1 in MuJoCo Simulator
![video](./display.gif)

# Build Package
## Install ROS2 according to your operating system
Refer to [ROS2](https://ros.org/) official doc.

## Dependency
```bash
sudo apt-get install ros-<ros2-distro>-eigenpy
sudo apt-get install ros-<ros2-distro>-pinocchio
```

## Build
```bash
git clone https://github.com/81578823/Bipedal_MPC.git
```

```bash
cd Bipedal_MPC & bash build.sh
echo "source Bipedal_MPC/install/setup.bash" >> ~/.bashrc
```

# Run Package
## Run Simulation
```bash
ros2 launch sim sim_launch.py 
```

## Run Controller
```bash
ros2 launch management management_launch.py 
```





