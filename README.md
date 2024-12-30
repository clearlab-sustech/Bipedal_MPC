# Model Predictive Control for Bipedal Robot TRON 1 in MuJoCo Simulator
![video](./display.gif)
# Build Package
## Install ROS2 
Refer to [ROS2](https://ros.org/) official doc.

## Build

```bash
cd ${source folder} & bash build.sh
echo "source ${source folder}/install/setup.bash" >> ~/.bashrc
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





