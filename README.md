# ROS2 implement for Arc*: Asynchronous Event-based Corner Detection 

This code is the ROS2 implementation of the Arc* algorithm described in the paper  "**Asynchronous Corner Detection and Tracking for Event Cameras in Real Time**", Alzugaray & Chli, RA-L 2018.

# Instructions

Currently only support ROS2 (Humble, Iron, etc.), ROS1 will soon be support.

## Build

Make your own ros workspace and clone the repo.

```bash
mkdir -p <ros2_ws>/src # You can change the workspace folder name
cd <ros2_ws>/src

git clone https://github.com/Ericsii/arc_star.git
cd ..
rosdep install --from-paths src --ignore-src -y -r # Install all the dependencies from `src`

colcon build --symlink-install # Compile the package
```

## Run the program

```bash
. ./install/setup.[bash|zsh|sh] # Setup path variables
ros2 launch arc_star arc_star.launch.py # Launch the arc_star_node
```

You can modify the configuration in `config/params.yaml` for your sensor resolution and different output encoding of events.

## Publication
If you use this work, please cite the following [publication](https://www.research-collection.ethz.ch/handle/20.500.11850/277131): 

Ignacio Alzugaray and Margarita Chli. "**Asynchronous Corner Detection and Tracking for Event Cameras in Real Time.**" IEEE Robotics and Automation Letters (RA-L), 2018. 

```bibtex
@ARTICLE{alzugaray18ral
    author={I. Alzugaray and M. Chli},
    journal={IEEE Robotics and Automation Letters},
    title={Asynchronous Corner Detection and Tracking for Event Cameras in Real Time},
    year={2018},
    volume={3},
    number={4},
    pages={3177-3184},
    doi={10.1109/LRA.2018.2849882},
    ISSN={2377-3766},
    month={Oct}}
```

# License

The source code is released under the MIT License.