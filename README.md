# VLA-Entrance-Test
## Overview
Repo for VLA project entrance test.

## Structure
```
|---outputs/ # Videos demonstrating robot/environment interaction.
|---src/ # Contains python source code for simulation, tasks, and robot controls.
|---config.py # Modify to configure simulation.
|---run_sim.py # Run to launch simulation with task.
```

## Setup
- Create a workspace directory and clone this repository inside.
- Then within the workspace directory, clone the other required repos
### Installation
#### Python venv
```
python -m venv .venv
source .venv/bin/activate
deactivate

```
#### cyclonedds
https://github.com/unitreerobotics/unitree_sdk2_python
```
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```
#### unitree_sdk2_python
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python

export CYCLONEDDS_HOME="/Users/mattiashollmann/Desktop/Projects/vla_Project/cyclonedds/install"

pip3 install -e .
```
#### mujoco-python
```
pip3 install mujoco
```
#### joystick
```
pip3 install pygame
```
#### unitree_mujoco
```
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```
#### Verify unitree_mujoco works
```
cd unitree_mujoco/simulate_python
mjpython ./unitree_mujoco.py
```
#### Changes to unitree_mujoco config 
```
INTERFACE="Io0"
USE_JOYSTICK=0
ROBOT="g1"
```
#### Additional information
- Make sure there are no folders with spaces in the path e.g. "VLA Project" to vla_project
