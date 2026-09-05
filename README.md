# VLA-Entrance-Test
## Overview
This project consists of one configurable task that is interfaced with mujoco.
The task involves the picking up of a cylinder and dropping it inside a square bowl.
The robot is restricted to moving its waist, left arm, and left arm.
The following can to vary between episodes:
- Initial positions of the cylinder and bowl.
- Color of the cylindera and bowl.
- Number of cylinder-bowl pairs (NOT IMPLEMENTED YET)

### Data collection
Each episode writes two files:
```
data/ep_n.npz # time series arrays
data/ep_n.json # episode metadata
```
#### Time-series data
- RGB images from left wrist camera and head camera
- Arm chain joint angles in radians, see `config.py` for order
- The end-effector / grasp position
- The time (t)
- Orientation of wrist and fingers
- Commanded target positions `d.ctrl(...)` of hand
#### Metadata
- RNG seed
- Success (whether the cylinder sits inside the bowl)
- Natural language instruction
- Object color(s)
- Total simulation steps
- Total frames
- Initial and final object positions
- Number of actions
- Whether the robot timed out

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

## Running the simulation
```
cd VLA-Entrance-Test
mjpython run_sim.py
```
