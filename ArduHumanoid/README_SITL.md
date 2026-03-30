# ArduHumanoid SITL

Native humanoid vehicle type for ArduPilot with Gazebo Harmonic simulation. GSoC 2026.

## Requirements
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) built and installed
- Gazebo Harmonic
- MAVProxy: `pip install mavproxy`

## Build ArduHumanoid
```bash
./waf configure --board sitl
./waf build --target bin/humanoid
```

## Run SITL

**Terminal 1 — ArduPilot:**
```bash
Tools/autotest/sim_vehicle.py -v ArduCopter --model JSON --console
```

**Terminal 2 — Gazebo:**
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot/humanoid_sitl/models:$HOME/ardupilot_gazebo/models
gz sim humanoid_sitl/worlds/humanoid.sdf
```