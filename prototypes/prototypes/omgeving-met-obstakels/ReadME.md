# Context
This prototype is the first realistic simulated room with obstacles. The goal was to test how well the robot and its sensors handle a populated room, rather than an empty room.

To keep the simulation setup maintainable, the model was split up into separate reusable "plugins" and obstacle models rather than one large monolithic `.sdf` file.

It served as our basis for modularity in our project. It inspired us to make our robot modular as well. Sensors were thus not defined directly inside the robot's .sdf file but were attached to the model as separate plugin snippets from `models/gazebo/plugins/`.


## Plugin location
| File | Sensor |
|---|---|
| `lidar_sensor.sdf` | GPU LiDAR (`gpu_lidar`) |
| `front_camera.sdf` | Front-facing camera |
| `rear_camera.sdf` | Rear-facing camera |
| `thermal_camera.sdf` | Thermal camera |
| `logical_audio_sensor.sdf` | Logical audio sensor |


## Obstacles — `models/gazebo/obstacles/`
Obstacles are defined as individual Gazebo models, organised into two categories:

- **`shapes/`** — simple geometric shapes (boxes, spheres, cylinders) for basic obstacle testing
- **`walls/`** — wall segments for constructing room layouts

Obstacles are included in the environment/room .sdf files.

## Prototype Files — `prototypes/omgeving-met-obstakels/`
| File | Description |
|---|---|
| `omgevingV1.sdf` | First version of the obstacle room simulation |
| `kamer/kamerV1.sdf` | First version of the modular room layout used in the simulation |
