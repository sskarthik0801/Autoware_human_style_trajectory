# Human Driving Replay for Autoware

**Learning to drive from human decisions: record how a person drives in simulation, understand it, and have Autoware's autonomous stack reproduce it.**

![Human-driven paths](docs/images/paths.png)

## The idea

Rule-based autonomous driving stacks are tuned by engineers: speed limits, comfort limits, gap thresholds. A human driver makes the same decisions (when to slow down, how hard to brake, how wide to take a turn, when to wait) from experience and from what is happening around the car.

This project is a first step toward an AV that **makes decisions the way a human driver did in the past, in a similar situation**:

1. **Collect.** A human drives the ego vehicle manually in the [AWSIM](https://github.com/tier4/AWSIM) simulator while the full [Autoware](https://github.com/autowarefoundation/autoware) stack runs. Everything is recorded to ROS2 bags: the car's pose, speed and steering, and (optionally) the surrounding objects, traffic lights and route.
2. **Understand.** The bags are turned into a description of *how* the human drove: path, speed profile, acceleration and braking, jerk, steering and lateral acceleration. Different drives on the same roads show different driving *styles*.
3. **Replicate.** The recorded human trajectory, with the speed the human chose at each point, is fed into Autoware's planning pipeline. Autoware's own controller then drives the car autonomously along the human's path at the human's speed.

```
 ┌──────────────┐   ros2 bag   ┌──────────────────┐   style features   ┌───────────────┐
 │ Human drives │ ───────────▶ │ analyze_drive.py │ ─────────────────▶ │ understanding │
 │   in AWSIM   │              └──────────────────┘                    └───────────────┘
 └──────────────┘        │
                         │     ┌──────────────────────────┐  Trajectory   ┌─────────────────────┐
                         └───▶ │ trajectory_replayer.py   │ ────────────▶ │ Autoware controller │──▶ AWSIM ego car
                               │ (bag → windowed path +   │               │ (MPC / pure pursuit │    drives itself
                               │  human speed profile)    │               │  + velocity smoother)│
                               └──────────────────────────┘               └─────────────────────┘
```

## Repository layout

```
.
├── scripts/
│   ├── record_drive.sh          # record a human drive (ego state, optionally perception)
│   ├── analyze_drive.py         # extract driving-style features + plots (no ROS needed)
│   └── trajectory_replayer.py   # ROS2 node: replay a human drive through Autoware
├── data/bags/                   # recorded human drives (ROS2 bag, sqlite3)
│   ├── test_drive/
│   ├── style_drive/ style_drive_1/ style_drive_2/ style_drive_3/
├── docs/
│   ├── how_it_works.md          # technical details of the replayer
│   └── images/                  # generated plots + features.json
└── requirements.txt
```

## Data: recorded human drives

All drives were recorded in AWSIM's Nishi-Shinjuku map with a human driving the ego vehicle by keyboard. Each bag contains:

| Topic | Type | What it captures |
|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/Odometry` | ego pose + twist (path, yaw rate) |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | speed the human chose |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | steering input |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | frames |

### What the drives show

Generated with `python3 scripts/analyze_drive.py data/bags/* --out docs/images`:

| feature | test_drive | style_drive | style_drive_1 | style_drive_2 | style_drive_3 |
|---|---:|---:|---:|---:|---:|
| duration [s] | 56.3 | 159.5 | 99.1 | 73.9 | 117.5 |
| path length [m] | 328.6 | 1146.5 | 915.9 | 338.2 | 1087.2 |
| mean moving speed [m/s] | 8.31 | 8.91 | 11.59 | 10.18 | 10.16 |
| max speed [m/s] | 13.91 | 21.50 | 18.93 | 17.50 | 23.91 |
| p99 acceleration [m/s²] | 1.08 | 1.80 | 1.65 | 1.65 | 1.69 |
| p99 braking [m/s²] | 2.08 | 4.25 | 3.63 | 13.27* | 3.71 |
| RMS jerk [m/s³] | 1.46 | 1.45 | 1.35 | 7.86* | 1.74 |
| p99 lateral accel [m/s²] | 0.88 | 5.72 | 8.72 | 18.08* | 5.17 |
| fraction of time stopped | 0.30 | 0.19 | 0.20 | 0.55 | 0.09 |

\* `style_drive_2` contains an abrupt event: about 57 s in, the car is reversing at about −11 m/s and its speed drops to 0 in one sample. Treat it as an edge-case or failure example, not a normal driving style.

Even on the same road, the drives differ in style. `style_drive_1` cruises fastest (11.6 m/s mean), `style_drive_3` reaches the highest top speed and almost never stops, and `test_drive` is the gentlest. A replicating AV should keep those differences, not flatten them into one tuned profile.

![Driving profiles](docs/images/profiles.png)

## Replicating a drive with Autoware

`trajectory_replayer.py` reads a bag and:

1. pairs every recorded pose with the nearest-in-time human speed,
2. downsamples the path by distance (`--min-dist`) and recomputes headings from the path geometry,
3. publishes a sliding window of the path as an `autoware_planning_msgs/Trajectory` on `/planning/scenario_planning/lane_driving/trajectory`. That is the input of Autoware's velocity smoother, so the normal *velocity smoother → planning validator → controller* chain still runs on top of the human path,
4. advances the window as the car progresses (closest-point search on `/localization/kinematic_state`) and republishes the final pose as the goal, so the route stays active.

See [docs/how_it_works.md](docs/how_it_works.md) for details.

## Quick start

### 1. Analyze the human drives (no ROS required)

```bash
pip install -r requirements.txt
python3 scripts/analyze_drive.py data/bags/* --out docs/images
```

### 2. Replay a drive autonomously in AWSIM

Requirements: Ubuntu 22.04, ROS2 Humble, an [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation/) build, the [AWSIM](https://tier4.github.io/AWSIM/) binary and the Nishi-Shinjuku map.

```bash
# Terminal 1: simulator
./AWSIM-Demo.x86_64

# Terminal 2: Autoware
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit \
  map_path:=$HOME/autoware_map/nishishinjuku_autoware_map

# Terminal 3: replay the human drive
source ~/autoware/install/setup.bash
python3 scripts/trajectory_replayer.py --bag data/bags/style_drive_1 --dry-run   # inspect first
python3 scripts/trajectory_replayer.py --bag data/bags/style_drive_1 --min-dist 0.5
```

Then in RViz: set the initial pose (**2D Pose Estimate**) if needed and press **AUTO**. The replayer publishes the goal itself.

| Option | Default | Meaning |
|---|---|---|
| `--bag` | (required) | bag directory |
| `--min-dist` | `3.0` | minimum spacing between trajectory points [m] |
| `--hz` | `10` | trajectory publish rate |
| `--dry-run` | off | print the trajectory summary only |
| `--autoware-install` | `~/autoware/install` | used to register Autoware message types for bag decoding |

### 3. Record your own drive

```bash
source ~/autoware/install/setup.bash
./scripts/record_drive.sh my_drive                     # ego state only
./scripts/record_drive.sh my_drive --with-perception   # + surrounding objects / traffic lights
```

## Current limitations

- **Replication is trajectory-level.** The AV reproduces *what* the human did on that route (path and speed), not yet *why*. It does not generalize to a new route or a new traffic situation.
- **The included bags record ego state only.** The surrounding context (other vehicles, pedestrians, traffic lights) that the human reacted to is supported by `record_drive.sh --with-perception` but is not in these recordings yet.
- Speed in the published trajectory is clamped to 2–14 m/s, and reversing is not replayed (speed is taken as an absolute value).
- Keyboard driving in AWSIM gives bang-bang steering (short full-lock pulses), which makes the raw steering signal noisy compared with a wheel.

## Roadmap: from replaying to deciding like a human

1. **Record context alongside decisions.** Log perception objects, traffic lights and the route with every drive.
2. **Segment drives into decisions.** Split into events such as *follow*, *stop at light*, *yield*, *turn* and *overtake*, and label the scene state at each one (gap to the lead vehicle, its speed, distance to the stop line, light state).
3. **Learn the human's decision policy.** Fit per-event models (for example time gap vs. speed while following, or braking onset distance at red lights), then move to imitation learning / behavior cloning from *scene → speed and path*.
4. **Retrieve and adapt.** For a new situation, find the most similar past human situation and use that decision as the planner's target, with Autoware's safety layers (planning validator, AEB) kept as the guard.
5. **Evaluate.** Compare AV vs. human on the same scenarios: speed and acceleration profile error, path deviation, comfort metrics and safety margins.

## Tech stack

ROS2 Humble · Autoware · AWSIM (Unity) · Python (`rclpy`, `rosbags`, NumPy, Matplotlib)
