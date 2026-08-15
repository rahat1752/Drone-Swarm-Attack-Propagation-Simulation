# Drone Swarm Attack Propagation Simulation Testbed

This repository contains a ROS 2 / PX4 / Gazebo simulation testbed for studying how a local manipulation of inter-drone coordination state can propagate through a UAV swarm and affect downstream agents.

The testbed supports:

- a five-UAV leader-follower swarm;
- configurable directed swarm topologies;
- waypoint-based sweeping missions;
- state spoofing attacks;
- coordination-channel jamming;
- temporal misalignment attacks (delay, jitter, and replay);
- configurable attack targets and progressive attack severity;
- CSV logging of vehicle state, swarm state, formation error, pairwise distance, and mission events;
- post-processing of propagation, formation, safety, timing, entropy, and mission-impact metrics.

This repository is provided for anonymous artifact evaluation. Author-identifying information is intentionally omitted.

---

## 1. System Overview

The experiment uses the following coordination path:

```text
PX4 vehicle state
        |
        v
state_broadcaster
        |
        | /swarm/drone_i_state_raw
        v
attack_manager
        |
        | /swarm/drone_i_state
        v
chain_controller
        |
        v
PX4 offboard setpoints
```

The `attack_manager` sits between the raw vehicle-state broadcaster and the swarm formation controller. It can alter the integrity, availability, or freshness of the coordination state seen by dependent drones.

The leader mission is generated independently by `mission_controller`, while `state_logger` records both physical vehicle state and coordination-level state for later analysis.

---

## 2. Evaluated Swarm Topology

The default topology is an inverted-V dependency tree:

```text
        D1
       /  \
     D2    D3
     |      |
     D5     D4
```

Directed dependencies are:

```text
D1 -> D2 -> D5
D1 -> D3 -> D4
```

The default formation is defined in:

```text
configs/formation_inverted_v_tree.json
```

The repository also contains alternative chain configurations for experimentation.

---

## 3. Tested Software Environment

The testbed was developed for:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- PX4 SITL
- PX4 `gz_x500` vehicle model
- `px4_msgs` compatible with the selected PX4 revision
- Micro XRCE-DDS Agent
- Python 3
- NumPy
- pandas

For strict reproduction, use matching PX4 and `px4_msgs` revisions. The PX4 source tree and Micro XRCE-DDS Agent are external dependencies and are not included in this repository.

---

## 4. Repository Layout

A clean checkout should contain approximately:

```text
DroneSys_ws/
├── src/
│   └── swarm_chain/
│       ├── configs/
│       │   ├── baseline_config.json
│       │   ├── spoofing_config.json
│       │   ├── jamming_config.json
│       │   ├── tma_delay.json
│       │   ├── tma_jitter.json
│       │   ├── tma_replay.json
│       │   ├── sweeping_mission.json
│       │   └── formation_inverted_v_tree.json
│       ├── launch/
│       │   ├── chain_attack.launch.py
│       │   └── baseline_chain.launch.py
│       ├── swarm_chain/
│       │   ├── attack_manager.py
│       │   ├── chain_controller.py
│       │   ├── mission_controller.py
│       │   ├── state_broadcaster.py
│       │   ├── state_logger.py
│       │   └── metrics_chain_entropy.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── README.md
```

Generated ROS 2 directories such as `build/`, `install/`, and `log/` should not be committed.

---

## 5. Build the ROS 2 Workspace

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

From the workspace root:

```bash
cd ~/DroneSys_ws
colcon build --symlink-install
source install/setup.bash
```

If NumPy or pandas is not already installed:

```bash
sudo apt install python3-numpy python3-pandas
```

Verify that the package is visible:

```bash
ros2 pkg list | grep swarm_chain
```

---

## 6. Start the DDS Agent

In a separate terminal:

```bash
MicroXRCEAgent udp4 -p 8888
```

Keep this terminal running throughout the experiment.

---

## 7. Start the Five PX4 SITL Vehicles

The following commands assume that PX4 SITL has already been built and that the current directory is the PX4-Autopilot repository.

### D1

```bash
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1
```

### D2

```bash
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-5,-5" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 2
```

### D3

```bash
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="5,-5" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 3
```

### D4

```bash
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="10,-10" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 4
```

### D5

```bash
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-10,-10" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 5
```

The Gazebo spawn coordinates only determine the initial placement of the vehicles. The in-flight parent-relative formation is defined separately by the formation JSON file.

After all five vehicles are running, verify that ROS 2 can see the PX4 topics:

```bash
ros2 topic list | grep vehicle_local_position
```

The expected namespaces include:

```text
/px4_1/
/px4_2/
/px4_3/
/px4_4/
/px4_5/
```

---

## 8. Run a Baseline Mission

For a baseline, use the same attack pipeline but provide `baseline_config.json`, which disables manipulation while preserving the same communication path.

From the ROS 2 workspace:

```bash
cd ~/DroneSys_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Run:

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/baseline_config.json \
  output_dir:=$PWD/runs/baseline_01
```

Repeat the experiment with a new output directory for each trial:

```text
runs/baseline_01
runs/baseline_02
runs/baseline_03
...
```

Do not reuse an output directory for a failed or repeated trial if the previous CSV files are still present.

---

## 9. Run Attack Experiments

The same launch file is used for all attacks. Only the attack configuration and output directory change.

### State Spoofing

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/spoofing_config.json \
  output_dir:=$PWD/runs/spoofing_D1_01
```

### Coordination-Channel Jamming

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/jamming_config.json \
  output_dir:=$PWD/runs/jamming_D1_01
```

### TMA: Delay

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/tma_delay.json \
  output_dir:=$PWD/runs/tma_delay_D1_01
```

### TMA: Jitter

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/tma_jitter.json \
  output_dir:=$PWD/runs/tma_jitter_D1_01
```

### TMA: Replay

```bash
ros2 launch swarm_chain chain_attack.launch.py \
  mission_file:=$PWD/src/swarm_chain/configs/sweeping_mission.json \
  formation_file:=$PWD/src/swarm_chain/configs/formation_inverted_v_tree.json \
  attack_config:=$PWD/src/swarm_chain/configs/tma_replay.json \
  output_dir:=$PWD/runs/tma_replay_D1_01
```

---

## 10. Attack Configuration

Attack behavior is controlled by JSON files in `configs/`.

The common fields are:

```json
{
  "enabled": true,
  "target_drone": 1,
  "attack_type": "spoof",
  "start_after_mission_s": 20.0,
  "ramp_duration_s": 60.0
}
```

### `target_drone`

Selects the directly manipulated drone.

Examples:

```text
1 = leader
2 or 3 = intermediate parent
4 or 5 = leaf
```

### `start_after_mission_s`

Time after the `MISSION` phase begins before attack injection starts.

### `ramp_duration_s`

Time over which the configured attack magnitude increases from its initial value to its final value.

### Spoofing

```json
"spoof": {
  "bias_start": [0.0, 0.0, 0.0],
  "bias_end": [20.0, 10.0, 0.0]
}
```

The bias is added to the target drone's shared position state.

### Jamming

```json
"jamming": {
  "drop_probability_start": 0.4,
  "drop_probability_end": 0.99
}
```

The attack manager probabilistically drops coordination-state messages from the selected drone.

### TMA Delay

```json
"tma": {
  "mode": "delay",
  "delay_start_ms": 0.0,
  "delay_end_ms": 10000.0
}
```

### TMA Jitter

```json
"tma": {
  "mode": "jitter",
  "delay_start_ms": 50.0,
  "delay_end_ms": 10000.0,
  "jitter_start_ms": 10.0,
  "jitter_end_ms": 9950.0
}
```

### TMA Replay

```json
"tma": {
  "mode": "replay",
  "replay_window_ms": 10000.0
}
```

Users can create additional configuration files to vary target role, attack onset, severity, or ramp duration without modifying the attack-manager source.

---

## 11. Mission Configuration

The default sweeping mission is:

```text
configs/sweeping_mission.json
```

It defines:

- altitude;
- virtual-reference speed; and
- the sequence of XY waypoints.

The default experiment uses a lawnmower/sweeping trajectory and an altitude of `-15 m` in PX4 NED coordinates.

---

## 12. Data Logging

`state_logger` automatically records experiment data while the mission is running.

Each run directory contains timestamped CSV files such as:

```text
vehicle_log_*.csv
swarm_state_log_*.csv
formation_error_*.csv
pairwise_distance_*.csv
mission_events_*.csv
```

### Vehicle log

Contains physical PX4 vehicle state, including:

- position;
- velocity;
- speed;
- PX4 timestamp; and
- state age.

### Swarm-state log

Contains the coordination state seen at the ROS 2 swarm layer, including:

- receive time;
- message timestamp;
- transport/state age; and
- position.

### Formation-error log

Contains parent-child formation measurements, including:

- drone and parent identifiers;
- graph hop;
- 3-D, XY, and Z formation error;
- expected and actual position;
- parent/child update age;
- swarm-state age; and
- parent-child timestamp skew.

### Pairwise-distance log

Contains inter-drone separation for every unique drone pair.

### Mission-events log

Records mission-phase events used to identify the experiment interval.

---

## 13. Verify the Running Pipeline

During a run:

```bash
ros2 node list
```

Expected nodes include:

```text
/state_broadcaster
/attack_manager
/chain_controller
/state_logger
/mission_controller
```

Check swarm topics:

```bash
ros2 topic list | grep /swarm/drone
```

Expected topics include both raw and post-attack state:

```text
/swarm/drone_1_state_raw
/swarm/drone_1_state
...
/swarm/drone_5_state_raw
/swarm/drone_5_state
```

For a quick sanity check:

```bash
ros2 topic echo /swarm/drone_1_state_raw --once
ros2 topic echo /swarm/drone_1_state --once
```

---

## 14. Compute Metrics

The analysis script reads the latest timestamped CSV files from each run directory.

### Create a baseline reference

Example using three baseline trials:

```bash
ros2 run swarm_chain metrics_chain_entropy \
  --make-baseline \
  --baseline-dirs \
    $PWD/runs/baseline_01 \
    $PWD/runs/baseline_02 \
    $PWD/runs/baseline_03 \
  --baseline-out $PWD/runs/baseline_reference.csv
```

This creates:

```text
baseline_reference.csv
baseline_reference_runs.csv
```

### Evaluate an attack against the baseline

```bash
ros2 run swarm_chain metrics_chain_entropy \
  --data-dir $PWD/runs/spoofing_D1_01 \
  --baseline-summary $PWD/runs/baseline_reference.csv
```

The analysis produces summary and intermediate CSV outputs in the selected run directory.

Metrics include measurements related to:

- mission duration;
- mean, maximum, and percentile formation error;
- graph-based attack propagation;
- minimum and mean inter-drone distance;
- near-collision samples, episodes, and duration;
- timing/state age;
- Shannon entropy; and
- baseline-relative mission impact.

---

## 15. Repeating Experiments

For statistical evaluation, use a new directory for every run:

```text
runs/baseline_01/
runs/baseline_02/
runs/spoofing_D1_01/
runs/spoofing_D1_02/
runs/jamming_D1_01/
runs/tma_replay_D1_01/
...
```

If a run fails because a vehicle does not arm, the simplest procedure is to discard that run directory and repeat the trial with a new directory. This prevents logs from multiple attempts from being mixed.

---

## 16. Notes on Interpretation

This testbed distinguishes between:

- **compromised drone/state:** the coordination stream directly manipulated by the attack manager; and
- **affected drone:** an uncompromised downstream drone whose physical behavior changes because it depends on manipulated or stale parent information.

The simulator therefore studies propagation of the **physical coordination effect**, not propagation of software compromise between hosts.

The attack implementations operate on shared coordination state inside the simulation. They should be interpreted as controlled emulations of integrity, availability, and freshness violations rather than as implementations of a specific wireless exploit.

---

## 17. Safety and Scope

This repository is intended for simulation-based research and defensive evaluation.

The included attack models operate inside the local ROS 2 simulation/test environment and are designed to evaluate UAV swarm robustness, propagation behavior, and defense mechanisms.

---

## 18. Citation

Citation information is intentionally omitted during anonymous review. A citation entry can be added after the review process.
