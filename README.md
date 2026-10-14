# epos4_drive

ROS 2 Humble C++ package to control a Maxon motor with an EPOS4 positioning controller in **Cyclic Synchronous Torque (CST)** mode at **200 Hz**, providing actual position feedback in **ticks** and **degrees** on separate topics.

Built on top of [`EposLib`](../EposLib) and Lely CANopen.

---

## 1. Features
- **200 Hz Control Loop**: Matches a 5000 µs CANopen SYNC period configured in `bus.yml`.
- **Physical Torque Setpoints**: Accepts input torque in $\text{N}\cdot\text{m}$ (`std_msgs/msg/Float64`).
- **Dual Position Feedback**:
  - `position_ticks`: Raw encoder quadcounts (`std_msgs/msg/Float64` for compatibility with `bus_can_drive2`).
  - `position_deg`: Angle at the output shaft in degrees (`std_msgs/msg/Float64`), accounting for CPR and gear ratio.
- **Full Parameterization**: All parameters (CAN interface, node IDs, mechanism CPR, gear ratio, safety limits, topic names) configurable via `config/epos4_params.yaml`.
- **Safety Supervision**:
  - **Overspeed Protection**: Immediately cuts torque to 0 if motor speed exceeds `max_speed_rpm`.
  - **Command Watchdog**: Automatically zeroes torque if commands cease for longer than `watchdog_timeout_ms`.
  - **Torque Clamp**: Clamps inputs within `[-max_torque_nm, +max_torque_nm]`.
  - **Graceful Shutdown**: Sends 0 torque before disabling the drive upon SIGINT or shutdown.
- **Jetson Orin Nano Compatible**: Fully compatible with Linux SocketCAN (`can0`) and tested alongside `talon_joints_node` (CTRE Phoenix).

---

## 2. Topics & Parameters

### Subscribed Topics
- `/arm_teleop/joint3/torque_nm` (`std_msgs/msg/Float64`): Commanded torque in $\text{N}\cdot\text{m}$.

### Published Topics
- `/arm_feedback/joint3_ticks` (`std_msgs/msg/Float64`): Position in encoder ticks (quadcounts).
- `/arm_feedback/joint3_deg` (`std_msgs/msg/Float64`): Position in degrees at output shaft.

### Parameters (`config/epos4_params.yaml`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `can_interface` | string | `"can0"` | SocketCAN interface |
| `node_id` | int | `5` | EPOS4 CANopen Node ID |
| `master_node_id` | int | `1` | CANopen Master Node ID |
| `dcf_path` | string | `""` | Master DCF file path (uses installed default if empty) |
| `loop_hz` | double | `200.0` | Node control and publishing frequency (Hz) |
| `encoder_cpr` | int | `500` | Encoder counts per revolution (quadcounts = 4 * CPR) |
| `gear_ratio` | double | `1.0` | Output turns per motor turn (e.g. `0.01` for 100:1) |
| `initial_angle` | double | `180.0` | Initial output angle at power-on (degrees) |
| `sign` | double | `1.0` | Direction sign (+1.0 or -1.0) for right-hand rule |
| `max_speed_rpm` | int | `3000` | Speed cutoff limit (RPM) |
| `max_torque_nm` | double | `0.20` | Maximum allowable torque command (N·m) |
| `watchdog_timeout_ms` | int | `500` | Timeout window before zeroing torque |
| `torque_topic` | string | `"/arm_teleop/joint3/torque_nm"` | Torque command topic name |
| `position_ticks_topic` | string | `"/arm_feedback/joint3_ticks"` | Position ticks topic name |
| `position_deg_topic` | string | `"/arm_feedback/joint3_deg"` | Position degrees topic name |

---

## 3. Build & Execution

### Build
From the workspace root (`ros2_ws`):
```bash
colcon build --packages-select eposlib epos4_drive --symlink-install
source install/setup.bash
```

### Launch
To launch with default parameters:
```bash
ros2 launch epos4_drive epos4_torque_launch.py
```

To launch with a custom YAML file:
```bash
ros2 launch epos4_drive epos4_torque_launch.py params_file:=/path/to/custom_params.yaml
```

### Direct Run
```bash
ros2 run epos4_drive epos4_torque_node --ros-args --params-file src/epos4_drive/config/epos4_params.yaml
```

---

## 4. Running alongside `talon_joints_node` on Jetson Orin Nano
Both nodes communicate over SocketCAN (`can0`).
1. Set up `can0` at 1 Mbit/s before launching:
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```
2. Both `talon_joints_node` and `epos4_torque_node` can run simultaneously. Linux SocketCAN multiplexes CAN frames from both processes.

