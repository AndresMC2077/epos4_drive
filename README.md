# epos4_drive

ROS 2 Humble C++ package to control dual (or multi-axis) Maxon motors with EPOS4 positioning controllers in **Cyclic Synchronous Torque (CST)** mode at **200 Hz** over a single shared CANopen master, providing actual position feedback in **ticks** and **degrees** on separate topics per motor.

Built on top of [`EposLib`](../EposLib) and Lely CANopen.

---

## 1. Architecture & Features
- **Single Shared CANopen Master (`epos4::CanBus`)**: Both EPOS4 drives (`node_5` for `joint2`, `node_4` for `joint3`) share one CANopen master on `can0` broadcasting a 200 Hz (`5000 µs`) SYNC pulse.
- **No Extra Threads**: Uses a single 200 Hz ROS 2 wall timer on the default single-threaded executor. All cyclic CAN reads/writes are lock-free atomics.
- **Independent Per-Motor Configuration (including EDS)**:
  - Each motor has its own EDS file (`config/red_epos/joint2.eds`, `config/red_epos/joint3.eds`) referenced per node in `config/red_epos/bus.yml`.
  - Every motor parameter (`node_id`, `required`, `encoder_cpr`, `gear_ratio`, `initial_angle`, `sign`, `c_torque`, `max_speed_rpm`, `max_torque_nm`, `watchdog_timeout_ms`, and topic names) is independently configured under `<motor_name>.*` in `config/epos4_params.yaml`.
- **Fault Isolation (`required: false`)**: If one drive fails to respond at startup or faults at runtime, only that motor is isolated/zeroed while the other motor continues operating normally.

---

## 2. Topics & Parameters

### Default Topics
| Joint | Node ID | Subscribed Command (`std_msgs/msg/Float64`) | Published Ticks (`std_msgs/msg/Float64`) | Published Degrees (`std_msgs/msg/Float64`) |
|---|---|---|---|---|
| `joint2` | `5` | `/arm_teleop/joint2/current` | `/arm_feedback/joint2_ticks` | `/arm_feedback/joint2_deg` |
| `joint3` | `4` | `/arm_teleop/joint3/current` | `/arm_feedback/joint3_ticks` | `/arm_feedback/joint3_deg` |

### Bus-Level Parameters (`config/epos4_params.yaml`)
| Parameter | Type | Default | Description |
|---|---|---|---|
| `can_interface` | string | `"can0"` | SocketCAN interface |
| `master_node_id` | int | `1` | CANopen Master Node ID |
| `dcf_path` | string | `""` | Master DCF path (uses installed `config/red_epos/master.dcf` if empty) |
| `loop_hz` | double | `200.0` | Control & publishing frequency (Hz) |
| `motor_names` | string[] | `["joint2", "joint3"]` | List of motor prefixes to initialize |

### Per-Motor Parameters (`<joint>.*`)
| Parameter | Type | Default (`joint2` / `joint3`) | Description |
|---|---|---|---|
| `<joint>.node_id` | int | `5` / `4` | EPOS4 CANopen Node ID (must match `config/red_epos/bus.yml`) |
| `<joint>.required` | bool | `false` | If `true`, node aborts startup when this drive fails to boot |
| `<joint>.encoder_cpr` | int | `4096` | Encoder pulses per revolution (`quadcounts = 4 * CPR`) |
| `<joint>.gear_ratio` | double | `0.01` | Output turns per motor turn (`1.0 / 100.0`) |
| `<joint>.initial_angle` | double | `0.0` | Output angle at power-on (degrees) |
| `<joint>.sign` | double | `-1.0` | Direction sign (`+1.0` or `-1.0`) |
| `<joint>.c_torque` | double | `0.00525` | Motor torque constant ($\text{N}\cdot\text{m}/\text{A}$) |
| `<joint>.max_speed_rpm` | int | `3000` | Overspeed cutoff limit (RPM) |
| `<joint>.max_torque_nm` | double | `0.20` | Maximum torque clamp ($\text{N}\cdot\text{m}$) |
| `<joint>.watchdog_timeout_ms` | int | `500` | Command timeout before zeroing torque (ms) |
| `<joint>.torque_topic` | string | `"/arm_teleop/<joint>/current"` | Command topic |
| `<joint>.position_ticks_topic` | string | `"/arm_feedback/<joint>_ticks"` | Ticks feedback topic |
| `<joint>.position_deg_topic` | string | `"/arm_feedback/<joint>_deg"` | Degrees feedback topic |

---

## 3. Build & Launch

```bash
colcon build --packages-select eposlib epos4_drive --symlink-install
source install/setup.bash
ros2 launch epos4_drive epos4_torque_launch.py
```
