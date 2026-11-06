// epos4_torque_node.cpp: ROS 2 Humble node for dual/multi-axis Cyclic
// Synchronous Torque (CST) control of Maxon motors via EPOS4 and EposLib.
//
// Architecture:
//   - ONE shared epos4::CanBus (one CANopen master on can0 sending SYNC at 200 Hz).
//   - N independent MotorDriver instances (default: joint2 on node 5, joint3 on node 4),
//     each with its own parameters, topics, watchdog, overspeed guard, and fault isolation.
//   - Single 200 Hz ROS 2 wall timer on the default single-threaded executor (no extra threads).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <ros2units/units.h>
#include "epos4/CanBus.hpp"
#include "epos4/hardware/Epos4.hpp"
#include "epos4/signals/Errors.hpp"

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

namespace
{
std::int64_t NowSteadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();
}
}  // namespace

// MotorDriver: per-joint EPOS4 instance, parameters, pub/sub, and control state
class MotorDriver
{
public:
  MotorDriver(
    rclcpp::Node & node,
    epos4::CanBus & bus,
    const std::string & name,
    int default_node_id)
  : name_(name),
    logger_(node.get_logger())
  {
    const std::string p = name_ + ".";

    node_id_ = static_cast<std::uint8_t>(
      node.declare_parameter<int>(p + "node_id", default_node_id));
    required_ = node.declare_parameter<bool>(p + "required", false);

    encoder_cpr_ = node.declare_parameter<int>(p + "encoder_cpr", 4096);
    gear_ratio_ = node.declare_parameter<double>(p + "gear_ratio", 0.01);
    initial_angle_ = node.declare_parameter<double>(p + "initial_angle", 0.0);
    sign_ = node.declare_parameter<double>(p + "sign", -1.0);
    c_torque_ = node.declare_parameter<double>(p + "c_torque", 0.00525);

    max_speed_rpm_ = std::abs(node.declare_parameter<int>(p + "max_speed_rpm", 3000));
    max_torque_nm_ = std::abs(node.declare_parameter<double>(p + "max_torque_nm", 0.20));
    watchdog_timeout_ms_ = node.declare_parameter<int>(p + "watchdog_timeout_ms", 500);

    torque_topic_ = node.declare_parameter<std::string>(
      p + "torque_topic", "/arm_teleop/" + name_ + "/current");
    pos_ticks_topic_ = node.declare_parameter<std::string>(
      p + "position_ticks_topic", "/arm_feedback/" + name_ + "_ticks");
    pos_deg_topic_ = node.declare_parameter<std::string>(
      p + "position_deg_topic", "/arm_feedback/" + name_ + "_deg");

    counts_per_rev_ = (4.0 * static_cast<double>(encoder_cpr_)) / gear_ratio_;
    initial_ticks_ = (initial_angle_ / 360.0) * counts_per_rev_;

    RCLCPP_INFO(logger_,
      "[%s] node_id=%u required=%d CPR=%d (qc=%d) gear=%.4f init_deg=%.1f sign=%.1f "
      "c_torque=%.5f max_torque=%.3fNm max_speed=%dRPM watchdog=%dms",
      name_.c_str(), node_id_, static_cast<int>(required_),
      encoder_cpr_, encoder_cpr_ * 4, gear_ratio_, initial_angle_, sign_,
      c_torque_, max_torque_nm_, max_speed_rpm_, watchdog_timeout_ms_);
    RCLCPP_INFO(logger_,
      "[%s] topics: sub=%s pub_ticks=%s pub_deg=%s",
      name_.c_str(), torque_topic_.c_str(), pos_ticks_topic_.c_str(), pos_deg_topic_.c_str());

    // Must be constructed BEFORE CanBus::Start() so the master routes PDOs to it
    motor_ = std::make_unique<epos4::Epos4>(bus, node_id_);
    motor_->SetMechanism(static_cast<std::uint32_t>(encoder_cpr_ * 4), gear_ratio_);
  }

  // Called AFTER CanBus::Start(). Boots the drive into CST mode or isolates it
  // if it fails and required_ == false.
  bool boot(rclcpp::Node & node)
  {
    RCLCPP_INFO(logger_, "[%s] Waiting for EPOS4 node %u to answer SDO...", name_.c_str(), node_id_);
    if (!motor_->WaitUntilReady(5000ms)) {
      return failStartup("EPOS4 did not answer SDO within timeout (check wiring, bitrate, node_id)");
    }

    for (int i = 0; i < 40 && !motor_->IsPdoActive(); ++i) {
      std::this_thread::sleep_for(50ms);
    }
    if (!motor_->IsPdoActive()) {
      return failStartup("PDOs are not active (check red_epos/bus.yml and master.dcf)");
    }

    if (motor_->IsFaulted()) {
      RCLCPP_WARN(logger_, "[%s] Drive reported fault: %s. Clearing...",
        name_.c_str(), motor_->DescribeLastError().c_str());
      if (!motor_->ClearFault(3000ms)) {
        return failStartup("Failed to clear EPOS4 fault condition");
      }
    }

    auto & rated_torque_sig = motor_->GetMotorRatedTorque().Refresh();
    if (rated_torque_sig.GetStatus() || rated_torque_sig.GetValue() == 0) {
      return failStartup(
        "Motor rated torque (0x6076) is 0 or unreadable (configure MotorConfigs in drive)");
    }
    RCLCPP_INFO(logger_, "[%s] Motor rated torque: %.4f N*m",
      name_.c_str(), rated_torque_sig.GetValue() / 1e6);

    if (!motor_->Enable(2000ms)) {
      return failStartup("Failed to enable EPOS4 power stage");
    }

    if (auto ec = motor_->EnterCyclicTorqueMode()) {
      motor_->Disable();
      return failStartup("Failed to enter Cyclic Synchronous Torque mode: " + ec.message());
    }

    // Initialize last command timestamp in the past so torque stays 0 until first message
    last_cmd_ns_.store(
      NowSteadyNs() - std::chrono::nanoseconds(10s).count(),
      std::memory_order_relaxed);
    watchdog_warned_ = true;

    pub_ticks_ = node.create_publisher<std_msgs::msg::Float64>(pos_ticks_topic_, 10);
    pub_deg_ = node.create_publisher<std_msgs::msg::Float64>(pos_deg_topic_, 10);
    sub_torque_ = node.create_subscription<std_msgs::msg::Float64>(
      torque_topic_, 10,
      std::bind(&MotorDriver::onTorqueCommand, this, std::placeholders::_1));

    active_ = true;
    RCLCPP_INFO(logger_, "[%s] EPOS4 node %u active in Cyclic Synchronous Torque mode.",
      name_.c_str(), node_id_);
    return true;
  }

  // Called every cycle (200 Hz) from the single node timer. Lock-free CAN access.
  void tick(rclcpp::Clock & clock)
  {
    if (!active_ || !motor_) {
      return;
    }

    if (!motor_->IsCyclicHealthy(50ms)) {
      motor_->StageTargetTorque(units::torque::newton_meter_t{0.0});
      const std::uint16_t emcy = motor_->GetCachedErrorCode();
      if (emcy != 0) {
        RCLCPP_ERROR_THROTTLE(logger_, clock, 1000,
          "[%s] EPOS4 node %u unhealthy (EMCY 0x%04X: %s) - torque zeroed",
          name_.c_str(), node_id_, emcy, epos4::signals::DeviceErrorName(emcy).c_str());
      } else {
        RCLCPP_ERROR_THROTTLE(logger_, clock, 1000,
          "[%s] EPOS4 node %u cyclic health failure (missing PDOs or drive left Operation Enabled) - torque zeroed",
          name_.c_str(), node_id_);
      }
      return;
    }

    // 1. Lock-free feedback from inbound TPDO cache
    const std::int32_t raw_ticks = motor_->GetCachedPosition();
    const std::int32_t vel_rpm = motor_->GetCachedVelocity();

    // 2. Convert quadcounts to output shaft ticks & degrees with initial_angle and sign
    const double output_ticks = initial_ticks_ + sign_ * static_cast<double>(raw_ticks);
    const double output_deg =
      initial_angle_ + sign_ * (static_cast<double>(raw_ticks) / counts_per_rev_) * 360.0;

    // 3. Publish position feedback
    std_msgs::msg::Float64 ticks_msg;
    ticks_msg.data = output_ticks;
    pub_ticks_->publish(ticks_msg);

    std_msgs::msg::Float64 deg_msg;
    deg_msg.data = output_deg;
    pub_deg_->publish(deg_msg);

    // 4. Safety Supervision: Watchdog & Overspeed
    double target_torque = cmd_torque_nm_.load(std::memory_order_relaxed);
    const auto elapsed_ms = (NowSteadyNs() - last_cmd_ns_.load(std::memory_order_relaxed)) / 1'000'000;

    if (elapsed_ms > watchdog_timeout_ms_) {
      target_torque = 0.0;
      if (!watchdog_warned_) {
        RCLCPP_WARN(logger_,
          "[%s] Watchdog timeout (%ld ms > %d ms): zeroing torque",
          name_.c_str(), static_cast<long>(elapsed_ms), watchdog_timeout_ms_);
        watchdog_warned_ = true;
      }
    }

    if (std::abs(vel_rpm) > max_speed_rpm_) {
      RCLCPP_WARN_THROTTLE(logger_, clock, 500,
        "[%s] Overspeed detected (%d RPM > %d RPM limit)! Cutting torque to 0.",
        name_.c_str(), vel_rpm, max_speed_rpm_);
      target_torque = 0.0;
    }

    // 5. Lock-free setpoint staging (sent on next SYNC)
    motor_->StageTargetTorque(units::torque::newton_meter_t{sign_ * target_torque});
  }

  void stageZeroTorque()
  {
    if (active_ && motor_) {
      motor_->StageTargetTorque(units::torque::newton_meter_t{0.0});
    }
  }

  void shutdown()
  {
    if (active_ && motor_) {
      if (motor_->IsCyclicModeActive()) {
        motor_->ExitCyclicMode();
      }
      motor_->Disable();
      active_ = false;
    }
  }

  bool isActive() const { return active_; }
  const std::string & getName() const { return name_; }

private:
  bool failStartup(const std::string & reason)
  {
    if (required_) {
      RCLCPP_FATAL(logger_, "[%s] (node %u): %s", name_.c_str(), node_id_, reason.c_str());
      throw std::runtime_error("[" + name_ + "] " + reason);
    }
    RCLCPP_WARN(logger_,
      "[%s] (node %u): %s -> motor INACTIVE (required=false)",
      name_.c_str(), node_id_, reason.c_str());
    active_ = false;
    return false;
  }

  void onTorqueCommand(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Translate input current (A) to torque (N*m) via torque constant c_torque
    const double torque_nm = c_torque_ * msg->data;
    const double clamped = std::clamp(torque_nm, -max_torque_nm_, max_torque_nm_);
    cmd_torque_nm_.store(clamped, std::memory_order_relaxed);
    last_cmd_ns_.store(NowSteadyNs(), std::memory_order_relaxed);
    watchdog_warned_ = false;
  }

  std::string name_;
  rclcpp::Logger logger_;

  // Per-motor parameters
  std::uint8_t node_id_{5};
  bool required_{false};
  int encoder_cpr_{4096};
  double gear_ratio_{0.01};
  double initial_angle_{0.0};
  double sign_{-1.0};
  double c_torque_{0.00525};
  int max_speed_rpm_{3000};
  double max_torque_nm_{0.20};
  int watchdog_timeout_ms_{500};

  std::string torque_topic_;
  std::string pos_ticks_topic_;
  std::string pos_deg_topic_;

  // Precomputed mechanism constants
  double counts_per_rev_{1638400.0};
  double initial_ticks_{0.0};

  // EPOS4 hardware handle
  std::unique_ptr<epos4::Epos4> motor_;
  bool active_{false};
  bool watchdog_warned_{false};

  // ROS 2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ticks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_deg_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_torque_;

  // Lock-free command state
  std::atomic<double> cmd_torque_nm_{0.0};
  std::atomic<std::int64_t> last_cmd_ns_{0};
};

// Epos4TorqueNode: owns the shared CanBus and the single 200 Hz timer
class Epos4TorqueNode : public rclcpp::Node
{
public:
  Epos4TorqueNode()
  : Node("epos4_torque_node")
  {
    const std::string default_dcf = DEFAULT_DCF;

    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    master_node_id_ = static_cast<std::uint8_t>(
      this->declare_parameter<int>("master_node_id", 1));
    dcf_path_ = this->declare_parameter<std::string>("dcf_path", "");
    if (dcf_path_.empty()) {
      dcf_path_ = default_dcf;
    }
    loop_hz_ = this->declare_parameter<double>("loop_hz", 200.0);

    const std::vector<std::string> default_motors{"joint2", "joint3"};
    const std::vector<std::string> motor_names =
      this->declare_parameter<std::vector<std::string>>("motor_names", default_motors);

    RCLCPP_INFO(this->get_logger(),
      "Initializing EPOS4 Torque Node: iface=%s, master_id=%u, loop=%.1fHz, motors=%zu",
      can_interface_.c_str(), master_node_id_, loop_hz_, motor_names.size());
    RCLCPP_INFO(this->get_logger(), "Master DCF: %s", dcf_path_.c_str());

    // 1. Create shared CANopen bus
    epos4::CanBus::Options bus_options;
    bus_options.interface = can_interface_;
    bus_options.masterDcf = dcf_path_;
    bus_options.masterNodeId = master_node_id_;

    bus_ = std::make_unique<epos4::CanBus>(bus_options);

    // 2. Construct and register all Epos4 devices BEFORE starting the bus
    //    Default node IDs: joint2 -> 5, joint3 -> 4
    for (std::size_t i = 0; i < motor_names.size(); ++i) {
      const int default_id = (i == 0) ? 5 : (i == 1 ? 4 : static_cast<int>(5 + i));
      motors_.push_back(
        std::make_unique<MotorDriver>(*this, *bus_, motor_names[i], default_id));
    }

    // 3. Start the CANopen bus and master thread
    bus_->Start();

    // 4. Boot each registered motor
    std::size_t active_count = 0;
    try {
      for (auto & m : motors_) {
        if (m->boot(*this)) {
          ++active_count;
        }
      }
    } catch (...) {
      shutdownAll();
      throw;
    }

    RCLCPP_INFO(this->get_logger(),
      "EPOS4 Torque Node started (%zu/%zu motors active).",
      active_count, motors_.size());

    // 5. Single periodic timer at loop_hz (200 Hz) driving all active motors
    const auto timer_period = std::chrono::microseconds(
      static_cast<std::int64_t>(1e6 / loop_hz_));
    control_timer_ = this->create_wall_timer(
      timer_period, std::bind(&Epos4TorqueNode::onControlCycle, this));
  }

  ~Epos4TorqueNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down EPOS4 Torque Node safely...");
    shutdownAll();
    RCLCPP_INFO(this->get_logger(), "EPOS4 node shutdown complete.");
  }

private:
  void onControlCycle()
  {
    auto & clk = *this->get_clock();
    for (auto & m : motors_) {
      m->tick(clk);
    }
  }

  void shutdownAll()
  {
    for (auto & m : motors_) {
      m->stageZeroTorque();
    }
    std::this_thread::sleep_for(25ms);

    for (auto & m : motors_) {
      m->shutdown();
    }
    motors_.clear();

    if (bus_) {
      bus_->Stop();
    }
  }

  std::string can_interface_{"can0"};
  std::uint8_t master_node_id_{1};
  std::string dcf_path_;
  double loop_hz_{200.0};

  std::unique_ptr<epos4::CanBus> bus_;
  std::vector<std::unique_ptr<MotorDriver>> motors_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<Epos4TorqueNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Fatal exception in epos4_torque_node: %s", ex.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
