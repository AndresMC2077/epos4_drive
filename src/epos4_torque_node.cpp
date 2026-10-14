// epos4_torque_node.cpp: ROS 2 Humble node for Cyclic Synchronous Torque (CST)
// control of a Maxon motor via EPOS4 and EposLib.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <ros2units/units.h>
#include "epos4/hardware/Epos4.hpp"
#include "epos4/CanBus.hpp"

using namespace std::chrono_literals;

class Epos4TorqueNode : public rclcpp::Node
{
public:
  Epos4TorqueNode()
  : Node("epos4_torque_node"),
    last_cmd_tp_(std::chrono::steady_clock::now())
  {
    // -------------------------------------------------------------------------
    // 1. Declare and retrieve ROS 2 parameters
    // -------------------------------------------------------------------------
    const std::string default_dcf = DEFAULT_DCF;

    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    node_id_ = static_cast<std::uint8_t>(this->declare_parameter<int>("node_id", 5));
    master_node_id_ = static_cast<std::uint8_t>(this->declare_parameter<int>("master_node_id", 1));
    dcf_path_ = this->declare_parameter<std::string>("dcf_path", "");
    if (dcf_path_.empty()) {
      dcf_path_ = default_dcf;
    }

    loop_hz_ = this->declare_parameter<double>("loop_hz", 200.0);
    encoder_cpr_ = this->declare_parameter<int>("encoder_cpr", 500);
    gear_ratio_ = this->declare_parameter<double>("gear_ratio", 1.0);
    initial_angle_ = this->declare_parameter<double>("initial_angle", 180.0);
    sign_ = this->declare_parameter<double>("sign", 1.0);

    max_speed_rpm_ = std::abs(this->declare_parameter<int>("max_speed_rpm", 3000));
    max_torque_nm_ = std::abs(this->declare_parameter<double>("max_torque_nm", 0.20));
    watchdog_timeout_ms_ = this->declare_parameter<int>("watchdog_timeout_ms", 500);

    torque_topic_ = this->declare_parameter<std::string>(
      "torque_topic", "/arm_teleop/joint3/torque_nm");
    pos_ticks_topic_ = this->declare_parameter<std::string>(
      "position_ticks_topic", "/arm_feedback/joint3_ticks");
    pos_deg_topic_ = this->declare_parameter<std::string>(
      "position_deg_topic", "/arm_feedback/joint3_deg");

    RCLCPP_INFO(this->get_logger(),
      "Initializing EPOS4 Torque Node: iface=%s, node_id=%u, master_id=%u, loop=%.1fHz",
      can_interface_.c_str(), node_id_, master_node_id_, loop_hz_);
    RCLCPP_INFO(this->get_logger(), "DCF file: %s", dcf_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Mechanism: CPR=%d (quadcounts=%d), gear_ratio=%.4f, initial_angle=%.1f deg, sign=%.1f",
      encoder_cpr_, encoder_cpr_ * 4, gear_ratio_, initial_angle_, sign_);
    RCLCPP_INFO(this->get_logger(),
      "Limits: max_torque=%.3f N*m, max_speed=%d RPM, watchdog=%d ms",
      max_torque_nm_, max_speed_rpm_, watchdog_timeout_ms_);
    RCLCPP_INFO(this->get_logger(),
      "Topics: sub=%s, pub_ticks=%s, pub_deg=%s",
      torque_topic_.c_str(), pos_ticks_topic_.c_str(), pos_deg_topic_.c_str());

    // -------------------------------------------------------------------------
    // 2. Setup ROS Publishers and Subscriber
    // -------------------------------------------------------------------------
    pub_ticks_ = this->create_publisher<std_msgs::msg::Float64>(pos_ticks_topic_, 10);
    pub_deg_ = this->create_publisher<std_msgs::msg::Float64>(pos_deg_topic_, 10);

    sub_torque_ = this->create_subscription<std_msgs::msg::Float64>(
      torque_topic_, 10,
      std::bind(&Epos4TorqueNode::onTorqueCommand, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // 3. Initialize CANopen master & EPOS4 driver
    // -------------------------------------------------------------------------
    epos4::CanBus::Options bus_options;
    bus_options.interface = can_interface_;
    bus_options.masterDcf = dcf_path_;
    bus_options.masterNodeId = master_node_id_;

    bus_ = std::make_unique<epos4::CanBus>(bus_options);
    motor_ = std::make_unique<epos4::Epos4>(*bus_, node_id_);

    // Configure mechanism scale: 4 quadcounts per encoder pulse
    motor_->SetMechanism(static_cast<std::uint32_t>(encoder_cpr_ * 4), gear_ratio_);

    // Devices MUST be registered with the bus before Start() is called
    bus_->Start();

    // -------------------------------------------------------------------------
    // 4. Drive boot sequence and state machine setup
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "Waiting for EPOS4 node %u to answer SDO...", node_id_);
    if (!motor_->WaitUntilReady(5000ms)) {
      bus_->Stop();
      throw std::runtime_error("EPOS4 did not answer within timeout. Check wiring, bitrate, and node ID.");
    }
    RCLCPP_INFO(this->get_logger(), "EPOS4 responded. Checking PDO communication...");

    // Wait for PDOs to be active (master must boot slave into NMT Operational)
    for (int i = 0; i < 40 && !motor_->IsPdoActive(); ++i) {
      std::this_thread::sleep_for(50ms);
    }
    if (!motor_->IsPdoActive()) {
      bus_->Stop();
      throw std::runtime_error("EPOS4 PDOs are not active. Check bus.yml and DCF configuration.");
    }

    // Clear existing fault if any
    if (motor_->IsFaulted()) {
      RCLCPP_WARN(this->get_logger(), "Drive reported fault: %s. Clearing...",
        motor_->DescribeLastError().c_str());
      if (!motor_->ClearFault(3000ms)) {
        bus_->Stop();
        throw std::runtime_error("Failed to clear EPOS4 fault condition.");
      }
    }

    // Verify Motor Rated Torque (0x6076) is properly configured
    auto & rated_torque_sig = motor_->GetMotorRatedTorque().Refresh();
    if (rated_torque_sig.GetStatus() || rated_torque_sig.GetValue() == 0) {
      bus_->Stop();
      throw std::runtime_error(
        "Motor rated torque (0x6076) is 0 or cannot be read. "
        "Configure motor current and torque constant (MotorConfigs) in drive non-volatile memory.");
    }
    RCLCPP_INFO(this->get_logger(), "Motor rated torque: %.4f N*m",
      rated_torque_sig.GetValue() / 1e6);

    // Transition drive to CiA 402 Operation Enabled
    RCLCPP_INFO(this->get_logger(), "Enabling EPOS4 drive power stage...");
    if (!motor_->Enable(2000ms)) {
      bus_->Stop();
      throw std::runtime_error("Failed to enable EPOS4 drive power stage.");
    }

    // Switch to Cyclic Synchronous Torque (CST) mode
    if (auto ec = motor_->EnterCyclicTorqueMode()) {
      motor_->Disable();
      bus_->Stop();
      throw std::runtime_error("Failed to enter Cyclic Synchronous Torque mode: " + ec.message());
    }
    RCLCPP_INFO(this->get_logger(), "EPOS4 in Cyclic Synchronous Torque mode. Ready for commands.");

    // -------------------------------------------------------------------------
    // 5. Start high-frequency periodic control timer
    // -------------------------------------------------------------------------
    const auto timer_period = std::chrono::microseconds(
      static_cast<std::int64_t>(1e6 / loop_hz_));
    control_timer_ = this->create_wall_timer(
      timer_period, std::bind(&Epos4TorqueNode::onControlCycle, this));
  }

  ~Epos4TorqueNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down EPOS4 Torque Node safely...");
    if (motor_) {
      // Stage 0 torque and allow several cycles for frames to be dispatched
      motor_->StageTargetTorque(units::torque::newton_meter_t{0.0});
      std::this_thread::sleep_for(25ms);

      if (motor_->IsCyclicModeActive()) {
        motor_->ExitCyclicMode();
      }
      motor_->Disable();
    }
    if (bus_) {
      bus_->Stop();
    }
    RCLCPP_INFO(this->get_logger(), "EPOS4 node shutdown complete.");
  }

private:
  void onTorqueCommand(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Clamp commanded torque to configured safe boundary
    const double clamped = std::clamp(msg->data, -max_torque_nm_, max_torque_nm_);
    cmd_torque_nm_.store(clamped, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(time_mutex_);
    last_cmd_tp_ = std::chrono::steady_clock::now();
  }

  void onControlCycle()
  {
    if (!motor_ || !motor_->IsCyclicHealthy(50ms)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "EPOS4 cyclic health failure (missing PDOs or drive faulted)!");
      if (motor_ && motor_->IsFaulted()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Drive fault: %s", motor_->DescribeLastError().c_str());
      }
      return;
    }

    // 1. Lock-free feedback acquisition from inbound PDO cache
    const std::int32_t raw_ticks = motor_->GetCachedPosition();
    const std::int32_t vel_rpm = motor_->GetCachedVelocity();

    // 2. Counts per revolution at output shaft
    const double counts_per_rev = (4.0 * encoder_cpr_) / gear_ratio_;

    // 3. Compute output angle and ticks with initial_angle and sign
    const double initial_ticks = (initial_angle_ / 360.0) * counts_per_rev;
    const double output_ticks = initial_ticks + sign_ * static_cast<double>(raw_ticks);
    const double output_deg = initial_angle_ + sign_ * (static_cast<double>(raw_ticks) / counts_per_rev) * 360.0;

    // 4. Publish position in ticks (Float64 for compatibility with bus_can_drive2)
    auto ticks_msg = std_msgs::msg::Float64();
    ticks_msg.data = output_ticks;
    pub_ticks_->publish(ticks_msg);

    // 5. Publish position in degrees (Float64)
    auto deg_msg = std_msgs::msg::Float64();
    deg_msg.data = output_deg;
    pub_deg_->publish(deg_msg);

    // 6. Safety Supervision
    double target_torque = cmd_torque_nm_.load(std::memory_order_relaxed);

    // Watchdog check: Zero torque if commands stop arriving
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_cmd_tp_).count();
      if (elapsed > watchdog_timeout_ms_) {
        target_torque = 0.0;
      }
    }

    // Overspeed check: Cut torque to 0 immediately if motor exceeds speed limit
    if (std::abs(vel_rpm) > max_speed_rpm_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Overspeed detected (%d RPM > %d RPM limit)! Cutting torque to 0.",
        vel_rpm, max_speed_rpm_);
      target_torque = 0.0;
    }

    // 7. Lock-free setpoint staging (dispatched on the next SYNC pulse)
    // sign_ ensures commanded torque direction aligns with joint angle direction
    motor_->StageTargetTorque(units::torque::newton_meter_t{sign_ * target_torque});
  }

  // Parameters
  std::string can_interface_{"can0"};
  std::uint8_t node_id_{5};
  std::uint8_t master_node_id_{1};
  std::string dcf_path_;
  double loop_hz_{200.0};
  int encoder_cpr_{500};
  double gear_ratio_{1.0};
  double initial_angle_{180.0};
  double sign_{1.0};
  int max_speed_rpm_{3000};
  double max_torque_nm_{0.20};
  int watchdog_timeout_ms_{500};

  std::string torque_topic_;
  std::string pos_ticks_topic_;
  std::string pos_deg_topic_;

  // CAN & Motor instances
  std::unique_ptr<epos4::CanBus> bus_;
  std::unique_ptr<epos4::Epos4> motor_;

  // ROS 2 Interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ticks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_deg_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_torque_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  std::atomic<double> cmd_torque_nm_{0.0};
  std::mutex time_mutex_;
  std::chrono::steady_clock::time_point last_cmd_tp_;
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

