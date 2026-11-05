// Gazebo 的“电机层”：策略只以 50 Hz 更新目标，本控制器从 ros2_control
// 的实时关节状态计算 PD 力矩。不能用 50 Hz 的 ROS 关节消息闭合电机环。
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hexapod_control
{
constexpr size_t N = 18;
struct MotorCommand
{
  // 行优先的 5x18 数据：[目标位置, 目标速度, kp, kd, 前馈力矩]。
  std::array<double, 5 * N> data{};
  double stamp{-1.0};
};

class PdController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("effort_limit", 30.0);
    auto_declare<double>("command_timeout", 0.25);
    auto_declare<double>("damping_kd", 11.0);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joints_) cfg.names.push_back(joint + "/effort");
    return cfg;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joints_) {
      cfg.names.push_back(joint + "/position");
      cfg.names.push_back(joint + "/velocity");
    }
    return cfg;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    limit_ = get_node()->get_parameter("effort_limit").as_double();
    timeout_ = get_node()->get_parameter("command_timeout").as_double();
    damping_ = get_node()->get_parameter("damping_kd").as_double();
    if (joints_.size() != N || !std::isfinite(limit_) || limit_ <= 0 ||
      !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(damping_) || damping_ < 0)
      return controller_interface::CallbackReturn::ERROR;
    subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() != 5 * N) return;
        MotorCommand command;
        for (size_t i = 0; i < 5 * N; ++i) {
          if (!std::isfinite(msg->data[i])) return;
          command.data[i] = msg->data[i];
        }
        for (size_t i = 0; i < N; ++i) {
          if (command.data[2*N+i] < 0 || command.data[2*N+i] > 300 ||
            command.data[3*N+i] < 0 || command.data[3*N+i] > 30) return;
        }
        command.stamp = get_node()->now().seconds();
        buffer_.writeFromNonRT(command);
      });
    // 遥测预先分配内存；实时环内 trylock，发布线程繁忙时跳过这一帧。
    state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      get_node()->create_publisher<sensor_msgs::msg::JointState>("~/state", 10));
    state_publisher_->msg_.name = joints_;
    state_publisher_->msg_.position.resize(N);
    state_publisher_->msg_.velocity.resize(N);
    state_publisher_->msg_.effort.resize(N);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 不能假设硬件借出的接口次序：按完整 joint/interface 名称建立索引。
    for (size_t j = 0; j < N; ++j) {
      command_index_[j] = find_interface(command_interfaces_, joints_[j] + "/effort");
      position_index_[j] = find_interface(state_interfaces_, joints_[j] + "/position");
      velocity_index_[j] = find_interface(state_interfaces_, joints_[j] + "/velocity");
      if (command_index_[j] < 0 || position_index_[j] < 0 || velocity_index_[j] < 0)
        return controller_interface::CallbackReturn::ERROR;
    }
    // 启动握手期间暂时保持当前位置；Python 端随后接管状态机。
    MotorCommand initial;
    initial.stamp = get_node()->now().seconds();
    for (size_t j = 0; j < N; ++j) {
      initial.data[j] = state_interfaces_[position_index_[j]].get_value();
      initial.data[2*N+j] = 150.0;
      initial.data[3*N+j] = 11.0;
    }
    buffer_.writeFromNonRT(initial);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & interface : command_interfaces_) interface.set_value(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    const auto * command = buffer_.readFromRT();
    const double age = time.seconds() - command->stamp;
    // 暂停不会消耗仿真时间；命令超时或时间戳异常时进入阻尼，避免旧目标残留。
    const bool stale = command->stamp < 0 || age < -0.01 || age > timeout_;
    const bool publish = (time.seconds()-last_publish_ >= 0.01 || time.seconds() < last_publish_) &&
      state_publisher_->trylock();
    for (size_t j = 0; j < N; ++j) {
      const double q = state_interfaces_[position_index_[j]].get_value();
      const double dq = state_interfaces_[velocity_index_[j]].get_value();
      double torque = 0.0;
      if (std::isfinite(q) && std::isfinite(dq)) {
        torque = stale ? -damping_ * dq :
          command->data[2*N+j] * (command->data[j] - q) +
          command->data[3*N+j] * (command->data[N+j] - dq) + command->data[4*N+j];
      }
      torque = std::clamp(torque, -limit_, limit_);
      command_interfaces_[command_index_[j]].set_value(torque);
      if (publish) {
        state_publisher_->msg_.position[j] = q;
        state_publisher_->msg_.velocity[j] = dq;
        state_publisher_->msg_.effort[j] = torque;
      }
    }
    if (publish) {
      state_publisher_->msg_.header.stamp = time;
      state_publisher_->unlockAndPublish();
      last_publish_ = time.seconds();
    }
    return controller_interface::return_type::OK;
  }

private:
  template<class Interfaces>
  int find_interface(const Interfaces & interfaces, const std::string & name)
  {
    for (size_t i = 0; i < interfaces.size(); ++i)
      if (interfaces[i].get_name() == name) return static_cast<int>(i);
    return -1;
  }
  std::vector<std::string> joints_;
  std::array<int, N> command_index_{}, position_index_{}, velocity_index_{};
  double limit_{30.0}, timeout_{0.25}, damping_{11.0};
  double last_publish_{-1.0};
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> state_publisher_;
  realtime_tools::RealtimeBuffer<MotorCommand> buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
};
}  // namespace hexapod_control
PLUGINLIB_EXPORT_CLASS(hexapod_control::PdController, controller_interface::ControllerInterface)
