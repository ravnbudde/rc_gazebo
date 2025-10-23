#include "vehicle_controller.hpp"
#include <iostream>

VehicleController::VehicleController(const double timer_period, const double timeout_duration) :
  Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  wheel_radius_{0.067/2},
  max_steering_angle_{0.5},
  max_velocity_{0.6},
  wheel_base_{0.145},
  track_width_{0.133},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  // Subscribers
  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10, std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));

  // Publishers
  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    const double sin_angle = sin(abs(steering_angle_));
    const double cos_angle = cos(abs(steering_angle_));

    if (steering_angle_ > 0.0) {
      // Left and right wheel angles when steering left
      left_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                              (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));

      right_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      // Left and right wheel angles when steering right (mirror left with negative signs)
      left_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));

      right_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                                (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    // Calculate turning radius and angular velocity
    const double turning_radius = wheel_base_ / tan(abs(steering_angle_));  // [m]
    const double vehicle_angular_velocity = velocity_ / turning_radius;     // [rad/s]

    // Compute inner and outer wheel radius
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0) {
      // Vehicle turning left
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    } else {
      // Vehicle turning right
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    // Determine the maximum wheel velocity
    const double max_wheel_velocity = std::max(abs(left_wheel_velocity), 
                                               abs(right_wheel_velocity));

    // Scale both wheel velocities proportionally if the maximum is exceeded
    if (max_wheel_velocity > max_velocity_) {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed_time{(current_time - last_velocity_time_).nanoseconds()};
  const auto steering_elapsed_time{(current_time - last_steering_time_).nanoseconds()};

  // std::cout << "[Timer] wheel_velocity=[" 
  //         << wheel_angular_velocity_[0] << ", " 
  //         << wheel_angular_velocity_[1] << "], "
  //         << "wheel_steering=[" 
  //         << wheel_steering_angle_[0] << ", " 
  //         << wheel_steering_angle_[1] << "]"
  //         << std::endl;

  // Reset velocity to zero if timeout
  if (velocity_elapsed_time > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }

  // Reset steering angle to zero if timeout
  if (steering_elapsed_time > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_steering_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (msg->data < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  } else {
    steering_angle_ = msg->data;
  }

  // std::cout << "[Steering callback] msg=" << msg->data 
  //             << " steering_angle_=" << steering_angle_
  //             << " max_steering_angle_=" << max_steering_angle_ 
  //             << std::endl;

  const auto wheel_angles{ackermann_steering_angle()};

  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_velocity_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->data < -max_velocity_) {
    velocity_ = -max_velocity_;
  } else {
    velocity_ = msg->data;
  }

  // std::cout << "[Velocity callback] msg=" << msg->data 
  //             << " velocity_=" << velocity_ 
  //             << " wheel_radius_=" << wheel_radius_ 
  //             << std::endl;

  const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  wheel_angular_velocity_ = {(wheel_velocity.first / wheel_radius_),
                             (wheel_velocity.second / wheel_radius_)};
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}
