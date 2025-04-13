#include "ackermann_controller.hpp"

namespace pet
{

AckermannController::AckermannController(const double timer_period, const double timeout_duration) :
  Node{"ackermann_controller"},
  m_timeout_duration{timeout_duration},
  m_last_velocity_time{get_clock()->now()},
  m_last_steering_time{get_clock()->now()},
  m_body_width{0.0},
  m_body_length{0.0},
  m_wheel_radius{0.0},
  m_wheel_width{0.0},
  m_max_steering_angle{0.0},
  m_max_velocity{0.0},
  m_wheel_base{0.0},
  m_track_width{0.0},
  m_steering_angle{0.0},
  m_velocity{0.0},
  m_wheel_angular_velocity{0.0, 0.0},
  m_wheel_steering_angle{0.0, 0.0}
{
  // Declare the used parameters
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("body_width", m_body_width);
  get_parameter("body_length", m_body_length);
  get_parameter("wheel_radius", m_wheel_radius);
  get_parameter("wheel_width", m_wheel_width);
  get_parameter("max_steering_angle", m_max_steering_angle);
  get_parameter("max_velocity", m_max_velocity);

  // Set the track width and wheel base
  m_track_width = m_body_width + (2 * m_wheel_width / 2);
  m_wheel_base = m_body_length - (2 * m_wheel_radius);

  // Subscribers
  m_steering_angle_subscriber = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&AckermannController::steering_angle_callback, this, std::placeholders::_1));

    m_velocity_subscriber = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10, std::bind(&AckermannController::velocity_callback, this, std::placeholders::_1));

  // Publishers
  m_position_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

    m_velocity_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  // Timer loop
  m_timer = create_wall_timer(std::chrono::duration<double>(timer_period),
                              std::bind(&AckermannController::timer_callback, this));
}

std::pair<double, double> AckermannController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  // Steering angle is not zero nor too small
  if (abs(m_steering_angle) > 1e-3) {
    const double sin_angle = sin(abs(m_steering_angle));
    const double cos_angle = cos(abs(m_steering_angle));

    if (m_steering_angle > 0.0) {
      // Left and right wheel angles when steering left
      left_wheel_angle = atan((2 * m_wheel_base * sin_angle) /
                              (2 * m_wheel_base * cos_angle - m_track_width * sin_angle));

      right_wheel_angle = atan((2 * m_wheel_base * sin_angle) /
                               (2 * m_wheel_base * cos_angle + m_track_width * sin_angle));
    } else {
      // Left and right wheel angles when steering right (mirror left with negative signs)
      left_wheel_angle = -atan((2 * m_wheel_base * sin_angle) /
                               (2 * m_wheel_base * cos_angle + m_track_width * sin_angle));

      right_wheel_angle = -atan((2 * m_wheel_base * sin_angle) /
                                (2 * m_wheel_base * cos_angle - m_track_width * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> AckermannController::rear_differential_velocity()
{
  double left_wheel_velocity{m_velocity};
  double right_wheel_velocity{m_velocity};

  // Steering angle is not zero nor too small
  if (abs(m_steering_angle) > 1e-3) {
    // Calculate turning radius and angular velocity
    const double turning_radius = m_wheel_base / tan(abs(m_steering_angle));  // [m]
    const double vehicle_angular_velocity = m_velocity / turning_radius;     // [rad/s]

    // Compute inner and outer wheel radius
    const double inner_radius = turning_radius - (m_track_width / 2.0);
    const double outer_radius = turning_radius + (m_track_width / 2.0);

    if (m_steering_angle > 0.0) {
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
    if (max_wheel_velocity > m_max_velocity) {
      const double scaling_factor = m_max_velocity / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void AckermannController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed_time{(current_time - m_last_velocity_time).nanoseconds()};
  const auto steering_elapsed_time{(current_time - m_last_steering_time).nanoseconds()};

  // Reset velocity to zero if timeout
  if (velocity_elapsed_time > m_timeout_duration) {
    m_wheel_angular_velocity = {0.0, 0.0};
  }

  // Reset steering angle to zero if timeout
  if (steering_elapsed_time > m_timeout_duration) {
    m_wheel_steering_angle = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = m_wheel_steering_angle;
  m_position_publisher->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = m_wheel_angular_velocity;
  m_velocity_publisher->publish(velocity_msg);
}

void AckermannController::steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  m_last_steering_time = get_clock()->now();  // Update timestamp

  if (msg->data > m_max_steering_angle) {
    m_steering_angle = m_max_steering_angle;
  } else if (msg->data < -m_max_steering_angle) {
    m_steering_angle = -m_max_steering_angle;
  } else {
    m_steering_angle = msg->data;
  }

  const auto wheel_angles{ackermann_steering_angle()};

  m_wheel_steering_angle = {wheel_angles.first, wheel_angles.second};
}

void AckermannController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  m_last_velocity_time = get_clock()->now();  // Update timestamp

  if (msg->data > m_max_velocity) {
    m_velocity = m_max_velocity;
  } else if (msg->data < -m_max_velocity) {
    m_velocity = -m_max_velocity;
  } else {
    m_velocity = msg->data;
  }

  const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  m_wheel_angular_velocity = {(wheel_velocity.first / m_wheel_radius),
                             (wheel_velocity.second / m_wheel_radius)};
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannController>());
  rclcpp::shutdown();
  return 0;
}

} // namespace pet
