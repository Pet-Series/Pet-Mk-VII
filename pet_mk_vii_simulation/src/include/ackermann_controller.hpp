#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <utility>
#include <vector>

namespace pet
{

class AckermannController : public rclcpp::Node
{

public:
  AckermannController(const double timer_period = 1e-2, const double timeout_duration = 1e9);

private:
  /**
   * @brief Calculates the steering angles for the left and right wheels using the 
   * Ackermann steering geometry.
   * 
   * @details This function computes the angles for the left and right front wheels based on the 
   * input steering angle and the vehicle's dimensions:
   * - `m_wheel_base` (distance between front and rear axles).
   * - `m_track_width` (distance between the left and right wheels).
   * 
   * @return A pair of left and right steering angles [rad].
   */
  std::pair<double, double> ackermann_steering_angle();

  /**
   * @brief Calculates the rear differential wheel velocities for a vehicle with a rear 
   * differential axle, considering the steering angle and limiting the velocities to a maximum 
   * allowable value.
   * 
   * @details This function computes rear wheel velocities based on Ackermann steering geometry:
   * - Both wheels have the same velocity for straight-line motion.
   * - For turning, it computes individual wheel velocities based on the turning radius.
   * - If any wheel velocity exceeds the maximum velocity, both are scaled proportionally.
   * 
   * @return A pair of left and right wheel velocities [m/s].
   */
  std::pair<double, double> rear_differential_velocity();

  /**
   * @brief Periodically checks for timeout and publishes steering angles and wheel velocities.
   * 
   * @details This timer callback checks if a timeout has occurred since the last received 
   * message for desired steering angle or velocity. If a timeout is detected, it resets 
   * the wheel steering angles and angular velocities to zero. The function also 
   * publishes the current steering positions and wheel velocities to the appropriate topics.
   */
  void timer_callback();

  /**
   * @brief Callback function for receiving and handling desired steering angle.
   * 
   * @details This callback updates the steering angle based on the input message and clamps 
   * the value to the maximum allowable steering angle. The function calculates the 
   * corresponding Ackermann steering angles for the left and right wheels and updates 
   * the internal state of the controller.
   * 
   * @param msg A shared pointer to the incoming message containing the steering angle [rad].
   */
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback function for receiving and handling desired velocity.
   * 
   * @details This callback updates the vehicle velocity based on the input message and clamps 
   * the value to the maximum allowable velocity. The function calculates the wheel 
   * velocities based on the rear differential model and updates the internal state 
   * with the corresponding angular velocities of the wheels.
   * 
   * @param msg A shared pointer to the incoming message containing the velocity [m/s].
   */
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double m_timeout_duration;
  rclcpp::Time m_last_velocity_time;
  rclcpp::Time m_last_steering_time;

  double m_body_width;
  double m_body_length;
  double m_wheel_radius;
  double m_wheel_width;
  double m_max_steering_angle;
  double m_max_velocity;
  double m_wheel_base;
  double m_track_width;

  double m_steering_angle;
  double m_velocity;

  std::vector<double> m_wheel_angular_velocity;
  std::vector<double> m_wheel_steering_angle;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_steering_angle_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_velocity_subscriber;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_position_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_velocity_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
};

} // namespace pet
