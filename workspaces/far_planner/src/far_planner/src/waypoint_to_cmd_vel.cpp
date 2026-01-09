/*
 * Waypoint to Cmd_Vel Converter
 * Subscribes to /way_point from far_planner and publishes /cmd_vel
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>

class WaypointToCmdVel : public rclcpp::Node
{
public:
  WaypointToCmdVel() : Node("waypoint_to_cmd_vel")
  {
    // Declare parameters
    this->declare_parameter("max_linear_speed", 0.8);
    this->declare_parameter("min_linear_speed", 0.15);
    this->declare_parameter("max_angular_speed", 1.5);
    this->declare_parameter("min_angular_speed", 0.2);
    this->declare_parameter("linear_kp", 0.8);
    this->declare_parameter("angular_kp", 3.0);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("angle_tolerance", 0.5);  // More relaxed - ~28 degrees
    this->declare_parameter("rotate_in_place_threshold", 1.57);  // ~90 degrees
    this->declare_parameter("stuck_timeout", 3.0);
    this->declare_parameter("stuck_distance_threshold", 0.1);
    
    // Get parameters
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    min_linear_speed_ = this->get_parameter("min_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    min_angular_speed_ = this->get_parameter("min_angular_speed").as_double();
    linear_kp_ = this->get_parameter("linear_kp").as_double();
    angular_kp_ = this->get_parameter("angular_kp").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    rotate_in_place_threshold_ = this->get_parameter("rotate_in_place_threshold").as_double();
    stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
    stuck_distance_threshold_ = this->get_parameter("stuck_distance_threshold").as_double();
    
    // Subscribers
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point", 10, 
      std::bind(&WaypointToCmdVel::waypointCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&WaypointToCmdVel::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer for control loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // Faster update rate
      std::bind(&WaypointToCmdVel::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Waypoint to Cmd_Vel node initialized");
    RCLCPP_INFO(this->get_logger(), "  Max linear: %.2f m/s, Min linear: %.2f m/s", 
                max_linear_speed_, min_linear_speed_);
    RCLCPP_INFO(this->get_logger(), "  Max angular: %.2f rad/s, Min angular: %.2f rad/s", 
                max_angular_speed_, min_angular_speed_);
    
    has_waypoint_ = false;
    has_odom_ = false;
    stuck_start_time_ = this->now();
    last_progress_time_ = this->now();
    consecutive_stuck_count_ = 0;
  }

private:
  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    double new_x = msg->point.x;
    double new_y = msg->point.y;
    
    // Check if this is a new waypoint
    if (!has_waypoint_ || 
        std::abs(new_x - target_x_) > 0.01 || 
        std::abs(new_y - target_y_) > 0.01) {
      target_x_ = new_x;
      target_y_ = new_y;
      waypoint_time_ = this->now();
      last_progress_time_ = this->now();
      consecutive_stuck_count_ = 0;
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "New waypoint: (%.2f, %.2f)", target_x_, target_y_);
    }
    has_waypoint_ = true;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double prev_x = robot_x_;
    double prev_y = robot_y_;
    
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_yaw_);
    
    // Track position history for stuck detection
    if (has_odom_) {
      double moved = std::sqrt(std::pow(robot_x_ - prev_x, 2) + 
                               std::pow(robot_y_ - prev_y, 2));
      position_history_.push_back(moved);
      if (position_history_.size() > 20) {  // Keep last 20 samples (1 second at 50Hz)
        position_history_.pop_front();
      }
    }
    
    has_odom_ = true;
  }
  
  bool isStuck()
  {
    if (position_history_.size() < 20) return false;  // Need full history
    
    // Calculate total movement in last second
    double total_movement = 0.0;
    for (double dist : position_history_) {
      total_movement += dist;
    }
    
    // More lenient threshold - consider stuck only if truly not moving
    return total_movement < (stuck_distance_threshold_ * 0.5);  // Half the threshold
  }
  
  void controlLoop()
  {
    if (!has_waypoint_ || !has_odom_) {
      publishZeroVelocity();
      return;
    }
    
    // Check if waypoint is too old (safety feature) - DISABLED for continuous operation
    // The planner continuously publishes waypoints, so we don't timeout
    auto time_diff = (this->now() - waypoint_time_).seconds();
    if (time_diff > 10.0) {  // Very long timeout - only for safety
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waypoint is too old (%.1fs), but continuing", time_diff);
      // Don't stop - let the planner handle this
      // publishZeroVelocity();
      // return;
    }
    
    // Calculate distance and angle to target
    double dx = target_x_ - robot_x_;
    double dy = target_y_ - robot_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_angle = std::atan2(dy, dx);
    double angle_diff = normalizeAngle(target_angle - robot_yaw_);
    
    // Check if we've reached the goal - DON'T STOP, let planner handle waypoint updates
    if (distance < goal_tolerance_) {
      // Publish very slow velocity to stay near waypoint until new one arrives
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.05;  // Very slow creep forward
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Near waypoint (dist: %.2fm), waiting for next waypoint", distance);
      return;
    }
    
    // Detect if stuck - but be less aggressive
    if (isStuck() && distance > goal_tolerance_ * 2.0) {  // Only check if not near goal
      auto stuck_duration = (this->now() - last_progress_time_).seconds();
      if (stuck_duration > stuck_timeout_ * 2.0) {  // Double the timeout
        consecutive_stuck_count_++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Robot appears stuck! Distance: %.2fm, Angle error: %.2f deg (count: %d)",
                             distance, angle_diff * 180.0 / M_PI, consecutive_stuck_count_);
        
        // Try recovery - push forward with minimum speed regardless of angle
        if (consecutive_stuck_count_ < 20) {  // More recovery attempts
          geometry_msgs::msg::Twist recovery_cmd;
          recovery_cmd.linear.x = min_linear_speed_ * 1.5;  // Push harder
          recovery_cmd.angular.z = std::copysign(min_angular_speed_ * 1.2, angle_diff);
          cmd_vel_pub_->publish(recovery_cmd);
          return;
        } else {
          // After many attempts, reset counter and keep trying
          consecutive_stuck_count_ = 0;
          RCLCPP_WARN(this->get_logger(), "Resetting stuck counter, continuing navigation");
        }
      }
    } else {
      last_progress_time_ = this->now();
      consecutive_stuck_count_ = 0;
    }
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Adaptive control based on angle error
    if (std::abs(angle_diff) > rotate_in_place_threshold_) {
      // Large angle error - rotate in place
      cmd_vel.linear.x = 0.0;
      double angular_cmd = angular_kp_ * angle_diff;
      // Apply minimum rotation speed to overcome friction
      if (std::abs(angular_cmd) < min_angular_speed_) {
        angular_cmd = std::copysign(min_angular_speed_, angular_cmd);
      }
      cmd_vel.angular.z = angular_cmd;
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Rotating in place: angle_error=%.1f deg", 
                           angle_diff * 180.0 / M_PI);
    } 
    else if (std::abs(angle_diff) > angle_tolerance_) {
      // Moderate angle error - slow forward motion with correction
      double speed_factor = 1.0 - (std::abs(angle_diff) / rotate_in_place_threshold_);
      speed_factor = std::max(0.3, speed_factor);  // Minimum 30% speed
      
      cmd_vel.linear.x = std::min(linear_kp_ * distance * speed_factor, 
                                  max_linear_speed_ * speed_factor);
      cmd_vel.angular.z = angular_kp_ * angle_diff;
      
      // Apply minimum speeds
      if (std::abs(cmd_vel.linear.x) < min_linear_speed_) {
        cmd_vel.linear.x = std::copysign(min_linear_speed_, cmd_vel.linear.x);
      }
    } 
    else {
      // Small angle error - full speed ahead with minor corrections
      cmd_vel.linear.x = std::min(linear_kp_ * distance, max_linear_speed_);
      cmd_vel.angular.z = angular_kp_ * angle_diff * 0.5;  // Gentler turning
      
      // Apply minimum linear speed
      if (std::abs(cmd_vel.linear.x) < min_linear_speed_) {
        cmd_vel.linear.x = std::copysign(min_linear_speed_, cmd_vel.linear.x);
      }
    }
    
    // Clamp velocities to max limits
    cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -max_linear_speed_, max_linear_speed_);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_speed_, max_angular_speed_);
    
    // Debug output
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Navigating: Dist=%.2fm, Angle=%.1fdeg, LinVel=%.2fm/s, AngVel=%.2frad/s",
                          distance, angle_diff * 180.0 / M_PI, 
                          cmd_vel.linear.x, cmd_vel.angular.z);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "Distance: %.2fm, Angle: %.1fdeg, Linear: %.2fm/s, Angular: %.2frad/s",
                          distance, angle_diff * 180.0 / M_PI, 
                          cmd_vel.linear.x, cmd_vel.angular.z);
    
    cmd_vel_pub_->publish(cmd_vel);
  }
  
  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
  }
  
  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
  
  // Subscribers and Publishers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  double max_linear_speed_;
  double min_linear_speed_;
  double max_angular_speed_;
  double min_angular_speed_;
  double linear_kp_;
  double angular_kp_;
  double goal_tolerance_;
  double angle_tolerance_;
  double rotate_in_place_threshold_;
  double stuck_timeout_;
  double stuck_distance_threshold_;
  
  // State variables
  double robot_x_, robot_y_, robot_yaw_;
  double target_x_, target_y_;
  bool has_waypoint_;
  bool has_odom_;
  rclcpp::Time waypoint_time_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time stuck_start_time_;
  int consecutive_stuck_count_;
  std::deque<double> position_history_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
