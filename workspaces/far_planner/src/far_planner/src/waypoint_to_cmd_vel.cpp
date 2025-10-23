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

class WaypointToCmdVel : public rclcpp::Node
{
public:
  WaypointToCmdVel() : Node("waypoint_to_cmd_vel")
  {
    // Declare parameters
    this->declare_parameter("max_linear_speed", 0.5);
    this->declare_parameter("max_angular_speed", 1.0);
    this->declare_parameter("linear_kp", 1.0);
    this->declare_parameter("angular_kp", 2.0);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("angle_tolerance", 0.1);
    
    // Get parameters
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    linear_kp_ = this->get_parameter("linear_kp").as_double();
    angular_kp_ = this->get_parameter("angular_kp").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    
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
      std::chrono::milliseconds(100),
      std::bind(&WaypointToCmdVel::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Waypoint to Cmd_Vel node initialized");
    
    has_waypoint_ = false;
    has_odom_ = false;
  }

private:
  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    target_x_ = msg->point.x;
    target_y_ = msg->point.y;
    has_waypoint_ = true;
    waypoint_time_ = this->now();
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
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
    
    has_odom_ = true;
  }
  
  void controlLoop()
  {
    if (!has_waypoint_ || !has_odom_) {
      publishZeroVelocity();
      return;
    }
    
    // Check if waypoint is too old (safety feature)
    auto time_diff = (this->now() - waypoint_time_).seconds();
    if (time_diff > 2.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waypoint is too old, stopping robot");
      publishZeroVelocity();
      return;
    }
    
    // Calculate distance and angle to target
    double dx = target_x_ - robot_x_;
    double dy = target_y_ - robot_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_angle = std::atan2(dy, dx);
    double angle_diff = normalizeAngle(target_angle - robot_yaw_);
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Check if we've reached the goal
    if (distance < goal_tolerance_) {
      publishZeroVelocity();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Reached waypoint");
      return;
    }
    
    // If angle error is large, rotate in place first
    if (std::abs(angle_diff) > angle_tolerance_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angular_kp_ * angle_diff;
    } else {
      // Move forward and adjust heading
      cmd_vel.linear.x = std::min(linear_kp_ * distance, max_linear_speed_);
      cmd_vel.angular.z = angular_kp_ * angle_diff;
    }
    
    // Clamp velocities
    cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -max_linear_speed_, max_linear_speed_);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_speed_, max_angular_speed_);
    
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
  double max_angular_speed_;
  double linear_kp_;
  double angular_kp_;
  double goal_tolerance_;
  double angle_tolerance_;
  
  // State variables
  double robot_x_, robot_y_, robot_yaw_;
  double target_x_, target_y_;
  bool has_waypoint_;
  bool has_odom_;
  rclcpp::Time waypoint_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
