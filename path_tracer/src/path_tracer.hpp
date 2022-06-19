#ifndef __PATH_TRACER_HPP__
#define __PATH_TRACER_HPP__

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class PathTracer : public rclcpp::Node
{
  public:
    PathTracer();
    ~PathTracer();
    
  private:
    void timerCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void constructMarker(visualization_msgs::msg::Marker& marker);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Path path_message_ = nav_msgs::msg::Path();
    visualization_msgs::msg::Marker trace_marker_;
    visualization_msgs::msg::MarkerArray trace_markers_;
    int32_t id_counter_ {0};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_ {"base_footprint"};
    std::string source_frame_ {"odom"};

};

#endif