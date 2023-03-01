#include "path_tracer.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace path_tracer {

    PathTracer::PathTracer()
        : Node("path_tracer")
        , odom_subscription_(this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&PathTracer::odomCallback, this, _1)))
        , path_publisher_(this->create_publisher<nav_msgs::msg::Path>("~/cfr_path", 10))
        , markers_publisher_(
          this->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker", 10))
        , timer_(
          this->create_wall_timer(500ms, std::bind(&PathTracer::timerCallback, this)))
        , tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock()))
        , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        this->constructMarker(trace_marker_);
    }

    void PathTracer::timerCallback()
    {
        geometry_msgs::msg::PoseStamped pose_stamped{};
        path_message_.header = odom_.header;
        pose_stamped.pose = odom_.pose.pose;
        path_message_.poses.emplace_back(pose_stamped);
        path_publisher_->publish(path_message_);

        geometry_msgs::msg::TransformStamped transformStamped;

        try {
            transformStamped =
            tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
        }

        catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform %s from %s: %s",
                        target_frame_.c_str(), source_frame_.c_str(), ex.what());
            return;
        }

        trace_marker_.id = id_counter_++;

        trace_marker_.pose = odom_.pose.pose;
        trace_marker_.pose.position.z = -0.02;

        trace_markers_.markers.push_back(trace_marker_);
        markers_publisher_->publish(trace_markers_);
    }

    void PathTracer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_.header = msg->header;
        odom_.pose = msg->pose;
    }

    void PathTracer::constructMarker(visualization_msgs::msg::Marker& marker)
    {
        rclcpp::Clock clock;

        marker.header.frame_id = source_frame_;
        marker.header.stamp = rclcpp::Time();
        marker.ns = "path";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = id_counter_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = 0.8;
        marker.scale.y = 1.6;
        marker.scale.z = 0.01;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.r = 0.0f;
        marker.color.a = 0.5f;
        marker.lifetime = rclcpp::Duration(0);
    }

} // namespace path_tracer