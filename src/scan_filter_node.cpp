#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <cmath>

using namespace std;

class DynamicScanFilter : public rclcpp::Node
{
public:
  DynamicScanFilter()
  : Node("dynamic_scan_filter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter("person_radius", 0.8);
    // setting inf makes the laser beams which is falling on the model(person standing) radius in the gazbeo become free space
    this->declare_parameter("filter_mode", "inf");  

    this->get_parameter("person_radius", person_radius_);
    this->get_parameter("filter_mode", filter_mode_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DynamicScanFilter::scanCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/person_pose_info_with_frame", 10, std::bind(&DynamicScanFilter::poseCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/scan_filter_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Dynamic scan filter running. Radius=%.2f, Mode=%s", person_radius_, filter_mode_.c_str());
  }

private:
  /*void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    geometry_msgs::msg::PoseStamped person_pose_world, person_pose_lidar;
    person_pose_world.header = msg->header;
    person_pose_world.pose = msg->poses[0];

    person_pose_world.pose.position.x += 5.6;     // offset is provided, to mark exactly where the model is in the world

    try {
      // Transform adjusted global pose to lidar frame for filtering
      person_pose_lidar = tf_buffer_.transform(person_pose_world, "lidar_link", tf2::durationFromSec(0.2));

      person_x_ = person_pose_lidar.pose.position.x;
      person_y_ = person_pose_lidar.pose.position.y;
      person_valid_ = true;

      publishMarker(person_pose_world.header, person_pose_world.pose.position.x, person_pose_world.pose.position.y);
    }
    catch (tf2::TransformException &ex) {
      // RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      person_valid_ = false;
    }
  }*/
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      person_valid_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped person_pose_world, person_pose_lidar;
    person_pose_world.header = msg->header;   
    person_pose_world.pose = msg->poses[0];
    person_pose_world.pose.position.x += 5.6;  // offset is provided, to mark exactly where the model is in the world

    bool transformed = false;

    try {
      person_pose_lidar = tf_buffer_.transform(person_pose_world, "lidar_link", tf2::durationFromSec(0.2));
      transformed = true;
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "TF (with msg stamp) transform failed: %s — will try latest transform", ex.what());
    }

    if (!transformed) {
      try {
        auto person_pose_latest = person_pose_world;
        person_pose_latest.header.stamp = rclcpp::Time(0); 
        person_pose_lidar = tf_buffer_.transform(person_pose_latest, "lidar_link", tf2::durationFromSec(0.2));
        transformed = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF: used latest transform fallback");
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform (latest) failed too: %s", ex.what());
        person_valid_ = false;
        return;
      }
    }

    if (transformed) {
      person_x_ = person_pose_lidar.pose.position.x;
      person_y_ = person_pose_lidar.pose.position.y;
      person_valid_ = true;

      publishMarker(person_pose_world.header, person_pose_world.pose.position.x, person_pose_world.pose.position.y);
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto filtered_scan = *msg;
    int beams_filtered = 0;

    if (person_valid_) {
      double angle = msg->angle_min;
      for (size_t i = 0; i < msg->ranges.size(); i++, angle += msg->angle_increment) {
        double r = msg->ranges[i];
        if (r < msg->range_min || r > msg->range_max) continue;

        double x = r * cos(angle);
        double y = r * sin(angle);

        double dx = x - person_x_;
        double dy = y - person_y_;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist <= person_radius_) {
          filtered_scan.ranges[i] = std::numeric_limits<float>::infinity(); // To clear (free) space where moving object is
          // filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN(); // To ignore those beams entirely (no clearing, no marking)
          beams_filtered++;
        }
      }
    }

    if (person_valid_) {
      // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
      //   "Filtered %d beams around person at (%.2f, %.2f) [lidar frame]",
      //   beams_filtered, person_x_, person_y_);
    }

    scan_pub_->publish(filtered_scan);
  }

  void publishMarker(const std_msgs::msg::Header &header, double global_x, double global_y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = "map";  
    marker.ns = "scan_filter";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = global_x;
    marker.pose.position.y = global_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = person_radius_ * 0.5;
    marker.scale.y = person_radius_ * 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double person_radius_;
  double person_x_{0.0}, person_y_{0.0};
  bool person_valid_{false};
  std::string filter_mode_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicScanFilter>());
  rclcpp::shutdown();
  return 0;
}

