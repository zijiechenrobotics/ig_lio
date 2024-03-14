



#ifndef POINTCLOUD_PREPROCESS_H_
#define POINTCLOUD_PREPROCESS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "point_type.h" // Ensure this is compatible with ROS2

// Assuming Livox ROS2 driver provides a similar message type or you have adapted it
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <glog/logging.h>

enum class LidarType { LIVOX, VELODYNE, OUSTER };

struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(float, time, time))

// for Ouster LiDAR
struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(
        uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))


class PointCloudPreprocess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    Config() {}

    int point_filter_num{4};
    LidarType lidar_type = LidarType::VELODYNE;
    // only use for Velodyne
    double time_scale{1000.0};
    double max_radius{150.0};
    double min_radius{0.5};
  };

  PointCloudPreprocess() = delete;

  PointCloudPreprocess(Config config = Config())
      : config_(config) {}

  ~PointCloudPreprocess() = default;

  void Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg,
               pcl::PointCloud<PointType>::Ptr& cloud_out,
               const double last_start_time = 0.0);

  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
               pcl::PointCloud<PointType>::Ptr& cloud_out);

 private:
  template <typename T>
  bool HasInf(const T& p);

  template <typename T>
  bool HasNan(const T& p);

  template <typename T>
  bool IsNear(const T& p1, const T& p2);

  void ProcessVelodyne(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                       pcl::PointCloud<PointType>::Ptr& cloud_out);

  void ProcessOuster(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                     pcl::PointCloud<PointType>::Ptr& cloud_out);
  bool InRadius(const PointType& p) {
    double radius = p.x * p.x + p.y * p.y + p.z * p.z;
    return (radius < (config_.max_radius * config_.max_radius) &&
            radius > (config_.min_radius * config_.min_radius));
  }
  int num_scans_ = 128;
  bool has_time_ = false;

  Config config_;

  pcl::PointCloud<PointType> cloud_sort_;
};

#endif  // POINTCLOUD_PREPROCESS_H_
