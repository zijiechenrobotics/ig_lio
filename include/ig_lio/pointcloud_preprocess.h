/*
 * @Description: preprocess point clouds
 * @Autor: Zijie Chen
 * @Date: 2023-12-28 09:45:32
 */

#ifndef POINTCLOUD_PREPROCESS_H_
#define POINTCLOUD_PREPROCESS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <glog/logging.h>

#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver/CustomMsg.h>

#include "point_type.h"

enum class LidarType { LIVOX, VELODYNE, OUSTER };

// for Velodyne LiDAR
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
// struct OusterPointXYZIRT {
//   PCL_ADD_POINT4D;
//   float intensity;
//   double timestamp;
//   uint16_t reflectivity;
//   uint8_t ring;
//   uint16_t noise;
//   uint32_t range;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     OusterPointXYZIRT,
//     (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
//         double, timestamp, timestamp)(uint16_t, reflectivity, reflectivity)(
//         uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

class PointCloudPreprocess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    Config(){};

    int point_filter_num{4};
    LidarType lidar_type = LidarType::VELODYNE;
    // only use for velodyne
    double time_scale{1000.0};
  };

  PointCloudPreprocess() = delete;

  PointCloudPreprocess(Config config = Config())
      : config_(config) {}

  ~PointCloudPreprocess() = default;

  void Process(const livox_ros_driver::CustomMsg::ConstPtr& msg,
               pcl::PointCloud<PointType>::Ptr& cloud_out,
               const double last_start_time = 0.0);

  void Process(const sensor_msgs::PointCloud2::ConstPtr& msg,
               pcl::PointCloud<PointType>::Ptr& cloud_out);

 private:
  template <typename T>
  bool HasInf(const T& p);

  template <typename T>
  bool HasNan(const T& p);

  template <typename T>
  bool IsNear(const T& p1, const T& p2);

  void ProcessVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       pcl::PointCloud<PointType>::Ptr& cloud_out);

  void ProcessOuster(const sensor_msgs::PointCloud2::ConstPtr& msg,
                     pcl::PointCloud<PointType>::Ptr& cloud_out);

  int num_scans_ = 128;
  bool has_time_ = false;

  Config config_;

  pcl::PointCloud<PointType> cloud_sort_;
};

#endif