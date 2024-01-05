/*
 * @Description: LIO
 * @Autor: Zijie Chen
 * @Date: 2023-12-28 11:09:59
 */

#ifndef LIO_H_
#define LIO_H_

#include <deque>
#include <numeric>

#include <sensor_msgs/Imu.h>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.hpp>

#include <pcl/common/transforms.h>

#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "ig_lio/faster_voxel_grid.h"
#include "ig_lio/point_type.h"
#include "ig_lio/utilities.hpp"
#include "ig_lio/voxel_map.h"

enum MeasurementType { LIDAR, GNSS };
enum GNSSStatus { RTK_FIXED, RTK_FLOAT, NONE };

struct SensorMeasurement {
  MeasurementType measurement_type_;
  // ros time
  double bag_time_{0.0};
  // The time of the first laser point in the scan
  double lidar_start_time_{0.0};
  // The time of the last laser point in the scan
  double lidar_end_time_{0.0};
  CloudPtr cloud_ptr_{};
  std::deque<sensor_msgs::Imu> imu_buff_;

  Eigen::Matrix4d gnss_pose_ = Eigen::Matrix4d::Identity();
  GNSSStatus gnss_status_ = GNSSStatus::NONE;
  bool has_gnss_ori_{false};
};

class LIO {
 public:
  struct Config {
    Config(){};

    double init_ori_cov{1.0};
    double init_pos_cov{1.0};
    double init_vel_cov{1.0};
    double init_ba_cov{1.0};
    double init_bg_cov{1.0};
    double acc_cov{1.0};
    double gyr_cov{1.0};
    double ba_cov{1.0};
    double bg_cov{1.0};

    double gravity{9.81};
    Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();

    size_t max_iterations{30};
    double gicp_constraint_gain{100.0};
    double point2plane_constraint_gain{1000.0};

    bool enable_undistort{true};
    bool enable_outlier_rejection{true};

    double current_scan_resolution{0.5};
    double voxel_map_resolution{0.5};

    double max_radius{150.0};
    double min_radius{0.5};
  };

  LIO(Config config = Config())
      : config_(config)
      , cloud_cov_ptr_(new CloudCovType())
      , cloud_DS_ptr_(new CloudType()) {
    fast_voxel_grid_ptr_ =
        std::make_shared<FasterVoxelGrid>(config_.current_scan_resolution);

    VoxelMap::Config voxel_map_config;
    voxel_map_config.resolution = config_.voxel_map_resolution;
    voxel_map_config.search_method = "NEARBY_7";  // nearby_7 is more stable
    voxel_map_ptr_ = std::make_shared<VoxelMap>(voxel_map_config);

    correspondences_array_.reserve(10000);
    g_ = Eigen::Vector3d(0.0, 0.0, config_.gravity);
  }

  bool MeasurementUpdate(SensorMeasurement& sensor_measurement);

  bool Predict(const double time,
               const Eigen::Vector3d& acc_1,
               const Eigen::Vector3d& gyr_1);

  bool StaticInitialization(SensorMeasurement& sensor_measurement);

  bool AHRSInitialization(SensorMeasurement& sensor_measurement);

  bool InRadius(const PointType& p) {
    double radius = p.x * p.x + p.y * p.y + p.z * p.z;
    return (radius < (config_.max_radius * config_.max_radius) &&
            radius > (config_.min_radius * config_.min_radius));
  }

  bool IsInit() { return lio_init_; }

  Eigen::Matrix4d GetCurrentPose() { return curr_state_.pose; }

  Eigen::Vector3d GetCurrentVel() { return curr_state_.vel; }

  Eigen::Vector3d GetCurrentBa() { return curr_state_.ba; }

  Eigen::Vector3d GetCurrentBg() { return curr_state_.bg; }

  size_t GetFinalIterations() { return iter_num_; }

 private:
  static constexpr int IndexErrorOri{0};
  static constexpr int IndexErrorPos{3};
  static constexpr int IndexErrorVel{6};
  static constexpr int IndexErrorBiasAcc{9};
  static constexpr int IndexErrorBiasGyr{12};

  static constexpr int IndexNoiseAccLast{0};
  static constexpr int IndexNoiseGyrLast{3};
  static constexpr int IndexNoiseAccCurr{6};
  static constexpr int IndexNoiseGyrCurr{9};
  static constexpr int IndexNoiseBiasAcc{12};
  static constexpr int IndexNoiseBiasGyr{15};

  struct PoseHistory {
    double time_ = 0.0;
    Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d un_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d un_gyr_ = Eigen::Vector3d::Zero();
  };
  std::deque<PoseHistory> pose_history_;  // for pointcloud

  struct Correspondence {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Vector3d mean_A = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_B = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mahalanobis = Eigen::Matrix3d::Zero();
    Eigen::Vector4d plane_coeff = Eigen::Vector4d::Zero();
  };
  std::vector<std::shared_ptr<Correspondence>> correspondences_buff_;
  tbb::concurrent_vector<std::shared_ptr<Correspondence>>
      correspondences_array_;

  struct State {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  };

  bool NominalStateUpdate(const double dt,
                          const Eigen::Vector3d& acc_0,
                          const Eigen::Vector3d& acc_1,
                          const Eigen::Vector3d& gyr_0,
                          const Eigen::Vector3d& gyr_1,
                          const Eigen::Matrix4d& T_prev,
                          const Eigen::Vector3d& vel_prev,
                          Eigen::Matrix4d& T_curr,
                          Eigen::Vector3d& vel_curr,
                          Eigen::Vector3d& un_acc,
                          Eigen::Vector3d& un_gyr);

  bool ErrorStateUpdate(const double dt,
                        const Eigen::Vector3d& acc_0,
                        const Eigen::Vector3d& acc_1,
                        const Eigen::Vector3d& gyr_0,
                        const Eigen::Vector3d& gyr_1);

  bool UndistortPointCloud(const double bag_time,
                           const double lidar_end_time,
                           CloudPtr& cloud_ptr);

  bool StepOptimize(const SensorMeasurement& sensor_measurement,
                    Eigen::Matrix<double, 15, 1>& delta_x);

  double ConstructGICPConstraints(Eigen::Matrix<double, 15, 15>& H,
                                  Eigen::Matrix<double, 15, 1>& b);

  double ConstructPoint2PlaneConstraints(Eigen::Matrix<double, 15, 15>& H,
                                         Eigen::Matrix<double, 15, 1>& b);

  double ConstructImuPriorConstraints(Eigen::Matrix<double, 15, 15>& H,
                                      Eigen::Matrix<double, 15, 1>& b);

  double ComputeError(const SensorMeasurement& sensor_measurement,
                      const State& state);

  bool IsConverged(const Eigen::Matrix<double, 15, 1>& delta_x);

  bool ComputeFinalCovariance(const Eigen::Matrix<double, 15, 1>& delta_x);

  bool CorrectState(const State& state,
                    const Eigen::Matrix<double, 15, 1>& delta_x,
                    State& corrected_state);

  bool GNStep(const SensorMeasurement& sensor_measurement,
              Eigen::Matrix<double, 15, 15>& H,
              Eigen::Matrix<double, 15, 1>& b,
              const double y0,
              Eigen::Matrix<double, 15, 1>& delta_x);

  Config config_;

  double lio_time_{0.0};
  bool lio_init_{false};

  Eigen::Vector3d g_ = Eigen::Vector3d(0.0, 0.0, 9.81);

  State curr_state_;
  State prev_state_;

  Eigen::Vector3d acc_0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_0_ = Eigen::Vector3d::Zero();

  Eigen::Matrix<double, 15, 15> P_ = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 15> F_ = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 18> B_ = Eigen::Matrix<double, 15, 18>::Zero();
  Eigen::Matrix<double, 18, 18> Q_ = Eigen::Matrix<double, 18, 18>::Zero();

  double transformation_epsilon_{0.001};
  bool need_converge_{false};
  size_t iter_num_{0};
  Eigen::Matrix<double, 15, 15> final_hessian_ =
      Eigen::Matrix<double, 15, 15>::Zero();

  size_t effect_feat_num_;
  CloudCovPtr cloud_cov_ptr_{};
  CloudPtr cloud_DS_ptr_{};
  std::shared_ptr<VoxelMap> voxel_map_ptr_{};
  std::shared_ptr<FasterVoxelGrid> fast_voxel_grid_ptr_{};
  size_t lidar_frame_count_{0};
  size_t keyframe_count_{0};

  // for initialization
  Eigen::Vector3d mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
  Eigen::Vector3d mean_gyr_ = Eigen::Vector3d(0, 0, 0);
  // <acc, gyr>
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> imu_init_buff_;
  bool first_imu_frame_{true};
  size_t imu_init_count_{0};
  size_t max_init_count_{20};

  Eigen::Matrix4d last_keyframe_pose_ = Eigen::Matrix4d::Identity();

  size_t effect_const_num_{0};
  double ava_effect_feat_num_{0.0};
};

#endif