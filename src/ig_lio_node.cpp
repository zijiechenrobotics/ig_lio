
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <tf/transform_broadcaster.h>

//c++
#include <boost/filesystem.hpp>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>


//ig lio
#include "ig_lio/lio.h"
#include "ig_lio/logger.hpp"
#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

//ros2 
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>



namespace fs = boost::filesystem;
constexpr double kAccScale = 9.80665;
// Global flag for clean shutdown
std::atomic<bool> FLAG_EXIT(false);
using std::placeholders::_1;

class IG_LIO_NODE : public rclcpp::Node {
public:
  IG_LIO_NODE(std::string package_path) : Node("ig_lio_node"){
    // Setup signal handler
    signal(SIGINT, IG_LIO_NODE::SigHandle);
    DeclareParams();
    GetParams();
    package_path_ = package_path;

    if (lidar_type_string == "velodyne") {
      lidar_type_ = LidarType::VELODYNE;
    } else if (lidar_type_string == "ouster") {
      lidar_type_ = LidarType::OUSTER;
    } else if (lidar_type_string == "livox") {
      lidar_type_ = LidarType::LIVOX;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error lidar type!");
      rclcpp::shutdown();
    }
    //Initialize variables 
    Initialize();
    //Register pub/sub
    Topics();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);




    // Start the loop in a separate thread
    processing_thread_ = std::thread(&IG_LIO_NODE::processingLoop, this);
  }

  ~IG_LIO_NODE() {
    // Set exit flag and join the thread on destruction
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
  }

private:

  void processingLoop() {
      rclcpp::WallRate rate(std::chrono::microseconds(200));
      while (rclcpp::ok() && !FLAG_EXIT.load()) {
          Process(); // Your processing function
          rate.sleep();
      }
      timer.PrintAll();
      rclcpp::shutdown();
  }




  void DeclareParams(){
    // Initialize publishers, subscribers, parameters, etc.
    this->declare_parameter<std::string>("odom/imu_topic", "imu/data");
    this->declare_parameter<std::string>("odom/lidar_topic", "velodyne_points");
    this->declare_parameter<std::string>("odom/lidar_type", "velodyne");

    this->declare_parameter<std::string>("odom/odom_frame","odom");
    this->declare_parameter<std::string>("odom/robot_frame","base_link");
    this->declare_parameter<std::string>("odom/imu_frame", "imu_link");
    this->declare_parameter<std::string>("odom/lidar_frame","velodyne");
    this->declare_parameter<std::string>("map/map_frame","odom" );

    this->declare_parameter<double>("odom/time_scale", 1000);
    this->declare_parameter<int>("odom/point_filter_num", 6);
    // Declare and get parameters for init LIO
    this->declare_parameter<double>("odom/scan_resolution", 0.5);
    this->declare_parameter<double>("odom/voxel_map_resolution", 0.5);
    this->declare_parameter<int>("odom/max_iterations", 10);
    // Declare and get various covariance and gravity parameters
    this->declare_parameter<double>("odom/acc_cov", 0.1);
    this->declare_parameter<double>("odom/gyr_cov", 0.1);
    this->declare_parameter<double>("odom/ba_cov", 0.000001);
    this->declare_parameter<double>("odom/bg_cov", 0.000001);
    this->declare_parameter<double>("odom/init_ori_cov", 0.0001);
    this->declare_parameter<double>("odom/init_pos_cov", 0.0001);
    this->declare_parameter<double>("odom/init_vel_cov", 100.0);
    this->declare_parameter<double>("odom/init_ba_cov", 0.0001);
    this->declare_parameter<double>("odom/init_bg_cov", 0.0001);
    this->declare_parameter<double>("odom/gravity", 9.80665);

    // Declare and get parameters for GICP constraints, outlier rejection, and others
    this->declare_parameter<double>("odom/gicp_constraints_gain", 100.0);
    this->declare_parameter<double>("odom/point2plane_constraints_gain", 1000.0);
    this->declare_parameter<bool>("odom/enable_undistort", true);
    this->declare_parameter<bool>("odom/enable_outlier_rejection", true);
    this->declare_parameter<bool>("odom/enable_acc_correct", false);
    this->declare_parameter<bool>("odom/enable_ahrs_initalization", true);
    // Declare and get parameters for min and max radius
    this->declare_parameter<double>("odom/min_radius", 1.0);
    this->declare_parameter<double>("odom/max_radius", 150.0);

    // For vector parameters like extrinsic, it's a bit more complex
    // Declare and get extrinsic parameters (vectors)
    this->declare_parameter<std::vector<double>>("extrinsics/imu2lidar/t", default_t_imu_lidar);
    this->declare_parameter<std::vector<double>>("extrinsics/imu2lidar/r", default_R_imu_lidar);

    this->declare_parameter<std::vector<double>>("extrinsics/robot2imu/t", default_t_imu_lidar);
    this->declare_parameter<std::vector<double>>("extrinsics/robot2imu/r", default_R_imu_lidar);

    this->declare_parameter<std::vector<double>>("extrinsics/robot2lidar/t", default_t_imu_lidar);
    this->declare_parameter<std::vector<double>>("extrinsics/robot2lidar/r", default_R_imu_lidar);


  }

  void GetParams(){
    this->get_parameter("odom/imu_topic", imu_topic);
    this->get_parameter("odom/lidar_topic", lidar_topic);
    this->get_parameter("odom/lidar_type", lidar_type_string);
    this->get_parameter("odom/odom_frame", odom_frame);
    this->get_parameter("odom/robot_frame", robot_frame);    
    this->get_parameter("odom/imu_frame", imu_frame);
    this->get_parameter("odom/lidar_frame", lidar_frame);
    this->get_parameter("map/map_frame", map_frame);

    this->get_parameter("odom/time_scale", time_scale);
    this->get_parameter("odom/point_filter_num", point_filter_num);
    this->get_parameter("odom/scan_resolution", scan_resolution);
    this->get_parameter("odom/voxel_map_resolution", voxel_map_resolution);
    this->get_parameter("odom/max_iterations", max_iterations);
    // Retrieve the covariance and gravity parameters
    this->get_parameter("odom/acc_cov", acc_cov);
    this->get_parameter("odom/gyr_cov", gyr_cov);
    this->get_parameter("odom/ba_cov", ba_cov);
    this->get_parameter("odom/bg_cov", bg_cov);
    this->get_parameter("odom/init_ori_cov", init_ori_cov);
    this->get_parameter("odom/init_pos_cov", init_pos_cov);
    this->get_parameter("odom/init_vel_cov", init_vel_cov);
    this->get_parameter("odom/init_ba_cov", init_ba_cov);
    this->get_parameter("odom/init_bg_cov", init_bg_cov);
    this->get_parameter("odom/gravity", gravity);

    this->get_parameter("odom/gicp_constraints_gain", gicp_constraints_gain);
    this->get_parameter("odom/point2plane_constraints_gain", point2plane_constraints_gain);
    this->get_parameter("odom/enable_undistort", enable_undistort);
    this->get_parameter("odom/enable_outlier_rejection", enable_outlier_rejection);
    this->get_parameter("odom/enable_acc_correct", enable_acc_correct);
    this->get_parameter("odom/enable_ahrs_initalization", enable_ahrs_initalization);

    // Retrieve the paramodom/eters as shown previously
    this->get_parameter("odom/min_radius", min_radius);
    this->get_parameter("odom/max_radius", max_radius);

    this->get_parameter("extrinsics/imu2lidar/t", t_imu_lidar_v);
    this->get_parameter("extrinsics/imu2lidar/r", R_imu_lidar_v);
    this->get_parameter("extrinsics/robot2imu/t", robot2imu_t);
    this->get_parameter("extrinsics/robot2imu/r", robot2imu_r);
    this->get_parameter("extrinsics/robot2lidar/t", robot2lidar_t);
    this->get_parameter("extrinsics/robot2lidar/r", robot2lidar_r);


    

  }

  void Initialize(){
    // 1. pointcloud_preprocess
    // Determine lidar type...
    LOG(INFO) << "time_scale: " << time_scale << std::endl
              << "point_filter_num: " << point_filter_num;
    PointCloudPreprocess::Config cloud_preprocess_config;
    cloud_preprocess_config.lidar_type = lidar_type_;
    cloud_preprocess_config.point_filter_num = point_filter_num;
    cloud_preprocess_config.time_scale = time_scale;
    cloud_preprocess_config.max_radius = max_radius;
    cloud_preprocess_config.min_radius = min_radius;
    cloud_preprocess_ptr =
        std::make_shared<PointCloudPreprocess>(cloud_preprocess_config);

    // 2. init LIO
      LOG(INFO) << "lidar type: " << lidar_type_string << std::endl
                << "scan_resoultion: " << scan_resolution << std::endl
                << "voxel_map_resolution: " << voxel_map_resolution << std::endl
                << "max_iterations: " << max_iterations << std::endl
                << "acc_cov: " << acc_cov << std::endl
                << "gyr_cov: " << gyr_cov << std::endl
                << "ba_cov: " << ba_cov << std::endl
                << "bg_cov: " << bg_cov << std::endl
                << "gravity: " << gravity << std::endl
                << "init_ori_cov: " << init_ori_cov << std::endl
                << "init_pos_cov: " << init_pos_cov << std::endl
                << "init_vel_cov: " << init_vel_cov << std::endl
                << "init_ba_cov: " << init_ba_cov << std::endl
                << "init_bg_cov: " << init_bg_cov << std::endl
                << "gicp_constraints_gain: " << gicp_constraints_gain << std::endl
                << "point2plane_constraints_gain: " << point2plane_constraints_gain
                << std::endl
                << "enable_undistort: " << enable_undistort << std::endl
                << "enable_acc_correct: " << enable_acc_correct << std::endl
                << "enable_outlier_rejection: " << enable_outlier_rejection
                << std::endl
                << "enable_ahrs_initalization: " << enable_ahrs_initalization
                << std::endl
                << "min_radius: " << min_radius << std::endl
                << "max_radius: " << max_radius;
    // 3. load extrinsic          
    T_imu_lidar = Eigen::Matrix4d::Identity();
    T_imu_lidar.block<3, 1>(0, 3) =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            t_imu_lidar_v.data(), 3, 1);
    T_imu_lidar.block<3, 3>(0, 0) =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            R_imu_lidar_v.data(), 3, 3);

    this->extrinsics.robot2imu.t = Eigen::Vector3f(robot2imu_t[0], robot2imu_t[1], robot2imu_t[2]);
    this->extrinsics.robot2imu.R = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(robot2imu_r.begin(), robot2imu_r.end()).data(), 3, 3);
    this->extrinsics.robot2imu_T = Eigen::Matrix4f::Identity();
    this->extrinsics.robot2imu_T.block(0, 3, 3, 1) = this->extrinsics.robot2imu.t;
    this->extrinsics.robot2imu_T.block(0, 0, 3, 3) = this->extrinsics.robot2imu.R;

    this->extrinsics.robot2lidar.t = Eigen::Vector3f(robot2lidar_t[0], robot2lidar_t[1], robot2lidar_t[2]);
    this->extrinsics.robot2lidar.R = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(robot2lidar_r.begin(), robot2lidar_r.end()).data(), 3, 3);

    this->extrinsics.robot2lidar_T = Eigen::Matrix4f::Identity();
    this->extrinsics.robot2lidar_T.block(0, 3, 3, 1) = this->extrinsics.robot2lidar.t;
    this->extrinsics.robot2lidar_T.block(0, 0, 3, 3) = this->extrinsics.robot2lidar.R;


    LIO::Config lio_config;
    lio_config.acc_cov = acc_cov;
    lio_config.gyr_cov = gyr_cov;
    lio_config.ba_cov = ba_cov;
    lio_config.bg_cov = bg_cov;

    lio_config.gravity = gravity;
    lio_config.init_ori_cov = init_ori_cov;
    lio_config.init_pos_cov = init_pos_cov;
    lio_config.init_vel_cov = init_vel_cov;
    lio_config.init_ba_cov = init_ba_cov;
    lio_config.init_bg_cov = init_bg_cov;

    lio_config.gicp_constraint_gain = gicp_constraints_gain;
    lio_config.point2plane_constraint_gain = point2plane_constraints_gain;
    lio_config.enable_outlier_rejection = enable_outlier_rejection;
    lio_config.enable_undistort = enable_undistort;
    lio_config.max_iterations = max_iterations;
    lio_config.current_scan_resolution = scan_resolution;
    lio_config.voxel_map_resolution = voxel_map_resolution;
    lio_config.min_radius = min_radius;
    lio_config.max_radius = max_radius;

    lio_config.T_imu_lidar = T_imu_lidar;

    lio_ptr = std::make_shared<LIO>(lio_config);

    voxel_filter.setLeafSize(0.5, 0.5, 0.5);

    // save trajectory
    fs::path result_path = fs::path(package_path_) / "result" / "lio_odom.txt";
    if (!fs::exists(result_path.parent_path())) {
      fs::create_directories(result_path.parent_path());	
    }
    odom_stream.open(result_path, std::ios::out);
    if (!odom_stream.is_open()) {
      LOG(ERROR) << "failed to open: " << result_path;
      exit(0);
    }


    LOG(INFO) << "Done initializing variables " << std::endl;
  }


  void Topics(){

    // Setup subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&IG_LIO_NODE::ImuCallBack, this, _1));
    if (lidar_type_ == LidarType::LIVOX) {
      cloud_sub_ = nullptr;
      livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic, 10, std::bind(&IG_LIO_NODE::LivoxCloudCallBack, this, std::placeholders::_1));
    } else {
      livox_sub_ = nullptr;
      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10, std::bind(&IG_LIO_NODE::CloudCallBack, this, std::placeholders::_1));
    }


    // Setup publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio_odom", 10);
    current_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/current_scan", 10);
    keyframe_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_scan", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    LOG(INFO) << "Done setting pub/sub" << std::endl;
  }

void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_ptr) {
  static double last_imu_timestamp = 0.0;
  static sensor_msgs::msg::Imu last_imu = *msg_ptr;
  // parameters for EMA filter
  static double a = 0.8;
  static double b = 1.0 - a;

  sensor_msgs::msg::Imu imu_msg = *msg_ptr;
  imu_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;

  // if (abs(timediff_lidar_wrt_imu) > 0.1 && timediff_correct_flag) {
  //   imu_msg.header.stamp =
  //       ros::Time().fromSec(imu_timestamp + timediff_lidar_wrt_imu);
  // }
  if (std::abs(timediff_lidar_wrt_imu) > 0.1 && timediff_correct_flag) {
    // In ROS2, directly manipulate the builtin_interfaces::msg::Time structure
    auto corrected_time = rclcpp::Time(imu_msg.header.stamp) + rclcpp::Duration::from_seconds(timediff_lidar_wrt_imu);
    
    // Now update the imu_msg header stamp with the corrected time
    imu_msg.header.stamp.sec = corrected_time.seconds();
    imu_msg.header.stamp.nanosec = corrected_time.nanoseconds() % static_cast<uint64_t>(1e9);
  }
  {
    std::lock_guard<std::mutex> lock(buff_mutex);

    if (imu_timestamp < last_imu_timestamp) {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buff.clear();
    }
    last_imu_timestamp = imu_timestamp;

    // EMA filter for accelerometer
    imu_msg.linear_acceleration.x =
        msg_ptr->linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
    imu_msg.linear_acceleration.y =
        msg_ptr->linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
    imu_msg.linear_acceleration.z =
        msg_ptr->linear_acceleration.z * a + last_imu.linear_acceleration.z * b;
    last_imu = *msg_ptr;

    // Some Livox datasets miss the gravitational constant in the accelerometer
    if (enable_acc_correct) {
      imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x * kAccScale;
      imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.y * kAccScale;
      imu_msg.linear_acceleration.z = imu_msg.linear_acceleration.z * kAccScale;
    }

    imu_buff.push_back(imu_msg);
  }
}

//process Velodyne and Outser
void CloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  static double last_lidar_timestamp = 0.0;
  timer.Evaluate(
      [&]() {
        lidar_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        CloudPtr cloud_ptr(new CloudType());
        cloud_preprocess_ptr->Process(msg, cloud_ptr);

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
          }
          last_lidar_timestamp = lidar_timestamp;

          cloud_buff.push_back(
              std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, cloud_ptr));
        }

        // LOG(INFO) << "lidar buff size: " << cloud_buff.size();
      },
      "Cloud Preprocess (Standard)");
}

// // process livox

void LivoxCloudCallBack(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  static double last_lidar_timestamp = 0.0;
  static CloudPtr temp_cloud_ptr(new CloudType());
  static bool first_scan_flag = true;
  static double first_scan_timestamp = 0.0;

  timer.Evaluate(
      [&]() {
        lidar_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          // livox synchronizes with external IMU timestamps
          if (!timediff_correct_flag &&
              abs(lidar_timestamp - imu_timestamp) > 1.0 && !imu_buff.empty()) {
            timediff_correct_flag = true;
            timediff_lidar_wrt_imu = lidar_timestamp + 0.1 - imu_timestamp;
            // clear unnecessary imu data after the synchronization
            imu_buff.clear();
            LOG(INFO) << "timediff_lidar_wrt_imu: " << timediff_lidar_wrt_imu
                      << std::endl;
          }
        }

        // livox has been synchronize with IMU
        if (!timediff_correct_flag &&
            abs(lidar_timestamp - imu_timestamp) < 1.0) {
          timediff_correct_flag = true;
        }
        if (!timediff_correct_flag) {
          LOG(INFO) << "Livox LiDAR has not Sync with other sensor!!!"
                    << std::endl;
          return;
        }

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          // prevent timestamp disorder
          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
            last_lidar_timestamp = lidar_timestamp;
          }

          if (first_scan_flag) {
            first_scan_timestamp = lidar_timestamp;
            first_scan_flag = false;
          }

          cloud_preprocess_ptr->Process(
              msg, temp_cloud_ptr, first_scan_timestamp);

          first_scan_flag = true;
          last_lidar_timestamp = lidar_timestamp;

          CloudPtr cloud_ptr(new CloudType(*temp_cloud_ptr));
          cloud_buff.push_back(std::make_pair(first_scan_timestamp, cloud_ptr));
          temp_cloud_ptr->clear();
        }
      },
      "Cloud Preprocess (Livox)");
}



bool SyncMeasurements() {
  static bool measurement_pushed = false;
  static bool process_lidar = false;
  static SensorMeasurement local_sensor_measurement;
  static double lidar_mean_scantime = 0.0;
  static size_t lidar_scan_num = 0;

  if (cloud_buff.empty() || imu_buff.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(buff_mutex);

  double lidar_end_time = 0.0;
  if (!measurement_pushed) {
    if (!process_lidar) {
      CloudPtr cloud_sort(new CloudType());
      *cloud_sort = *cloud_buff.front().second;
      std::sort(cloud_sort->points.begin(),
                cloud_sort->points.end(),
                [](const PointType& x, const PointType& y) -> bool {
                  return (x.curvature < y.curvature);
                });
      local_sensor_measurement.cloud_ptr_ = cloud_sort;
      local_sensor_measurement.bag_time_ = cloud_buff.front().first;
      if (!local_sensor_measurement.cloud_ptr_->points.empty()) {
        local_sensor_measurement.lidar_start_time_ =
            cloud_buff.front().first +
            local_sensor_measurement.cloud_ptr_->points.front().curvature /
                (double)(1000);
      } else {
        local_sensor_measurement.lidar_start_time_ = cloud_buff.front().first;
      }

      if (local_sensor_measurement.cloud_ptr_->size() <= 1) {
        LOG(WARNING) << "Too Few Points in Cloud!!!" << std::endl;
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else if (local_sensor_measurement.cloud_ptr_->points.back().curvature /
                     (double)(1000) <
                 0.5 * lidar_mean_scantime) {
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else {
        lidar_scan_num++;
        lidar_end_time =
            local_sensor_measurement.bag_time_ +
            local_sensor_measurement.cloud_ptr_->points.back().curvature /
                (double)(1000);
        lidar_mean_scantime +=
            ((local_sensor_measurement.cloud_ptr_->points.back().curvature -
              local_sensor_measurement.cloud_ptr_->points.front().curvature) /
                 (double)(1000) -
             lidar_mean_scantime) /
            (double)(lidar_scan_num);
      }

      if (enable_undistort) {
        local_sensor_measurement.lidar_end_time_ = lidar_end_time;
      } else {
        local_sensor_measurement.lidar_end_time_ =
            local_sensor_measurement.bag_time_;
      }

      process_lidar = true;
    }

    bool get_gnss_measurement = false;
    while (!gnss_buff.empty()) {
      if (gnss_buff.front().header.stamp.sec +
                  imu_buff.front().header.stamp.nanosec * 1e-9 >
          sensor_measurement.bag_time_) {
        if (gnss_buff.front().header.stamp.sec +
                  gnss_buff.front().header.stamp.nanosec * 1e-9 >
            local_sensor_measurement.bag_time_) {
          LOG(INFO) << "gnss too new" << std::endl;
          break;
        }

        if ((int)(gnss_buff.front().twist.covariance[0]) == 1) {
          sensor_measurement.gnss_status_ = GNSSStatus::RTK_FIXED;
        } else {
          sensor_measurement.gnss_status_ = GNSSStatus::NONE;
        }

        sensor_measurement.measurement_type_ = MeasurementType::GNSS;
        sensor_measurement.bag_time_ = gnss_buff.front().header.stamp.sec +
                                          gnss_buff.front().header.stamp.nanosec * 1e-9;
        sensor_measurement.lidar_start_time_ =
            gnss_buff.front().header.stamp.sec +
                  gnss_buff.front().header.stamp.nanosec * 1e-9;
        sensor_measurement.lidar_end_time_ =
            gnss_buff.front().header.stamp.sec +
                  gnss_buff.front().header.stamp.nanosec * 1e-9;

        sensor_measurement.has_gnss_ori_ = false;
        Eigen::Vector3d temp_t =
            Eigen::Vector3d(gnss_buff.front().pose.pose.position.x,
                            gnss_buff.front().pose.pose.position.y,
                            gnss_buff.front().pose.pose.position.z);

        if ((gnss_buff.front().pose.pose.orientation.w *
                 gnss_buff.front().pose.pose.orientation.w +
             gnss_buff.front().pose.pose.orientation.x *
                 gnss_buff.front().pose.pose.orientation.x +
             gnss_buff.front().pose.pose.orientation.y *
                 gnss_buff.front().pose.pose.orientation.y +
             gnss_buff.front().pose.pose.orientation.z *
                 gnss_buff.front().pose.pose.orientation.z) < 1) {
          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              Eigen::Matrix3d::Identity();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          LOG(INFO) << "get gnss measurement." << std::endl;
        } else {
          Eigen::Quaterniond temp_q(gnss_buff.front().pose.pose.orientation.w,
                                    gnss_buff.front().pose.pose.orientation.x,
                                    gnss_buff.front().pose.pose.orientation.y,
                                    gnss_buff.front().pose.pose.orientation.z);

          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              temp_q.toRotationMatrix();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          sensor_measurement.has_gnss_ori_ = true;
          LOG(INFO) << "get gnss measurement with ori." << std::endl;
        }

        get_gnss_measurement = true;

        break;
      } else {
        gnss_buff.pop_front();
        LOG(INFO) << "gnss too old" << std::endl;
      }
    }

    if (!get_gnss_measurement) {
      sensor_measurement = local_sensor_measurement;
      sensor_measurement.measurement_type_ = MeasurementType::LIDAR;
    }

    measurement_pushed = true;

    if (sensor_measurement.measurement_type_ == MeasurementType::LIDAR) {
      cloud_buff.pop_front();
      process_lidar = false;
    } else if (sensor_measurement.measurement_type_ == MeasurementType::GNSS) {
      gnss_buff.pop_front();
    }
  }

  if (imu_buff.back().header.stamp.sec +
                  imu_buff.back().header.stamp.nanosec * 1e-9 <
      sensor_measurement.lidar_end_time_) {
    return false;
  }

  sensor_measurement.imu_buff_.clear();
  while (!imu_buff.empty()) {
    double imu_time = imu_buff.front().header.stamp.sec +
                  imu_buff.front().header.stamp.nanosec * 1e-9;
    if (imu_time < sensor_measurement.lidar_end_time_) {
      sensor_measurement.imu_buff_.push_back(imu_buff.front());
      imu_buff.pop_front();
    } else {
      break;
    }
  }
  sensor_measurement.imu_buff_.push_back(imu_buff.front());

  measurement_pushed = false;
  return true;
}

// The main process of iG-LIO
void Process() {
  // Step 1: Time synchronization
  if (!SyncMeasurements()) {
    return;
  }

  // Step 2: Use AHRS or static initialization
  // If the imu message has orientation channel, LIO can be initialized via AHRS
  if (!lio_ptr->IsInit()) {
    if (enable_ahrs_initalization) {
      lio_ptr->AHRSInitialization(sensor_measurement);
    } else {
      lio_ptr->StaticInitialization(sensor_measurement);
    }
    return;
  }

  // Step 3: Prediction
  for (size_t i = 0; i < sensor_measurement.imu_buff_.size(); ++i) {
    double time;
    if (i == sensor_measurement.imu_buff_.size() - 1) {
      // Ensure that the integration is intergrated to the measurement time.
      time = sensor_measurement.lidar_end_time_;
    } else {
      time = sensor_measurement.imu_buff_.at(i).header.stamp.sec + sensor_measurement.imu_buff_.at(i).header.stamp.nanosec * 1e-9;
    }
    Eigen::Vector3d acc(
        sensor_measurement.imu_buff_.at(i).linear_acceleration.x,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.y,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.z);
    Eigen::Vector3d gyr(sensor_measurement.imu_buff_.at(i).angular_velocity.x,
                        sensor_measurement.imu_buff_.at(i).angular_velocity.y,
                        sensor_measurement.imu_buff_.at(i).angular_velocity.z);
    lio_ptr->Predict(time, acc, gyr);
  }

  // Too little points for measurement update!
  if (sensor_measurement.cloud_ptr_->size() <= 1) {
    LOG(WARNING) << "no point, skip this scan";
    return;
  }

  // Setp 4: Measurement Update
  timer.Evaluate([&] { lio_ptr->MeasurementUpdate(sensor_measurement); },
                 "measurement update");

  LOG(INFO) << "iter_num: " << lio_ptr->GetFinalIterations() << std::endl
            << "ba: " << lio_ptr->GetCurrentBa().transpose()
            << " ba_norm: " << lio_ptr->GetCurrentBa().norm()
            << " bg: " << lio_ptr->GetCurrentBg().transpose() * 180.0 / M_PI
            << " bg_norm: " << lio_ptr->GetCurrentBg().norm() * 180.0 / M_PI
            << std::endl;

  // // Setp 5: Send to rviz for visualization
  Eigen::Matrix4d result_pose = lio_ptr->GetCurrentPose();

  // // odometry message
  auto nanosec_part = static_cast<uint32_t>((sensor_measurement.lidar_end_time_ - static_cast<uint64_t>(sensor_measurement.lidar_end_time_)) * 1e9);
  auto sec_part = static_cast<int32_t>(sensor_measurement.lidar_end_time_);
  // Create or update the rclcpp::Time object
  rclcpp::Time current_time_stamp = rclcpp::Time(sec_part, nanosec_part);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = this->odom_frame;
  odom_msg.child_frame_id = this->robot_frame;
  odom_msg.header.stamp = current_time_stamp;
  Eigen::Quaterniond temp_q(result_pose.block<3, 3>(0, 0));
  odom_msg.pose.pose.orientation.x = temp_q.x();
  odom_msg.pose.pose.orientation.y = temp_q.y();
  odom_msg.pose.pose.orientation.z = temp_q.z();
  odom_msg.pose.pose.orientation.w = temp_q.w();
  odom_msg.pose.pose.position.x = result_pose(0, 3);
  odom_msg.pose.pose.position.y = result_pose(1, 3);
  odom_msg.pose.pose.position.z = result_pose(2, 3);
  odom_pub_->publish(odom_msg);
  publishToRos(result_pose, temp_q, current_time_stamp);


  // publish dense scan
  CloudPtr trans_cloud(new CloudType());
  pcl::transformPointCloud(
      *sensor_measurement.cloud_ptr_, *trans_cloud, result_pose);
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*trans_cloud, scan_msg);
  scan_msg.header.frame_id = this->odom_frame;
  scan_msg.header.stamp.sec = sec_part;
  scan_msg.header.stamp.nanosec = nanosec_part;
  current_scan_pub_->publish(scan_msg);
  // publish keyframe path and scan
  static bool is_first_keyframe = true;
  static Eigen::Matrix4d last_keyframe = result_pose;
  Eigen::Matrix4d delta_p = last_keyframe.inverse() * result_pose;
  // double norm_ = Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm();
  double norm_ = Sophus::SO3d(lio_ptr->correctRotationMatrix(delta_p.block<3, 3>(0, 0))).log().norm();
  if (is_first_keyframe || delta_p.block<3, 1>(0, 3).norm() > 1.0 ||
      norm_ > 0.18) {
          LOG(INFO) << "Done checking delta p" << std::endl;

    if (is_first_keyframe) {
      is_first_keyframe = false;
    }

    last_keyframe = result_pose;

    // // publish downsample scan
    CloudPtr cloud_DS(new CloudType());
    voxel_filter.setInputCloud(sensor_measurement.cloud_ptr_);
    voxel_filter.filter(*cloud_DS);
    CloudPtr trans_cloud_DS(new CloudType());
    pcl::transformPointCloud(*cloud_DS, *trans_cloud_DS, result_pose);
    sensor_msgs::msg::PointCloud2 keyframe_scan_msg;
    pcl::toROSMsg(*trans_cloud_DS, keyframe_scan_msg);
    keyframe_scan_msg.header.frame_id = this->odom_frame;
    keyframe_scan_msg.header.stamp = scan_msg.header.stamp;
    keyframe_scan_pub_->publish(keyframe_scan_msg);
    // publich path
    path_array.header.stamp = scan_msg.header.stamp;
    path_array.header.frame_id = this->odom_frame;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = scan_msg.header.stamp;
    pose_stamped.header.frame_id = this->odom_frame;
    pose_stamped.pose.position.x = result_pose(0, 3);
    pose_stamped.pose.position.y = result_pose(1, 3);
    pose_stamped.pose.position.z = result_pose(2, 3);
    pose_stamped.pose.orientation.w = temp_q.w();
    pose_stamped.pose.orientation.x = temp_q.x();
    pose_stamped.pose.orientation.y = temp_q.y();
    pose_stamped.pose.orientation.z = temp_q.z();
    path_array.poses.push_back(pose_stamped);
    path_pub_->publish(path_array);

    
  }

  // Setp 6: Save trajectory for evo evaluation
  static size_t delay_count = 0;
  if (delay_count > 50) {
    Eigen::Matrix4d lio_pose = result_pose;
    Eigen::Quaterniond lio_q(lio_pose.block<3, 3>(0, 0));
    odom_stream << std::fixed << std::setprecision(6)
                << sensor_measurement.lidar_end_time_ << " "
                << std::setprecision(15) << lio_pose(0, 3) << " "
                << lio_pose(1, 3) << " " << lio_pose(2, 3) << " " << lio_q.x()
                << " " << lio_q.y() << " " << lio_q.z() << " " << lio_q.w()
                << std::endl;
  } else {
    delay_count++;
  }
}

  
  static void SigHandle(int sig) {
    FLAG_EXIT.store(true);
    std::cout << "Catch sig " << sig << std::endl;
  }


  void publishToRos(Eigen::Matrix4d& result_pose, Eigen::Quaterniond& temp_q, rclcpp::Time& current_time_stamp){

    // transform: odom to robot_frame
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = current_time_stamp; 
    transformStamped.header.frame_id = this->odom_frame;
    transformStamped.child_frame_id = this->robot_frame;

    // Set the translation
    transformStamped.transform.translation.x = result_pose(0, 3);
    transformStamped.transform.translation.y = result_pose(1, 3);
    transformStamped.transform.translation.z = result_pose(2, 3);

    // Set the rotation
    tf2::Quaternion q_tf;
    q_tf.setX(temp_q.x());
    q_tf.setY(temp_q.y());
    q_tf.setZ(temp_q.z());
    q_tf.setW(temp_q.w());

    transformStamped.transform.rotation.x = q_tf.x();
    transformStamped.transform.rotation.y = q_tf.y();
    transformStamped.transform.rotation.z = q_tf.z();
    transformStamped.transform.rotation.w = q_tf.w();

    // Send the transform
    tf_broadcaster_->sendTransform(transformStamped);

    // transform: robot to imu
    transformStamped.header.stamp = current_time_stamp;
    transformStamped.header.frame_id = this->robot_frame;
    transformStamped.child_frame_id = this->imu_frame;

    transformStamped.transform.translation.x = this->extrinsics.robot2imu.t[0];
    transformStamped.transform.translation.y = this->extrinsics.robot2imu.t[1];
    transformStamped.transform.translation.z = this->extrinsics.robot2imu.t[2];

    Eigen::Quaternionf q(this->extrinsics.robot2imu.R);
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();

    tf_broadcaster_->sendTransform(transformStamped);


    // transform: robot to lidar
    transformStamped.header.stamp = current_time_stamp;
    transformStamped.header.frame_id = this->robot_frame;
    transformStamped.child_frame_id = this->lidar_frame;

    transformStamped.transform.translation.x = this->extrinsics.robot2lidar.t[0];
    transformStamped.transform.translation.y = this->extrinsics.robot2lidar.t[1];
    transformStamped.transform.translation.z = this->extrinsics.robot2lidar.t[2];
    

    Eigen::Quaternionf qq(this->extrinsics.robot2lidar.R);
    transformStamped.transform.rotation.w = qq.w();
    transformStamped.transform.rotation.x = qq.x();
    transformStamped.transform.rotation.y = qq.y();
    transformStamped.transform.rotation.z = qq.z();

    tf_broadcaster_->sendTransform(transformStamped);
  }








  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;


  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string imu_topic;
  std::string lidar_topic;
  std::string lidar_type_string;
  std::string odom_frame;
  std::string robot_frame;
  std::string imu_frame;
  std::string lidar_frame;
  std::string map_frame;
  LidarType lidar_type_ = LidarType::LIVOX;
  bool enable_acc_correct;
  bool enable_undistort;
  bool enable_ahrs_initalization;
  Eigen::Matrix4d T_imu_lidar;

  // parameters used to synchronize livox time with the external imu
  double timediff_lidar_wrt_imu = 0.0;
  double lidar_timestamp = 0.0;
  double imu_timestamp = 0.0;
  bool timediff_correct_flag;
  double acc_cov, gyr_cov, bg_cov, ba_cov, init_ori_cov, init_pos_cov,
      init_vel_cov, init_ba_cov, init_bg_cov, gravity;
  std::mutex buff_mutex;
  double time_scale;
  int point_filter_num;
  double gicp_constraints_gain;
  double point2plane_constraints_gain;
  bool enable_outlier_rejection;
  
  double scan_resolution, voxel_map_resolution;
  int max_iterations;
  double min_radius, max_radius;
  // data deque
  std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> cloud_buff;
  std::deque<sensor_msgs::msg::Imu> imu_buff;
  std::deque<nav_msgs::msg::Odometry> gnss_buff;
  std::vector<double> t_imu_lidar_v, R_imu_lidar_v; 
  std::vector<double> robot2imu_t, robot2imu_r, robot2lidar_t, robot2lidar_r; 
  std::string package_path_;
  // Set default values for t_imu_lidar and R_imu_lidar
  std::vector<double> default_t_imu_lidar = {0.0, 0.0, 0.0};
  std::vector<double> default_R_imu_lidar = {1.0, 0.0, 0.0,  // First row of identity matrix
                                            0.0, 1.0, 0.0,  // Second row
                                            0.0, 0.0, 1.0}; // Third row
  nav_msgs::msg::Path path_array;

  Timer timer;
  std::shared_ptr<PointCloudPreprocess> cloud_preprocess_ptr;
  SensorMeasurement sensor_measurement;
  std::shared_ptr<LIO> lio_ptr;
  pcl::VoxelGrid<PointType> voxel_filter;
  std::fstream odom_stream;
  // LIO and other related objects
  std::shared_ptr<LIO> lio_ptr_;
  std::shared_ptr<PointCloudPreprocess> cloud_preprocess_ptr_;
  
  std::thread processing_thread_;
  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 robot2imu;
    SE3 robot2lidar;
    Eigen::Matrix4f robot2imu_T;
    Eigen::Matrix4f robot2lidar_T;
  }; Extrinsics extrinsics;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string package_path = ament_index_cpp::get_package_share_directory("ig_lio");
  Logger logger(argv, package_path);
  auto node = std::make_shared<IG_LIO_NODE>(package_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
