/*
 * @Description: incremental voxel map
 * @Autor: Zijie Chen
 * @Date: 2023-12-27 23:44:38
 */

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include <execution>
#include <list>
#include <unordered_map>

#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <boost/sort/spreadsort/integer_sort.hpp>

#include <glog/logging.h>

#include <Eigen/Core>

#include "ig_lio/point_type.h"

struct Grid {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Grid(size_t grid_max_points) { points_array_.reserve(2 * grid_max_points); }

  size_t hash_idx{0};
  Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inv_cov_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d cov_sum_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d points_sum_ = Eigen::Vector3d::Zero();
  size_t points_num_{0};
  bool is_valid_{false};
  std::vector<Eigen::Vector3d> points_array_;
};

struct point_hash_idx {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point_;
  size_t hash_idx_;  //

  point_hash_idx() = default;
  point_hash_idx(Eigen::Vector3d point, size_t hash_idx)
      : point_(point)
      , hash_idx_(hash_idx) {}

  bool operator<(const point_hash_idx& p) const {
    return (hash_idx_ < p.hash_idx_);
  }
};

// for KNN
struct point_distance {
 public:
  point_distance() = default;
  point_distance(const Eigen::Vector3d& point,
                 const double distance,
                 const size_t point_idx)
      : point_(point)
      , distance_(distance)
      , point_idx_(point_idx) {}

  inline bool operator()(const point_distance& p1, const point_distance& p2) {
    return p1.distance_ < p2.distance_;
  }

  inline bool operator<(const point_distance& rhs) {
    return distance_ < rhs.distance_;
  }

  Eigen::Vector3d point_;
  double distance_{0.0};
  size_t point_idx_{0};
};

class VoxelMap {
 public:
  struct Config {
    Config(){};

    double resolution{0.5};
    // using "NEARBY_7" is the most stable
    std::string search_method = "NEARBY_7";
    size_t capacity{5000000};
    size_t grid_max_points{20};
  };

  VoxelMap() = delete;

  VoxelMap(Config config = Config());

  bool AddCloud(const CloudPtr& input_cloud_ptr);

  bool GetSurroundingGrids(const PointType& point, std::vector<size_t>& grids);

  size_t ComputeHashIndex(const Eigen::Vector3d& point);

  void ComputeCovariance(std::shared_ptr<Grid>& grid_ptr);

  bool IsSameGrid(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

  bool GetCentroidAndCovariance(const size_t hash_idx,
                                Eigen::Vector3d& centorid,
                                Eigen::Matrix3d& cov);

  bool KNNByCondition(const Eigen::Vector3d& point,
                      const size_t K,
                      const double range,
                      std::vector<Eigen::Vector3d>& results);

  size_t GetVoxelMapSize() { return voxel_map_.size(); }

  std::vector<Eigen::Vector3d> delta_P_;
  double resolution_{1.0};
  double inv_resolution_{1.0};
  size_t capacity_{5000000};
  size_t grid_max_points_{20};

  using MyHashMap = tbb::concurrent_hash_map<size_t, std::shared_ptr<Grid>>;
  using MyVector = tbb::concurrent_vector<std::shared_ptr<Grid>>;
  using MyAccessor = MyHashMap::accessor;

  std::shared_ptr<MyHashMap> temp_voxel_map_ptr_;
  std::shared_ptr<MyVector> temp_voxel_array_ptr_;

  std::unordered_map<
      size_t,
      typename std::list<std::pair<size_t, std::shared_ptr<Grid>>>::iterator>
      voxel_map_;
  std::list<std::pair<size_t, std::shared_ptr<Grid>>> grids_cache_;

  size_t HASH_P_{116101};
  size_t MAX_N_{10000000000};
};

#endif