#include "ig_lio/voxel_map.h"

VoxelMap::VoxelMap(Config config) {
  resolution_ = config.resolution;
  inv_resolution_ = 1.0 / resolution_;
  capacity_ = config.capacity;
  grid_max_points_ = config.grid_max_points;

  delta_P_.clear();
  if (config.search_method == "NEARBY_1") {
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  } else if (config.search_method == "NEARBY_7") {
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
  } else if (config.search_method == "NEARBY_19") {
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
  } else if (config.search_method == "NEARBY_26") {
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
    delta_P_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, resolution_, resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, resolution_, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(resolution_, -resolution_, resolution_));
    delta_P_.push_back(
        Eigen::Vector3d(resolution_, -resolution_, -resolution_));
    delta_P_.push_back(Eigen::Vector3d(-resolution_, resolution_, resolution_));
    delta_P_.push_back(
        Eigen::Vector3d(-resolution_, resolution_, -resolution_));
    delta_P_.push_back(
        Eigen::Vector3d(-resolution_, -resolution_, resolution_));
    delta_P_.push_back(
        Eigen::Vector3d(-resolution_, -resolution_, -resolution_));
  }

  temp_voxel_map_ptr_ = std::make_shared<MyHashMap>();
  temp_voxel_array_ptr_ = std::make_shared<MyVector>();
}

bool VoxelMap::AddCloud(const CloudPtr& input_cloud_ptr) {
  if (input_cloud_ptr->empty()) {
    LOG(INFO) << "input cloud is empty";
    return false;
  }

  std::vector<point_hash_idx> point_buff;
  point_buff.resize(input_cloud_ptr->size());

  // Follow the algorithm 1 in the paper
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, input_cloud_ptr->size()),
      [&, this](tbb::blocked_range<size_t> r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          point_hash_idx p;
          p.point_ = input_cloud_ptr->points[i].getVector3fMap().cast<double>();
          p.hash_idx_ = ComputeHashIndex(p.point_);
          point_buff[i] = p;
        }
      });

  tbb::parallel_sort(point_buff.begin(), point_buff.end());

  size_t error_grids = 0;
  for (size_t i = 0; i < point_buff.size();) {
    size_t j = i;
    size_t curr_hash_idx = point_buff.at(i).hash_idx_;
    Eigen::Vector3d point_sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
    size_t count = 0;
    std::vector<Eigen::Vector3d> points_array_temp;
    for (; j < point_buff.size(); ++j) {
      if (curr_hash_idx == point_buff.at(j).hash_idx_) {
        // prevent hash collision
        if (IsSameGrid(point_buff.at(i).point_, point_buff.at(j).point_)) {
          point_sum += point_buff.at(j).point_;
          cov_sum +=
              point_buff.at(j).point_ * point_buff.at(j).point_.transpose();
          count++;
          points_array_temp.emplace_back(point_buff.at(j).point_);
        } else {
          // hash collision!
          error_grids++;
        }

      } else {
        break;
      }
    }

    auto iter = voxel_map_.find(curr_hash_idx);
    if (iter == voxel_map_.end()) {
      // create a new grid
      std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_max_points_);
      grid->points_sum_ = point_sum;
      grid->points_num_ = count;
      grid->cov_sum_ = cov_sum;
      grid->points_array_ = points_array_temp;
      // compute centroid
      grid->centroid_ =
          grid->points_sum_ / static_cast<double>(grid->points_num_);
      // compute covariance
      ComputeCovariance(grid);

      // LRU cache
      grids_cache_.push_front({curr_hash_idx, grid});
      voxel_map_.insert({curr_hash_idx, grids_cache_.begin()});
      if (voxel_map_.size() >= capacity_) {
        voxel_map_.erase(grids_cache_.back().first);
        grids_cache_.pop_back();
      }
    } else {
      Eigen::Vector3d centroid = point_sum / static_cast<double>(count);
      // prevent hash collision
      if (!IsSameGrid(iter->second->second->centroid_, centroid)) {
        // hash collision!
        error_grids++;
        // remove this grid
        grids_cache_.erase(iter->second);

        // create a new grid
        std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_max_points_);
        grid->points_sum_ = point_sum;
        grid->points_num_ = count;
        grid->cov_sum_ = cov_sum;
        grid->points_array_ = points_array_temp;
        // compute centroid
        grid->centroid_ =
            grid->points_sum_ / static_cast<double>(grid->points_num_);
        // compute covariance
        ComputeCovariance(grid);

        grids_cache_.push_front({curr_hash_idx, grid});
        voxel_map_[curr_hash_idx] = grids_cache_.begin();
      } else {
        // If the number of points is greater than 50, the probability has
        // stabilized and no need to update.
        if (iter->second->second->points_num_ < 50) {
          iter->second->second->points_sum_ += point_sum;
          iter->second->second->points_num_ += count;
          iter->second->second->cov_sum_ += cov_sum;
          // compute centroid
          iter->second->second->centroid_ =
              iter->second->second->points_sum_ /
              static_cast<double>(iter->second->second->points_num_);
          // compute covariance
          ComputeCovariance(iter->second->second);

          // Improve KNN efficiency by limiting the number of points per
          // grid.
          if (iter->second->second->points_num_ < grid_max_points_) {
            iter->second->second->points_array_.insert(
                iter->second->second->points_array_.end(),
                points_array_temp.begin(),
                points_array_temp.end());
          }
        }

        grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
        voxel_map_[curr_hash_idx] = grids_cache_.begin();
      }
    }
    i = j;
  }

  LOG(INFO) << "total_point " << input_cloud_ptr->size()
            << " error_point: " << error_grids;
  return true;
}

void VoxelMap::ComputeCovariance(std::shared_ptr<Grid>& grid_ptr) {
  if (grid_ptr->points_num_ >= 6) {
    Eigen::Matrix3d covariance =
        (grid_ptr->cov_sum_ -
         grid_ptr->points_sum_ * grid_ptr->centroid_.transpose()) /
        (static_cast<double>(grid_ptr->points_num_) - 1.0);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector3d values(1, 1, 1e-3);
    Eigen::Matrix3d modified_cov =
        svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

    grid_ptr->inv_cov_ = modified_cov.inverse();
    grid_ptr->cov_ = modified_cov;
    grid_ptr->is_valid_ = true;
  } else {
    // The number of points is too little to calculate a valid probability.
    grid_ptr->is_valid_ = false;
  }
}

bool VoxelMap::KNNByCondition(const Eigen::Vector3d& point,
                              const size_t K,
                              const double range,
                              std::vector<Eigen::Vector3d>& results) {
  std::vector<point_distance> point_dist;
  point_dist.reserve(delta_P_.size() * grid_max_points_);
  double range2 = range * range;

  for (const auto& delta : delta_P_) {
    Eigen::Vector3d nearby_point = point + delta;
    size_t hash_idx = ComputeHashIndex(nearby_point);
    Eigen::Vector3d centroid;

    auto iter = voxel_map_.find(hash_idx);
    if (iter != voxel_map_.end() &&
        IsSameGrid(nearby_point, iter->second->second->centroid_)) {
      for (const auto& p : iter->second->second->points_array_) {
        double dist = (point - p).squaredNorm();

        if (dist < range2) {
          point_dist.emplace_back(point_distance(
              p, dist, (&p - iter->second->second->points_array_.data())));
        }
      }
    }
  }

  if (point_dist.empty()) {
    return false;
  }

  if (point_dist.size() > K) {
    std::nth_element(
        point_dist.begin(), point_dist.begin() + K - 1, point_dist.end());
    point_dist.resize(K);
  }
  std::nth_element(point_dist.begin(), point_dist.begin(), point_dist.end());

  results.clear();
  for (const auto& it : point_dist) {
    results.emplace_back(it.point_);
  }
  return true;
}

bool VoxelMap::GetCentroidAndCovariance(const size_t hash_idx,
                                        Eigen::Vector3d& centorid,
                                        Eigen::Matrix3d& cov) {
  auto iter = voxel_map_.find(hash_idx);
  if (iter != voxel_map_.end() && iter->second->second->is_valid_) {
    centorid = iter->second->second->centroid_;
    cov = iter->second->second->cov_;
    return true;
  } else {
    return false;
  }
}

size_t VoxelMap::ComputeHashIndex(const Eigen::Vector3d& point) {
  double loc_xyz[3];
  for (size_t i = 0; i < 3; ++i) {
    loc_xyz[i] = point[i] * inv_resolution_;
    if (loc_xyz[i] < 0) {
      loc_xyz[i] -= 1.0;
    }
  }

  size_t x = static_cast<size_t>(loc_xyz[0]);
  size_t y = static_cast<size_t>(loc_xyz[1]);
  size_t z = static_cast<size_t>(loc_xyz[2]);

  return ((((z)*HASH_P_) % MAX_N_ + (y)) * HASH_P_) % MAX_N_ + (x);
}

bool VoxelMap::IsSameGrid(const Eigen::Vector3d& p1,
                          const Eigen::Vector3d& p2) {
  int hx_1 = floor(p1.x() * inv_resolution_);
  int hy_1 = floor(p1.y() * inv_resolution_);
  int hz_1 = floor(p1.z() * inv_resolution_);
  int hx_2 = floor(p2.x() * inv_resolution_);
  int hy_2 = floor(p2.y() * inv_resolution_);
  int hz_2 = floor(p2.z() * inv_resolution_);

  return ((hx_1 == hx_2) && (hy_1 == hy_2) && (hz_1 == hz_2));
}