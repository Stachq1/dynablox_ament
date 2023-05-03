
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */


#pragma once

#include <iostream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZ PointType;


namespace common {
struct Config {
  float m_res = 0.1;  // resolution of the map
  int num_threads = 4;

  // Cluster =============>
  // Filter out clusters with too few or many points.
  int min_cluster_size = 25;
  int max_cluster_size = 2500;
  // filter out clusters whose AABB is larger or smaller than this [m].
  float min_extent = 0.25f;
  float max_extent = 2.5f;
  // Grow ever free detections by 1 (false) or 2 (true) voxels.
  bool grow_clusters_twice = true;
  // Connectivity used when clustering voxels. (6, 18, 26)
  int neighbor_connectivity = 6;
  // If true check separation per point, if false per voxel.
  bool check_cluster_separation_exact = false;
  // merge clusters whose points are closer than the minimum separation [m].
  float min_cluster_separation = 0.2;

  // Tracking =============>
  // Numbers of frames a cluster needs to be tracked to be considered dynamic.
  int min_track_duration = 0;
  // Maximum distance a cluster may have moved to be considered a track [m].
  float max_tracking_distance = 1.f;

  bool verbose_ = true;

};

}  // namespace common