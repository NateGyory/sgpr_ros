#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <armadillo>

struct GraphLaplacian {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
  arma::sp_mat laplacian;
  double radius;
  double smallestDistance;
  arma::vec eigenvalues;
};
