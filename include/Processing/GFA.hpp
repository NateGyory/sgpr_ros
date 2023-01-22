#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cfenv>

#include "Types/Scene.h"

namespace Processing {
namespace GFA {

template <typename T> inline bool swap_if_gt(T &a, T &b) {
  if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

inline void GetGFAFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                          std::vector<double> &eigenvalue_feature,
                          pcl::PointXYZRGB &centroid) {

  // Find the variances.
  const size_t kNPoints = cloud->points.size();
  pcl::PointCloud<pcl::PointXYZRGB> variances;
  for (size_t i = 0u; i < kNPoints; ++i) {
    variances.push_back(pcl::PointXYZRGB());
    variances.points[i].x = cloud->points[i].x - centroid.x;
    variances.points[i].y = cloud->points[i].y - centroid.y;
    variances.points[i].z = cloud->points[i].z - centroid.z;
  }

  // Find the covariance matrix. Since it is symmetric, we only bother with the
  // upper diagonal.
  const std::vector<size_t> row_indices_to_access = {0, 0, 0, 1, 1, 2};
  const std::vector<size_t> col_indices_to_access = {0, 1, 2, 1, 2, 2};
  Eigen::Matrix3f covariance_matrix;
  for (size_t i = 0u; i < row_indices_to_access.size(); ++i) {
    const size_t row = row_indices_to_access[i];
    const size_t col = col_indices_to_access[i];
    double covariance = 0;
    for (size_t k = 0u; k < kNPoints; ++k) {
      covariance +=
          variances.points[k].data[row] * variances.points[k].data[col];
    }
    covariance /= kNPoints;
    covariance_matrix(row, col) = covariance;
    covariance_matrix(col, row) = covariance;
  }

  // Compute eigenvalues of covariance matrix.
  constexpr bool compute_eigenvectors = false;
  Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix,
                                                         compute_eigenvectors);
  std::vector<float> eigenvalues(3, 0.0);
  eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
  eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
  eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
  if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
      eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
      eigenvalues_solver.eigenvalues()[2].imag() != 0.0) {
    std::cout << "Eigenvalues should not have non-zero imaginary component."
              << std::endl;
    exit(0);
  }

  // Sort eigenvalues from smallest to largest.
  swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
  swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
  swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

  // Normalize eigenvalues.
  double sum_eigenvalues =
      eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
  double e1 = eigenvalues.at(0) / sum_eigenvalues;
  double e2 = eigenvalues.at(1) / sum_eigenvalues;
  double e3 = eigenvalues.at(2) / sum_eigenvalues;
  if (e1 == e2 || e2 == e3 || e1 == e3) {
    std::cout << "Eigenvalues should not be equal." << std::endl;
    exit(0);
  }

  // Store inside features.
  const double sum_of_eigenvalues = e1 + e2 + e3;
  constexpr double kOneThird = 1.0 / 3.0;
  if (e1 == 0.0) {
    std::cout << "e1 should not be zero" << std::endl;
    exit(0);
  }
  if (sum_eigenvalues == 0.0) {
    std::cout << "sum of eigenvalues should not be 0.0" << std::endl;
    exit(0);
  }

  const double kNormalizationPercentile = 1.0;

  const double kLinearityMax = 28890.9 * kNormalizationPercentile;
  const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
  const double kScatteringMax = 124811 * kNormalizationPercentile;
  const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
  const double kAnisotropyMax = 124810 * kNormalizationPercentile;
  const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
  const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;

  const double kNPointsMax = 13200 * kNormalizationPercentile;

  eigenvalue_feature.push_back((e1 - e2) / e1 / kLinearityMax);
  eigenvalue_feature.push_back((e2 - e3) / e1 / kPlanarityMax);
  eigenvalue_feature.push_back(e3 / e1 / kScatteringMax);
  eigenvalue_feature.push_back(std::pow(e1 * e2 * e3, kOneThird) /
                               kOmnivarianceMax);
  eigenvalue_feature.push_back((e1 - e3) / e1 / kAnisotropyMax);
  eigenvalue_feature.push_back((e1 * std::log(e1)) + (e2 * std::log(e2)) +
                               (e3 * std::log(e3)) / kEigenEntropyMax);
  eigenvalue_feature.push_back(e3 / sum_of_eigenvalues / kChangeOfCurvatureMax);

  pcl::PointXYZRGB point_min, point_max;

  pcl::getMinMax3D(*cloud, point_min, point_max);

  double diff_x, diff_y, diff_z;

  diff_x = point_max.x - point_min.x;
  diff_y = point_max.y - point_min.y;
  diff_z = point_max.z - point_min.z;

  if (diff_z < diff_x && diff_z < diff_y) {
    eigenvalue_feature.push_back(0.2);
  } else {
    eigenvalue_feature.push_back(0.0);
  }

  // eigenvalue_feature.push_back(FeatureValue("n_points", kNPoints /
  // kNPointsMax));

  if (eigenvalue_feature.size() != 8) {
    std::cout << "ERROR eigenvalue_feature vector not of size 8" << std::endl;
    exit(0);
  }

  // Check that there were no overflows, underflows, or invalid float
  // operations.
  if (std::fetestexcept(FE_OVERFLOW)) {

    std::cout << "Overflow error in eigenvalue feature computation."
              << std::endl;
    exit(0);
  } else if (std::fetestexcept(FE_UNDERFLOW)) {
    std::cout << "Underflow error in eigenvalue feature computation."
              << std::endl;
    exit(0);
  } else if (std::fetestexcept(FE_INVALID)) {
    std::cout << "Invalid Flag error in eigenvalue feature computation."
              << std::endl;
    exit(0);
  } else if (std::fetestexcept(FE_DIVBYZERO)) {
    std::cout << "Divide by zero error in eigenvalue feature computation."
              << std::endl;
    ;
    exit(0);
  }
}

inline void computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            pcl::PointXYZRGB &centroid) {
  pcl::CentroidPoint<pcl::PointXYZRGB> center;
  for (int i = 0; i < cloud->size(); i++) {
    center.add(cloud->points[i]);
  }

  center.get(centroid);
}

inline void calculateGFA(SpectralObject &spectral_object) {
  pcl::PointXYZRGB centroid;
  computeCentroid(spectral_object.cloud, centroid);

  // Get the GFA features
  GetGFAFeature(spectral_object.cloud, spectral_object.gfaFeatures, centroid);
}

inline void CalculateGFAFeatures(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &calculateGFA);
}

}; // namespace GFA
}; // namespace Processing
