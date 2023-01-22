#ifndef POINT_CLOUD
#define POINT_CLOUD

#include <Types/Scene.h>
#include <Utils/Visualization.hpp>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <cmath>
#include <memory>
#include <pcl/common/io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include "Types/GraphLaplacian.h"

namespace Processing {
namespace PointCloud {

// TODO probably need to template this
inline void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             int max_points) {
  double leafInc = 0.01;
  double leafValue = 0.1;
  while (cloud->points.size() > max_points) {
    std::cout << "Filtering cloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leafValue, leafValue, leafValue);
    sor.filter(*vox_cloud);

    cloud.reset();
    cloud = vox_cloud;
    leafValue += leafInc;
  }
}

inline void computeMCAR(SpectralObject &spectral_object) {
  spectral_object.kdTree.setInputCloud(spectral_object.cloud);

  int size = spectral_object.cloud->size();

  double radius = 0.01f;
  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;

    spectral_object.kdTree.radiusSearch(i, radius, indicies_found,
                                        squaredDistances, 2);
    int num_edges = indicies_found.size() - 1;

    if (num_edges == 0) {
      std::cout << "Increasing radius" << std::endl;
      i--;
      radius += 0.01f;
    }
  }

  spectral_object.mcar = radius;
}

inline void findObjectPointCloud(SpectralObject &spectral_object,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                 int max_pts) {
  spectral_object.cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  uint8_t r = (uint8_t)strtol(spectral_object.ply_color.substr(0, 2).c_str(),
                              nullptr, 16);
  uint8_t g = (uint8_t)strtol(spectral_object.ply_color.substr(2, 2).c_str(),
                              nullptr, 16);
  uint8_t b = (uint8_t)strtol(spectral_object.ply_color.substr(4, 2).c_str(),
                              nullptr, 16);

  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(
      new pcl::ConditionAnd<pcl::PointXYZRGB>());
  range_cond->addComparison(
      pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
              "r", pcl::ComparisonOps::EQ, r)));
  range_cond->addComparison(
      pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
              "g", pcl::ComparisonOps::EQ, g)));
  range_cond->addComparison(
      pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
              "b", pcl::ComparisonOps::EQ, b)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.filter(*cloud_filtered);

  filterPointCloud(cloud_filtered, max_pts);
  spectral_object.cloud = cloud_filtered;
}

inline void ExtractObjectPointClouds(Scene &scene, int max_pts) {
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPLYFile(scene.ply_file_path, *cloud2);
  pcl::fromPCLPointCloud2(*cloud2, *cloud);

  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                [&, max_pts](SpectralObject &so) {
                  findObjectPointCloud(so, cloud, max_pts);
                });
}

inline void MinimallyConnectedAdaptiveRadius(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &computeMCAR);
}

inline void CalculateGFAFeatures(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &Processing::GFA::calculateGFA);
}

}; // namespace PointCloud
}; // namespace Processing

#endif // !POINT_CLOUD
