#ifndef POINT_CLOUD
#define POINT_CLOUD

#include <Types/Scene.h>
#include <Utils/Visualization.hpp>

#include <cmath>
#include <memory>
#include <pcl/common/io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <unordered_map>

#include "Processing/GFA.hpp"
#include "Types/GraphLaplacian.h"

namespace Processing {
namespace PointCloud {

inline std::unordered_map<int, std::string> id_rgb = {
    {0, "000000"},   {1, "ff0000"},   {10, "6496f5"},  {11, "64e6f5"},
    {13, "6450fa"},  {15, "1e3c96"},  {16, "0000ff"},  {18, "501eb4"},
    {20, "0000ff"},  {30, "ff1e1e"},  {31, "ff28c8"},  {32, "961e5a"},
    {40, "ff00ff"},  {44, "ff96ff"},  {48, "4b004b"},  {49, "af004b"},
    {50, "ffc800"},  {51, "ff7832"},  {52, "ff9600"},  {60, "96ffaa"},
    {70, "00af00"},  {71, "873c00"},  {72, "96f050"},  {80, "fff096"},
    {81, "ff0000"},  {99, "32ffff"},  {252, "6496f5"}, {253, "ff28c8"},
    {254, "ff1e1e"}, {255, "961e5a"}, {256, "0000ff"}, {257, "6450fa"},
    {258, "501eb4"}, {259, "0000ff"}};

inline std::unordered_map<std::string, int> rgb_id = {
    {"000000", 0},   {"ff0000", 1},   {"6496f5", 10},  {"64e6f5", 11},
    {"6450fa", 13},  {"1e3c96", 15},  {"0000ff", 16},  {"501eb4", 18},
    {"0000ff", 20},  {"ff1e1e", 30},  {"ff28c8", 31},  {"961e5a", 32},
    {"ff00ff", 40},  {"ff96ff", 44},  {"4b004b", 48},  {"af004b", 49},
    {"ffc800", 50},  {"ff7832", 51},  {"ff9600", 52},  {"96ffaa", 60},
    {"00af00", 70},  {"873c00", 71},  {"96f050", 72},  {"fff096", 80},
    {"ff0000", 81},  {"32ffff", 99},  {"6496f5", 252}, {"ff28c8", 253},
    {"ff1e1e", 254}, {"961e5a", 255}, {"0000ff", 256}, {"6450fa", 257},
    {"501eb4", 258}, {"0000ff", 259}};

inline std::unordered_map<int, std::string> id_label = {
    {0, "unlabeled"},
    {1, "outlier"},
    {10, "car"},
    {11, "bicycle"},
    {13, "bus"},
    {15, "motorcycle"},
    {16, "on-rails"},
    {18, "truck"},
    {20, "other-vehicle"},
    {30, "person"},
    {31, "bicyclist"},
    {32, "motorcyclist"},
    {40, "road"},
    {44, "parking"},
    {48, "sidewalk"},
    {49, "other-ground"},
    {50, "building"},
    {51, "fence"},
    {52, "other-structure"},
    {60, "lane-marking"},
    {70, "vegetation"},
    {71, "trunk"},
    {72, "terrain"},
    {80, "pole"},
    {81, "traffic-sign"},
    {99, "other-object"},
    {252, "moving-car"},
    {253, "moving-bicyclist"},
    {254, "moving-person"},
    {255, "moving-motorcyclist"},
    {256, "moving-on-rails"},
    {257, "moving-bus"},
    {258, "moving-truck"},
    {259, "moving-other-vehicle"}};

inline bool ShouldSkip(SpectralObject &so) {
  bool ret = true;
  switch (so.global_id) {
  case 10:
  case 11:
  case 13:
  case 15:
  case 16:
  case 18:
  case 20:
  case 40:
  case 44:
  case 48:
  case 49:
  case 50:
  case 51:
  case 52:
  case 60:
  case 70:
  case 71:
  case 72:
  case 80:
  case 81:
    ret = false;
    break;
  default:
    break;
  }

  return ret;
}

inline void PopulateSpectralObjs(Scene &scene) {
  // For every possible color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPLYFile(scene.ply_file_path, *cloud);

  std::unordered_map<std::string, bool> color_visited_map;
  for (auto kv : rgb_id) {

    if (color_visited_map.find(kv.first) != color_visited_map.end()) {
      continue;
    }

    SpectralObject so;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    uint8_t r = (uint8_t)strtol(kv.first.substr(0, 2).c_str(), nullptr, 16);
    uint8_t g = (uint8_t)strtol(kv.first.substr(2, 2).c_str(), nullptr, 16);
    uint8_t b = (uint8_t)strtol(kv.first.substr(4, 2).c_str(), nullptr, 16);

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

    if (cloud_filtered->size() > 0) {
      // create a spectral object
      so.cloud = cloud_filtered;
      so.global_id = kv.second;
      so.label = id_label[kv.second];
      so.ply_color = kv.first;
      color_visited_map[kv.first] = true;
      scene.spectral_objects.push_back(so);
    }
  }
}

// inline void GetSemanticKittiObjs(Scene &scene, int max_pts) {
//
//   PopulateSpectralObjs(scene);
//
//   spectral_object.cloud =
//   std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
//       std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//   uint8_t r = (uint8_t)strtol(spectral_object.ply_color.substr(0, 2).c_str(),
//                               nullptr, 16);
//   uint8_t g = (uint8_t)strtol(spectral_object.ply_color.substr(2, 2).c_str(),
//                               nullptr, 16);
//   uint8_t b = (uint8_t)strtol(spectral_object.ply_color.substr(4, 2).c_str(),
//                               nullptr, 16);
//
//   pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(
//       new pcl::ConditionAnd<pcl::PointXYZRGB>());
//   range_cond->addComparison(
//       pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
//           new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
//               "r", pcl::ComparisonOps::EQ, r)));
//   range_cond->addComparison(
//       pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
//           new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
//               "g", pcl::ComparisonOps::EQ, g)));
//   range_cond->addComparison(
//       pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
//           new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
//               "b", pcl::ComparisonOps::EQ, b)));
//
//   pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
//   condrem.setCondition(range_cond);
//   condrem.setInputCloud(cloud);
//   condrem.filter(*cloud_filtered);
//
//   // filterPointCloud(cloud_filtered, max_pts);
//   spectral_object.cloud = cloud_filtered;
// }

inline void computeFPS(SpectralObject &so, int size) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fps_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::FarthestPointSampling<pcl::PointXYZRGB> fps;
  fps.setInputCloud(so.cloud);
  fps.setSample(size);
  fps.setSeed(time(0));
  fps.filter(*fps_cloud);

  so.cloud.reset();
  so.cloud = fps_cloud;
}

inline void computeSOR(SpectralObject &so, int meanK, double stdThresh) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  sor.setInputCloud(so.cloud);
  sor.setMeanK(3);
  sor.setStddevMulThresh(5.0);
  sor.filter(*cloud_filtered);

  so.cloud.reset();
  so.cloud = cloud_filtered;
}

// TODO probably need to template this
inline void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             int max_points) {
  double leafInc = 0.01;
  double leafValue = 0.1;
  while (cloud->points.size() > max_points) {
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
  if (size == 1) {
    spectral_object.mcar = 100.0;
    return;
  }

  double radius = 0.01f;
  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;

    spectral_object.kdTree.radiusSearch(i, radius, indicies_found,
                                        squaredDistances, 2);
    int num_edges = indicies_found.size() - 1;

    if (num_edges == 0) {
      i--;
      radius += 0.01f;
    }
  }

  spectral_object.mcar = radius;
}

inline void findObjectPointCloud(SpectralObject &spectral_object,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                 int max_pts) {
  spectral_object.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
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

  // filterPointCloud(cloud_filtered, max_pts);
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
                Processing::GFA::calculateGFA);
}

inline void ComputeSceneSOR(Scene &scene, int meanK, double stdThresh) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                [meanK, stdThresh](SpectralObject &so) {
                  computeSOR(so, meanK, stdThresh);
                });
}

}; // namespace PointCloud
}; // namespace Processing

#endif // !POINT_CLOUD
