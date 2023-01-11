#pragma once

#include <pcl/point_cloud.h>
#include <string>

#include <matplot/matplot.h>
#include <pcl/io/ply_io.h>

#include "Processing/Eigen.hpp"
#include "Processing/Laplacian.hpp"
#include "Processing/PointCloud.hpp"

#include "Types/GraphLaplacian.h"

using ScenePair = std::pair<Scene, Scene>;
//using GraphLaplacianPair =
//    std::pair<std::shared_ptr<GraphLaplacian>, std::shared_ptr<GraphLaplacian>>;
using namespace matplot;

class NovelMethodTestingPipeline {
public:
  NovelMethodTestingPipeline() = default;
  ~NovelMethodTestingPipeline() = default;

  void ParsePointCloudPair(std::string f_ply1, std::string f_ply2,
                           int max_points);
  void ComputeEdges(int edge_heuristic);
  void ComputeLaplacian(int laplacian_type);
  void ComputeEigs(int eigs_num);
  void SaveEigenvalues(std::string file_name);
  void PlotHistograms();

  double GetRadius1() { return mScenePair.first.spectral_objects[0].mcar; }
  double GetRadius2() { return mScenePair.second.spectral_objects[0].mcar; }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud1() { return mScenePair.first.spectral_objects[0].cloud; }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud2() { return mScenePair.second.spectral_objects[0].cloud; }

private:
  ScenePair mScenePair;
};
