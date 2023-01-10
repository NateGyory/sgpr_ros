#pragma once

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

  //double GetRadius1() { return mGraphLaplacianPair.first->radius; }
  //double GetRadius2() { return mGraphLaplacianPair.second->radius; }
  //PointCloudPair GetPointCloudPair() { return mPointCloudPair; }

private:
  ScenePair mScenePair;
};
