#ifndef PIPELINE
#define PIPELINE

#include <algorithm>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vector>
#include <utility>

#include "DataLoaders/DataLoader.h"
#include "DataLoaders/Matterport3D.h"
#include "DataLoaders/RScanDataLoader.h"
#include "DataLoaders/SemanticKitti.h"
#include "Scene.h"

#include <KeyFrames/KeyFrameDB.h>

#include <Processing/Eigen.hpp>
#include <Processing/Laplacian.hpp>
#include <Processing/PointCloud.hpp>

#include <Types/GraphLaplacian.h>

using PointCloudPair = std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr>;

using GraphLaplacianPair =
    std::pair<std::shared_ptr<GraphLaplacian>, std::shared_ptr<GraphLaplacian>>;

class Pipeline {
public:
  Pipeline(int type) : mType(type) {};
  ~Pipeline() = default;

  void PostProcess(int dataset); // NOTE: not using
  void RealTime(){};             // NOTE: not using
                                 //
  // Dataset Comparison Pipeline
  void ParseDataset(int dataset);
  void ExtractObjectPointClouds();
  void MCAR();
  void IDW();
  void Eigs();

  // PointCloud Comparison Pipeline
  void ParsePointCloudPair(std::string f_ply1, std::string f_ply2, int max_points);
  void FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int max_points);
  void ComputeEdges(int edge_heuristic);
  void ComputeLaplcian(int laplacian_type);
  void ComputeEigs(int eigs_num);
  void SaveEigenvalues(std::string file_name);
  void PlotHistograms();

  PointCloudPair GetPointCloudPair() { return mPointCloudPair; }
  double GetRadius1() { return mGraphLaplacianPair.first->radius; }
  double GetRadius2() { return mGraphLaplacianPair.second->radius; }

private:
  void initDataLoader(int dataset);

  int mType;

  scene_map_t mSceneMap;
  spDataLoader mDataLoader;
  spKeyFrameDB mKeyFrameDB;

  // NMT pipeline
  PointCloudPair mPointCloudPair;
  GraphLaplacianPair mGraphLaplacianPair;
};

#endif // !PIPELINE
