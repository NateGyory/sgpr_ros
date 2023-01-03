#ifndef PIPELINE
#define PIPELINE

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "DataLoaders/DataLoader.h"
#include "DataLoaders/Matterport3D.h"
#include "DataLoaders/RScanDataLoader.h"
#include "DataLoaders/SemanticKitti.h"
#include "Scene.h"

#include <KeyFrames/KeyFrameDB.h>

#include <Processing/Eigen.hpp>
#include <Processing/Laplacian.hpp>
#include <Processing/PointCloud.hpp>

using PointCloudPair = std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr>;

class Pipeline {
public:
  Pipeline() = default;
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
  void ParsePointCloudPair(std::string f_ply1, std::string f_ply2);
  void CreateViewers();
  void CloudViz(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);

  PointCloudPair GetPointCloudPair() { return mPointCloudPair; }

private:
  void initDataLoader(int dataset);

  scene_map_t mSceneMap;
  spDataLoader mDataLoader;
  spKeyFrameDB mKeyFrameDB;
  PointCloudPair mPointCloudPair;
};

#endif // !PIPELINE
