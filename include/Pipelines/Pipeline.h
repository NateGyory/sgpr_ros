#pragma once

#include <algorithm>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vector>
#include <utility>

#include "Types/Scene.h"

#include <KeyFrames/KeyFrameDB.h>

#include <Processing/Eigen.hpp>
#include <Processing/Laplacian.hpp>
#include <Processing/PointCloud.hpp>

class Pipeline {
public:
  virtual ~Pipeline(){};

  virtual void ParseDataset() = 0;
  virtual void ExtractObjectPointClouds(int max_pts) = 0;
  virtual void ComputeEdges(int edge_heuristic) = 0;
  virtual void ComputeLaplacian(int laplacian_type) = 0;
  virtual void ComputeEigs(int max_eigs) = 0;
  virtual void SaveEigenvalues(std::string file_name) = 0;
  virtual void GetQueryScans(std::vector<std::string> &query_scans) = 0;
  virtual void GetRefScans(std::vector<std::string> &ref_scans) = 0;
  virtual const char* GetMappedRefScan(std::vector<std::string> &query_scans, int query_scan_idx) = 0;

protected:
  scene_map_t mSceneMap;
  spKeyFrameDB mKeyFrameDB;

  std::pair<std::string, std::string> mQRScanPair;
  std::pair<int, int> mQRObjIdx;
};
