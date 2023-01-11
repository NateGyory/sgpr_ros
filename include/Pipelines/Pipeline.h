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

#include <Types/GraphLaplacian.h>

using GraphLaplacianPair =
    std::pair<std::shared_ptr<GraphLaplacian>, std::shared_ptr<GraphLaplacian>>;

class Pipeline {
public:
  virtual ~Pipeline(){};

  virtual void ParseDataset() = 0;
  virtual void ExtractObjectPointClouds(int max_pts) = 0;
  virtual void ComputeEdges(int edge_heuristic) = 0;
  virtual void ComputeLaplacian(int laplacian_type) = 0;
  virtual void ComputeEigs(int max_eigs) = 0;
  virtual void SaveEigenvalues(std::string file_name) = 0;

protected:
  scene_map_t mSceneMap;
  spKeyFrameDB mKeyFrameDB;
};
