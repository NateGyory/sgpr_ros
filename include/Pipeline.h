#ifndef PIPELINE
#define PIPELINE

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "DataLoaders/DataLoader.h"
#include "DataLoaders/Matterport3D.h"
#include "DataLoaders/RScanDataLoader.h"
#include "DataLoaders/SemanticKitti.h"
#include "Scene.h"

#include <KeyFrames/KeyFrameDB.h>

#include <Processing/PointCloud.hpp>
#include <Processing/Laplacian.hpp>
#include <Processing/Eigen.hpp>

class Pipeline {
public:
  Pipeline() = default;
  ~Pipeline() = default;

  void PostProcess(int dataset); // NOTE: not using
  void RealTime(){}; // NOTE: not using
                     //
  void ParseDataset(int dataset);
  void ExtractObjectPointClouds();
  void MCAR();
  void IDW();
  void Eigs();


private:
  void initDataLoader(int dataset);

  scene_map_t mSceneMap;
  spDataLoader mDataLoader;
  spKeyFrameDB mKeyFrameDB;
};

#endif // !PIPELINE
