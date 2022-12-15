#include <Pipeline.h>

void Pipeline::PostProcess(int dataset) {
  initDataLoader(dataset);
  mDataLoader->ParseConfig();

  // For all reference scans
  // 1) Process PointCloud
  // 2) Process Radius
  // 3) Process Laplacian
  // 4) Process Eigenvalues
  // 5) Create keyframe
  // 6) Populate KeyFrameDB with this info

  // For all query scans 
  // 1) Process PointCloud
  // 2) Process Radius
  // 3) Process Laplacian
  // 4) Process Eigenvalues
  // 5) Create keyframe
  // 6) Match keyframe against KeyframeDB
}

void Pipeline::initDataLoader(int dataset) {
  switch (dataset) {
  case 0:
    mDataLoader = std::make_shared<RScanDataLoader>();
    break;
  case 1:
    mDataLoader = std::make_shared<Matterport3D>();
    break;
  case 2:
    mDataLoader = std::make_shared<SemanticKitti>();
    break;
  default:;
    // TODO add exit error and Fatal log statement
  }
}
