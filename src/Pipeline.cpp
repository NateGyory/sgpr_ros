#include <Pipeline.h>

void Pipeline::PostProcess(int dataset) {
  initDataLoader(dataset);
  mDataLoader->ParseConfig(mSceneMap);

  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          &Processing::PointCloud::ExtractObjectPointClouds);

  // 2) TODO Process Radius
  // 3) TODO Process Laplacian
  // 4) TODO Process Eigenvalues
  // 5) TODO Create keyframe
  // 6) TODO Populate KeyFrameDB with this info

  // For all query scans 
  // 1) TODO Process PointCloud
  // 2) TODO Process Radius
  // 3) TODO Process Laplacian
  // 4) TODO Process Eigenvalues
  // 5) TODO Create keyframe
  // 6) TODO Match keyframe against KeyframeDB
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
