#include <Pipeline.h>

void Pipeline::ParseDataset(int dataset) {
  std::cout << "Parsing Dataset" << std::endl;
  mSceneMap.clear();
  initDataLoader(dataset);
  mDataLoader->ParseConfig(mSceneMap);
  std::cout << "Finished Parsing Dataset" << std::endl;
}

void Pipeline::ExtractObjectPointClouds() {
  std::cout << "ExtractObjectPointClouds" << std::endl;
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::PointCloud::ExtractObjectPointClouds(pair.second);
          });
  std::cout << "Finished ExtractObjectPointClouds" << std::endl;
}

void Pipeline::MCAR() {
  std::cout << "MCAR" << std::endl;
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::PointCloud::MinimallyConnectedAdaptiveRadius(pair.second);
          });
  std::cout << "Finished MCAR" << std::endl;
}

void Pipeline::IDW() {
  std::cout << "IDW" << std::endl;
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::Laplacian::IDWLaplacian(pair.second);
          });
  std::cout << "Finished IDW" << std::endl;
}

void Pipeline::Eigs() {
  std::cout << "eigs" << std::endl;
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::Eigen::Eigendecomposition(pair.second, 50);
          });
  std::cout << "Finished eigs" << std::endl;
}

void Pipeline::PostProcess(int dataset) {
  initDataLoader(dataset);
  mDataLoader->ParseConfig(mSceneMap);

  // TODO do this in paralled
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::PointCloud::ExtractObjectPointClouds(pair.second);
          });
  

  // TODO find radius distribution
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::PointCloud::MinimallyConnectedAdaptiveRadius(pair.second);
          });

  // 3) TODO Process Laplacian
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::Laplacian::IDWLaplacian(pair.second);
          });

  // 4) TODO Process Eigenvalues
  std::for_each(
          mSceneMap.begin(),
          mSceneMap.end(),
          [](std::pair<const std::string, Scene> &pair) {
            Processing::Eigen::Eigendecomposition(pair.second, 50);
          });

  // 5) TODO Create keyframe
  //std::for_each(
  //        mSceneMap.begin(),
  //        mSceneMap.end(),
  //        [](std::pair<std::string, Scene> &pair) {
  //          // TODO Create keyframe
  //        });
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
