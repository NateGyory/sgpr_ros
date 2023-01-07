#include <Pipeline.h>

// void Pipeline::CloudViz(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
//                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
//
//   pcl::visualization::CloudViewer viewer1("Cloud Viewer 1");
//   pcl::visualization::CloudViewer viewer2("Cloud Viewer 2");
//   viewer1.showCloud(cloud1);
//   viewer2.showCloud(cloud2);
//   std::cout << '\n' << "Press Enter";
//   while (std::cin.get() != '\n') {
//   }
// }

void Pipeline::ComputeEdges(int edge_heuristic, double &radius1, double &radius2) {
  switch (edge_heuristic) {
  case 0:
    //pl.Knn();
  case 1:
    radius1 = Processing::PointCloud::cloudComputeMCAR(mPointCloudPair.first);
    radius2 = Processing::PointCloud::cloudComputeMCAR(mPointCloudPair.second);
    break;
  case 2:
    //pl.FullyConnected();
    break;
  default:
    break;
  }
}

void Pipeline::ParsePointCloudPair(std::string f_ply1, std::string f_ply2) {
  mPointCloudPair = std::make_pair(
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));

  if (pcl::io::loadPLYFile(f_ply1, *mPointCloudPair.first) == -1) {
    PCL_ERROR("Error reading PLY file\n");
  }

  if (pcl::io::loadPLYFile(f_ply2, *mPointCloudPair.second) == -1) {
    PCL_ERROR("Error reading PLY file\n");
  }
}

void Pipeline::ParseDataset(int dataset) {
  std::cout << "Parsing Dataset" << std::endl;
  mSceneMap.clear();
  initDataLoader(dataset);
  mDataLoader->ParseConfig(mSceneMap);
  std::cout << "Finished Parsing Dataset" << std::endl;
}

void Pipeline::ExtractObjectPointClouds() {
  std::cout << "ExtractObjectPointClouds" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::ExtractObjectPointClouds(pair.second);
                });
  std::cout << "Finished ExtractObjectPointClouds" << std::endl;
}

void Pipeline::MCAR() {
  std::cout << "MCAR" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::MinimallyConnectedAdaptiveRadius(
                      pair.second);
                });
  std::cout << "Finished MCAR" << std::endl;
}

void Pipeline::IDW() {
  std::cout << "IDW" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Laplacian::IDWLaplacian(pair.second);
                });
  std::cout << "Finished IDW" << std::endl;
}

void Pipeline::Eigs() {
  std::cout << "eigs" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Eigen::Eigendecomposition(pair.second, 50);
                });
  std::cout << "Finished eigs" << std::endl;
}

void Pipeline::PostProcess(int dataset) {
  initDataLoader(dataset);
  mDataLoader->ParseConfig(mSceneMap);

  // TODO do this in paralled
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::ExtractObjectPointClouds(pair.second);
                });

  // TODO find radius distribution
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::MinimallyConnectedAdaptiveRadius(
                      pair.second);
                });

  // 3) TODO Process Laplacian
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Laplacian::IDWLaplacian(pair.second);
                });

  // 4) TODO Process Eigenvalues
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Eigen::Eigendecomposition(pair.second, 50);
                });

  // 5) TODO Create keyframe
  // std::for_each(
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
