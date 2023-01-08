#include <Pipeline.h>
#include <matplot/matplot.h>

using namespace matplot;

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

void Pipeline::PlotHistograms() {
  auto f = figure(true);
  f->width(f->width() * 3);
  f->height(f->height() * 2.5);
  f->x_position(10);
  f->y_position(10);

  std::vector<double> ref = arma::conv_to<std::vector<double>>::from(
      mGraphLaplacianPair.first->eigenvalues);
  std::vector<double> query = arma::conv_to<std::vector<double>>::from(
      mGraphLaplacianPair.second->eigenvalues);

  double min_ref = *std::min_element(ref.begin(), ref.end());
  double max_ref = *std::max_element(ref.begin(), ref.end());
  double min_query = *std::min_element(query.begin(), query.end());
  double max_query = *std::max_element(query.begin(), query.end());

  double min = std::min(min_ref, min_query);
  double max = std::max(max_ref, max_query);

  double bin_width = (max - min) / 25;

  auto h1 = hist(ref);
  h1->face_color("r");
  h1->edge_color("r");
  h1->bin_width(bin_width);
  hold(on);
  auto h2 = hist(query);
  h2->face_color("b");
  h2->edge_color("b");
  h2->bin_width(bin_width);
  title("Eigenvalue Spectras");
  f->draw();
  show();
}

void Pipeline::ComputeEigs(int eigs_num) {
  Processing::Eigen::NMT::computeEigenValues(mGraphLaplacianPair.first,
                                             eigs_num);
  Processing::Eigen::NMT::computeEigenValues(mGraphLaplacianPair.second,
                                             eigs_num);
}

void Pipeline::ComputeLaplcian(int laplacian_type) {
  switch (laplacian_type) {
  case 0:
    Processing::Laplacian::NMT::genericLaplacian(mGraphLaplacianPair.first);
    Processing::Laplacian::NMT::genericLaplacian(mGraphLaplacianPair.second);
  case 1:
    // TODO Normalized laplacian
    break;
  default:
    break;
  }
}

void Pipeline::ComputeEdges(int edge_heuristic) {
  mGraphLaplacianPair.first.reset();
  mGraphLaplacianPair.second.reset();
  mGraphLaplacianPair = std::make_pair(std::make_shared<GraphLaplacian>(),
                                       std::make_shared<GraphLaplacian>());
  switch (edge_heuristic) {
  case 0:
    // pl.Knn();
  case 1:
    Processing::PointCloud::NMT::computeMCAR(mPointCloudPair.first,
                                             mGraphLaplacianPair.first);
    Processing::PointCloud::NMT::computeMCAR(mPointCloudPair.second,
                                             mGraphLaplacianPair.second);
    break;
  case 2:
    // pl.FullyConnected();
    break;
  default:
    break;
  }
}

void Pipeline::FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  double leafInc = 0.01;
  double leafValue = 0.1;
  while (cloud->points.size() > 10000) {
    std::cout << "Filtering cloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vox_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leafValue, leafValue, leafValue);
    sor.filter(*vox_cloud);

    cloud.reset();
    cloud = vox_cloud;
    leafValue += leafInc;
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

  // Filter point clouds
  FilterCloud(mPointCloudPair.first);
  FilterCloud(mPointCloudPair.second);
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
  std::for_each(
      mSceneMap.begin(), mSceneMap.end(),
      [](std::pair<const std::string, Scene> &pair) {
        Processing::PointCloud::DatasetTesting::ExtractObjectPointClouds(
            pair.second);
      });
  std::cout << "Finished ExtractObjectPointClouds" << std::endl;
}

void Pipeline::MCAR() {
  std::cout << "MCAR" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::DatasetTesting::
                      MinimallyConnectedAdaptiveRadius(pair.second);
                });
  std::cout << "Finished MCAR" << std::endl;
}

void Pipeline::IDW() {
  std::cout << "IDW" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Laplacian::DatasetTesting::IDWLaplacian(
                      pair.second);
                });
  std::cout << "Finished IDW" << std::endl;
}

void Pipeline::Eigs() {
  std::cout << "eigs" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::Eigen::DatasetTesting::Eigendecomposition(
                      pair.second, 50);
                });
  std::cout << "Finished eigs" << std::endl;
}

// void Pipeline::PostProcess(int dataset) {
//   initDataLoader(dataset);
//   mDataLoader->ParseConfig(mSceneMap);
//
//   // TODO do this in paralled
//   std::for_each(mSceneMap.begin(), mSceneMap.end(),
//                 [](std::pair<const std::string, Scene> &pair) {
//                   Processing::PointCloud::DatasetTesting::ExtractObjectPointClouds(pair.second);
//                 });
//
//   // TODO find radius distribution
//   std::for_each(mSceneMap.begin(), mSceneMap.end(),
//                 [](std::pair<const std::string, Scene> &pair) {
//                   Processing::PointCloud::DatasetTesting::MinimallyConnectedAdaptiveRadius(
//                       pair.second);
//                 });
//
//   // 3) TODO Process Laplacian
//   std::for_each(mSceneMap.begin(), mSceneMap.end(),
//                 [](std::pair<const std::string, Scene> &pair) {
//                   Processing::Laplacian::IDWLaplacian(pair.second);
//                 });
//
//   // 4) TODO Process Eigenvalues
//   std::for_each(mSceneMap.begin(), mSceneMap.end(),
//                 [](std::pair<const std::string, Scene> &pair) {
//                   Processing::Eigen::Eigendecomposition(pair.second, 50);
//                 });
//
//   // 5) TODO Create keyframe
//   // std::for_each(
//   //        mSceneMap.begin(),
//   //        mSceneMap.end(),
//   //        [](std::pair<std::string, Scene> &pair) {
//   //          // TODO Create keyframe
//   //        });
//   // 6) TODO Populate KeyFrameDB with this info
//
//   // For all query scans
//   // 1) TODO Process PointCloud
//   // 2) TODO Process Radius
//   // 3) TODO Process Laplacian
//   // 4) TODO Process Eigenvalues
//   // 5) TODO Create keyframe
//   // 6) TODO Match keyframe against KeyframeDB
// }

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
