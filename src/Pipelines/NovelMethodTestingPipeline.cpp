#include "Pipelines/NovelMethodTestingPipeline.h"
#include "Processing/PointCloud.hpp"
#include "Types/Scene.h"
#include <memory>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <utility>

void NovelMethodTestingPipeline::ParsePointCloudPair(std::string f_ply1,
                                                     std::string f_ply2,
                                                     int max_points) {
  Scene s1, s2;

  s1.scan_id = "1";
  s1.ply_file_path = f_ply1;
  s1.is_reference = true;
  s1.reference_id_match = "";
  s1.spectral_objects.emplace_back(SpectralObject());
  s1.spectral_objects[0].cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  s2.scan_id = "2";
  s2.ply_file_path = f_ply2;
  s2.is_reference = false;
  s2.reference_id_match = "1";
  s2.spectral_objects.emplace_back(SpectralObject());
  s2.spectral_objects[0].cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // TODO Read into an XYZ then copy to an XYZRGB cloud
  // pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  if (pcl::io::loadPLYFile(f_ply1, *s1.spectral_objects[0].cloud) == -1) {
    PCL_ERROR("Error reading PLY file\n");
  }

  // pcl::copyPointCloud(xyz_cloud, *s1.spectral_objects[0].cloud);
  // xyz_cloud.clear();

  if (pcl::io::loadPLYFile(f_ply2, *s2.spectral_objects[0].cloud) == -1) {
    PCL_ERROR("Error reading PLY file\n");
  }

  // pcl::copyPointCloud(xyz_cloud, *s1.spectral_objects[0].cloud);
  //int32_t rgb = (static_cast<uint32_t>(255) << 16 |
  //               static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));

  for (auto &p : s1.spectral_objects[0].cloud->points) {
    p.r = 255;
    p.g = 255;
    p.b = 255;
  }
  for (auto &p : s2.spectral_objects[0].cloud->points) {
    p.r = 255;
    p.g = 255;
    p.b = 255;
  }

  Processing::PointCloud::filterPointCloud(s1.spectral_objects[0].cloud,
                                           max_points);
  Processing::PointCloud::filterPointCloud(s2.spectral_objects[0].cloud,
                                           max_points);

  mScenePair = std::make_pair(s1, s2);
}

void NovelMethodTestingPipeline::ComputeEdges(int edge_heuristic) {
  // mGraphLaplacianPair.first.reset();
  // mGraphLaplacianPair.second.reset();
  // mGraphLaplacianPair = std::make_pair(std::make_shared<GraphLaplacian>(),
  //                                      std::make_shared<GraphLaplacian>());

  // TODO need a reset function for the future

  switch (edge_heuristic) {
  case 0:
    // pl.Knn();
    break;
  case 1:
    Processing::PointCloud::computeMCAR(mScenePair.first.spectral_objects[0]);
    Processing::PointCloud::computeMCAR(mScenePair.second.spectral_objects[0]);
    break;
  case 2:
    // pl.FullyConnected();
    break;
  default:
    break;
  }
}

void NovelMethodTestingPipeline::ComputeLaplacian(int laplacian_type) {
  switch (laplacian_type) {
  case 0:
    Processing::Laplacian::genericLaplacian(
        mScenePair.first.spectral_objects[0]);
    Processing::Laplacian::genericLaplacian(
        mScenePair.second.spectral_objects[0]);
    break;
  case 1:
    Processing::Laplacian::normalizedLaplacian(
        mScenePair.first.spectral_objects[0]);
    Processing::Laplacian::normalizedLaplacian(
        mScenePair.second.spectral_objects[0]);
    break;
  default:
    break;
  }
}

void NovelMethodTestingPipeline::ComputeEigs(int eigs_num) {
  Processing::Eigen::computeEigenvalues(mScenePair.first.spectral_objects[0],
                                        eigs_num);
  Processing::Eigen::computeEigenvalues(mScenePair.second.spectral_objects[0],
                                        eigs_num);
}

void NovelMethodTestingPipeline::SaveEigenvalues(std::string file_name) {
  std::string file_path =
      "/home/nate/Development/catkin_ws/src/sgpr_ros/data/" + file_name;

  json j;
  std::ofstream o(file_path);

  // {
  //   reference_scans: [
  //     scan_id:
  //     ply_color:
  //     {
  //       label:
  //       global_id:
  //       eigenvalues:
  //     }
  //   ],
  //   query_scans: [
  //     scan_id:
  //     reference_scan_id:
  //     ply_color:
  //     {
  //       label:
  //       global_id:
  //       eigenvalues:
  //     }
  //   ],
  // }

  j["reference_scans"] = std::vector<json>();
  j["query_scans"] = std::vector<json>();

  Processing::Eigen::SaveEigenvalues(j["reference_scans"],
                                     mScenePair.first.spectral_objects[0]);
  Processing::Eigen::SaveEigenvalues(j["query_scans"],
                                     mScenePair.second.spectral_objects[0]);

  o << std::setw(4) << j << std::endl;
}

void NovelMethodTestingPipeline::PlotHistograms() {
  auto f = figure(true);
  f->width(f->width() * 3);
  f->height(f->height() * 2.5);
  f->x_position(10);
  f->y_position(10);

  std::vector<double> ref = arma::conv_to<std::vector<double>>::from(
      mScenePair.first.spectral_objects[0].eigenvalues);
  std::vector<double> query = arma::conv_to<std::vector<double>>::from(
      mScenePair.second.spectral_objects[0].eigenvalues);

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
