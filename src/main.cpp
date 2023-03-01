#include <algorithm>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <cmath>
#include <iostream>
#include <matplot/freestanding/axes_functions.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>

#include <sgpr_ros/Eigenvalues.h>

#include "Pipelines/NovelMethodTestingPipeline.h"
#include "Pipelines/RScanPipeline.h"

#include "Processing/Eigen.hpp"
#include "Processing/Files.hpp"
#include "Processing/Laplacian.hpp"
#include "Processing/PointCloud.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_state.h"
#include "ros/duration.h"
#include <stdio.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <vtkIOStream.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

using namespace std::chrono_literals;
using namespace matplot;

struct Result {
  // int obj_match_vote;
  // int obj_dnm_vote;
  // int obj_total;
  double obj_match_ratio;
  bool is_match;
};

ros::ServiceClient evaluation_service_client;
pcl::visualization::PCLVisualizer::Ptr viewer;
matplot::figure_handle f;

// matplot::figure_handle f_gfa;

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

std::string GetSequenceString(int val) {
  std::string seq_string;
  switch (val) {
  case 0:
    seq_string = "/00/";
    break;
  case 1:
    seq_string = "/01/";
    break;
  case 2:
    seq_string = "/02/";
    break;
  case 3:
    seq_string = "/03/";
    break;
  case 4:
    seq_string = "/04/";
    break;
  case 5:
    seq_string = "/05/";
    break;
  case 6:
    seq_string = "/06/";
    break;
  case 7:
    seq_string = "/07/";
    break;
  case 8:
    seq_string = "/08/";
    break;
  default:
    break;
  }
  return seq_string;
}

void PlotSpectra() {
  if (f.use_count() == 0) {
    f = matplot::figure(true);
    f->width(f->width() * 3);
    f->height(f->height() * 2.5);
    f->x_position(72);
    f->y_position(760);
    f->size(960, 380);
  }

  while (1) {
    if (ImGuiState::DatasetTesting::update_hist) {
      ImGuiState::DatasetTesting::eigs_mtx.lock();
      cla();
      std::vector<double> ref =
          ImGuiState::DatasetTesting::eig_srv.request.r_eigs;
      std::vector<double> query =
          ImGuiState::DatasetTesting::eig_srv.request.q_eigs;

      if (ref.size() > 0 && query.size() > 0) {
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
        ImGuiState::DatasetTesting::update_hist = false;
        ImGuiState::DatasetTesting::eigs_mtx.unlock();
      }
    }
  }
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void) {
  if (event.getKeySym() == "r" && event.keyDown()) {
    std::cout << "r was pressed => removing all text" << std::endl;
    ImGuiState::DatasetTesting::mtx.lock();
    ImGuiState::DatasetTesting::show_radius =
        !ImGuiState::DatasetTesting::show_radius;
    ImGuiState::DatasetTesting::radius_toggled = true;
    ImGuiState::DatasetTesting::mtx.unlock();
  }
}

void BackgroundVizThread() {

  int v0(0);
  int v1(1);

  // TODO I can probably delete this
  if (viewer.use_count() == 0) {

    viewer = std::make_shared<pcl::visualization::PCLVisualizer>();
    viewer->setPosition(72, 0);
    viewer->setSize(600, 200);
    viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);

    viewer->initCameraParameters();
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v0);
    viewer->setBackgroundColor(0, 0, 0, v0);
    // viewer->addCoordinateSystem(1.0, "coord1", v0);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
    // viewer->addCoordinateSystem(1.0, "coord2", v1);
  }

  while (!viewer->wasStopped()) {
    ImGuiState::DatasetTesting::mtx.lock();
    if (ImGuiState::DatasetTesting::radius_toggled) {
      if (ImGuiState::DatasetTesting::show_radius) {

        auto start = std::chrono::steady_clock::now();
        unsigned int max_nn = 1000;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdTree;

        // TODO Maybe separate this so we are not duplicating code
        kdTree.setInputCloud(ImGuiState::DatasetTesting::q_so.cloud);
        int size1 = ImGuiState::DatasetTesting::q_so.cloud->size();

        for (int i = 0; i < size1; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, ImGuiState::DatasetTesting::q_so.mcar,
                              indicies_found, squaredDistances, max_nn);

          pcl::PointXYZRGB pt_r =
              ImGuiState::DatasetTesting::q_so.cloud->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q =
                ImGuiState::DatasetTesting::q_so.cloud->points[idx];
            // viewer->addLine(pt_r, pt_q, "line", 0);
            std::string string_id =
                "0" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v0);
          }
        }

        kdTree.setInputCloud(ImGuiState::DatasetTesting::r_so.cloud);
        int size2 = ImGuiState::DatasetTesting::r_so.cloud->size();

        for (int i = 0; i < size2; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, ImGuiState::DatasetTesting::r_so.mcar,
                              indicies_found, squaredDistances, max_nn);

          pcl::PointXYZRGB pt_r =
              ImGuiState::DatasetTesting::r_so.cloud->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q =
                ImGuiState::DatasetTesting::r_so.cloud->points[idx];
            std::string string_id =
                "1" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v1);
          }
        }
      } else {
        viewer->removeAllShapes(v0);
        viewer->removeAllShapes(v1);
      }
      ImGuiState::DatasetTesting::radius_toggled = false;
    }

    if (ImGuiState::DatasetTesting::update_cloud) {
      // TODO try and replace this with updateCloud instead if everything
      // works
      viewer->removeAllPointClouds(v0);
      viewer->removeAllPointClouds(v1);
      viewer->removeAllShapes(v0);
      viewer->removeAllShapes(v1);

      viewer->addPointCloud(ImGuiState::DatasetTesting::q_so.cloud, "cloud1",
                            v0);
      viewer->addPointCloud(ImGuiState::DatasetTesting::r_so.cloud, "cloud2",
                            v1);

      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1", v0);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2", v1);

      // Add Label and ID
      std::string name1 =
          ImGuiState::DatasetTesting::q_so.label + ": " +
          std::to_string(ImGuiState::DatasetTesting::q_so.global_id) + " : " +
          std::to_string(ImGuiState::DatasetTesting::q_so.scene_id);
      std::string name2 =
          ImGuiState::DatasetTesting::r_so.label + ": " +
          std::to_string(ImGuiState::DatasetTesting::r_so.global_id) + " : " +
          std::to_string(ImGuiState::DatasetTesting::r_so.scene_id);

      viewer->addText(name1, 0, 0, 25, 1, 1, 1, "text1", v0);
      viewer->addText(name2, 0, 0, 25, 1, 1, 1, "text2", v1);
      // Add radii
      std::string rsize1 =
          "Radius Size: " +
          std::to_string(ImGuiState::DatasetTesting::q_so.mcar);
      std::string rsize2 =
          "Radius Size: " +
          std::to_string(ImGuiState::DatasetTesting::r_so.mcar);
      viewer->addText(rsize1, 0, 25, 25, 1, 1, 1, "text3", v0);
      viewer->addText(rsize2, 0, 25, 25, 1, 1, 1, "text4", v1);

      // Add cloud size
      std::string csize1 =
          "Cloud Size: " +
          std::to_string(ImGuiState::DatasetTesting::q_so.cloud->size());
      std::string csize2 =
          "Cloud Size: " +
          std::to_string(ImGuiState::DatasetTesting::r_so.cloud->size());
      viewer->addText(csize1, 0, 50, 25, 1, 1, 1, "text5", v0);
      viewer->addText(csize2, 0, 50, 25, 1, 1, 1, "text6", v1);

      if (ImGuiState::DatasetTesting::show_radius) {
        unsigned int max_nn = 1000;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdTree;

        // TODO Maybe separate this so we are not duplicating code
        kdTree.setInputCloud(ImGuiState::DatasetTesting::q_so.cloud);
        int size1 = ImGuiState::DatasetTesting::q_so.cloud->size();

        for (int i = 0; i < size1; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, ImGuiState::DatasetTesting::q_so.mcar,
                              indicies_found, squaredDistances, max_nn);

          pcl::PointXYZRGB pt_r =
              ImGuiState::DatasetTesting::q_so.cloud->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q =
                ImGuiState::DatasetTesting::q_so.cloud->points[idx];
            // viewer->addLine(pt_r, pt_q, "line", 0);
            std::string string_id =
                "0" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v0);
          }
        }

        kdTree.setInputCloud(ImGuiState::DatasetTesting::r_so.cloud);
        int size2 = ImGuiState::DatasetTesting::r_so.cloud->size();

        for (int i = 0; i < size2; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, ImGuiState::DatasetTesting::r_so.mcar,
                              indicies_found, squaredDistances, max_nn);

          pcl::PointXYZRGB pt_r =
              ImGuiState::DatasetTesting::r_so.cloud->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q =
                ImGuiState::DatasetTesting::r_so.cloud->points[idx];
            std::string string_id =
                "1" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v1);
          }
        }
      }

      ImGuiState::DatasetTesting::update_cloud = false;
    }
    ImGuiState::DatasetTesting::mtx.unlock();
    viewer->spinOnce(1);
    // std::this_thread::sleep_for(100ms);
  }

  viewer->close();
  std::cout << "closed!" << std::endl;
}

GLFWwindow *initGUI() {
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit())
    exit(1);

  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  // Create window with graphics context
  GLFWwindow *window =
      glfwCreateWindow(500, 650, "SGPR Data Explorer", NULL, NULL);
  if (window == NULL)
    exit(1);
  glfwSetWindowPos(window, 1032, 0);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return window;
}

void semanticKittiTestingPipeline(std::shared_ptr<Pipeline> &pl) {

  ImGui::Begin("Dataset Testing Pipeline");

  // --------------------------------------------------------------
  ImGui::Text("Dataset");

  ImGui::Combo("Choose Dataset", &ImGuiState::DatasetTesting::dataset_idx,
               ImGuiState::datasets, IM_ARRAYSIZE(ImGuiState::datasets));

  // ImGui::InputInt("Max number of points in point cloud",
  //                 &ImGuiState::DatasetTesting::max_pts);

  ImGui::InputInt("Start Ref ID", &ImGuiState::DatasetTesting::start_ref_scene);
  ImGui::InputInt("Final Ref ID", &ImGuiState::DatasetTesting::last_ref_scene);
  ImGui::InputInt("Start Query ID",
                  &ImGuiState::DatasetTesting::start_query_scene);
  ImGui::InputInt("Final Query ID",
                  &ImGuiState::DatasetTesting::last_query_scene);
  ImGui::InputInt("Sequence", &ImGuiState::DatasetTesting::sequence);

  if (ImGui::Button("Button 1")) {
    pl.reset();
    switch (ImGuiState::DatasetTesting::dataset_idx) {
    case 0:
      pl = std::make_shared<RScanPipeline>();
      break;
    case 1:
      // pl = std::make_shared<MatterPortPipeline>();
      break;
    case 2:
      pl = std::make_shared<RScanPipeline>();
      // pl->last_scene_id = ImGuiState::DatasetTesting::last_scene;
      // pl->start_scene_id = ImGuiState::DatasetTesting::start_scene;
      //  pl = std::make_shared<SemanticKittiPipeline>();
      break;
    default:
      break;
    }

    pl->ParseDataset(ImGuiState::DatasetTesting::dataset_idx,
                     ImGuiState::DatasetTesting::sequence);
    pl->ExtractObjsSemanticKitti(ImGuiState::DatasetTesting::max_pts);
    // pl->ExtractObjectPointClouds(ImGuiState::DatasetTesting::max_pts);
    ImGuiState::DatasetTesting::dataset_parsed = true;
    ImGuiState::DatasetTesting::sequence_string =
        GetSequenceString(ImGuiState::DatasetTesting::sequence);
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::DatasetParsed())
      ? ImGui::Text("Success! Parse datset")
      : ImGui::Text("Click to parse dataset");

  // if (!ImGuiState::DatasetTesting::DatasetParsed()) {
  //   ImGui::BeginDisabled();
  // }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Preprocessing");

  ImGui::Checkbox("SOR", &ImGuiState::DatasetTesting::sor_check);
  ImGui::Checkbox("Double SOR", &ImGuiState::DatasetTesting::double_sor);

  if (!ImGuiState::DatasetTesting::sor_check)
    ImGui::BeginDisabled();

  ImGui::InputInt("MeanK", &ImGuiState::DatasetTesting::meanK);
  ImGui::InputDouble("StdDevThresh", &ImGuiState::DatasetTesting::stdThresh);

  if (!ImGuiState::DatasetTesting::sor_check)
    ImGui::EndDisabled();

  ImGui::Combo("Filtering", &ImGuiState::DatasetTesting::filtering_idx,
               ImGuiState::filtering, IM_ARRAYSIZE(ImGuiState::filtering));

  ImGui::RadioButton("Smallest Cloud",
                     &ImGuiState::DatasetTesting::filtering_opts, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Percent", &ImGuiState::DatasetTesting::filtering_opts, 1);
  ImGui::SameLine();
  ImGui::RadioButton("Sample Size", &ImGuiState::DatasetTesting::filtering_opts,
                     2);

  if (!ImGuiState::DatasetTesting::FilterPercent())
    ImGui::BeginDisabled();

  ImGui::InputDouble("%", &ImGuiState::DatasetTesting::filter_percent);

  if (!ImGuiState::DatasetTesting::FilterPercent())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::SampleSize())
    ImGui::BeginDisabled();

  ImGui::InputInt("Sample #", &ImGuiState::DatasetTesting::sample_size);

  if (!ImGuiState::DatasetTesting::SampleSize())
    ImGui::EndDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Graph Formulation");

  ImGui::Combo(
      "Edge Heuristic", &ImGuiState::DatasetTesting::edge_heuristic_idx,
      ImGuiState::edge_heuristics, IM_ARRAYSIZE(ImGuiState::edge_heuristics));

  ImGui::Checkbox("Same Radius", &ImGuiState::DatasetTesting::same_radius);

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Laplacian");

  ImGui::Combo("Laplacian Algorithm",
               &ImGuiState::DatasetTesting::laplacian_idx,
               ImGuiState::laplacians, IM_ARRAYSIZE(ImGuiState::laplacians));

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Eigenvalues");

  ImGui::RadioButton("All Eigenvalues",
                     &ImGuiState::DatasetTesting::eigendecomposition_method, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Specific # of Eigenvalues",
                     &ImGuiState::DatasetTesting::eigendecomposition_method, 1);

  if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
    ImGui::BeginDisabled();

  ImGui::InputInt("# of Eigenvalues to compute",
                  &ImGuiState::DatasetTesting::eigs_number);

  if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
    ImGui::EndDisabled();

  // --------------------------------------------------------------
  // ImGui::Separator();
  // ImGui::Text("Choose Scans");

  // if (ImGuiState::DatasetTesting::GetQueryScans()) {
  //   pl->GetQueryScans(ImGuiState::DatasetTesting::query_scans);
  // }

  // const char *selected_query_scan =
  //     ImGuiState::DatasetTesting::GetSelectedQueryScan();

  // if (ImGui::BeginCombo("Choose A Query Scan", selected_query_scan)) {
  //   for (int i = 0; i < ImGuiState::DatasetTesting::query_scans.size(); i++)
  //   {
  //     const bool isSelected = (ImGuiState::DatasetTesting::query_scan_idx ==
  //     i); if
  //     (ImGui::Selectable(ImGuiState::DatasetTesting::query_scans[i].c_str(),
  //                           isSelected)) {
  //       ImGuiState::DatasetTesting::query_scan_idx = i;
  //     }

  //    if (isSelected) {
  //      ImGui::SetItemDefaultFocus();
  //    }
  //  }
  //  ImGui::EndCombo();
  //}

  // const char *selected_ref_scan =
  //     (ImGuiState::DatasetTesting::query_scans.size() > 0)
  //         ? pl->GetMappedRefScan(ImGuiState::DatasetTesting::query_scans,
  //                                ImGuiState::DatasetTesting::query_scan_idx)
  //         : "";

  // if (ImGuiState::DatasetTesting::GetRefScans())
  //   pl->GetRefScans(ImGuiState::DatasetTesting::ref_scans);

  // if (ImGuiState::DatasetTesting::UpdateRefScanSelected()) {
  //   ImGuiState::DatasetTesting::last_query_scan_idx =
  //       ImGuiState::DatasetTesting::query_scan_idx;
  //   std::string ref = std::string(selected_ref_scan);
  //   auto it = std::find(ImGuiState::DatasetTesting::ref_scans.begin(),
  //                       ImGuiState::DatasetTesting::ref_scans.end(), ref);

  //  if (it != ImGuiState::DatasetTesting::ref_scans.end()) {
  //    ImGuiState::DatasetTesting::ref_scan_idx =
  //        it - ImGuiState::DatasetTesting::ref_scans.begin();
  //  } else {
  //    std::cout << "ERROR reference scan not in list" << std::endl;
  //    std::exit(1);
  //  }
  //}

  // if (ImGui::BeginCombo("Choose A Reference Scan", selected_ref_scan)) {
  //   for (int i = 0; i < ImGuiState::DatasetTesting::ref_scans.size(); i++) {
  //     const bool isSelected = (ImGuiState::DatasetTesting::ref_scan_idx ==
  //     i); if
  //     (ImGui::Selectable(ImGuiState::DatasetTesting::ref_scans[i].c_str(),
  //                           isSelected)) {
  //       ImGuiState::DatasetTesting::ref_scan_idx = i;
  //     }

  //    if (isSelected) {
  //      ImGui::SetItemDefaultFocus();
  //    }
  //  }
  //  ImGui::EndCombo();
  //}

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Place Recognition");

  ImGui::InputInt("Scan buffer", &ImGuiState::DatasetTesting::scan_buffer);
  ImGui::InputDouble("Threshold For Match",
                     &ImGuiState::DatasetTesting::match_thresh);

  if (ImGui::Button("Longer Improved Method")) {
    int total_compared = 0;
    int TP = 0;
    int FP = 0;
    int FN = 0;
    int TN = 0;
    bool TN_flag = false;
    ImGuiState::DatasetTesting::sequence_string =
        GetSequenceString(ImGuiState::DatasetTesting::sequence);
    std::vector<SemanticKittiResult> results;

    std::cout << "Sequence string: "
              << ImGuiState::DatasetTesting::sequence_string << std::endl;

    int query_scan_idx = ImGuiState::DatasetTesting::start_query_scene;
    pl = std::make_shared<RScanPipeline>();
    // pl->last_scene_id = ImGuiState::DatasetTesting::last_scene;
    // pl->start_scene_id = ImGuiState::DatasetTesting::start_scene;

    while (query_scan_idx <= ImGuiState::DatasetTesting::last_query_scene) {
      std::string query_key = std::to_string(query_scan_idx) + ".ply";
      std::cout << "Query: " << query_key << std::endl;

      // get the query scene
      Scene q_scene;
      q_scene.ply_file_path = "/home/nate/Datasets/SemanticKittiPLY/" +
                              ImGuiState::DatasetTesting::sequence_string +
                              "/" + query_key;
      Processing::PointCloud::PopulateSpectralObjs(q_scene);

      int ref_scan_idx = ImGuiState::DatasetTesting::start_ref_scene;
      //while (ref_scan_idx <
      //       query_scan_idx - ImGuiState::DatasetTesting::scan_buffer) {
      while (ref_scan_idx <= ImGuiState::DatasetTesting::last_ref_scene) {
        std::string ref_key = std::to_string(ref_scan_idx) + ".ply";
        std::cout << "ref_key: " << ref_key << std::endl;
        Scene r_scene;
        r_scene.ply_file_path = "/home/nate/Datasets/SemanticKittiPLY/" +
                                ImGuiState::DatasetTesting::sequence_string +
                                "/" + ref_key;

        Processing::PointCloud::PopulateSpectralObjs(r_scene);

        // Compare all the objects in the reference and query scene
        int obj_match_vote = 0;
        int obj_dnm_vote = 0;
        int total_objs = 0;
        bool ks_result, ad_result;
        for (auto &so : q_scene.spectral_objects) {

          // Determine if to skip the object or not
          if (Processing::PointCloud::ShouldSkip(so))
            continue;

          total_objs++;

          // Find the matching object
          auto begin_it = r_scene.spectral_objects.begin();
          auto end_it = r_scene.spectral_objects.end();

          std::string ply_color = so.ply_color;

          auto it = std::find_if(begin_it, end_it,
                                 [ply_color](const SpectralObject &r_so) {
                                   return r_so.ply_color == ply_color;
                                 });

          int ref_obj_idx = it - begin_it;

          if (it != end_it) {
            // found the shared object
            // Now we need to run all the code and determine if the objs are the
            // same
            if (so.cloud->size() <= 1 ||
                r_scene.spectral_objects[ref_obj_idx].cloud->size() <= 1) {
              std::cout << "Skipping Label: " << so.label << std::endl;
              continue;
            }

            // ImGuiState::DatasetTesting::mtx.lock();

            ImGuiState::DatasetTesting::q_so = so;

            ImGuiState::DatasetTesting::r_so =
                r_scene.spectral_objects[ref_obj_idx];

            Processing::PointCloud::computeSOR(
                ImGuiState::DatasetTesting::q_so,
                ImGuiState::DatasetTesting::meanK,
                ImGuiState::DatasetTesting::stdThresh);
            Processing::PointCloud::computeSOR(
                ImGuiState::DatasetTesting::r_so,
                ImGuiState::DatasetTesting::meanK,
                ImGuiState::DatasetTesting::stdThresh);

            double size =
                pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
                            ImGuiState::DatasetTesting::sample_size,
                            ImGuiState::DatasetTesting::filter_percent,
                            ImGuiState::DatasetTesting::q_so.cloud->size(),
                            ImGuiState::DatasetTesting::r_so.cloud->size());

            Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
                                               size);
            Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
                                               size);

            if (ImGuiState::DatasetTesting::double_sor) {
              Processing::PointCloud::computeSOR(
                  ImGuiState::DatasetTesting::q_so,
                  ImGuiState::DatasetTesting::meanK,
                  ImGuiState::DatasetTesting::stdThresh);
              Processing::PointCloud::computeSOR(
                  ImGuiState::DatasetTesting::r_so,
                  ImGuiState::DatasetTesting::meanK,
                  ImGuiState::DatasetTesting::stdThresh);
            }

            Processing::PointCloud::computeMCAR(
                ImGuiState::DatasetTesting::q_so);
            Processing::PointCloud::computeMCAR(
                ImGuiState::DatasetTesting::r_so);

            if (ImGuiState::DatasetTesting::same_radius) {
              double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
                                     ImGuiState::DatasetTesting::r_so.mcar);
              ImGuiState::DatasetTesting::q_so.mcar = mcar;
              ImGuiState::DatasetTesting::r_so.mcar = mcar;
            }

            pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
                          ImGuiState::DatasetTesting::q_so);
            pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
                          ImGuiState::DatasetTesting::r_so);

            int number_eigs = ImGuiState::DatasetTesting::q_so.cloud->size();
            if (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
              number_eigs = ImGuiState::DatasetTesting::eigs_number;
            }

            Processing::Eigen::computeEigenvalues(
                ImGuiState::DatasetTesting::q_so, number_eigs);
            Processing::Eigen::computeEigenvalues(
                ImGuiState::DatasetTesting::r_so, number_eigs);

            // ImGuiState::DatasetTesting::eigs_mtx.lock();

            ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
                arma::conv_to<std::vector<double>>::from(
                    ImGuiState::DatasetTesting::q_so.eigenvalues);
            ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
                arma::conv_to<std::vector<double>>::from(
                    ImGuiState::DatasetTesting::r_so.eigenvalues);
            ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
                ImGuiState::DatasetTesting::q_so.gfaFeatures;
            ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
                ImGuiState::DatasetTesting::r_so.gfaFeatures;

            // Eval service
            if (evaluation_service_client.call(
                    ImGuiState::DatasetTesting::eig_srv)) {
              // ROS_INFO("eval service success!!! %f",
              //          ImGuiState::DatasetTesting::eig_srv.response.results[0]);

              ks_result =
                  ImGuiState::DatasetTesting::eig_srv.response.results[0];
              ad_result =
                  ImGuiState::DatasetTesting::eig_srv.response.results[1];
            } else {
              // ROS_ERROR("eval service failed");
              std::cout << "ROS srv failed" << std::endl;
            }

            // TODO need to throw the evaluation code here
            if (ks_result || ad_result) {
              obj_match_vote++;
              // std::cout << "Label: " << so.label << std::endl;
            } else {
              obj_dnm_vote++;
            }

            ImGuiState::DatasetTesting::update_cloud = true;
            ImGuiState::DatasetTesting::update_hist = true;
            // ImGuiState::DatasetTesting::mtx.unlock();
            // ImGuiState::DatasetTesting::eigs_mtx.unlock();
          }
        }

        // TODO need to add the eval totaling here, ref scene is done
        SemanticKittiResult scene_result;
        scene_result.sequence = ImGuiState::DatasetTesting::sequence_string;
        scene_result.query_id = query_key;
        scene_result.ref_id = ref_key;
        scene_result.obj_match_ratio = obj_match_vote / double(total_objs);
        // scene_result.obj_match_ratio =
        //     obj_match_vote / (double(obj_match_vote + obj_dnm_vote));
        //  scene_result.obj_match_ratio =
        //      obj_match_vote / double(r_kv.second.spectral_objects.size());
        //  scene_result.is_match = r_kv.first ==
        //  q_kv.second.reference_id_match;
        results.push_back(scene_result);

        ref_scan_idx++;
      }

      query_scan_idx++;
    }

    // TODO Save to a file
    Processing::Files::SaveSemanticKittiResults(results, ImGuiState::DatasetTesting::start_ref_scene, ImGuiState::DatasetTesting::start_query_scene);
  }

  if (ImGui::Button("Compute Spectral Features")) {
    // SOR
    pl->ComputeSOR(ImGuiState::DatasetTesting::meanK,
                   ImGuiState::DatasetTesting::stdThresh);

    // FPS
    pl->ComputeFPS(ImGuiState::DatasetTesting::filtering_opts,
                   ImGuiState::DatasetTesting::sample_size,
                   ImGuiState::DatasetTesting::filter_percent);

    // Double sided SOR
    if (ImGuiState::DatasetTesting::double_sor) {
      pl->ComputeSOR(ImGuiState::DatasetTesting::meanK,
                     ImGuiState::DatasetTesting::stdThresh);
    }

    // MCAR
    pl->ComputeEdges(ImGuiState::DatasetTesting::edge_heuristic_idx);

    // Laplacian
    pl->ComputeLaplacian(ImGuiState::DatasetTesting::laplacian_idx);

    // Eigendecomposition
    pl->ComputeEigs(ImGuiState::DatasetTesting::eigs_number);
  }

  if (ImGui::Button("Find Loop Closure's")) {
    int total_compared = 0;
    int TP = 0;
    int FP = 0;
    int FN = 0;
    int TN = 0;
    bool TN_flag = false;
    // key - query scan
    // value - array<Pair<string,Pair<int,int>>
    std::vector<SemanticKittiResult> results;

    // FIRST SAVE GFA
    // pl->SaveGFA();

    // Okay from here we have to customize for SemanticKitti
    // For every scan, test against all previous scans

    int query_scan_idx = 0;

    while (query_scan_idx <= ImGuiState::DatasetTesting::last_query_scene) {
      std::string query_key = std::to_string(query_scan_idx) + ".ply";
      std::cout << "Query: " << query_key << std::endl;

      int ref_scan_idx = 0;
      while (ref_scan_idx <
             query_scan_idx - ImGuiState::DatasetTesting::scan_buffer) {
        std::string ref_key = std::to_string(ref_scan_idx) + ".ply";
        std::cout << "ref_key: " << ref_key << std::endl;
        Scene ref_scan_id = pl->mSceneMap[ref_key];

        // Compare all the objects in the reference and query scene
        int obj_match_vote = 0;
        int obj_dnm_vote = 0;
        int total_objs = 0;
        bool ks_result, ad_result;
        for (auto &so : pl->mSceneMap[query_key].spectral_objects) {

          // Determine if to skip the object or not
          if (Processing::PointCloud::ShouldSkip(so))
            continue;

          total_objs++;

          // Find the matching object
          auto begin_it = pl->mSceneMap[ref_key].spectral_objects.begin();
          auto end_it = pl->mSceneMap[ref_key].spectral_objects.end();

          std::string ply_color = so.ply_color;

          auto it = std::find_if(begin_it, end_it,
                                 [ply_color](const SpectralObject &r_so) {
                                   return r_so.ply_color == ply_color;
                                 });

          int ref_obj_idx = it - begin_it;

          if (it != end_it) {
            // found the shared object
            // Now we need to run all the code and determine if the objs are
            // the same
            if (so.cloud->size() <= 1 || pl->mSceneMap[ref_key]
                                                 .spectral_objects[ref_obj_idx]
                                                 .cloud->size() <= 1) {
              std::cout << "Skipping Label: " << so.label << std::endl;
              continue;
            }

            // ImGuiState::DatasetTesting::mtx.lock();

            ImGuiState::DatasetTesting::q_so = so;

            ImGuiState::DatasetTesting::r_so =
                pl->mSceneMap[ref_key].spectral_objects[ref_obj_idx];

            // Processing::PointCloud::computeSOR(
            //     ImGuiState::DatasetTesting::q_so,
            //     ImGuiState::DatasetTesting::meanK,
            //     ImGuiState::DatasetTesting::stdThresh);
            // Processing::PointCloud::computeSOR(
            //     ImGuiState::DatasetTesting::r_so,
            //     ImGuiState::DatasetTesting::meanK,
            //     ImGuiState::DatasetTesting::stdThresh);

            // double size =
            //     pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
            //                 ImGuiState::DatasetTesting::sample_size,
            //                 ImGuiState::DatasetTesting::filter_percent,
            //                 ImGuiState::DatasetTesting::q_so.cloud->size(),
            //                 ImGuiState::DatasetTesting::r_so.cloud->size());

            // Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
            //                                    size);
            // Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
            //                                    size);

            // if (ImGuiState::DatasetTesting::double_sor) {
            //   Processing::PointCloud::computeSOR(
            //       ImGuiState::DatasetTesting::q_so,
            //       ImGuiState::DatasetTesting::meanK,
            //       ImGuiState::DatasetTesting::stdThresh);
            //   Processing::PointCloud::computeSOR(
            //       ImGuiState::DatasetTesting::r_so,
            //       ImGuiState::DatasetTesting::meanK,
            //       ImGuiState::DatasetTesting::stdThresh);
            // }

            // Processing::PointCloud::computeMCAR(
            //     ImGuiState::DatasetTesting::q_so);
            // Processing::PointCloud::computeMCAR(
            //     ImGuiState::DatasetTesting::r_so);

            // if (ImGuiState::DatasetTesting::same_radius) {
            //   double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
            //                          ImGuiState::DatasetTesting::r_so.mcar);
            //   ImGuiState::DatasetTesting::q_so.mcar = mcar;
            //   ImGuiState::DatasetTesting::r_so.mcar = mcar;
            // }

            // pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
            //               ImGuiState::DatasetTesting::q_so);
            // pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
            //               ImGuiState::DatasetTesting::r_so);

            // int number_eigs =
            // ImGuiState::DatasetTesting::q_so.cloud->size(); if
            // (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
            //   number_eigs = ImGuiState::DatasetTesting::eigs_number;
            // }

            // Processing::Eigen::computeEigenvalues(
            //     ImGuiState::DatasetTesting::q_so, number_eigs);
            // Processing::Eigen::computeEigenvalues(
            //     ImGuiState::DatasetTesting::r_so, number_eigs);

            // ImGuiState::DatasetTesting::eigs_mtx.lock();

            ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
                arma::conv_to<std::vector<double>>::from(
                    ImGuiState::DatasetTesting::q_so.eigenvalues);
            ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
                arma::conv_to<std::vector<double>>::from(
                    ImGuiState::DatasetTesting::r_so.eigenvalues);
            ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
                ImGuiState::DatasetTesting::q_so.gfaFeatures;
            ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
                ImGuiState::DatasetTesting::r_so.gfaFeatures;

            // Eval service
            if (evaluation_service_client.call(
                    ImGuiState::DatasetTesting::eig_srv)) {
              // ROS_INFO("eval service success!!! %f",
              //          ImGuiState::DatasetTesting::eig_srv.response.results[0]);

              ks_result =
                  ImGuiState::DatasetTesting::eig_srv.response.results[0];
              ad_result =
                  ImGuiState::DatasetTesting::eig_srv.response.results[1];
            } else {
              ROS_ERROR("eval service failed");
            }

            // TODO need to throw the evaluation code here
            if (ks_result || ad_result) {
              obj_match_vote++;
              // std::cout << "Label: " << so.label << std::endl;
            } else {
              obj_dnm_vote++;
            }

            ImGuiState::DatasetTesting::update_cloud = true;
            ImGuiState::DatasetTesting::update_hist = true;
            // ImGuiState::DatasetTesting::mtx.unlock();
            // ImGuiState::DatasetTesting::eigs_mtx.unlock();
          }
        }

        // TODO need to add the eval totaling here, ref scene is done
        SemanticKittiResult scene_result;
        scene_result.sequence = ImGuiState::DatasetTesting::sequence_string;
        scene_result.query_id = query_key;
        scene_result.ref_id = ref_key;
        scene_result.obj_match_ratio = obj_match_vote / double(total_objs);
        // scene_result.obj_match_ratio =
        //     obj_match_vote / (double(obj_match_vote + obj_dnm_vote));
        //  scene_result.obj_match_ratio =
        //      obj_match_vote / double(r_kv.second.spectral_objects.size());
        //  scene_result.is_match = r_kv.first ==
        //  q_kv.second.reference_id_match;
        results.push_back(scene_result);

        ref_scan_idx += 10;
      }

      query_scan_idx += 10;
    }

    // TODO Save to a file
    Processing::Files::SaveSemanticKittiResults(results, ImGuiState::DatasetTesting::start_ref_scene, ImGuiState::DatasetTesting::start_query_scene);

    // TODO add final eval tally here
    // Will eventually need to figure out where the actual loop closures are
    // and save to a file with the pred and ground truth labels for PR Curve
    // results for (auto &result : results) {
    //  if (isnan(result.obj_match_ratio))
    //    continue;
    //  if (result.obj_match_ratio > ImGuiState::DatasetTesting::match_thresh)
    //  {
    //    std::cout << "query_id: " << result.query_id << std::endl;
    //    std::cout << "ref_id: " << result.ref_id << std::endl;
    //    std::cout << "score: " << result.obj_match_ratio << std::endl;
    //  }
    //}

    // TODO: save the pose array with markers where there are loop closures
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Eval Obj Pairs");

  if (ImGui::Button("Init Scene")) {
    std::cout << "INIT Pressed" << std::endl;
    bool first_flag = true;
    for (auto const kv : pl->mSceneMap) {
      if (first_flag) {
        ImGuiState::DatasetTesting::key1 = kv.first;
        first_flag = false;
      } else {
        ImGuiState::DatasetTesting::key2 = kv.first;
        break;
      }
    }

    ImGuiState::DatasetTesting::query_obj_idx = 0;
  }

  if (ImGui::Button("Compaire Object Pair")) {

    if (pl->mSceneMap[ImGuiState::DatasetTesting::key1]
            .spectral_objects.size() ==
        ImGuiState::DatasetTesting::query_obj_idx)
      ImGuiState::DatasetTesting::query_obj_idx = 0;

    // Get the first two scen scan ids

    // For all the objs in first scene
    ImGuiState::DatasetTesting::q_so =
        pl->mSceneMap[ImGuiState::DatasetTesting::key1]
            .spectral_objects[ImGuiState::DatasetTesting::query_obj_idx];

    auto begin_it = pl->mSceneMap[ImGuiState::DatasetTesting::key2]
                        .spectral_objects.begin();
    auto end_it =
        pl->mSceneMap[ImGuiState::DatasetTesting::key2].spectral_objects.end();

    std::string ply_color = ImGuiState::DatasetTesting::q_so.ply_color;

    auto it =
        std::find_if(begin_it, end_it, [ply_color](const SpectralObject &r_so) {
          return r_so.ply_color == ply_color;
        });

    int ref_obj_idx = it - begin_it;

    if (it != end_it) {
      ImGuiState::DatasetTesting::mtx.lock();

      ImGuiState::DatasetTesting::r_so =
          pl->mSceneMap[ImGuiState::DatasetTesting::key2]
              .spectral_objects[ref_obj_idx];

      Processing::PointCloud::computeSOR(ImGuiState::DatasetTesting::q_so,
                                         ImGuiState::DatasetTesting::meanK,
                                         ImGuiState::DatasetTesting::stdThresh);
      Processing::PointCloud::computeSOR(ImGuiState::DatasetTesting::r_so,
                                         ImGuiState::DatasetTesting::meanK,
                                         ImGuiState::DatasetTesting::stdThresh);

      double size = pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
                                ImGuiState::DatasetTesting::sample_size,
                                ImGuiState::DatasetTesting::filter_percent,
                                ImGuiState::DatasetTesting::q_so.cloud->size(),
                                ImGuiState::DatasetTesting::r_so.cloud->size());

      Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
                                         size);
      Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
                                         size);

      if (ImGuiState::DatasetTesting::double_sor) {
        Processing::PointCloud::computeSOR(
            ImGuiState::DatasetTesting::q_so, ImGuiState::DatasetTesting::meanK,
            ImGuiState::DatasetTesting::stdThresh);
        Processing::PointCloud::computeSOR(
            ImGuiState::DatasetTesting::r_so, ImGuiState::DatasetTesting::meanK,
            ImGuiState::DatasetTesting::stdThresh);
      }

      Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::q_so);
      Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::r_so);

      if (ImGuiState::DatasetTesting::same_radius) {
        double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
                               ImGuiState::DatasetTesting::r_so.mcar);
        ImGuiState::DatasetTesting::q_so.mcar = mcar;
        ImGuiState::DatasetTesting::r_so.mcar = mcar;
      }

      pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
                    ImGuiState::DatasetTesting::q_so);
      pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
                    ImGuiState::DatasetTesting::r_so);

      int number_eigs = ImGuiState::DatasetTesting::q_so.cloud->size();
      if (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
        number_eigs = ImGuiState::DatasetTesting::eigs_number;
      }

      Processing::Eigen::computeEigenvalues(ImGuiState::DatasetTesting::q_so,
                                            number_eigs);
      Processing::Eigen::computeEigenvalues(ImGuiState::DatasetTesting::r_so,
                                            number_eigs);

      ImGuiState::DatasetTesting::eigs_mtx.lock();

      ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
          arma::conv_to<std::vector<double>>::from(
              ImGuiState::DatasetTesting::q_so.eigenvalues);
      ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
          arma::conv_to<std::vector<double>>::from(
              ImGuiState::DatasetTesting::r_so.eigenvalues);
      ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
          ImGuiState::DatasetTesting::q_so.gfaFeatures;
      ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
          ImGuiState::DatasetTesting::r_so.gfaFeatures;

      // Eval service
      if (evaluation_service_client.call(ImGuiState::DatasetTesting::eig_srv)) {
        ROS_INFO("eval service success!!! %f",
                 ImGuiState::DatasetTesting::eig_srv.response.results[0]);
      } else {
        ROS_ERROR("eval service failed");
      }

      ImGuiState::DatasetTesting::update_cloud = true;
      ImGuiState::DatasetTesting::update_hist = true;
      ImGuiState::DatasetTesting::mtx.unlock();
      ImGuiState::DatasetTesting::eigs_mtx.unlock();

    } else {
      ImGuiState::DatasetTesting::ref_obj_exists = false;
    }

    ImGuiState::DatasetTesting::query_obj_idx++;
  }

  if (!ImGuiState::DatasetTesting::ReadyToStep())
    ImGui::BeginDisabled();

  if (!ImGuiState::DatasetTesting::RefObjExists())
    ImGui::Text("Ref Object Does Not Exist");

  if (!ImGuiState::DatasetTesting::ReadyToStep())
    ImGui::EndDisabled();

  // if (!ImGuiState::DatasetTesting::DatasetParsed()) {
  //   ImGui::EndDisabled();
  // }

  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

// void datasetTestingPipeline(std::shared_ptr<Pipeline> &pl) {
//
//   ImGui::Begin("Dataset Testing Pipeline");
//
//   // --------------------------------------------------------------
//   ImGui::Text("Dataset");
//
//   ImGui::Combo("Choose Dataset", &ImGuiState::DatasetTesting::dataset_idx,
//                ImGuiState::datasets, IM_ARRAYSIZE(ImGuiState::datasets));
//
//   // ImGui::InputInt("Max number of points in point cloud",
//   //                 &ImGuiState::DatasetTesting::max_pts);
//
//   if (ImGui::Button("Button 1")) {
//     pl.reset();
//     switch (ImGuiState::DatasetTesting::dataset_idx) {
//     case 0:
//       pl = std::make_shared<RScanPipeline>();
//       break;
//     case 1:
//       // pl = std::make_shared<MatterPortPipeline>();
//       break;
//     case 2:
//       pl = std::make_shared<RScanPipeline>();
//       // pl = std::make_shared<SemanticKittiPipeline>();
//       break;
//     default:
//       break;
//     }
//
//     pl->ParseDataset(ImGuiState::DatasetTesting::dataset_idx);
//     pl->ExtractObjectPointClouds(ImGuiState::DatasetTesting::max_pts);
//     ImGuiState::DatasetTesting::dataset_parsed = true;
//   }
//
//   ImGui::SameLine();
//   (ImGuiState::DatasetTesting::DatasetParsed())
//       ? ImGui::Text("Success! Parse datset")
//       : ImGui::Text("Click to parse dataset");
//
//   if (!ImGuiState::DatasetTesting::DatasetParsed()) {
//     ImGui::BeginDisabled();
//   }
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Preprocessing");
//
//   ImGui::Checkbox("SOR", &ImGuiState::DatasetTesting::sor_check);
//   ImGui::Checkbox("Double SOR", &ImGuiState::DatasetTesting::double_sor);
//
//   if (!ImGuiState::DatasetTesting::sor_check)
//     ImGui::BeginDisabled();
//
//   ImGui::InputInt("MeanK", &ImGuiState::DatasetTesting::meanK);
//   ImGui::InputDouble("StdDevThresh",
//   &ImGuiState::DatasetTesting::stdThresh);
//
//   if (!ImGuiState::DatasetTesting::sor_check)
//     ImGui::EndDisabled();
//
//   ImGui::Combo("Filtering", &ImGuiState::DatasetTesting::filtering_idx,
//                ImGuiState::filtering, IM_ARRAYSIZE(ImGuiState::filtering));
//
//   ImGui::RadioButton("Smallest Cloud",
//                      &ImGuiState::DatasetTesting::filtering_opts, 0);
//   ImGui::SameLine();
//   ImGui::RadioButton("Percent",
//   &ImGuiState::DatasetTesting::filtering_opts, 1); ImGui::SameLine();
//   ImGui::RadioButton("Sample Size",
//   &ImGuiState::DatasetTesting::filtering_opts,
//                      2);
//
//   if (!ImGuiState::DatasetTesting::FilterPercent())
//     ImGui::BeginDisabled();
//
//   ImGui::InputDouble("%", &ImGuiState::DatasetTesting::filter_percent);
//
//   if (!ImGuiState::DatasetTesting::FilterPercent())
//     ImGui::EndDisabled();
//
//   if (!ImGuiState::DatasetTesting::SampleSize())
//     ImGui::BeginDisabled();
//
//   ImGui::InputInt("Sample #", &ImGuiState::DatasetTesting::sample_size);
//
//   if (!ImGuiState::DatasetTesting::SampleSize())
//     ImGui::EndDisabled();
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Graph Formulation");
//
//   ImGui::Combo(
//       "Edge Heuristic", &ImGuiState::DatasetTesting::edge_heuristic_idx,
//       ImGuiState::edge_heuristics,
//       IM_ARRAYSIZE(ImGuiState::edge_heuristics));
//
//   ImGui::Checkbox("Same Radius", &ImGuiState::DatasetTesting::same_radius);
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Laplacian");
//
//   ImGui::Combo("Laplacian Algorithm",
//                &ImGuiState::DatasetTesting::laplacian_idx,
//                ImGuiState::laplacians,
//                IM_ARRAYSIZE(ImGuiState::laplacians));
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Eigenvalues");
//
//   ImGui::RadioButton("All Eigenvalues",
//                      &ImGuiState::DatasetTesting::eigendecomposition_method,
//                      0);
//   ImGui::SameLine();
//   ImGui::RadioButton("Specific # of Eigenvalues",
//                      &ImGuiState::DatasetTesting::eigendecomposition_method,
//                      1);
//
//   if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
//     ImGui::BeginDisabled();
//
//   ImGui::InputInt("# of Eigenvalues to compute",
//                   &ImGuiState::DatasetTesting::eigs_number);
//
//   if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
//     ImGui::EndDisabled();
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Choose Scans");
//
//   if (ImGuiState::DatasetTesting::GetQueryScans()) {
//     pl->GetQueryScans(ImGuiState::DatasetTesting::query_scans);
//   }
//
//   const char *selected_query_scan =
//       ImGuiState::DatasetTesting::GetSelectedQueryScan();
//
//   if (ImGui::BeginCombo("Choose A Query Scan", selected_query_scan)) {
//     for (int i = 0; i < ImGuiState::DatasetTesting::query_scans.size();
//     i++) {
//       const bool isSelected = (ImGuiState::DatasetTesting::query_scan_idx
//       == i); if
//       (ImGui::Selectable(ImGuiState::DatasetTesting::query_scans[i].c_str(),
//                             isSelected)) {
//         ImGuiState::DatasetTesting::query_scan_idx = i;
//       }
//
//       if (isSelected) {
//         ImGui::SetItemDefaultFocus();
//       }
//     }
//     ImGui::EndCombo();
//   }
//
//   const char *selected_ref_scan =
//       (ImGuiState::DatasetTesting::query_scans.size() > 0)
//           ? pl->GetMappedRefScan(ImGuiState::DatasetTesting::query_scans,
//                                  ImGuiState::DatasetTesting::query_scan_idx)
//           : "";
//
//   if (ImGuiState::DatasetTesting::GetRefScans())
//     pl->GetRefScans(ImGuiState::DatasetTesting::ref_scans);
//
//   if (ImGuiState::DatasetTesting::UpdateRefScanSelected()) {
//     ImGuiState::DatasetTesting::last_query_scan_idx =
//         ImGuiState::DatasetTesting::query_scan_idx;
//     std::string ref = std::string(selected_ref_scan);
//     auto it = std::find(ImGuiState::DatasetTesting::ref_scans.begin(),
//                         ImGuiState::DatasetTesting::ref_scans.end(), ref);
//
//     if (it != ImGuiState::DatasetTesting::ref_scans.end()) {
//       ImGuiState::DatasetTesting::ref_scan_idx =
//           it - ImGuiState::DatasetTesting::ref_scans.begin();
//     } else {
//       std::cout << "ERROR reference scan not in list" << std::endl;
//       std::exit(1);
//     }
//   }
//
//   if (ImGui::BeginCombo("Choose A Reference Scan", selected_ref_scan)) {
//     for (int i = 0; i < ImGuiState::DatasetTesting::ref_scans.size(); i++)
//     {
//       const bool isSelected = (ImGuiState::DatasetTesting::ref_scan_idx ==
//       i); if
//       (ImGui::Selectable(ImGuiState::DatasetTesting::ref_scans[i].c_str(),
//                             isSelected)) {
//         ImGuiState::DatasetTesting::ref_scan_idx = i;
//       }
//
//       if (isSelected) {
//         ImGui::SetItemDefaultFocus();
//       }
//     }
//     ImGui::EndCombo();
//   }
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Eval Options");
//
//   ImGui::RadioButton("Eval All", &ImGuiState::DatasetTesting::eval_opts,
//   0); ImGui::SameLine(); ImGui::RadioButton("Eval From Config",
//   &ImGuiState::DatasetTesting::eval_opts,
//                      1);
//
//   ImGui::Checkbox("Debug Bulk Eval",
//   &ImGuiState::DatasetTesting::debug_eval);
//
//   if (ImGui::Button("Funky")) {
//
//     int total_compared = 0;
//     int TP = 0;
//     int FP = 0;
//     int FN = 0;
//     int TN = 0;
//     bool TN_flag = false;
//     int class_id_correct = 0;
//     int class_id_wrong = 0;
//     std::vector<int> truth_label;
//     std::vector<int> pred_label;
//     std::vector<std::vector<double>> query_gfa;
//     std::vector<std::vector<double>> ref_gfa;
//     std::vector<int> q_global_id;
//     std::vector<int> q_scene_id;
//     std::vector<int> r_global_id;
//     std::vector<int> r_scene_id;
//     for (auto &kv : pl->mSceneMap) {
//       if (kv.second.is_reference)
//         continue;
//
//       std::cout << "-------------------------" << std::endl;
//       std::cout << "For Query Scan: " << kv.first << std::endl;
//       // Find the matching reference_scan
//       std::string ref_id = kv.second.reference_id_match;
//
//       // For all the objects in the query scan
//       for (auto &q_so : kv.second.spectral_objects) {
//         // match exists: try to determin TP, FP, FN against all objects
//         bool match_found = false;
//         bool ks_result, ad_result;
//         for (auto &r_so : pl->mSceneMap[ref_id].spectral_objects) {
//
//           if (q_so.global_id == r_so.global_id) {
//
//             // if (r_so.scene_id == q_so.scene_id) {
//             //   Do all the fancy matches
//             ImGuiState::DatasetTesting::mtx.lock();
//             ImGuiState::DatasetTesting::q_so = q_so;
//             ImGuiState::DatasetTesting::r_so = r_so;
//
//             query_gfa.push_back(ImGuiState::DatasetTesting::q_so.gfaFeatures);
//             q_global_id.push_back(ImGuiState::DatasetTesting::q_so.global_id);
//             q_scene_id.push_back(ImGuiState::DatasetTesting::q_so.scene_id);
//
//             ref_gfa.push_back(ImGuiState::DatasetTesting::r_so.gfaFeatures);
//             r_global_id.push_back(ImGuiState::DatasetTesting::r_so.global_id);
//             r_scene_id.push_back(ImGuiState::DatasetTesting::r_so.scene_id);
//
//             // if (r_so.scene_id == q_so.scene_id) {
//             Processing::PointCloud::computeSOR(
//                 ImGuiState::DatasetTesting::q_so,
//                 ImGuiState::DatasetTesting::meanK,
//                 ImGuiState::DatasetTesting::stdThresh);
//             Processing::PointCloud::computeSOR(
//                 ImGuiState::DatasetTesting::r_so,
//                 ImGuiState::DatasetTesting::meanK,
//                 ImGuiState::DatasetTesting::stdThresh);
//             //}
//
//             double size =
//                 pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
//                             ImGuiState::DatasetTesting::sample_size,
//                             ImGuiState::DatasetTesting::filter_percent,
//                             ImGuiState::DatasetTesting::q_so.cloud->size(),
//                             ImGuiState::DatasetTesting::r_so.cloud->size());
//
//             std::cout << "size: " << size << std::endl;
//
//             Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
//                                                size);
//             Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
//                                                size);
//
//             if (ImGuiState::DatasetTesting::double_sor) {
//               Processing::PointCloud::computeSOR(
//                   ImGuiState::DatasetTesting::q_so,
//                   ImGuiState::DatasetTesting::meanK,
//                   ImGuiState::DatasetTesting::stdThresh);
//               Processing::PointCloud::computeSOR(
//                   ImGuiState::DatasetTesting::r_so,
//                   ImGuiState::DatasetTesting::meanK,
//                   ImGuiState::DatasetTesting::stdThresh);
//             }
//
//             Processing::PointCloud::computeMCAR(
//                 ImGuiState::DatasetTesting::q_so);
//             Processing::PointCloud::computeMCAR(
//                 ImGuiState::DatasetTesting::r_so);
//
//             if (ImGuiState::DatasetTesting::same_radius) {
//               double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
//                                      ImGuiState::DatasetTesting::r_so.mcar);
//               ImGuiState::DatasetTesting::q_so.mcar = mcar;
//               ImGuiState::DatasetTesting::r_so.mcar = mcar;
//             }
//
//             // TODO will need to add Laplacian choice when adding more in
//             // future
//             pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                           ImGuiState::DatasetTesting::q_so);
//             pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                           ImGuiState::DatasetTesting::r_so);
//
//             int number_eigs =
//             ImGuiState::DatasetTesting::q_so.cloud->size(); if
//             (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
//               number_eigs = ImGuiState::DatasetTesting::eigs_number;
//             }
//
//             Processing::Eigen::computeEigenvalues(
//                 ImGuiState::DatasetTesting::q_so, number_eigs);
//             Processing::Eigen::computeEigenvalues(
//                 ImGuiState::DatasetTesting::r_so, number_eigs);
//
//             ImGuiState::DatasetTesting::update_cloud = true;
//             ImGuiState::DatasetTesting::mtx.unlock();
//
//             ImGuiState::DatasetTesting::eigs_mtx.lock();
//
//             ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
//                 arma::conv_to<std::vector<double>>::from(
//                     ImGuiState::DatasetTesting::q_so.eigenvalues);
//             ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
//                 arma::conv_to<std::vector<double>>::from(
//                     ImGuiState::DatasetTesting::r_so.eigenvalues);
//             ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
//                 ImGuiState::DatasetTesting::q_so.gfaFeatures;
//             ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
//                 ImGuiState::DatasetTesting::r_so.gfaFeatures;
//
//             ImGuiState::DatasetTesting::update_hist = true;
//             ImGuiState::DatasetTesting::eigs_mtx.unlock();
//
//             //if (ImGuiState::DatasetTesting::eig_srv.request.q_eigs.size()
//             ==
//             //        0 ||
//             //    ImGuiState::DatasetTesting::eig_srv.request.r_eigs.size()
//             == 0)
//             //  continue;
//
//             // Eval service
//             if (evaluation_service_client.call(
//                     ImGuiState::DatasetTesting::eig_srv)) {
//               // ROS_INFO("eval service success!!! %f",
//               // ImGuiState::DatasetTesting::eig_srv.response.results[0]);
//               ks_result =
//                   ImGuiState::DatasetTesting::eig_srv.response.results[0];
//               ad_result =
//                   ImGuiState::DatasetTesting::eig_srv.response.results[1];
//             } else {
//               ROS_ERROR("eval service failed");
//               exit(1);
//             }
//
//             // Save GFA
//             // {
//             //   query_gfa: []
//             //   reference_gfa: []
//             //   is_match_truth: []
//             // }
//
//             if (ks_result || ad_result) {
//               // if (r_so.scene_id == q_so.scene_id) {
//               pred_label.push_back(1);
//               if (r_so.global_id == q_so.global_id &&
//                   r_so.scene_id == q_so.scene_id) {
//                 truth_label.push_back(1);
//                 TP++;
//               } else {
//                 truth_label.push_back(0);
//                 FP++;
//               }
//               //          if (r_so.global_id == q_so.global_id)
//               //            class_id_correct++;
//               //          else
//               //            class_id_wrong++;
//             } else {
//               // if (r_so.scene_id == q_so.scene_id) {
//               pred_label.push_back(0);
//               if (r_so.global_id == q_so.global_id &&
//                   r_so.scene_id == q_so.scene_id) {
//                 truth_label.push_back(1);
//                 FN++;
//               } else {
//                 truth_label.push_back(0);
//                 TN++;
//               }
//               //            if (r_so.global_id == q_so.global_id)
//               //             class_id_wrong++;
//               //           else
//               //             class_id_correct++;
//             }
//             total_compared++;
//             if (ImGuiState::DatasetTesting::debug_eval) {
//               std::cout << '\n' << "Press Enter";
//               while (std::cin.get() != '\n') {
//               }
//             }
//           }
//         }
//       }
//     }
//
//     // Make sure total_compared equals all added up so there is no funny
//     // business
//
//     // double accuracy = (TP + TN) / double(total_compared);
//     // double class_accuracy =
//     //    class_id_correct / double(class_id_correct + class_id_wrong);
//     double instance_accuracy = (TP + TN) / double(TP + TN + FP + FN);
//     double precision = TP / double(TP + FP);
//     double recall = TP / double(TP + FN);
//     double f1_score = (2 * precision * recall) / (precision + recall);
//     // std::cout << "Class Accuracy: " << class_accuracy << std::endl;
//     std::cout << "Instance Accuracy: " << instance_accuracy << std::endl;
//     std::cout << "Precision: " << precision << std::endl;
//     std::cout << "Recall: " << recall << std::endl;
//     std::cout << "f1_score: " << f1_score << std::endl;
//     std::cout << "\nConfusion Matrix" << std::endl;
//     std::cout << "          Negative | Poistive" << std::endl;
//     std::cout << "Negative | " << TN << "    | " << FN << std::endl;
//     std::cout << "Positive | " << FP << "    | " << TP << std::endl;
//
//     EvalMetrics em;
//     em.dataset = (ImGuiState::DatasetTesting::dataset_idx == 0)
//                      ? "3RScan"
//                      : "SemanticKitti";
//     em.sample_size = ImGuiState::DatasetTesting::sample_size;
//     em.mean_k = ImGuiState::DatasetTesting::meanK;
//     em.std_thresh = ImGuiState::DatasetTesting::stdThresh;
//     em.laplacian = ImGuiState::DatasetTesting::GetLaplacianName();
//     // em.class_accuracy = class_accuracy;
//     em.instance_accuracy = instance_accuracy;
//     em.precision = precision;
//     em.recall = recall;
//     em.f1_score = f1_score;
//     em.threshold = 1.0;
//     em.truth_labels = truth_label;
//     em.pred_labels = pred_label;
//
//
//     GFAFeatures gfa;
//     gfa.query_gfa = query_gfa;
//     gfa.ref_gfa = ref_gfa;
//     gfa.match_results = truth_label;
//     gfa.q_global_id = q_global_id;
//     gfa.r_global_id = r_global_id;
//     gfa.q_scene_id = q_scene_id;
//     gfa.r_scene_id = r_scene_id;
//
//     // Save GFA results
//     Processing::Files::SaveGFAFeatures(gfa);
//
//     Processing::Files::SaveEvalMetrics(em);
//     // File name will be results/3RScan/laplacian_name/idx.json
//     // TODO need to save a, p, r, f1 to json file
//     // {
//     //   sample_size:
//     //   meak_k:
//     //   std_thresh:
//     //   laplacian:
//     //   Accuracy:
//     //   Precision:
//     //   Recall:
//     //   F1_Score
//     // }
//
//     if (total_compared != (TP + TN + FP + FN)) {
//       std::cout << "ERROR total compared DNE all added: " << total_compared
//                 << std::endl;
//       exit(1);
//     }
//   }
//
//   if (ImGui::Button("Eval Place Recognition Precision Recall")) {
//     int total_compared = 0;
//     int TP = 0;
//     int FP = 0;
//     int FN = 0;
//     int TN = 0;
//     bool TN_flag = false;
//     // key - query scan
//     // value - array<Pair<string,Pair<int,int>>
//     std::unordered_map<std::string, std::vector<Result>> result_map;
//
//     // FIRST SAVE GFA
//     pl->SaveGFA();
//
//     for (auto &q_kv : pl->mSceneMap) {
//       if (q_kv.second.is_reference)
//         continue;
//
//       // Compare against all reference scans
//       for (auto &r_kv : pl->mSceneMap) {
//         if (!r_kv.second.is_reference)
//           continue;
//
//         // For all the objects in the query scan
//         int obj_match_vote = 0;
//         int obj_dnm_vote = 0;
//         bool ks_result, ad_result;
//         for (auto &q_so : q_kv.second.spectral_objects) {
//
//           // If there is no corresponding obj then continue
//           auto begin_it = r_kv.second.spectral_objects.begin();
//           auto end_it = r_kv.second.spectral_objects.end();
//
//           int global_id = q_so.global_id;
//
//           auto it = std::find_if(begin_it, end_it,
//                                  [global_id](const SpectralObject &r_so) {
//                                    return r_so.global_id == global_id;
//                                  });
//
//           int ref_obj_idx = it - begin_it;
//
//           if (it == end_it)
//             continue;
//
//           ImGuiState::DatasetTesting::mtx.lock();
//           ImGuiState::DatasetTesting::q_so = q_so;
//           ImGuiState::DatasetTesting::r_so =
//               r_kv.second.spectral_objects[ref_obj_idx];
//
//           Processing::PointCloud::computeSOR(
//               ImGuiState::DatasetTesting::q_so,
//               ImGuiState::DatasetTesting::meanK,
//               ImGuiState::DatasetTesting::stdThresh);
//           Processing::PointCloud::computeSOR(
//               ImGuiState::DatasetTesting::r_so,
//               ImGuiState::DatasetTesting::meanK,
//               ImGuiState::DatasetTesting::stdThresh);
//
//           double size =
//               pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
//                           ImGuiState::DatasetTesting::sample_size,
//                           ImGuiState::DatasetTesting::filter_percent,
//                           ImGuiState::DatasetTesting::q_so.cloud->size(),
//                           ImGuiState::DatasetTesting::r_so.cloud->size());
//
//           Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
//                                              size);
//           Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
//                                              size);
//
//           if (ImGuiState::DatasetTesting::double_sor) {
//             Processing::PointCloud::computeSOR(
//                 ImGuiState::DatasetTesting::q_so,
//                 ImGuiState::DatasetTesting::meanK,
//                 ImGuiState::DatasetTesting::stdThresh);
//             Processing::PointCloud::computeSOR(
//                 ImGuiState::DatasetTesting::r_so,
//                 ImGuiState::DatasetTesting::meanK,
//                 ImGuiState::DatasetTesting::stdThresh);
//           }
//
//           Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::q_so);
//           Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::r_so);
//
//           if (ImGuiState::DatasetTesting::same_radius) {
//             double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
//                                    ImGuiState::DatasetTesting::r_so.mcar);
//             ImGuiState::DatasetTesting::q_so.mcar = mcar;
//             ImGuiState::DatasetTesting::r_so.mcar = mcar;
//           }
//
//           // TODO will need to add Laplacian choice when adding more in
//           // the future
//           pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                         ImGuiState::DatasetTesting::q_so);
//           pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                         ImGuiState::DatasetTesting::r_so);
//
//           int number_eigs = ImGuiState::DatasetTesting::q_so.cloud->size();
//           if (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
//             number_eigs = ImGuiState::DatasetTesting::eigs_number;
//           }
//
//           Processing::Eigen::computeEigenvalues(
//               ImGuiState::DatasetTesting::q_so, number_eigs);
//           Processing::Eigen::computeEigenvalues(
//               ImGuiState::DatasetTesting::r_so, number_eigs);
//
//           ImGuiState::DatasetTesting::update_cloud = true;
//           ImGuiState::DatasetTesting::mtx.unlock();
//
//           ImGuiState::DatasetTesting::eigs_mtx.lock();
//
//           ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
//               arma::conv_to<std::vector<double>>::from(
//                   ImGuiState::DatasetTesting::q_so.eigenvalues);
//           ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
//               arma::conv_to<std::vector<double>>::from(
//                   ImGuiState::DatasetTesting::r_so.eigenvalues);
//           ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
//               ImGuiState::DatasetTesting::q_so.gfaFeatures;
//           ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
//               ImGuiState::DatasetTesting::r_so.gfaFeatures;
//
//           ImGuiState::DatasetTesting::update_hist = true;
//           ImGuiState::DatasetTesting::eigs_mtx.unlock();
//
//           // Eval service
//           if (evaluation_service_client.call(
//                   ImGuiState::DatasetTesting::eig_srv)) {
//             // ROS_INFO("eval service success!!! %f",
//             // ImGuiState::DatasetTesting::eig_srv.response.results[0]);
//             ks_result =
//             ImGuiState::DatasetTesting::eig_srv.response.results[0];
//             ad_result =
//             ImGuiState::DatasetTesting::eig_srv.response.results[1];
//           } else {
//             ROS_ERROR("eval service failed");
//             exit(1);
//           }
//
//           if (ks_result || ad_result) {
//             obj_match_vote++;
//           } else {
//             obj_dnm_vote++;
//           }
//
//           if (ImGuiState::DatasetTesting::debug_eval) {
//             std::cout << '\n' << "Press Enter";
//             while (std::cin.get() != '\n') {
//             }
//           }
//         }
//
//         Result scene_result;
//         // scene_result.ref_scan_id = r_kv.first;
//         // scene_result.correct_ref_scan_id =
//         q_kv.second.reference_id_match;
//         // scene_result.obj_match_vote = obj_match_vote;
//         // scene_result.obj_dnm_vote = obj_dnm_vote;
//         // scene_result.obj_total = r_kv.second.spectral_objects.size();
//         // scene_result.obj_match_ratio = obj_match_vote /
//         // (double(obj_match_vote + obj_dnm_vote));
//         scene_result.obj_match_ratio =
//             obj_match_vote / double(r_kv.second.spectral_objects.size());
//         scene_result.is_match = r_kv.first ==
//         q_kv.second.reference_id_match;
//         result_map[r_kv.first].push_back(scene_result);
//       }
//     }
//
//     // {
//     //   results : [
//     //     ref_scan_id: scan_id
//     //     query_scan_truth: [... 0 or 1 for correct pred]
//     //     query_scan_pred: [... the pred percent]
//     //   ]
//     // }
//
//     std::string file_extension = ".json";
//     std::string path =
//         "/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/";
//     std::string lap_name = ImGuiState::DatasetTesting::GetLaplacianName();
//     std::string save_path = path + lap_name + file_extension;
//     std::ofstream o(save_path);
//     json data;
//     for (auto kv : result_map) {
//       json result;
//       result["ref_scan_id"] = kv.first;
//       // std::cout << "Ref: " << kv.first << std::endl;
//       for (auto res : kv.second) {
//         // std::cout << "pred ratio" << res.obj_match_ratio << std::endl;
//         // std::cout << "truth" << res.is_match << std::endl;
//         result["query_scan_truth"].push_back(res.is_match);
//         result["query_scan_pred"].push_back(res.obj_match_ratio);
//       }
//       data["results"].push_back(result);
//     }
//
//     o << std::setw(4) << data << std::endl;
//     o.close();
//
//   }
//
//   // --------------------------------------------------------------
//   ImGui::Separator();
//   ImGui::Text("Eval Obj Pairs");
//
//   if (ImGui::Button("Init Scene")) {
//     std::cout << "INIT Pressed" << std::endl;
//     ImGuiState::DatasetTesting::query_obj_scene_ids.clear();
//     ImGuiState::DatasetTesting::query_obj_idx = 0;
//     pl->GetQuerySpectralObjIds(ImGuiState::DatasetTesting::query_obj_scene_ids,
//                                std::string(selected_query_scan));
//   }
//
//   if (ImGui::Button("Compaire Object Pair")) {
//
//     if (pl->RefObjExists(std::string(selected_query_scan),
//                          ImGuiState::DatasetTesting::query_obj_idx,
//                          ImGuiState::DatasetTesting::ref_obj_idx)) {
//
//       ImGuiState::DatasetTesting::ref_obj_exists = true;
//       ImGuiState::DatasetTesting::mtx.lock();
//
//       pl->GetQueryRefCloudObjPair(
//           std::string(selected_query_scan), std::string(selected_ref_scan),
//           ImGuiState::DatasetTesting::query_obj_idx,
//           ImGuiState::DatasetTesting::ref_obj_idx,
//           ImGuiState::DatasetTesting::q_so,
//           ImGuiState::DatasetTesting::r_so);
//
//       Processing::PointCloud::computeSOR(ImGuiState::DatasetTesting::q_so,
//                                          ImGuiState::DatasetTesting::meanK,
//                                          ImGuiState::DatasetTesting::stdThresh);
//       Processing::PointCloud::computeSOR(ImGuiState::DatasetTesting::r_so,
//                                          ImGuiState::DatasetTesting::meanK,
//                                          ImGuiState::DatasetTesting::stdThresh);
//
//       double size = pl->GetSize(ImGuiState::DatasetTesting::filtering_opts,
//                                 ImGuiState::DatasetTesting::sample_size,
//                                 ImGuiState::DatasetTesting::filter_percent,
//                                 ImGuiState::DatasetTesting::q_so.cloud->size(),
//                                 ImGuiState::DatasetTesting::r_so.cloud->size());
//
//       Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::q_so,
//                                          size);
//       Processing::PointCloud::computeFPS(ImGuiState::DatasetTesting::r_so,
//                                          size);
//
//       if (ImGuiState::DatasetTesting::double_sor) {
//         Processing::PointCloud::computeSOR(
//             ImGuiState::DatasetTesting::q_so,
//             ImGuiState::DatasetTesting::meanK,
//             ImGuiState::DatasetTesting::stdThresh);
//         Processing::PointCloud::computeSOR(
//             ImGuiState::DatasetTesting::r_so,
//             ImGuiState::DatasetTesting::meanK,
//             ImGuiState::DatasetTesting::stdThresh);
//       }
//
//       Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::q_so);
//       Processing::PointCloud::computeMCAR(ImGuiState::DatasetTesting::r_so);
//
//       if (ImGuiState::DatasetTesting::same_radius) {
//         double mcar = std::max(ImGuiState::DatasetTesting::q_so.mcar,
//                                ImGuiState::DatasetTesting::r_so.mcar);
//         ImGuiState::DatasetTesting::q_so.mcar = mcar;
//         ImGuiState::DatasetTesting::r_so.mcar = mcar;
//       }
//
//       //
//       Processing::Laplacian::genericLaplacian(ImGuiState::DatasetTesting::q_so);
//       //
//       Processing::Laplacian::genericLaplacian(ImGuiState::DatasetTesting::r_so);
//       pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                     ImGuiState::DatasetTesting::q_so);
//       pl->Laplacian(ImGuiState::DatasetTesting::laplacian_idx,
//                     ImGuiState::DatasetTesting::r_so);
//
//       int number_eigs = ImGuiState::DatasetTesting::q_so.cloud->size();
//       if (ImGuiState::DatasetTesting::eigendecomposition_method == 1) {
//         number_eigs = ImGuiState::DatasetTesting::eigs_number;
//       }
//
//       Processing::Eigen::computeEigenvalues(ImGuiState::DatasetTesting::q_so,
//                                             number_eigs);
//       Processing::Eigen::computeEigenvalues(ImGuiState::DatasetTesting::r_so,
//                                             number_eigs);
//
//       ImGuiState::DatasetTesting::update_cloud = true;
//       ImGuiState::DatasetTesting::mtx.unlock();
//
//       ImGuiState::DatasetTesting::eigs_mtx.lock();
//
//       ImGuiState::DatasetTesting::eig_srv.request.q_eigs =
//           arma::conv_to<std::vector<double>>::from(
//               ImGuiState::DatasetTesting::q_so.eigenvalues);
//       ImGuiState::DatasetTesting::eig_srv.request.r_eigs =
//           arma::conv_to<std::vector<double>>::from(
//               ImGuiState::DatasetTesting::r_so.eigenvalues);
//       ImGuiState::DatasetTesting::eig_srv.request.q_gfa =
//           ImGuiState::DatasetTesting::q_so.gfaFeatures;
//       ImGuiState::DatasetTesting::eig_srv.request.r_gfa =
//           ImGuiState::DatasetTesting::r_so.gfaFeatures;
//
//       ImGuiState::DatasetTesting::update_hist = true;
//       ImGuiState::DatasetTesting::eigs_mtx.unlock();
//
//       // Eval service
//       if
//       (evaluation_service_client.call(ImGuiState::DatasetTesting::eig_srv))
//       {
//         ROS_INFO("eval service success!!! %f",
//                  ImGuiState::DatasetTesting::eig_srv.response.results[0]);
//       } else {
//         ROS_ERROR("eval service failed");
//       }
//
//     } else {
//       ImGuiState::DatasetTesting::ref_obj_exists = false;
//     }
//
//     ImGuiState::DatasetTesting::query_obj_idx++;
//   }
//
//   if (!ImGuiState::DatasetTesting::ReadyToStep())
//     ImGui::BeginDisabled();
//
//   if (!ImGuiState::DatasetTesting::RefObjExists())
//     ImGui::Text("Ref Object Does Not Exist");
//
//   if (!ImGuiState::DatasetTesting::ReadyToStep())
//     ImGui::EndDisabled();
//
//   if (!ImGuiState::DatasetTesting::DatasetParsed()) {
//     ImGui::EndDisabled();
//   }
//
//   ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
//               1000.0f / ImGui::GetIO().Framerate,
//               ImGui::GetIO().Framerate);
//   ImGui::End();
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "sgpr_ros_node", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  evaluation_service_client =
      n.serviceClient<sgpr_ros::Eigenvalues>("evaluation_service");

  evaluation_service_client.waitForExistence(ros::Duration(10));

  // Todo need to use the param server at somepoint
  // ros::param::get("dataset", dataset);

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::shared_ptr<Pipeline> datasetPipeline;

  // Background threads running
  // std::thread viz_t(BackgroundVizThread);
  // viz_t.detach();

  // std::thread spectra_t(PlotSpectra);
  // spectra_t.detach();

  GLFWwindow *window = initGUI();

  ImGuiState::DatasetTesting::dataset_idx = 2;
  // GUI loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // datasetTestingPipeline(datasetPipeline);
    semanticKittiTestingPipeline(datasetPipeline);

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 1;
}
