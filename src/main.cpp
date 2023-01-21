#include <boost/smart_ptr/make_shared_object.hpp>
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

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_state.h"
#include "ros/duration.h"
#include <stdio.h>
#include <vtkIOStream.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

using namespace std::chrono_literals;
using namespace matplot;

ros::ServiceClient evaluation_service_client;

pcl::visualization::PCLVisualizer::Ptr viewer;
matplot::figure_handle f;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr thread_cloud1;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr thread_cloud2;
bool update_cloud = true;
bool update_hist = false;
bool show_radius = false;
bool radius_toggled = false;
double thread_r1;
double thread_r2;
std::mutex mtx, eigs_mtx;

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
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
    if (update_hist) {
      eigs_mtx.lock();
      cla();
      std::vector<double> ref =
          ImGuiState::DatasetTesting::eig_srv.request.r_eigs;
      std::vector<double> query =
          ImGuiState::DatasetTesting::eig_srv.request.q_eigs;

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
      update_hist = false;
      eigs_mtx.unlock();
    }
  }
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void) {
  if (event.getKeySym() == "r" && event.keyDown()) {
    std::cout << "r was pressed => removing all text" << std::endl;
    mtx.lock();
    show_radius = !show_radius;
    radius_toggled = true;
    mtx.unlock();
  }
}

void BackgroundVizThread() {

  int v0(0);
  int v1(1);

  // TODO I can probably delete this
  if (viewer.use_count() == 0) {

    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
    viewer->setPosition(72, 0);
    viewer->setSize(600, 200);
    viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);

    viewer->initCameraParameters();
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v0);
    viewer->setBackgroundColor(0, 0, 0, v0);
    viewer->addCoordinateSystem(1.0, "coord1", v0);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
    viewer->addCoordinateSystem(1.0, "coord2", v1);
  }

  while (!viewer->wasStopped()) {
    mtx.lock();
    if (radius_toggled) {
      if (show_radius) {

        auto start = std::chrono::steady_clock::now();
        unsigned int max_nn = 1000;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdTree;

        // TODO Maybe separate this so we are not duplicating code
        kdTree.setInputCloud(thread_cloud1);
        int size1 = thread_cloud1->size();

        for (int i = 0; i < size1; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, thread_r1, indicies_found, squaredDistances,
                              max_nn);

          pcl::PointXYZRGB pt_r = thread_cloud1->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q = thread_cloud1->points[idx];
            // viewer->addLine(pt_r, pt_q, "line", 0);
            std::string string_id =
                "0" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v0);
          }
        }

        kdTree.setInputCloud(thread_cloud2);
        int size2 = thread_cloud2->size();

        for (int i = 0; i < size2; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, thread_r2, indicies_found, squaredDistances,
                              max_nn);

          pcl::PointXYZRGB pt_r = thread_cloud2->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q = thread_cloud2->points[idx];
            std::string string_id =
                "1" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v1);
          }
        }
      } else {
        viewer->removeAllShapes(v0);
        viewer->removeAllShapes(v1);
      }
      radius_toggled = false;
    }

    if (update_cloud) {
      // TODO try and replace this with updateCloud instead if everything works
      viewer->removeAllPointClouds(v0);
      viewer->removeAllPointClouds(v1);
      viewer->removeAllShapes(v0);
      viewer->removeAllShapes(v1);

      viewer->addPointCloud(thread_cloud1, "cloud1", v0);
      viewer->addPointCloud(thread_cloud2, "cloud2", v1);

      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1", v0);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2", v1);

      if (show_radius) {
        unsigned int max_nn = 1000;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdTree;

        // TODO Maybe separate this so we are not duplicating code
        kdTree.setInputCloud(thread_cloud1);
        int size1 = thread_cloud1->size();

        for (int i = 0; i < size1; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, thread_r1, indicies_found, squaredDistances,
                              max_nn);

          pcl::PointXYZRGB pt_r = thread_cloud1->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q = thread_cloud1->points[idx];
            // viewer->addLine(pt_r, pt_q, "line", 0);
            std::string string_id =
                "0" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v0);
          }
        }

        kdTree.setInputCloud(thread_cloud2);
        int size2 = thread_cloud2->size();

        for (int i = 0; i < size2; i++) {
          std::vector<int> indicies_found;
          std::vector<float> squaredDistances;
          kdTree.radiusSearch(i, thread_r2, indicies_found, squaredDistances,
                              max_nn);

          pcl::PointXYZRGB pt_r = thread_cloud2->points[i];

          for (int j = 1; j < indicies_found.size(); j++) {
            int idx = indicies_found[j];
            pcl::PointXYZRGB pt_q = thread_cloud2->points[idx];
            std::string string_id =
                "1" + std::to_string(i) + "-" + std::to_string(j);
            viewer->addLine(pt_r, pt_q, string_id, v1);
          }
        }
      }

      update_cloud = false;
    }
    mtx.unlock();
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

void novelMethodsTesting(NovelMethodTestingPipeline &pl) {

  ImGui::Begin("Novel Methods Testing");

  // --------------------------------------------------------------
  ImGui::Text("Point Clouds");

  if (ImGui::Button("DELETE ME")) {
    // Lock here
    mtx.lock();
    thread_cloud1.reset();
    thread_cloud1 = pl.GetCloud2();
    thread_cloud2.reset();
    thread_cloud2 = pl.GetCloud2();
    update_cloud = true;
    mtx.unlock();
  }

  ImGui::Combo("Choose PointCloud 1", &ImGuiState::NMT::point_cloud_idx1,
               ImGuiState::pointclouds, IM_ARRAYSIZE(ImGuiState::pointclouds));

  ImGui::Combo("Choose PointCloud 2", &ImGuiState::NMT::point_cloud_idx2,
               ImGuiState::pointclouds, IM_ARRAYSIZE(ImGuiState::pointclouds));

  ImGui::InputInt("Max number of points in point cloud",
                  &ImGuiState::NMT::max_pts);

  if (ImGui::Button("Button 1")) {
    pl.ParsePointCloudPair(
        ImGuiState::NMT::GetPlyFileName(ImGuiState::NMT::point_cloud_idx1),
        ImGuiState::NMT::GetPlyFileName(ImGuiState::NMT::point_cloud_idx2),
        ImGuiState::NMT::max_pts);
    ImGuiState::NMT::point_clouds_read = true;
  }

  ImGui::SameLine();
  (ImGuiState::NMT::PointCloudsRead())
      ? ImGui::Text("Success! read point clouds")
      : ImGui::Text("Click to read in point cloud files");

  // Extract object point clouds
  if (!ImGuiState::NMT::PointCloudsRead()) {
    ImGui::BeginDisabled();
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Graph Formulation");

  ImGui::Combo("Edge Heuristic", &ImGuiState::NMT::edge_heuristic_idx,
               ImGuiState::edge_heuristics,
               IM_ARRAYSIZE(ImGuiState::edge_heuristics));

  if (ImGui::Button("Button 2")) {
    pl.ComputeEdges(ImGuiState::NMT::edge_heuristic_idx);
    ImGuiState::NMT::edges_created = true;
  }

  ImGui::SameLine();
  (ImGuiState::NMT::EdgesCreated())
      ? ImGui::Text("Success! graph edges created")
      : ImGui::Text("Click to compute graph edges");

  if (ImGuiState::NMT::ShowRadius()) {
    ImGui::Text("Radius for Cloud 1 is: %f", pl.GetRadius1());
    ImGui::Text("Radius for Cloud 2 is: %f", pl.GetRadius2());
  }

  if (!ImGuiState::NMT::PointCloudsRead())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::EdgesCreated()) {
    ImGui::BeginDisabled();
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Laplacian");

  ImGui::Combo("Laplacian Algorithm", &ImGuiState::NMT::laplacian_idx,
               ImGuiState::laplacians, IM_ARRAYSIZE(ImGuiState::laplacians));

  if (ImGui::Button("Button 3")) {
    pl.ComputeLaplacian(ImGuiState::NMT::laplacian_idx);
    ImGuiState::NMT::laplacian_created = true;
  }

  ImGui::SameLine();
  (ImGuiState::NMT::LaplacianCreated())
      ? ImGui::Text("Success! Laplacian computed")
      : ImGui::Text("Click to compute Laplacian");

  if (!ImGuiState::NMT::EdgesCreated())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::LaplacianCreated())
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Eigenvalues");

  ImGui::RadioButton("All Eigenvalues",
                     &ImGuiState::NMT::eigendecomposition_method, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Specific # of Eigenvalues",
                     &ImGuiState::NMT::eigendecomposition_method, 1);

  // --- Input Box
  if (ImGuiState::NMT::eigendecomposition_method == 0)
    ImGui::BeginDisabled();

  ImGui::InputInt("# of Eigenvalues to compute", &ImGuiState::NMT::eigs_number);

  if (ImGuiState::NMT::eigendecomposition_method == 0)
    ImGui::EndDisabled();
  // --- ! Input Box

  if (ImGui::Button("Button 4")) {
    int eigs_num = (ImGuiState::NMT::eigendecomposition_method == 0)
                       ? -1
                       : ImGuiState::NMT::eigs_number;
    pl.ComputeEigs(eigs_num);
    ImGuiState::NMT::eigs = true;
  }

  ImGui::SameLine();
  (ImGuiState::NMT::ComputedEigs())
      ? ImGui::Text("Success! Eigenvalues computed")
      : ImGui::Text("Click to compute eigenvalues");

  if (!ImGuiState::NMT::LaplacianCreated())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::ComputedEigs())
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Save Eigenvalues");

  ImGui::InputText("file name to save eigenvalues",
                   ImGuiState::NMT::eigenvalue_json_f,
                   IM_ARRAYSIZE(ImGuiState::NMT::eigenvalue_json_f));
  if (ImGui::Button("Button 5")) {
    std::string file_name(ImGuiState::NMT::eigenvalue_json_f);
    pl.SaveEigenvalues(file_name);
    ImGuiState::NMT::saved_eigs = true;
  }

  ImGui::SameLine();
  (ImGuiState::NMT::SavedEigs()) ? ImGui::Text("Success! Eigenvalues saved")
                                 : ImGui::Text("Click to save eigenvalues");

  if (!ImGuiState::NMT::ComputedEigs())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::PointCloudsRead()) // || TODO viz is already open)
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Visualizer");

  if (ImGui::Button("Button 6")) {
    ImGuiState::NMT::pcl_viz = true;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = pl.GetCloud1();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = pl.GetCloud2();

    // std::thread t1(RunVizThread, cloud1, cloud2);
    // t1.detach();
  }

  ImGui::SameLine();
  (ImGuiState::NMT::PCLViz())
      ? ImGui::Text("Success! PCL visualizer is running")
      : ImGui::Text("Click to run PCL visualizer");

  if (!ImGuiState::NMT::EdgesCreated())
    ImGui::BeginDisabled();

  if (ImGui::Button("Button 7")) {
    ImGuiState::NMT::pcl_viz_connection = true;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = pl.GetCloud1();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = pl.GetCloud2();

    // std::thread t1(RunVizConnectionThread, cloud1, cloud2, pl.GetRadius1(),
    //                pl.GetRadius2());
    // t1.detach();
  }

  ImGui::SameLine();
  (ImGuiState::NMT::PCLConnectionViz())
      ? ImGui::Text("Success! PCL graph connection visualizer is running")
      : ImGui::Text("Click to run PCL graph connection visualizer");

  if (!ImGuiState::NMT::EdgesCreated())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::ComputedEigs()) // || TODO viz is already open)
    ImGui::BeginDisabled();

  if (ImGui::Button("Button 8")) {
    ImGuiState::NMT::matplot = true;
    pl.PlotHistograms();
  }

  ImGui::SameLine();
  (ImGuiState::NMT::MatplotViz())
      ? ImGui::Text("Success! Matplot++ visualizer is running")
      : ImGui::Text("Click to run MatPlot++ visualizer");

  if (!ImGuiState::NMT::ComputedEigs())
    ImGui::EndDisabled();

  if (!ImGuiState::NMT::PointCloudsRead())
    ImGui::EndDisabled();

  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

void datasetTestingPipeline(std::shared_ptr<Pipeline> &pl) {

  ImGui::Begin("Dataset Testing");

  // --------------------------------------------------------------
  ImGui::Text("Dataset");

  ImGui::Combo("Choose Dataset", &ImGuiState::DatasetTesting::dataset_idx,
               ImGuiState::datasets, IM_ARRAYSIZE(ImGuiState::datasets));

  ImGui::InputInt("Max number of points in point cloud",
                  &ImGuiState::DatasetTesting::max_pts);

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
      // pl = std::make_shared<SemanticKittiPipeline>();
      break;
    default:
      break;
    }

    pl->ParseDataset();
    pl->ExtractObjectPointClouds(ImGuiState::DatasetTesting::max_pts);
    ImGuiState::DatasetTesting::dataset_parsed = true;
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::DatasetParsed())
      ? ImGui::Text("Success! Parse datset")
      : ImGui::Text("Click to parse dataset");

  if (!ImGuiState::DatasetTesting::DatasetParsed()) {
    ImGui::BeginDisabled();
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("To Step Or Not To Step");

  ImGui::RadioButton("Step", &ImGuiState::DatasetTesting::should_step, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Don't Step", &ImGuiState::DatasetTesting::should_step, 1);

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Graph Formulation");

  ImGui::Combo(
      "Edge Heuristic", &ImGuiState::DatasetTesting::edge_heuristic_idx,
      ImGuiState::edge_heuristics, IM_ARRAYSIZE(ImGuiState::edge_heuristics));

  if (ImGui::Button("Button 2")) {
    pl->ComputeEdges(ImGuiState::DatasetTesting::edge_heuristic_idx);
    ImGuiState::DatasetTesting::edges_created = true;
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::EdgesCreated())
      ? ImGui::Text("Success! graph edges created")
      : ImGui::Text("Click to compute graph edges");

  // if (ImGuiState::ShowRadius()) {
  //   ImGui::Text("Radius for Cloud 1 is: %f", pl.GetRadius1());
  //   ImGui::Text("Radius for Cloud 2 is: %f", pl.GetRadius2());
  // }

  if (!ImGuiState::DatasetTesting::DatasetParsed())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::EdgesCreated()) {
    ImGui::BeginDisabled();
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Laplacian");

  ImGui::Combo("Laplacian Algorithm",
               &ImGuiState::DatasetTesting::laplacian_idx,
               ImGuiState::laplacians, IM_ARRAYSIZE(ImGuiState::laplacians));

  if (ImGui::Button("Button 3")) {
    pl->ComputeLaplacian(ImGuiState::DatasetTesting::laplacian_idx);
    ImGuiState::DatasetTesting::laplacian_created = true;
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::LaplacianCreated())
      ? ImGui::Text("Success! Laplacian computed")
      : ImGui::Text("Click to compute Laplacian");

  if (!ImGuiState::DatasetTesting::EdgesCreated())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::LaplacianCreated())
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Eigenvalues");

  ImGui::RadioButton("All Eigenvalues",
                     &ImGuiState::DatasetTesting::eigendecomposition_method, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Specific # of Eigenvalues",
                     &ImGuiState::DatasetTesting::eigendecomposition_method, 1);

  // --- Input Box
  if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
    ImGui::BeginDisabled();

  ImGui::InputInt("# of Eigenvalues to compute",
                  &ImGuiState::DatasetTesting::eigs_number);

  if (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
    ImGui::EndDisabled();
  // --- ! Input Box

  if (ImGui::Button("Button 4")) {
    int eigs_num = (ImGuiState::DatasetTesting::eigendecomposition_method == 0)
                       ? -1
                       : ImGuiState::DatasetTesting::eigs_number;
    pl->ComputeEigs(eigs_num);
    ImGuiState::DatasetTesting::eigs = true;
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::ComputedEigs())
      ? ImGui::Text("Success! Eigenvalues computed")
      : ImGui::Text("Click to compute eigenvalues");

  if (!ImGuiState::DatasetTesting::LaplacianCreated())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::ComputedEigs())
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Save Eigenvalues");

  ImGui::InputText("file name to save eigenvalues",
                   ImGuiState::DatasetTesting::eigenvalue_json_f,
                   IM_ARRAYSIZE(ImGuiState::DatasetTesting::eigenvalue_json_f));
  if (ImGui::Button("Button 5")) {
    std::string file_name(ImGuiState::DatasetTesting::eigenvalue_json_f);
    pl->SaveEigenvalues(file_name);
    ImGuiState::DatasetTesting::saved_eigs = true;
  }

  ImGui::SameLine();
  (ImGuiState::DatasetTesting::SavedEigs())
      ? ImGui::Text("Success! Eigenvalues saved")
      : ImGui::Text("Click to save eigenvalues");

  if (!ImGuiState::DatasetTesting::ShouldStep())
    ImGui::BeginDisabled();

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Choose Scans");

  if (ImGuiState::DatasetTesting::GetQueryScans()) {
    pl->GetQueryScans(ImGuiState::DatasetTesting::query_scans);
  }

  const char *selected_query_scan =
      ImGuiState::DatasetTesting::GetSelectedQueryScan();

  if (ImGui::BeginCombo("Choose A Query Scan", selected_query_scan)) {
    for (int i = 0; i < ImGuiState::DatasetTesting::query_scans.size(); i++) {
      const bool isSelected = (ImGuiState::DatasetTesting::query_scan_idx == i);
      if (ImGui::Selectable(ImGuiState::DatasetTesting::query_scans[i].c_str(),
                            isSelected)) {
        ImGuiState::DatasetTesting::query_scan_idx = i;
      }

      if (isSelected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  const char *selected_ref_scan =
      (ImGuiState::DatasetTesting::query_scans.size() > 0)
          ? pl->GetMappedRefScan(ImGuiState::DatasetTesting::query_scans,
                                 ImGuiState::DatasetTesting::query_scan_idx)
          : "";

  if (ImGuiState::DatasetTesting::GetRefScans())
    pl->GetRefScans(ImGuiState::DatasetTesting::ref_scans);

  if (ImGuiState::DatasetTesting::UpdateRefScanSelected()) {
    ImGuiState::DatasetTesting::last_query_scan_idx =
        ImGuiState::DatasetTesting::query_scan_idx;
    std::string ref = std::string(selected_ref_scan);
    auto it = std::find(ImGuiState::DatasetTesting::ref_scans.begin(),
                        ImGuiState::DatasetTesting::ref_scans.end(), ref);

    if (it != ImGuiState::DatasetTesting::ref_scans.end()) {
      ImGuiState::DatasetTesting::ref_scan_idx =
          it - ImGuiState::DatasetTesting::ref_scans.begin();
    } else {
      std::cout << "ERROR reference scan not in list" << std::endl;
      std::exit(1);
    }
  }

  if (ImGui::BeginCombo("Choose A Reference Scan", selected_ref_scan)) {
    for (int i = 0; i < ImGuiState::DatasetTesting::ref_scans.size(); i++) {
      const bool isSelected = (ImGuiState::DatasetTesting::ref_scan_idx == i);
      if (ImGui::Selectable(ImGuiState::DatasetTesting::ref_scans[i].c_str(),
                            isSelected)) {
        ImGuiState::DatasetTesting::ref_scan_idx = i;
      }

      if (isSelected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Choose Objects");

  ImGui::RadioButton("Compare All Matching Objects",
                     &ImGuiState::DatasetTesting::obj_compare_method, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Select Objects",
                     &ImGuiState::DatasetTesting::obj_compare_method, 1);

  // --------------------------------------------------------------
  ImGui::Separator();
  ImGui::Text("Visualizer");

  if (ImGui::Button("Init Objects")) {
    std::cout << "INIT Pressed" << std::endl;
    ImGuiState::DatasetTesting::query_obj_scene_ids.clear();
    ImGuiState::DatasetTesting::query_obj_idx = 0;
    pl->GetQuerySpectralObjIds(ImGuiState::DatasetTesting::query_obj_scene_ids,
                               std::string(selected_query_scan));
  }

  if (ImGui::Button("Vizualize Object Pairs")) {
    std::cout << "Step Pressed" << std::endl;

    if (pl->RefObjExists(std::string(selected_query_scan),
                         ImGuiState::DatasetTesting::query_obj_idx,
                         ImGuiState::DatasetTesting::ref_obj_idx)) {

      ImGuiState::DatasetTesting::cloud1.reset();
      ImGuiState::DatasetTesting::cloud2.reset();

      ImGuiState::DatasetTesting::cloud1 =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      ImGuiState::DatasetTesting::cloud2 =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

      ImGuiState::DatasetTesting::ref_obj_exists = true;
      // Get Point Clouds
      pl->GetQueryRefCloudObjPair(std::string(selected_query_scan),
                                  std::string(selected_ref_scan),
                                  ImGuiState::DatasetTesting::query_obj_idx,
                                  ImGuiState::DatasetTesting::ref_obj_idx,
                                  ImGuiState::DatasetTesting::cloud1,
                                  ImGuiState::DatasetTesting::cloud2);

      mtx.lock();
      thread_cloud1.reset();
      thread_cloud1 = ImGuiState::DatasetTesting::cloud1;
      thread_cloud2.reset();
      thread_cloud2 = ImGuiState::DatasetTesting::cloud2;
      update_cloud = true;
      mtx.unlock();

      //----------------------------------------------------------------------
      // DELETEME
      // Call viz thread
      // std::thread t1(RunVizThread, ImGuiState::DatasetTesting::cloud1,
      //               ImGuiState::DatasetTesting::cloud2);
      // t1.detach();
      //----------------------------------------------------------------------

      // Todo convert pcl point cloud to pointcloud 2 and send them
      thread_r1 = pl->GetRadius(std::string(selected_query_scan),
                                ImGuiState::DatasetTesting::query_obj_idx);
      thread_r2 = pl->GetRadius(std::string(selected_ref_scan),
                                ImGuiState::DatasetTesting::ref_obj_idx);

      std::cout << "Cloud1 size: " << thread_cloud1->size() << std::endl;
      std::cout << "Cloud2 size: " << thread_cloud2->size() << std::endl;

      std::cout << "r1 size:" << thread_r1 << std::endl;
      std::cout << "r2 size:" << thread_r2 << std::endl;

      // sensor_msgs::PointCloud2 ros_cloud1, ros_cloud2;
      // pcl::toROSMsg(*ImGuiState::DatasetTesting::cloud1, ros_cloud1);
      // pcl::toROSMsg(*ImGuiState::DatasetTesting::cloud2, ros_cloud2);

      // sgpr_ros::PointClouds pc_srv;
      // pc_srv.request.cloud1 = ros_cloud1;
      // pc_srv.request.cloud2 = ros_cloud2;
      // pc_srv.request.radius1 = r1;
      // pc_srv.request.radius2 = r2;
      // if (point_cloud_service_client.call(pc_srv)) {
      //   ROS_INFO("Point cloud service success!!! %d", pc_srv.response.done);
      // } else {
      //   ROS_ERROR("Point cloud service failed");
      // }

      // call point cloud connetion viz thread

      // std::cout << "r1: " << r1 << std::endl;
      // std::cout << "r2: " << r2 << std::endl;

      //----------------------------------------------------------------------
      // DELETEME
      // std::thread t2(RunVizConnectionThread,
      // ImGuiState::DatasetTesting::cloud1,
      //              ImGuiState::DatasetTesting::cloud2, r1, r2);
      // t2.detach();
      //----------------------------------------------------------------------

      eigs_mtx.lock();
      pl->GetEigs(ImGuiState::DatasetTesting::eig_srv,
                  std::string(selected_query_scan),
                  ImGuiState::DatasetTesting::query_obj_idx,
                  std::string(selected_ref_scan),
                  ImGuiState::DatasetTesting::ref_obj_idx);
      update_hist = true;
      eigs_mtx.unlock();

      // if (histogram_service_client.call(eig_srv)) {
      //   ROS_INFO("histogram service success!!! %f",
      //            eig_srv.response.results[0]);
      // } else {
      //   ROS_ERROR("histogram service failed");
      // }

      //// Eval service
      if (evaluation_service_client.call(ImGuiState::DatasetTesting::eig_srv)) {
        ROS_INFO("eval service success!!! %f",
                 ImGuiState::DatasetTesting::eig_srv.response.results[0]);
      } else {
        ROS_ERROR("eval service failed");
      }

      //----------------------------------------------------------------------
      // DELETEME
      /// Call matplot lib histogram func
      // pl->PlotHistograms(std::string(selected_ref_scan),
      //                    std::string(selected_query_scan),
      //                    ImGuiState::DatasetTesting::query_obj_idx,
      //                    ImGuiState::DatasetTesting::ref_obj_idx);

      //----------------------------------------------------------------------

      // Call rosservice for ad and ks test
      // sgpr_ros::Eigenvalues srv;
      // srv.request.a = 1;
      // srv.request.b = 2;
      // if (ks_service_client.call(srv)) {
      //  ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      //} else {
      //  ROS_ERROR("Failed to call service add_two_ints");
      //}

      // if (ad_service_client.call(srv)) {
      //   ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      // } else {
      //   ROS_ERROR("Failed to call service add_two_ints");
      // }

      ImGuiState::DatasetTesting::cloud1.reset();
      ImGuiState::DatasetTesting::cloud2.reset();

    } else {
      ImGuiState::DatasetTesting::ref_obj_exists = false;
    }

    ImGuiState::DatasetTesting::query_obj_idx++;
  }

  // TODO need a scene is done function
  if (!ImGuiState::DatasetTesting::ReadyToStep())
    ImGui::BeginDisabled();

  if (!ImGuiState::DatasetTesting::RefObjExists())
    ImGui::Text("Ref Object Does Not Exist");

  // TODO
  // First try and figure out why the viz thread is not closing
  // options:
  //  a) do get viz thread to close
  //  b) keep doing what we are doing
  //  c) (Probably the cleanest but most work) throw it in ros and create a
  //  service msg which will do all the viz stuff in different processes
  //}

  if (!ImGuiState::DatasetTesting::ReadyToStep())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::ShouldStep())
    ImGui::EndDisabled();

  if (!ImGuiState::DatasetTesting::ComputedEigs())
    ImGui::EndDisabled();

  // if (!ImGuiState::PointCloudsRead()) // || TODO viz is already open)
  //   ImGui::BeginDisabled();

  //// --------------------------------------------------------------
  // ImGui::Separator();
  // ImGui::Text("Visualizer");

  // if (ImGui::Button("Button 6")) {
  //   ImGuiState::pcl_viz = true;
  //   auto pointCloudPair = pl.GetPointCloudPair();

  //  std::thread t1(RunVizThread, pointCloudPair.first, pointCloudPair.second);
  //  t1.detach();
  //}

  // ImGui::SameLine();
  //(ImGuiState::PCLViz()) ? ImGui::Text("Success! PCL visualizer is running")
  //                        : ImGui::Text("Click to run PCL visualizer");

  // if (!ImGuiState::EdgesCreated())
  //   ImGui::BeginDisabled();

  // if (ImGui::Button("Button 7")) {
  //   ImGuiState::pcl_viz_connection = true;
  //   auto pointCloudPair = pl.GetPointCloudPair();

  //  std::thread t1(RunVizConnectionThread, pointCloudPair.first,
  //                 pointCloudPair.second, pl.GetRadius1(), pl.GetRadius2());
  //  t1.detach();
  //}

  // ImGui::SameLine();
  //(ImGuiState::PCLConnectionViz())
  //     ? ImGui::Text("Success! PCL graph connection visualizer is running")
  //     : ImGui::Text("Click to run PCL graph connection visualizer");

  // if (!ImGuiState::EdgesCreated())
  //   ImGui::EndDisabled();

  // if (!ImGuiState::ComputedEigs()) // || TODO viz is already open)
  //   ImGui::BeginDisabled();

  // if (ImGui::Button("Button 8")) {
  //   ImGuiState::matplot = true;
  //   pl->PlotHistograms();
  // }

  // ImGui::SameLine();
  //(ImGuiState::MatplotViz())
  //     ? ImGui::Text("Success! Matplot++ visualizer is running")
  //     : ImGui::Text("Click to run MatPlot++ visualizer");

  // if (!ImGuiState::ComputedEigs())
  //   ImGui::EndDisabled();

  // if (!ImGuiState::PointCloudsRead())
  //   ImGui::EndDisabled();

  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

// NOTE: Only for laptop. Throw it under Begin block for the window
// ImGui::SetWindowFontScale(5.0f);

int main(int argc, char **argv) {
  ros::init(argc, argv, "sgpr_ros_node");
  ros::NodeHandle n;

  evaluation_service_client =
      n.serviceClient<sgpr_ros::Eigenvalues>("evaluation_service");

  evaluation_service_client.waitForExistence(ros::Duration(10));

  // Todo need to use the param server at somepoint
  // ros::param::get("dataset", dataset);

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  NovelMethodTestingPipeline nmtPipeline;
  std::shared_ptr<Pipeline> datasetPipeline;

  nmtPipeline.ParsePointCloudPair("/home/nate/Datasets/teddyPly/b1.ply",
                                  "/home/nate/Datasets/teddyPly/b7.ply", 300);
  thread_cloud1 = nmtPipeline.GetCloud1();
  thread_cloud2 = nmtPipeline.GetCloud1();

  // Background threads running
  std::thread viz_t(BackgroundVizThread);
  viz_t.detach();

  std::thread spectra_t(PlotSpectra);
  spectra_t.detach();

  GLFWwindow *window = initGUI();

  // GUI loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    novelMethodsTesting(nmtPipeline);
    datasetTestingPipeline(datasetPipeline);

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

  // switch (dataset) {
  // case 0:
  // case 1:
  //   pl.PostProcess(dataset);
  //   break;
  // case 2:
  //   pl.RealTime();
  //   break;
  // default:
  //   //   ROS_DEBUG("NOT SUPPORTED");
  //   break;
  // }

  // ROS_DEBUG("Done Processing");

  return 1;
}
