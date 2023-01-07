#include <iostream>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>

#include <Pipeline.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_state.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

using namespace std::chrono_literals;

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void RunVizThread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->initCameraParameters();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZ>(cloud1, "Cloud 1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, "Cloud 2", v2);

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud 1");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud 2");
  viewer->addCoordinateSystem(1.0);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
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
      glfwCreateWindow(1280, 720, "SGPR Data Explorer", NULL, NULL);
  if (window == NULL)
    exit(1);
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

// void datasetComparisonTesting(Pipeline &pl) {
//
//   // Dropdown selectable box. dataset_idx holds the value for the dataset to
//   // use
//   ImGui::Combo("Choose Dataset", &ImGuiState::dataset_idx,
//   ImGuiState::datasets,
//                IM_ARRAYSIZE(ImGuiState::datasets));
// }

int main(int argc, char **argv) {

  GLFWwindow *window = initGUI();
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // Define Pipeline
  Pipeline pl;

  // GUI loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Pipeline");

    // Only for laptop
    //ImGui::SetWindowFontScale(5.0f);

    // --------------------------------------------------------------
    ImGui::Text("Point Clouds");

    ImGui::Combo("Choose PointCloud 1", &ImGuiState::point_cloud_idx1,
                 ImGuiState::pointclouds,
                 IM_ARRAYSIZE(ImGuiState::pointclouds));

    ImGui::Combo("Choose PointCloud 2", &ImGuiState::point_cloud_idx2,
                 ImGuiState::pointclouds,
                 IM_ARRAYSIZE(ImGuiState::pointclouds));

    if (ImGui::Button("Button 1")) {
      pl.ParsePointCloudPair(
          ImGuiState::GetPlyFileName(ImGuiState::point_cloud_idx1),
          ImGuiState::GetPlyFileName(ImGuiState::point_cloud_idx2));
      ImGuiState::point_clouds_read = true;
    }

    ImGui::SameLine();
    (ImGuiState::PointCloudsRead())
        ? ImGui::Text("Success! read point clouds")
        : ImGui::Text("Click to read in point cloud files");

    // Extract object point clouds
    if (!ImGuiState::PointCloudsRead()) {
      ImGui::BeginDisabled();
    }

    // --------------------------------------------------------------
    ImGui::Separator();
    ImGui::Text("Graph Formulation");

    // TODO: Choose edge heuristic algo
    ImGui::Combo("Edge Heuristic", &ImGuiState::edge_heuristic_idx,
                 ImGuiState::edge_heuristics,
                 IM_ARRAYSIZE(ImGuiState::edge_heuristics));

    if (ImGui::Button("Button 2")) {
      pl.ComputeEdges(ImGuiState::edge_heuristic_idx, ImGuiState::radius_1, ImGuiState::radius_2);
      ImGuiState::edges_created = true;
    }

    ImGui::SameLine();
    (ImGuiState::EdgesCreated()) ? ImGui::Text("Success! graph edges created")
                                 : ImGui::Text("Click to compute graph edges");

    if (ImGuiState::ShowRadius()) {
      ImGui::Text("Radius for Cloud 1 is: %f", ImGuiState::radius_1);
      ImGui::Text("Radius for Cloud 2 is: %f", ImGuiState::radius_2);
    }

    if (!ImGuiState::PointCloudsRead())
      ImGui::EndDisabled();

    if (!ImGuiState::EdgesCreated()) {
      ImGui::BeginDisabled();
    }

    // --------------------------------------------------------------
    ImGui::Separator();
    ImGui::Text("Laplacian");

    // TODO: Choose edge heuristic algo
    ImGui::Combo("Laplacian Algorithm", &ImGuiState::laplacian_idx,
                 ImGuiState::laplacians, IM_ARRAYSIZE(ImGuiState::laplacians));

    if (ImGui::Button("Button 3")) {
      // pl.MCAR();
      ImGuiState::laplacian_created = true;
    }

    ImGui::SameLine();
    (ImGuiState::LaplacianCreated())
        ? ImGui::Text("Success! Laplacian computed")
        : ImGui::Text("Click to compute Laplacian");

    if (!ImGuiState::EdgesCreated())
      ImGui::EndDisabled();

    if (!ImGuiState::LaplacianCreated())
      ImGui::BeginDisabled();

    // --------------------------------------------------------------
    ImGui::Separator();
    ImGui::Text("Eigenvalues");

    ImGui::RadioButton("All Eigenvalues",
                       &ImGuiState::eigendecomposition_method, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Specific # of Eigenvalues",
                       &ImGuiState::eigendecomposition_method, 1);

    // --- Input Box
    if (ImGuiState::eigendecomposition_method == 0)
      ImGui::BeginDisabled();

    ImGui::InputInt("# of Eigenvalues to compute", &ImGuiState::eigs_number);

    if (ImGuiState::eigendecomposition_method == 0)
      ImGui::EndDisabled();
    // --- ! Input Box

    if (ImGui::Button("Button 4")) {
      // pl.IDW();
      ImGuiState::eigs = true;
    }

    ImGui::SameLine();
    (ImGuiState::ComputedEigs()) ? ImGui::Text("Success! Eigenvalues computed")
                                 : ImGui::Text("Click to compute eigenvalues");

    if (!ImGuiState::LaplacianCreated())
      ImGui::EndDisabled();

    if (!ImGuiState::ComputedEigs())
      ImGui::BeginDisabled();

    // --------------------------------------------------------------
    ImGui::Separator();
    ImGui::Text("Analysis");

    if (ImGui::Button("Button 5")) {
      ImGuiState::ad_test = true;
    }

    ImGui::SameLine();
    (ImGuiState::ADTestRun()) ? ImGui::Text("Success! AD Test finished")
                              : ImGui::Text("Click to run AD Test");

    if (ImGui::Button("Button 6")) {
      ImGuiState::ks_test = true;
    }

    ImGui::SameLine();
    (ImGuiState::KSTestRun()) ? ImGui::Text("Success! KS Test finished")
                              : ImGui::Text("Click to run KS Test");

    if (!ImGuiState::ComputedEigs())
      ImGui::EndDisabled();

    if (!ImGuiState::PointCloudsRead()) // || TODO viz is already open)
      ImGui::BeginDisabled();

    // --------------------------------------------------------------
    ImGui::Separator();
    ImGui::Text("Visualizer");

    if (ImGui::Button("Button 7")) {
      ImGuiState::pcl_viz = true;
      auto pointCloudPair = pl.GetPointCloudPair();

      std::thread t1(RunVizThread, pointCloudPair.first, pointCloudPair.second);
      t1.detach();
    }

    // pcl::visualization::PCLVisualizer::Ptr viewer(
    //     new pcl::visualization::PCLVisualizer("3D Viewer"));

    // viewer->setBackgroundColor(0, 0, 0, 0);
    // viewer->setBackgroundColor(0, 0, 0, 1);
    // viewer->addPointCloud<pcl::PointXYZ>(pointCloudPair.first, "Cloud 1",
    // 0); viewer->addPointCloud<pcl::PointXYZ>(pointCloudPair.second, "Cloud
    // 2", 1); viewer->setPointCloudRenderingProperties(
    //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 1", 0);
    // viewer->setPointCloudRenderingProperties(
    //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 2", 1);
    // viewer->addCoordinateSystem(1.0, "Cloud 1", 0);
    // viewer->addCoordinateSystem(1.0, "Cloud 2", 1);
    // viewer->initCameraParameters();

    ImGui::SameLine();
    (ImGuiState::PCLViz()) ? ImGui::Text("Success! PCL visualizer is running")
                           : ImGui::Text("Click to run PCL visualizer");

    if (!ImGuiState::PointCloudsRead())
      ImGui::EndDisabled();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

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

  // ros::init(argc, argv, "sgpr_ros_node");
  // ros::NodeHandle n;

  // int dataset;
  // ros::param::get("dataset", dataset);

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
