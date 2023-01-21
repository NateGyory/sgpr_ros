#pragma once

#include "sgpr_ros/Eigenvalues.h"
#include "imgui.h"
#include <pcl/point_cloud.h>
#include <string>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
namespace ImGuiState {

// Shared
inline const char *pointclouds[] = {"/home/nate/Datasets/teddyPly/b1.ply",
                                    "/home/nate/Datasets/teddyPly/b2.ply"};
inline const char *datasets[] = {"3RScan", "Matterport3D", "SemanticKitti"};
inline const char *edge_heuristics[] = {"Knn", "MCAR", "Fully Connected"};
inline const char *laplacians[] = {"Generic", "Normalized IDW"};

namespace NMT {

// Indexes for drop down arrays
inline static int point_cloud_idx1 = 0;
inline static int point_cloud_idx2 = 1;
inline static int edge_heuristic_idx = 1;
inline static int laplacian_idx = 0;

// Input string boxes
inline static char eigenvalue_json_f[64] = "NMT_eigs.json";

// Radio indexes
inline static int eigendecomposition_method = 1;

// Input box values
inline static int eigs_number = 100;
inline static int max_pts = 300;

// GUI state variables
inline static bool point_clouds_read = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool eigs = false;
inline static bool saved_eigs = false;
inline static bool pcl_viz = false;
inline static bool pcl_viz_connection = false;
inline static bool matplot = false;

inline static bool PointCloudsRead() { return point_clouds_read; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ComputedEigs() { return eigs; }
inline static bool SavedEigs() { return saved_eigs; }
inline static bool PCLViz() { return pcl_viz; }
inline static bool PCLConnectionViz() { return pcl_viz; }
inline static bool MatplotViz() { return matplot; }

inline static std::string GetPlyFileName(int idx) {
  return &(*pointclouds[idx]);
}
inline static bool ShowRadius() {
  return edge_heuristic_idx == 1 && edges_created;
}
}; // namespace NMT

namespace DatasetTesting {

inline static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
inline static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;
inline static sgpr_ros::Eigenvalues eig_srv;


inline static std::vector<std::string> query_scans;
inline static std::vector<std::string> ref_scans;
inline static std::vector<int> query_obj_scene_ids;

// Indexes for drop down arrays
inline static int dataset_idx = 0;
inline static int edge_heuristic_idx = 1;
inline static int laplacian_idx = 0;
inline static int query_scan_idx = 0;
inline static int last_query_scan_idx = 0;
inline static int ref_scan_idx = 0;

// Input string boxes
inline static char eigenvalue_json_f[64] = "DT_eigs.json";

// Radio indexes
inline static int eigendecomposition_method = 1;
inline static int should_step = 0;
inline static int obj_compare_method = 0;

// Checkboxes
inline static bool c_visualize_point_clouds = true;
inline static bool c_visualize_graph_connections = true;
inline static bool c_visualize_spectra = true;
inline static bool c_anderson_darling_test = true;
inline static bool c_ks_test = true;

// Input box values
inline static int eigs_number = 100;
inline static int max_pts = 300;

// GUI state variables
inline static bool dataset_parsed = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool eigs = false;
inline static bool saved_eigs = false;
inline static bool ref_obj_exists = true;
// inline static bool pcl_viz = false; Not yet implemented
// inline static bool pcl_viz_connection = false;
// inline static bool matplot = false;

// query and ref obj idx for current obj comparing in the scene
inline static int query_obj_idx = 0;
inline static int ref_obj_idx = 0;

inline static bool DatasetParsed() { return dataset_parsed; }
inline static bool ShouldStep() { return should_step == 0; }
inline static bool GetQueryScans() {
  return query_scans.size() == 0 && DatasetParsed();
}
inline static bool GetRefScans() {
  return ref_scans.size() == 0 && DatasetParsed();
}
inline static bool UpdateRefScanSelected() {
  return query_scan_idx != last_query_scan_idx;
}
inline static const char *GetSelectedQueryScan() {
  return (query_scans.size() > 0) ? query_scans[query_scan_idx].c_str() : "";
}

inline static bool ReadyToStep() { return query_obj_scene_ids.size() > 0 && query_obj_idx < query_obj_scene_ids.size(); }
inline static bool RefObjExists() { return ref_obj_exists; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ComputedEigs() { return eigs; }
inline static bool SavedEigs() { return saved_eigs; }
// inline static bool PCLViz() { return pcl_viz; }
// inline static bool PCLConnectionViz() { return pcl_viz; }
// inline static bool MatplotViz() { return matplot; }

}; // namespace DatasetTesting
} // namespace ImGuiState
