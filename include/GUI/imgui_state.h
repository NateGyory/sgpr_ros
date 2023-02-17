#pragma once

#include "Types/Scene.h"
#include "imgui.h"
#include "sgpr_ros/Eigenvalues.h"
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>
namespace ImGuiState {

inline const char *datasets[] = {"3RScan", "Matterport3D", "SemanticKitti"};
inline const char *edge_heuristics[] = {"Knn", "MCAR", "Fully Connected"};
inline const char *laplacians[] = {"Generic", "Normalized", "IDW", "Geometric", "Geometric_IDW"};
inline const char *filtering[] = {"FPS", "VOX", "URS"};

namespace DatasetTesting {

inline static sgpr_ros::Eigenvalues eig_srv;
inline static SpectralObject q_so;
inline static SpectralObject r_so;
inline static bool update_cloud = false;
inline static bool update_hist = false;
inline static bool show_radius = false;
inline static bool radius_toggled = false;
inline static std::mutex mtx, eigs_mtx;

inline static std::vector<std::string> query_scans;
inline static std::vector<std::string> ref_scans;
inline static std::vector<int> query_obj_scene_ids;
inline static std::string key1;
inline static std::string key2;

// Indexes for drop down arrays
inline static int dataset_idx = 0;
inline static int edge_heuristic_idx = 1;
inline static int laplacian_idx = 0;
inline static int filtering_idx = 0;

inline static int query_scan_idx = 0;
inline static int last_query_scan_idx = 0;
inline static int ref_scan_idx = 0;

// Input string boxes
inline static char eigenvalue_json_f[64] = "eigs.json";

// Radio indexes
inline static int eigendecomposition_method = 1;
inline static int should_step = 0;
inline static int obj_compare_method = 0;
inline static int filtering_opts = 0;
inline static int eval_opts = 0;

// Checkboxes
inline static bool sor_check = true;
inline static bool double_sor = false;
inline static bool same_radius = true;
inline static bool debug_eval = false;

// Input box values
inline static int eigs_number = 100;
inline static int max_pts = 300;
inline static int meanK = 3;
inline static int sample_size = 1000;
inline static double stdThresh = 3;
inline static double filter_percent = 0.75f;
inline static int last_scene = 50;
inline static int scan_buffer = 20;
inline static double match_thresh = -1.0;
inline static int sequence = 5;

// GUI state variables
inline static bool dataset_parsed = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool eigs = false;
inline static bool saved_eigs = false;
inline static bool ref_obj_exists = true;

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

inline static bool ReadyToStep() {
  return query_obj_scene_ids.size() > 0 &&
         query_obj_idx < query_obj_scene_ids.size();
}
inline static bool RefObjExists() { return ref_obj_exists; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ComputedEigs() { return eigs; }
inline static bool SavedEigs() { return saved_eigs; }
inline static bool FilterPercent() { return filtering_opts == 1; }
inline static bool SampleSize() { return filtering_opts == 2; }
inline static std::string GetLaplacianName() {
  std::string name = "";
  switch (laplacian_idx) {
  case 0:
    name = "Generic";
    break;
  case 1:
    name = "Normalized";
    break;
  case 2:
    name = "IDW";
    break;
  case 3:
    name = "Geometric";
    break;
  case 4:
    name = "Geometric_IDW";
    break;
  default:
    break;
  }
  return name;
}

}; // namespace DatasetTesting
} // namespace ImGuiState
