#pragma once

#include "imgui.h"
#include <string>
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

// Indexes for drop down arrays
inline static int dataset_idx = 0;
inline static int edge_heuristic_idx = 1;
inline static int laplacian_idx = 0;

// Input string boxes
inline static char eigenvalue_json_f[64] = "DT_eigs.json";

// Radio indexes
inline static int eigendecomposition_method = 1;
inline static int should_step = 0;

// Input box values
inline static int eigs_number = 100;
inline static int max_pts = 300;

// GUI state variables
inline static bool dataset_parsed = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool eigs = false;
inline static bool saved_eigs = false;
//inline static bool pcl_viz = false; Not yet implemented
//inline static bool pcl_viz_connection = false;
//inline static bool matplot = false;

inline static bool DatasetParsed() { return dataset_parsed; }
inline static bool ShouldStep() { return should_step == 1; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ComputedEigs() { return eigs; }
inline static bool SavedEigs() { return saved_eigs; }
//inline static bool PCLViz() { return pcl_viz; }
//inline static bool PCLConnectionViz() { return pcl_viz; }
//inline static bool MatplotViz() { return matplot; }

}; // namespace DatasetTesting
} // namespace ImGuiState
