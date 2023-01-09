#pragma once

#include "imgui.h"
#include <string>
namespace ImGuiState {

// Drop down arrays
inline const char *datasets[] = {"3RScan", "Matterport3D", "SemanticKitti"};
inline const char *pointclouds[] = {"/home/nate/Datasets/teddyPly/b1.ply",
                                    "/home/nate/Datasets/teddyPly/b2.ply"};
inline const char *edge_heuristics[] = {"Knn", "MCAR", "Fully Connected"};
inline const char *laplacians[] = {"Generic", "Normalized IDW"};

// Input string boxes
inline static char eigenvalue_json_f[64] = "eigs.json";

// Indexes for drop down arrays
inline static int dataset_idx = 0;
inline static int point_cloud_idx1 = 0;
inline static int point_cloud_idx2 = 1;
inline static int edge_heuristic_idx= 1;
inline static int laplacian_idx= 0;

// Radio indexes
inline static int eigendecomposition_method = 1;

// Input box values
inline static int eigs_number = 100;
inline static int max_pts = 300;

// GUI state variables
inline static bool dataset_parsed = false;
inline static bool point_clouds_read = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool obj_pc = false;
inline static bool mcar = false;
inline static bool idw = false;
inline static bool eigs = false;
inline static bool saved_eigs = false;
inline static bool pcl_viz = false;
inline static bool pcl_viz_connection = false;
inline static bool matplot = false;

inline static bool IsDatasetParsed() { return dataset_parsed; }
inline static bool PointCloudsRead() { return point_clouds_read; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ExtractedObjectPointClouds() { return obj_pc; }
inline static bool ComputedMCAR() { return mcar; }
inline static bool ComputedIDW() { return idw; }
inline static bool ComputedEigs() { return eigs; }
inline static bool SavedEigs() { return saved_eigs; }
inline static bool PCLViz() { return pcl_viz; }
inline static bool PCLConnectionViz() { return pcl_viz; }
inline static bool ShowRadius() { return edge_heuristic_idx == 1 && edges_created; }
inline static bool MatplotViz() { return matplot; }

inline static std::string GetPlyFileName(int idx) {
  return &(*pointclouds[idx]);
}

// NOTE: Need to implement later when changing between datasets
// inline static bool DidDatasetChange() { return prev_dataset_idx ==
// dataset_idx; }
} // namespace ImGuiState
