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

// Indexes for drop down arrays
inline static int dataset_idx = 0;
inline static int point_cloud_idx1 = 0;
inline static int point_cloud_idx2 = 1;
inline static int edge_heuristic_idx= 1;
inline static int laplacian_idx= 1;

// Radio indexes
inline static int eigendecomposition_method = 0;

// Input box values
inline static int eigs_number = 100;
inline static double radius_1 = 0.0f;
inline static double radius_2 = 0.0f;

// GUI state variables
inline static bool dataset_parsed = false;
inline static bool point_clouds_read = false;
inline static bool edges_created = false;
inline static bool laplacian_created = false;
inline static bool obj_pc = false;
inline static bool mcar = false;
inline static bool idw = false;
inline static bool eigs = false;
inline static bool ad_test = false;
inline static bool ks_test = false;
inline static bool pcl_viz = false;

inline static bool IsDatasetParsed() { return dataset_parsed; }
inline static bool PointCloudsRead() { return point_clouds_read; }
inline static bool EdgesCreated() { return edges_created; }
inline static bool LaplacianCreated() { return laplacian_created; }
inline static bool ExtractedObjectPointClouds() { return obj_pc; }
inline static bool ComputedMCAR() { return mcar; }
inline static bool ComputedIDW() { return idw; }
inline static bool ComputedEigs() { return eigs; }
inline static bool ADTestRun() { return ad_test; }
inline static bool KSTestRun() { return ks_test; }
inline static bool PCLViz() { return pcl_viz; }
inline static bool ShowRadius() { return edge_heuristic_idx == 1 && edges_created; }

inline static std::string GetPlyFileName(int idx) {
  return &(*pointclouds[idx]);
}

// NOTE: Need to implement later when changing between datasets
// inline static bool DidDatasetChange() { return prev_dataset_idx ==
// dataset_idx; }
} // namespace ImGuiState
