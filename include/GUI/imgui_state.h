#include "imgui.h"
namespace ImGuiState {

inline const char *datasets[] = {"3RScan", "Matterport3D", "SemanticKitti"};
inline const char *pointclouds[] = {"/home/nate/Datasets/teddyPly/b1.ply", "/home/nate/Datasets/teddyPly/b2.ply"};

inline static int dataset_idx = 0;
inline static int point_cloud_idx1= 0;
inline static int point_cloud_idx2= 1;
inline static bool dataset_parsed = false;
inline static bool obj_pc= false;
inline static bool mcar = false;
inline static bool idw = false;
inline static bool eigs = false;

inline static bool IsDatasetParsed() { return dataset_parsed; }
inline static bool ExtractedObjectPointClouds() { return obj_pc; }
inline static bool ComputedMCAR() { return mcar; }
inline static bool ComputedIDW() { return idw; }
inline static bool ComputedEigs() { return eigs; }

// NOTE: Need to implement later when changing between datasets
//inline static bool DidDatasetChange() { return prev_dataset_idx == dataset_idx; }
}
