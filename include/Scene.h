#ifndef SCENE
#define SCENE

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <armadillo>

struct SpectralObject {
  int global_id;
  int scene_id;
  std::string label;
  std::string ply_color; // Consider deleting this not sure if useful
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  double radius;
  arma::sp_mat laplacian;
  arma::vec eigenvalues;
};

/*! \struct Scene
 *  \brief Struct containing scene info 
 *
 *  Struct containing scene info
 */

struct Scene {
  std::string scan_id;
  std::string ply_file_path;
  bool is_reference;
  std::string reference_id_match;
  std::vector<SpectralObject> spectral_objects;
};

using scene_map_t = std::unordered_map<std::string, Scene>;

#endif // !SCENE
