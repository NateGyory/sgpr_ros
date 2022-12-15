#ifndef DATA_LOADER
#define DATA_LOADER

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

#include <nlohmann/json.hpp>

#include <armadillo>

/*! \struct SpectralObject
 *  \brief Struct containing the spectral object info
 *
 *  Struct containing the spectral object info
 */

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

/*! \class DataLoader
 *  \brief Abstract class for data set loading
 *
 *  Abstract class for data set loading
 */

class DataLoader {
public:
  virtual ~DataLoader(){};

  virtual void ParseConfig() = 0;

protected:
  std::unordered_map<std::string, Scene> mSceneMap;
};

using spDataLoader = std::shared_ptr<DataLoader>;
using json = nlohmann::json;

#endif // !DATA_LOADER
