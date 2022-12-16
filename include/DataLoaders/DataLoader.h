#ifndef DATA_LOADER
#define DATA_LOADER

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Scene.h>

#include <ros/ros.h>

#include <nlohmann/json.hpp>


/*! \struct SpectralObject
 *  \brief Struct containing the spectral object info
 *
 *  Struct containing the spectral object info
 */


/*! \class DataLoader
 *  \brief Abstract class for data set loading
 *
 *  Abstract class for data set loading
 */

class DataLoader {
public:
  virtual ~DataLoader(){};

  virtual void ParseConfig(scene_map_t &scene_map) = 0;

protected:
};

using spDataLoader = std::shared_ptr<DataLoader>;
using json = nlohmann::json;

#endif // !DATA_LOADER
