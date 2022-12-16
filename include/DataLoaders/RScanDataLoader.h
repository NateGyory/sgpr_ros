#ifndef RSCAN_DATA_LOADER
#define RSCAN_DATA_LOADER

#include <DataLoaders/DataLoader.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

/*! \class 3RScanDataLoader
 *  \brief Data loader for 3RScan datset
 *
 *  Data loader for 3RScan dataset
 */

class RScanDataLoader : public DataLoader {
public:
  RScanDataLoader() = default; 
  ~RScanDataLoader() = default;

  void ParseConfig(scene_map_t &scene_map) override;
};

#endif // !RSCAN_DATA_LOADER
