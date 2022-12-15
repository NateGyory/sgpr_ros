#ifndef RSCAN_DATA_LOADER
#define RSCAN_DATA_LOADER

#include <DataLoaders/DataLoader.h>
#include <iostream>

/*! \class 3RScanDataLoader
 *  \brief Data loader for 3RScan datset
 *
 *  Data loader for 3RScan dataset
 */
class RScanDataLoader: public DataLoader
{
public:
  RScanDataLoader();
  ~RScanDataLoader() = default;

  void GetSemanticCloudMap() override;
};

#endif // !RSCAN_DATA_LOADER
