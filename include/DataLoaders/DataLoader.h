#ifndef DATA_LOADER
#define DATA_LOADER

#include <memory>
#include <string>

/*! \class DataLoader
 *  \brief Abstract class for data set loading
 *
 *  Abstract class for data set loading
 */

class DataLoader
{
public:
  virtual ~DataLoader() {};

  virtual void GetSemanticCloudMap() = 0;
};

using spDataLoader = std::shared_ptr<DataLoader>;

#endif // !DATA_LOADER
