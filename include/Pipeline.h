#ifndef PIPELINE
#define PIPELINE

#include <memory>
#include <string>

#include "DataLoaders/DataLoader.h"
#include "DataLoaders/Matterport3D.h"
#include "DataLoaders/RScanDataLoader.h"
#include "DataLoaders/SemanticKitti.h"

#include <KeyFrames/KeyFrameDB.h>

class Pipeline {
public:
  Pipeline() = default;
  ~Pipeline() = default;

  void PostProcess(int dataset);
  void RealTime(){};

private:
  void initDataLoader(int dataset);

  spDataLoader mDataLoader;
  spKeyFrameDB mKeyFrameDB;
};

#endif // !PIPELINE
