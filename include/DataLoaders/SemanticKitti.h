#ifndef SEMANTIC_KITTI
#define SEMANTIC_KITTI

#include <DataLoaders/DataLoader.h>

/*! \class MATTERPORT_3D
 *  \brief Data loader for SemanticKitti datset
 *
 *  Data loader for SemanticKitti dataset
 */
class SemanticKitti: public DataLoader
{
public:
  SemanticKitti();
  ~SemanticKitti() = default;

  void GetSemanticCloudMap() override;
};

#endif // !SEMANTIC_KITTI
