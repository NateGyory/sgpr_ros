#ifndef MATTERPORT_3D
#define MATTERPORT_3D

#include <DataLoaders/DataLoader.h>

/*! \class MATTERPORT_3D
 *  \brief Data loader for Matterport3D datset
 *
 *  Data loader for Matterport3D dataset
 */
class Matterport3D : public DataLoader {
public:
  Matterport3D() = default;
  ~Matterport3D() = default;

  void ParseConfig() override{};
};

#endif // !MATTERPORT_3D
