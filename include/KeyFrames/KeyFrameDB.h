#ifndef KEYFRAME_DB
#define KEYFRAME_DB

#include <memory>

/*! \class KeyFrameDB
 *  \brief Keyframe database
 *
 *  Keyframe database
 */

class KeyFrameDB {
public:
  KeyFrameDB();
  ~KeyFrameDB() = default;
};

using spKeyFrameDB = std::shared_ptr<KeyFrameDB>;

#endif // !KEYFRAME_DB
