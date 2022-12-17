#ifndef VISUALIZATION
#define VISUALIZATION

#include <pcl/visualization/cloud_viewer.h>

namespace Visualization {

inline void VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud);
  std::cout << '\n' << "Press Enter";
  while (std::cin.get() != '\n') {
  }
}

} // namespace Visualization

#endif // !VISUALIZATION
