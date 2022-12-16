
#ifndef VISUALIZATION
#define VISUALIZATION

namespace Visualization {

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    std::cout << '\n' << "Press Enter";
    while (std::cin.get() != '\n') {}
}

} // !Visualization

#endif // !VISUALIZATION

