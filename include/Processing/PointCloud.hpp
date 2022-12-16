#ifndef POINT_CLOUD
#define POINT_CLOUD

#include <Scene.h>
#include <Utils/Visualization.hpp>

namespace Processing {
namespace PontCloud {

void findObjectPointCloud(SpectralObject &spectral_object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        uint8_t r = (uint8_t) strtol(spectral_object.ply_color.substr(0,2).c_str(), nullptr, 16);
        uint8_t g = (uint8_t) strtol(spectral_object.ply_color.substr(2,2).c_str(), nullptr, 16);
        uint8_t b = (uint8_t) strtol(spectral_object.ply_color.substr(4,2).c_str(), nullptr, 16);

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
        range_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::EQ, r)));
        range_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::EQ, g)));
        range_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::EQ, b)));

        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.filter(*cloud_filtered);

        // TODO Filter after Testing here!!!

        spectral_object = cloud_filtered;
        Visualization::VisualizeCloud(cloud_filtered);
}

void ExtractObjectPointClouds(Scene &scene){
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPLYFile(scene.ply_file.path, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);

    std::for_each(
          scene.spectral_objects.begin(),
          scene.spectral_objects.end(),
          [&](SpectralObject &so) {
            findObjectPointCloud(so, cloud);
          });
}

}; // !PointCloud
}; // !Processing

#endif // !POINT_CLOUD
