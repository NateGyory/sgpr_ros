#pragma once

#include "Pipelines/Pipeline.h"
#include <string>
#include <vector>

class RScanPipeline : public Pipeline {
public:
  RScanPipeline() = default;
  ~RScanPipeline() = default;

  void ParseDataset() override;
  void ExtractObjectPointClouds(int max_pts) override;
  void ComputeEdges(int edge_heuristic) override;
  void ComputeLaplacian(int laplacian_type) override;
  void ComputeEigs(int max_eigs) override;
  void SaveEigenvalues(std::string file_name) override;
  // Todo considere moving this into Pipeline.h after adding new datasets
  void GetQueryScans(std::vector<std::string> &query_scans) override;
  void GetRefScans(std::vector<std::string> &ref_scans) override;
  const char *GetMappedRefScan(std::vector<std::string> &query_scans,
                               int query_scan_idx) override;
  void GetQuerySpectralObjIds(std::vector<int> &query_obj_scene_ids,
                              std::string query_scan) override;
  bool RefObjExists(std::string query_scan, int query_obj_idx,
                    int &ref_obj_idx) override;
  void GetQueryRefCloudObjPair(
      std::string query_scan, std::string ref_scan, int q_idx, int r_idx,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2) override;
  double GetRadius(std::string scan, int obj_idx) override;
  void PlotHistograms(std::string ref_scan, std::string query_scan,
                      int query_obj_idx, int ref_obj_idx) override;
  void GetEigs(sgpr_ros::Eigenvalues &eig_srv, std::string query_scan,
               int query_obj_idx, std::string ref_scan,
               int ref_obj_idx) override;
};
