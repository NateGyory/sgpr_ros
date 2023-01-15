#pragma once

#include <algorithm>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <string>
#include <utility>
#include <vector>

#include "Types/Scene.h"

#include <KeyFrames/KeyFrameDB.h>

#include <Processing/Eigen.hpp>
#include <Processing/Laplacian.hpp>
#include <Processing/PointCloud.hpp>

#include <matplot/matplot.h>

using namespace matplot;

class Pipeline {
public:
  virtual ~Pipeline(){};

  virtual void ParseDataset() = 0;
  virtual void ExtractObjectPointClouds(int max_pts) = 0;
  virtual void ComputeEdges(int edge_heuristic) = 0;
  virtual void ComputeLaplacian(int laplacian_type) = 0;
  virtual void ComputeEigs(int max_eigs) = 0;
  virtual void SaveEigenvalues(std::string file_name) = 0;
  virtual void GetQueryScans(std::vector<std::string> &query_scans) = 0;
  virtual void GetRefScans(std::vector<std::string> &ref_scans) = 0;
  virtual const char *GetMappedRefScan(std::vector<std::string> &query_scans,
                                       int query_scan_idx) = 0;
  virtual void GetQuerySpectralObjIds(std::vector<int> &query_obj_scene_ids,
                                      std::string query_scan) = 0;
  virtual bool RefObjExists(std::string query_scan, int query_obj_idx,
                            int &ref_obj_idx) = 0;
  virtual void
  GetQueryRefCloudObjPair(std::string query_scan, std::string ref_scan,
                          int q_idx, int r_idx,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2) = 0;
  virtual double GetRadius(std::string scan, int obj_idx) = 0;

  virtual void PlotHistograms(std::string ref_scan, std::string query_scan,
                              int query_obj_idx, int ref_obj_idx) = 0;

protected:
  scene_map_t mSceneMap;
  spKeyFrameDB mKeyFrameDB;

  std::pair<std::string, std::string> mQRScanPair;
  std::pair<int, int> mQRObjIdx;
};