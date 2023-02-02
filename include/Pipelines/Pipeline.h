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

#include <sgpr_ros/Eigenvalues.h>

#include <matplot/matplot.h>

using namespace matplot;

class Pipeline {
public:
  virtual ~Pipeline(){};

  virtual void ParseDataset() = 0;
  virtual void ExtractObjectPointClouds(int max_pts) = 0;
  virtual void ComputeSOR(int meanK, double stdThresh) = 0;
  virtual void ComputeFPS(int filtering_opts, int sample_size,
                          double percent) = 0;
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
  virtual void GetQueryRefCloudObjPair(std::string query_scan,
                                       std::string ref_scan, int q_idx,
                                       int r_idx, SpectralObject &q_so,
                                       SpectralObject &r_so) = 0;
  virtual double GetRadius(std::string scan, int obj_idx) = 0;

  virtual void PlotHistograms(std::string ref_scan, std::string query_scan,
                              int query_obj_idx, int ref_obj_idx) = 0;
  virtual void GetEigs(sgpr_ros::Eigenvalues &eig_srv, std::string query_scan,
                       int query_obj_idx, std::string ref_scan,
                       int ref_obj_idx) = 0;
  virtual int GetSize(int filtering_opts, int sample_size,
                      double filter_percent, int q_size, int r_size) = 0;
  virtual void Laplacian(int laplacian_type, SpectralObject &so) = 0;

  scene_map_t mSceneMap;

protected:
  spKeyFrameDB mKeyFrameDB;

  std::pair<std::string, std::string> mQRScanPair;
  std::pair<int, int> mQRObjIdx;
};
