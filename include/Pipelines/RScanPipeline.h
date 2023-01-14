#pragma once

#include "Pipelines/Pipeline.h"
#include <string>
#include <vector>

class RScanPipeline : public Pipeline {
public:
  RScanPipeline () = default;
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
  const char* GetMappedRefScan(std::vector<std::string> &query_scans, int query_scan_idx) override;
};
