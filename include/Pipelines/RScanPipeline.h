#pragma once

#include "Pipelines/Pipeline.h"
#include <string>

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
};
