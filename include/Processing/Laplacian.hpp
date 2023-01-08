#pragma once

#include "Scene.h"
#include "Types/GraphLaplacian.h"
#include <memory>

namespace Processing {
namespace Laplacian {
namespace NMT {

inline void
setSmallestDistance(std::shared_ptr<GraphLaplacian> graphLaplacian) {
  double min = 1000000.0;
  for (int i = 0; i < graphLaplacian->kdTree.getInputCloud()->size(); i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    graphLaplacian->kdTree.nearestKSearch(i, 2, indicies_found,
                                          squaredDistances);
    min = (squaredDistances[1] < min) ? sqrt(squaredDistances[1]) : min;
  }

  graphLaplacian->smallestDistance = min;
}

inline void genericLaplacian(std::shared_ptr<GraphLaplacian> graphLaplacian) {

  setSmallestDistance(graphLaplacian);
  double bias = 1.0 - graphLaplacian->smallestDistance;

  unsigned int max_nn = 1000;

  int size = graphLaplacian->kdTree.getInputCloud()->size();
  graphLaplacian->laplacian = arma::sp_mat(size, size);

  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    graphLaplacian->kdTree.radiusSearch(
        i, graphLaplacian->radius, indicies_found, squaredDistances, max_nn);

    int num_edges = indicies_found.size() - 1;
    graphLaplacian->laplacian(i, i) = num_edges;

    for (int j = 1; j < indicies_found.size(); j++) {
      graphLaplacian->laplacian(i, indicies_found[j]) =
          -1 / (sqrt(squaredDistances[j]) + bias);
      graphLaplacian->laplacian(indicies_found[j], i) =
          -1 / (sqrt(squaredDistances[j]) + bias);
    }
  }
}

}; // namespace NMT

namespace DatasetTesting {

inline void setSmallestDistance(SpectralObject &spectral_object) {
  double min = 1000000.0;
  for (int i = 0; i < spectral_object.cloud->size(); i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    spectral_object.kdTree.nearestKSearch(i, 2, indicies_found,
                                          squaredDistances);
    min = (squaredDistances[1] < min) ? sqrt(squaredDistances[1]) : min;
  }

  spectral_object.smallest_distance = min;
}

inline void idwCompute(SpectralObject &spectral_object) {

  setSmallestDistance(spectral_object);
  double bias = 1.0 - spectral_object.smallest_distance;

  unsigned int max_nn = 1000;

  int size = spectral_object.cloud->size();
  spectral_object.laplacian = arma::sp_mat(size, size);

  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    spectral_object.kdTree.radiusSearch(i, spectral_object.mcar, indicies_found,
                                        squaredDistances, max_nn);

    int num_edges = indicies_found.size() - 1;
    spectral_object.laplacian(i, i) = num_edges;

    for (int j = 1; j < indicies_found.size(); j++) {
      spectral_object.laplacian(i, indicies_found[j]) =
          -1 / (sqrt(squaredDistances[j]) + bias);
      spectral_object.laplacian(indicies_found[j], i) =
          -1 / (sqrt(squaredDistances[j]) + bias);
    }
  }
}

inline void IDWLaplacian(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &idwCompute);
}

}; // namespace DatasetTesting
}; // namespace Laplacian
}; // namespace Processing
