#pragma once

#include "Types/GraphLaplacian.h"
#include "Types/Scene.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace Processing {
namespace Laplacian {

inline void setSmallestDistance(SpectralObject &spectral_object) {
  double min = 1000000.0;
  for (int i = 0; i < spectral_object.cloud->size(); i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    spectral_object.kdTree.nearestKSearch(i, 2, indicies_found,
                                          squaredDistances);
    min = (std::sqrt(squaredDistances[1]) < min) ? sqrt(squaredDistances[1])
                                                 : min;
  }

  spectral_object.smallest_distance = min;
}

// NOTE: This is a utility function that allows us to check that the laplacians
// rows and cols add to 0. It is super slow so only use it as a one off check
inline void checkLaplacian(SpectralObject &spectral_object) {
  // Check laplacin Rows
  for (int i = 0; i < spectral_object.laplacian.n_rows; i++) {
    int sum = 0;
    for (int j = 0; j < spectral_object.laplacian.n_cols; j++) {
      sum += spectral_object.laplacian(i, j);
    }

    if (sum != 0) {
      std::cout << "ERROR, rows laplacian sum is not 0" << std::endl;
      exit(1);
    }
  }

  // Check laplacin Cols
  for (int i = 0; i < spectral_object.laplacian.n_cols; i++) {
    int sum = 0;

    for (int j = 0; j < spectral_object.laplacian.n_rows; j++) {
      sum += spectral_object.laplacian(j, i);
    }

    if (sum != 0) {
      std::cout << "ERROR, cols laplacian sum is not 0" << std::endl;
      exit(1);
    }
  }
}

// TODO this is not the actual generic laplacian. When we create the
// normalized
// IDW laplacian we need to change this back
inline void genericLaplacian(SpectralObject &spectral_object) {

  unsigned int max_nn = 1000;

  int size = spectral_object.cloud->size();
  spectral_object.laplacian = arma::sp_mat(size, size);

  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    spectral_object.kdTree.radiusSearch(i, spectral_object.mcar, indicies_found,
                                        squaredDistances, max_nn);

    int num_edges = indicies_found.size() - 1;
    if (num_edges == 0) {
      std::cout << "ERROR, Graph is Disconnected" << std::endl;
      std::exit(1);
    }

    spectral_object.laplacian(i, i) = num_edges;

    for (int j = 0; j < indicies_found.size(); j++) {
      if (indicies_found[j] == i)
        continue;
      spectral_object.laplacian(i, indicies_found[j]) = -1;
      spectral_object.laplacian(indicies_found[j], i) = -1;
    }
  }

  // Call checkLaplacian(spectral_object) to verify the laplacian is correct
}

inline void normalizedLaplacian(SpectralObject &so) {

  std::cout << "NORMALIZED" << std::endl;
  unsigned int max_nn = 1000;

  int size = so.cloud->size();
  so.laplacian = arma::sp_mat(size, size);

  // Make degree matrix first
  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    so.kdTree.radiusSearch(i, so.mcar, indicies_found, squaredDistances,
                           max_nn);

    int num_edges = indicies_found.size() - 1;
    if (num_edges == 0) {
      std::cout << "ERROR, Graph is Disconnected" << std::endl;
      std::exit(1);
    }

    so.laplacian(i, i) = num_edges;

    // for (int j = 0; j < indicies_found.size(); j++) {
    //   if (indicies_found[j] == i)
    //     continue;
    //   so.laplacian(i, indicies_found[j]) = -1;
    //   so.laplacian(indicies_found[j], i) = -1;
    // }
  }

  // Make adjecency matrix next
  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    so.kdTree.radiusSearch(i, so.mcar, indicies_found, squaredDistances,
                           max_nn);

    for (int j = 0; j < indicies_found.size(); j++) {
      if (indicies_found[j] == i)
        continue;
      so.laplacian(i, indicies_found[j]) =
          -1 / std::sqrt(so.laplacian(i, i) * so.laplacian(j, j));
      so.laplacian(indicies_found[j], i) =
          -1 / std::sqrt(so.laplacian(i, i) * so.laplacian(j, j));
    }
  }

  // Set diagonal to 1
  for (int i = 0; i < size; i++) {
    std::vector<int> indicies_found;
    std::vector<float> squaredDistances;
    so.kdTree.radiusSearch(i, so.mcar, indicies_found, squaredDistances,
                           max_nn);


    so.laplacian(i, i) = 1;
  }
}

inline void IDWLaplacian(SpectralObject &spectral_object) {

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
    if (num_edges == 0) {
      std::cout << "ERROR, Graph is Disconnected" << std::endl;
      std::exit(1);
    }

    spectral_object.laplacian(i, i) = num_edges;

    double norm = 0.0f;
    for (int j = 1; j < indicies_found.size(); j++) {
      double val = 1 / (sqrt(squaredDistances[j]) + bias);
      //norm += val;
      spectral_object.laplacian(i, indicies_found[j]) = -1 * val;
      spectral_object.laplacian(indicies_found[j], i) = -1 * val;
    }

    //spectral_object.laplacian(i, i) /= norm; // Normalize the degree value
  }
}

// inline void idwCompute(SpectralObject &spectral_object) {
//
//   setSmallestDistance(spectral_object);
//   double bias = 1.0 - spectral_object.smallest_distance;
//
//   unsigned int max_nn = 1000;
//
//   int size = spectral_object.cloud->size();
//   spectral_object.laplacian = arma::sp_mat(size, size);
//
//   for (int i = 0; i < size; i++) {
//     std::vector<int> indicies_found;
//     std::vector<float> squaredDistances;
//     spectral_object.kdTree.radiusSearch(i, spectral_object.mcar,
//     indicies_found,
//                                         squaredDistances, max_nn);
//
//     int num_edges = indicies_found.size() - 1;
//     spectral_object.laplacian(i, i) = num_edges;
//
//     for (int j = 1; j < indicies_found.size(); j++) {
//       spectral_object.laplacian(i, indicies_found[j]) =
//           -1 / (sqrt(squaredDistances[j]) + bias);
//       spectral_object.laplacian(indicies_found[j], i) =
//           -1 / (sqrt(squaredDistances[j]) + bias);
//     }
//   }
// }

inline void genLaplacian(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &genericLaplacian);
}

inline void normLaplacian(Scene &scene) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                &normalizedLaplacian);
}

}; // namespace Laplacian
}; // namespace Processing
