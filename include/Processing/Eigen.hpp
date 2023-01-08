#pragma once

#include "Scene.h"
#include "Types/GraphLaplacian.h"
#include <memory>

namespace Processing {
namespace Eigen {
namespace NMT {

inline void computeEigenValues(std::shared_ptr<GraphLaplacian> graphLaplacian, int eigs_num){

  arma::mat eigvec;
  int eig_n =
      (eigs_num == -1) ? graphLaplacian->laplacian.n_cols - 1 : eigs_num;

  if (eig_n > graphLaplacian->kdTree.getInputCloud()->size() - 1) {
    std::cout << "Skipping" << std::endl;
    return;
  }

  auto start = std::chrono::steady_clock::now();
  arma::eigs_sym(graphLaplacian->eigenvalues, eigvec, graphLaplacian->laplacian, eig_n);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Elapsed time in miliseconds: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << " ms" << std::endl;
}

} // namespace NMT

namespace DatasetTesting {
inline void eigendecompose(SpectralObject &spectral_object, int max_eigs) {
  arma::vec eigval;
  arma::mat eigvec;

  int eig_n =
      (max_eigs == -1) ? spectral_object.laplacian.n_cols - 1 : max_eigs;

  if (max_eigs >= spectral_object.cloud->size() - 1) {
    std::cout << "Skipping" << std::endl;
    return;
  }

  auto start = std::chrono::steady_clock::now();
  arma::eigs_sym(eigval, eigvec, spectral_object.laplacian, eig_n);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Elapsed time in miliseconds: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << " ms" << std::endl;

  spectral_object.eigenvalues = eigval;
}

inline void Eigendecomposition(Scene &scene, int max_eigs) {
  std::for_each(scene.spectral_objects.begin(), scene.spectral_objects.end(),
                [&](SpectralObject &so) { eigendecompose(so, max_eigs); });
}

}; // namespace DatasetTesting
}; // namespace Eigen
}; // namespace Processing
