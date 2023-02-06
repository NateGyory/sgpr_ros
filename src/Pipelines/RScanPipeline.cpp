#include "Pipelines/RScanPipeline.h"
#include "Processing/Laplacian.hpp"
#include "Processing/PointCloud.hpp"
#include <algorithm>
#include <pcl/common/io.h>
#include <string>
#include <vector>

// NOTE: for gdb debugging
std::string make_string(const char *x) { return x; }

void RScanPipeline::Laplacian(int laplacian_type, SpectralObject &so) {
  switch (laplacian_type) {
  case 0: // Generic
    Processing::Laplacian::genericLaplacian(so);
    break;
  case 1: // Normalized
    Processing::Laplacian::normalizedLaplacian(so);
    break;
  case 2: // IDW
    Processing::Laplacian::IDWLaplacian(so);
    break;
  case 3: // Geometric
    Processing::Laplacian::GeometricLaplacian(so);
    break;
  case 4: // Geometric_IDW
    Processing::Laplacian::GeometricIDWLaplacian(so);
    break;
  default:
    break;
  }
}

int RScanPipeline::GetSize(int filtering_opts, int sample_size,
                           double filter_percent, int q_size, int r_size) {

  int min_size = std::min(q_size, r_size);
  int ret = 0;
  switch (filtering_opts) {
  case 0:
    // TODO smallest cloud
    ret = min_size;
    break;
  case 1:
    // TODO percent
    ret = int(min_size * filter_percent);
    break;
  case 2:
    // TODO sample size
    ret = (min_size < sample_size) ? min_size : sample_size;
    break;
  default:
    break;
  }
  return ret;
}

void RScanPipeline::ParseDataset() {
  std::cout << "Parsing Dataset" << std::endl;
  std::string dataset_dir, config_dir, rscan_json, config_json, objects_json;
  mSceneMap.clear();

  //  ros::param::get("ros_pkg_dir", config_dir);
  //  ros::param::get("dataset_dir", dataset_dir);
  config_dir = "/home/nate/Development/catkin_ws/src/sgpr_ros";
  dataset_dir = "/home/nate/Datasets/3RScan";

  config_dir += "/config/3RScan";
  rscan_json = config_dir + "/" + "3RScan.json";
  config_json = config_dir + "/" + "config.json";
  objects_json = config_dir + "/" + "objects.json";

  std::ifstream configFile(config_json);
  std::ifstream objectFile(objects_json);
  std::ifstream refQueryMapFile(rscan_json);

  json configData = json::parse(configFile);
  json objectData = json::parse(objectFile);
  json refQueryMapData = json::parse(refQueryMapFile);

  std::string ply_file = configData["ply_filename"].get<std::string>();

  // TODO do this in parallel in future
  for (auto const &reference_scan : configData["reference_scans"]) {
    Scene ref_scene;
    ref_scene.scan_id = reference_scan["scan_id"].get<std::string>();
    ref_scene.ply_file_path =
        dataset_dir + "/" + ref_scene.scan_id + "/" + ply_file;
    ref_scene.is_reference = true;
    ref_scene.reference_id_match = "";

    // Get all the objects associated with the scan
    std::vector<json> objects;
    for (auto obj : objectData["scans"]) {
      if (obj["scan"] == ref_scene.scan_id) {
        objects = obj["objects"];
        break;
      }
    }

    // Fill map with the colors
    for (auto const obj : objects) {
      SpectralObject sp_object;
      sp_object.global_id = std::stoi(obj["global_id"].get<std::string>());
      sp_object.scene_id = std::stoi(obj["id"].get<std::string>());
      sp_object.label = obj["label"].get<std::string>();
      sp_object.ply_color = obj["ply_color"].get<std::string>();

      sp_object.ply_color.erase(0, 1);
      ref_scene.spectral_objects.push_back(sp_object);
    }

    mSceneMap[ref_scene.scan_id] = ref_scene;

    // For each reference scan we need to find all the query scans and
    // populate the queryscan map
    // TODO do this in parallel
    for (auto const &scan : refQueryMapData["scans"]) {
      if (scan["reference"] == ref_scene.scan_id) {
        std::vector<json> query_scans = scan["scans"];
        for (auto const &query_scan : query_scans) {
          Scene query_scene;
          query_scene.scan_id = query_scan["reference"].get<std::string>();
          query_scene.ply_file_path =
              dataset_dir + "/" + query_scene.scan_id + "/" + ply_file;
          query_scene.is_reference = false;
          query_scene.reference_id_match = ref_scene.scan_id;

          // Get all the objects associated with the scan
          std::vector<json> objects;
          for (auto obj : objectData["scans"]) {
            if (obj["scan"] == query_scene.scan_id) {
              objects = obj["objects"];
              break;
            }
          }

          for (auto const obj : objects) {
            SpectralObject sp_object;
            sp_object.global_id =
                std::stoi(obj["global_id"].get<std::string>());
            sp_object.scene_id = std::stoi(obj["id"].get<std::string>());
            sp_object.label = obj["label"].get<std::string>();
            sp_object.ply_color = obj["ply_color"].get<std::string>();

            sp_object.ply_color.erase(0, 1);
            query_scene.spectral_objects.push_back(sp_object);
          }

          mSceneMap[query_scene.scan_id] = query_scene;
        }
      }
    }
  }
  std::cout << "Finished Parsing Dataset" << std::endl;
}

void RScanPipeline::ExtractObjectPointClouds(int max_pts) {
  std::cout << "ExtractObjectPointClouds" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [max_pts](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::ExtractObjectPointClouds(pair.second,
                                                                   max_pts);
                });
  std::cout << "Finished ExtractObjectPointClouds" << std::endl;

  std::cout << "Calculating GFA Features" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::CalculateGFAFeatures(pair.second);
                });
  std::cout << "Finished Calculating GFA Features" << std::endl;
}

void RScanPipeline::ComputeSOR(int meanK, double stdThresh) {
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [meanK, stdThresh](std::pair<const std::string, Scene> &pair) {
                  Processing::PointCloud::ComputeSceneSOR(pair.second, meanK,
                                                          stdThresh);
                });
}

void RScanPipeline::ComputeSceneFPS(Scene &scene, int filtering_opts,
                                    int sample_size, double percent) {
  // Do not process if it is a reference scene
  if (scene.is_reference)
    return;
  for (SpectralObject &so : scene.spectral_objects) {
    // Find the matching object in the reference_scan
    auto begin_it =
        mSceneMap[scene.reference_id_match].spectral_objects.begin();
    auto end_it = mSceneMap[scene.reference_id_match].spectral_objects.end();

    int scene_id = so.scene_id;

    auto it =
        std::find_if(begin_it, end_it, [scene_id](const SpectralObject &r_so) {
          return r_so.scene_id == scene_id;
        });

    int ref_obj_idx = it - begin_it;

    if (it != end_it) {
      // Now do check
      double size =
          GetSize(filtering_opts, sample_size, percent, so.cloud->size(),
                  mSceneMap[scene.reference_id_match]
                      .spectral_objects[ref_obj_idx]
                      .cloud->size());

      Processing::PointCloud::computeFPS(so, size);
      Processing::PointCloud::computeFPS(
          mSceneMap[scene.reference_id_match].spectral_objects[ref_obj_idx],
          size);

      if (so.cloud->size() != mSceneMap[scene.reference_id_match]
                                  .spectral_objects[ref_obj_idx]
                                  .cloud->size()) {
        std::cout << "ERROR clouds not same" << std::endl;
        exit(1);
      }
    }
  }
}

void RScanPipeline::ComputeFPS(int filtering_opts, int sample_size,
                               double percent) {
  // TODO
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [this, filtering_opts, sample_size,
                 percent](std::pair<const std::string, Scene> &pair) {
                  ComputeSceneFPS(pair.second, filtering_opts, sample_size,
                                  percent);
                });
}

void RScanPipeline::ComputeEdges(int edge_heuristic) {
  std::cout << "Computing edges" << std::endl;

  switch (edge_heuristic) {
  case 0: // Knn
    break;
  case 1: // MCAR
    std::for_each(mSceneMap.begin(), mSceneMap.end(),
                  [](std::pair<const std::string, Scene> &pair) {
                    Processing::PointCloud::MinimallyConnectedAdaptiveRadius(
                        pair.second);
                  });
    break;
  case 2: // Fully Connected
    break;
  default:
    break;
  }

  std::cout << "Finished computing edges" << std::endl;
}

void RScanPipeline::ComputeLaplacian(int laplacian_type) {
  std::cout << "Compute Laplcain" << std::endl;
  switch (laplacian_type) {
  case 0: // Generic
    std::for_each(mSceneMap.begin(), mSceneMap.end(),
                  [](std::pair<const std::string, Scene> &pair) {
                    Processing::Laplacian::genLaplacian(pair.second);
                  });
    break;
  case 1: // Normalized
    std::for_each(mSceneMap.begin(), mSceneMap.end(),
                  [](std::pair<const std::string, Scene> &pair) {
                    Processing::Laplacian::normLaplacian(pair.second);
                  });
    break;
  default:
    break;
  }
  std::cout << "Finished computing Laplacian" << std::endl;
}

void RScanPipeline::ComputeEigs(int max_eigs) {
  std::cout << "Computing eigenvalues" << std::endl;
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [max_eigs](std::pair<const std::string, Scene> &pair) {
                  Processing::Eigen::Eigendecomposition(pair.second, max_eigs);
                });
  std::cout << "Finished computing eigenvalues" << std::endl;
}

void RScanPipeline::SaveEigenvalues(std::string file_name) {
  std::string file_path =
      "/home/nate/Development/catkin_ws/src/sgpr_ros/data/" + file_name;

  json j;
  std::ofstream o(file_path);

  // {
  //   reference_scans: [{
  //     scan_id:
  //     objects: [{
  //       label:
  //       global_id:
  //       scene_id
  //       ply_color:
  //       eigenvalues:
  //     }]
  //   }],
  //   query_scans: [
  //     scan_id:
  //     reference_scan_id:
  //     objects: [{
  //       label:
  //       global_id:
  //       scene_id
  //       ply_color:
  //       eigenvalues:
  //     }]
  //   }],
  // }

  std::vector<json> reference_scans;
  std::vector<json> query_scans;
  for (auto &kv : mSceneMap) {
    json scene;
    std::vector<json> scene_objects;

    scene["scan_id"] = kv.first;

    for (auto &so : kv.second.spectral_objects) {
      json object;
      object["label"] = so.label;
      object["global_id"] = so.global_id;
      object["scene_id"] = so.scene_id;
      object["ply_color"] = so.ply_color;
      object["eigenvalues"] = so.eigenvalues;
      scene_objects.push_back(object);
    }

    scene["objects"] = scene_objects;

    if (kv.second.is_reference) {
      reference_scans.push_back(scene);
    } else {
      scene["reference_scan_id"] = kv.second.reference_id_match;
      query_scans.push_back(scene);
    }
  }

  j["reference_scans"] = reference_scans;
  j["query_scans"] = query_scans;

  o << std::setw(4) << j << std::endl;
}

void RScanPipeline::GetQueryScans(std::vector<std::string> &query_scans) {
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [&query_scans](std::pair<const std::string, Scene> &pair) {
                  if (!pair.second.is_reference)
                    query_scans.push_back(pair.first);
                });
}

void RScanPipeline::GetRefScans(std::vector<std::string> &ref_scans) {
  std::for_each(mSceneMap.begin(), mSceneMap.end(),
                [&ref_scans](std::pair<const std::string, Scene> &pair) {
                  if (pair.second.is_reference)
                    ref_scans.push_back(pair.first);
                });
}

const char *
RScanPipeline::GetMappedRefScan(std::vector<std::string> &query_scans,
                                int query_scan_idx) {
  return (query_scans.size() == 0) ? ""
                                   : mSceneMap[query_scans[query_scan_idx]]
                                         .reference_id_match.c_str();
}

void RScanPipeline::GetQuerySpectralObjIds(
    std::vector<int> &query_obj_scene_ids, std::string query_scan) {
  for (auto spectral_obj : mSceneMap[query_scan].spectral_objects) {
    query_obj_scene_ids.push_back(spectral_obj.scene_id);
  }
}

bool RScanPipeline::RefObjExists(std::string query_scan, int query_obj_idx,
                                 int &ref_obj_idx) {
  int scene_id = mSceneMap[query_scan].spectral_objects[query_obj_idx].scene_id;
  std::string reference_scan = mSceneMap[query_scan].reference_id_match;

  auto begin_it = mSceneMap[reference_scan].spectral_objects.begin();
  auto end_it = mSceneMap[reference_scan].spectral_objects.end();

  auto it =
      std::find_if(begin_it, end_it, [scene_id](const SpectralObject &so) {
        return so.scene_id == scene_id;
      });

  ref_obj_idx = it - begin_it;

  return it != end_it;
}

void RScanPipeline::GetQueryRefCloudObjPair(std::string query_scan,
                                            std::string ref_scan, int q_idx,
                                            int r_idx, SpectralObject &q_so,
                                            SpectralObject &r_so) {
  q_so = mSceneMap[query_scan].spectral_objects[q_idx];
  r_so = mSceneMap[ref_scan].spectral_objects[r_idx];
}

double RScanPipeline::GetRadius(std::string scan, int obj_idx) {
  return mSceneMap[scan].spectral_objects[obj_idx].mcar;
}

void RScanPipeline::PlotHistograms(std::string ref_scan, std::string query_scan,
                                   int query_obj_idx, int ref_obj_idx) {
  auto f = figure(true);
  f->width(f->width() * 3);
  f->height(f->height() * 2.5);
  f->x_position(10);
  f->y_position(10);

  std::vector<double> ref = arma::conv_to<std::vector<double>>::from(
      mSceneMap[ref_scan].spectral_objects[ref_obj_idx].eigenvalues);
  std::vector<double> query = arma::conv_to<std::vector<double>>::from(
      mSceneMap[query_scan].spectral_objects[query_obj_idx].eigenvalues);

  double min_ref = *std::min_element(ref.begin(), ref.end());
  double max_ref = *std::max_element(ref.begin(), ref.end());
  double min_query = *std::min_element(query.begin(), query.end());
  double max_query = *std::max_element(query.begin(), query.end());

  double min = std::min(min_ref, min_query);
  double max = std::max(max_ref, max_query);

  double bin_width = (max - min) / 25;

  auto h1 = hist(ref);
  h1->face_color("r");
  h1->edge_color("r");
  h1->bin_width(bin_width);
  hold(on);
  auto h2 = hist(query);
  h2->face_color("b");
  h2->edge_color("b");
  h2->bin_width(bin_width);
  title("Eigenvalue Spectras");
  f->draw();
  show();
}

void RScanPipeline::GetEigs(sgpr_ros::Eigenvalues &eig_srv,
                            std::string query_scan, int query_obj_idx,
                            std::string ref_scan, int ref_obj_idx) {
  eig_srv.request.q_eigs = arma::conv_to<std::vector<double>>::from(
      mSceneMap[query_scan].spectral_objects[query_obj_idx].eigenvalues);
  eig_srv.request.r_eigs = arma::conv_to<std::vector<double>>::from(
      mSceneMap[ref_scan].spectral_objects[ref_obj_idx].eigenvalues);
  eig_srv.request.q_gfa =
      mSceneMap[query_scan].spectral_objects[query_obj_idx].gfaFeatures;
  eig_srv.request.r_gfa =
      mSceneMap[ref_scan].spectral_objects[ref_obj_idx].gfaFeatures;
}

void RScanPipeline::SaveGFA() {
  // {
  //   query : [{
  //     scan_id:
  //     ref_match_id:
  //     global_ids: []
  //     scene_ids: []
  //     gfa_features: []
  //   }],
  //
  //   ref : [{
  //     scan_id:
  //     global_ids: []
  //     scene_ids: []
  //     gfa_features: []
  //   }]
  // }

  std::vector<json> query_list;
  std::vector<json> ref_list;
  for(auto const &kv: mSceneMap) {
    if(kv.second.is_reference) {
      json reference;
      reference["scan_id"] = kv.second.scan_id;
      std::vector<int> global_ids;
      std::vector<int> scene_ids;
      std::vector<std::vector<double>> gfa_features;
      for (auto const so : kv.second.spectral_objects) {
        global_ids.push_back(so.global_id);
        scene_ids.push_back(so.scene_id);
        gfa_features.push_back(so.gfaFeatures);
      }
      reference["global_ids"] = global_ids;
      reference["scene_ids"] = scene_ids;
      reference["gfa_features"] = gfa_features;
      ref_list.push_back(reference);
    } else {
      json query;
      query["scan_id"] = kv.second.scan_id;
      query["ref_match_id"] = kv.second.reference_id_match;
      std::vector<int> global_ids;
      std::vector<int> scene_ids;
      std::vector<std::vector<double>> gfa_features;
      for (auto const so : kv.second.spectral_objects) {
        global_ids.push_back(so.global_id);
        scene_ids.push_back(so.scene_id);
        gfa_features.push_back(so.gfaFeatures);
      }
      query["global_ids"] = global_ids;
      query["scene_ids"] = scene_ids;
      query["gfa_features"] = gfa_features;
      query_list.push_back(query);
    }
  }

  std::string file_path = "/home/nate/Development/catkin_ws/src/sgpr_ros/results/gfa/pr.json";

  std::ofstream o(file_path);

  json data;
  data["reference"] = ref_list; 
  data["query"] = query_list; 

  o << std::setw(4) << data << std::endl;
}
