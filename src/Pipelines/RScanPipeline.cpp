#include "Pipelines/RScanPipeline.h"

// NOTE: for gdb debugging
std::string make_string(const char *x) { return x; }

void RScanPipeline::ParseDataset() {
  std::cout << "Parsing Dataset" << std::endl;
  std::string dataset_dir, config_dir, rscan_json, config_json, objects_json;

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

    // For each reference scan we need to find all the query scans and populate
    // the queryscan map
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
                  Processing::PointCloud::ExtractObjectPointClouds(pair.second, max_pts);
                });
  std::cout << "Finished ExtractObjectPointClouds" << std::endl;
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
                    Processing::Laplacian::IDWLaplacian(pair.second);
                  });
    break;
  case 1: // Normalized
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
