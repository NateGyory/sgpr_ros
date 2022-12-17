#include "DataLoaders/DataLoader.h"
#include <DataLoaders/RScanDataLoader.h>
#include <string>

// NOTE: for gdb debugging
std::string make_string(const char *x)
{
        return x;
}

void RScanDataLoader::ParseConfig(scene_map_t &scene_map) {
  std::string dataset_dir, config_dir, rscan_json, config_json, objects_json;

  ros::param::get("ros_pkg_dir", config_dir);
  ros::param::get("dataset_dir", dataset_dir);

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
    ref_scene.ply_file_path = dataset_dir + "/" + ref_scene.scan_id + "/" + ply_file;
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

    scene_map[ref_scene.scan_id] = ref_scene;

    // For each reference scan we need to find all the query scans and populate the queryscan map
    // TODO do this in parallel
    for (auto const &scan : refQueryMapData["scans"]) {
      if (scan["reference"] == ref_scene.scan_id) {
        std::vector<json> query_scans = scan["scans"];
        for (auto const &query_scan : query_scans) {
          Scene query_scene;
          query_scene.scan_id = query_scan["reference"].get<std::string>();
          query_scene.ply_file_path = dataset_dir + "/" + query_scene.scan_id + "/" + ply_file;
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
            sp_object.global_id = std::stoi(obj["global_id"].get<std::string>());
            sp_object.scene_id = std::stoi(obj["id"].get<std::string>());
            sp_object.label = obj["label"].get<std::string>();
            sp_object.ply_color = obj["ply_color"].get<std::string>();

            sp_object.ply_color.erase(0, 1);
            query_scene.spectral_objects.push_back(sp_object);
          }

          scene_map[query_scene.scan_id] = query_scene;
        }
      }
    }
  }
}
