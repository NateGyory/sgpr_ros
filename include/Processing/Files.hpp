#pragma once

#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::json;

struct SemanticKittiResult {
  std::string sequence;
  std::string query_id;
  std::string ref_id;
  double obj_match_ratio;
};

struct EvalMetrics {
  std::string dataset;
  int sample_size;
  int mean_k;
  double std_thresh;
  std::string laplacian;
  double class_accuracy;
  double instance_accuracy;
  double precision;
  double recall;
  double f1_score;
  double threshold;
  std::vector<int> pred_labels;
  std::vector<int> truth_labels;
};

struct GFAFeatures {
  std::vector<std::vector<double>> query_gfa;
  std::vector<std::vector<double>> ref_gfa;
  std::vector<int> match_results;
  std::vector<int> q_global_id;
  std::vector<int> q_scene_id;
  std::vector<int> r_global_id;
  std::vector<int> r_scene_id;
};

namespace Processing {
namespace Files {

inline void
SaveSemanticKittiResults(std::vector<SemanticKittiResult> &results) {
  std::string sequence = results[0].sequence;
  std::string path =
      "/home/nate/Development/catkin_ws/src/sgpr_ros/results/SemanticKitti/" +
      sequence + "/results.json";

  std::unordered_map<int, json> json_map;

  json j;
  // {
  //   comparisons:
  //   [
  //    {
  //       query_id:
  //       ref_id:
  //       probability:
  //    } ...
  //   ]
  // }

  for (auto &result : results) {
    json comparison;
    comparison["query_id"] = result.query_id;
    comparison["ref_id"] = result.ref_id;
    comparison["probability"] = result.obj_match_ratio;
    j["comparisons"].push_back(comparison);
  }

  std::ofstream o(path);
  o << std::setw(4) << j << std::endl;
  o.close();
}

inline std::vector<std::string> GetFilenameFromFolder(std::string folder_path) {
  std::vector<std::string> filenames;
  DIR *dirp = opendir(folder_path.c_str());
  if (dirp) {
    struct dirent *dp;
    while ((dp = readdir(dirp)) != nullptr) {
      filenames.push_back(dp->d_name);
    }
    closedir(dirp);
  }
  return filenames;
}

inline void SaveGFAFeatures(GFAFeatures &gfa) {
  std::string path = "/home/nate/Development/catkin_ws/src/sgpr_ros/results/"
                     "gfa/results.json";

  json data;
  data["q_gfa"] = gfa.query_gfa;
  data["r_gfa"] = gfa.ref_gfa;
  data["truth_labels"] = gfa.match_results;
  data["q_global_id"] = gfa.q_global_id;
  data["r_global_id"] = gfa.r_global_id;
  data["q_scene_id"] = gfa.q_scene_id;
  data["r_scene_id"] = gfa.r_scene_id;

  std::ofstream o(path);
  o << std::setw(4) << data << std::endl;
  o.close();
}

inline void SaveEvalMetrics(EvalMetrics &em) {
  std::string path = "/home/nate/Development/catkin_ws/src/sgpr_ros/results/" +
                     em.dataset + "/" + em.laplacian + "/";

  std::vector<std::string> files = GetFilenameFromFolder(path);
  int number = -1;
  for (auto file : files) {
    if (file == "." || file == "..")
      continue;
    int dot_position = file.find(".");
    if (dot_position != std::string::npos) {
      file.erase(dot_position, file.length() - dot_position);
    }
    int file_number = stoi(file);
    if (file_number > number)
      number = file_number;
  }
  number++;

  // Save the file now
  json data;
  data["sample_size"] = em.sample_size;
  data["mean_k"] = em.mean_k;
  data["std_thresh"] = em.std_thresh;
  data["laplacian"] = em.laplacian;
  data["instance_accuracy"] = em.instance_accuracy;
  data["class_accuracy"] = em.class_accuracy;
  data["precision"] = em.precision;
  data["recall"] = em.recall;
  data["f1_score"] = em.f1_score;
  data["threshold"] = em.threshold;
  data["pred_labels"] = em.pred_labels;
  data["truth_labels"] = em.truth_labels;

  std::string filename = std::to_string(number) + ".json";
  std::ofstream o(path + filename);
  o << std::setw(4) << data << std::endl;
  o.close();
}

}; // namespace Files
}; // namespace Processing
