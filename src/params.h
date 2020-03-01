#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <fstream>

struct PipelineParams {

  std::string DATASET;

  // filtering parameters
  float FILTER_RESOLUTION;
  Eigen::Vector4f CROPBOX_MIN;
  Eigen::Vector4f CROPBOX_MAX;
  Eigen::Vector4f ROOF_MIN

  // RANSAC parameters
  int MAX_ITERATIONS;
  float DISTANCE_TOLERANCE;

  // clustering parameters
  float CLUSTER_DISTANCE;
  int CLUSTER_MIN;
  int CLUSTER_MAX;

  // visualization options
  bool render_input_cloud;
  bool render_filtered_cloud;
  bool render_obstacles;
  bool render_plane;
  bool render_clusters;

  bool fromFile(const std::string &filePath) {
    std::ifstream file_(filePath);

    if (!file_.is_open()) {
      std::cerr << "Params file not found!" << std::endl;
      return false;
    }

    std::string line_;
    int i = 0;
    while (getline(file_, line_)) {
      if (line_[0] == '#') continue;
      if (line_.empty()) continue;

      std::stringstream check1(line_);
      std::string paramName;

      check1 >> paramName;
      if (paramName == "dataset:"){
        check1 >> DATASET;
      } else if (paramName == "filter_resolution:") {
        check1 >> FILTER_RESOLUTION;
      } else if (paramName == "crop_min_point:") {
        double x, y, z, i;
        check1 >> x >> y >> z >> i;
        CROPBOX_MIN = Eigen::Vector4f(x, y, z, i);
      } else if (paramName == "crop_max_point:") {
        double x, y, z, i;
        check1 >> x >> y >> z >> i;
        CROPBOX_MAX = Eigen::Vector4f(x, y, z, i);
      } else if (paramName == "max_iterations:") {
        check1 >> MAX_ITERATIONS;
      } else if (paramName == "distance_threshold:") {
        check1 >> DISTANCE_TOLERANCE;
      } else if (paramName == "cluster_tolerance:") {
        check1 >> CLUSTER_DISTANCE;
      } else if (paramName == "cluster_min_size:") {
        check1 >> CLUSTER_MIN;
      } else if (paramName == "cluster_max_size:") {
        check1 >> CLUSTER_MAX;
      } else if (paramName == "render_input_cloud:") {
        check1 >> render_input_cloud;
      } else if (paramName == "render_filtered_cloud:") {
        check1 >> render_filtered_cloud;
      } else if (paramName == "render_obstacles:") {
        check1 >> render_obstacles;
      } else if (paramName == "render_plane:") {
        check1 >> render_plane;
      } else if (paramName == "render_clusters:") {
        check1 >> render_clusters;
      } else {
        std::cerr << "Unrecognized pipeline parameter: " << paramName << std::endl;
        assert(0);
      }
    }
    file_.close();
    return true;
  }

};

#endif