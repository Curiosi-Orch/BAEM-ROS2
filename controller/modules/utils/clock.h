#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "utils/utilities.h"

class Clock {
 public:
  Clock(const std::string& config_path = "clock.yaml") {
    config_path_ = ament_index_cpp::get_package_share_directory("controller")+"/configs/"+config_path;
    YAML::Node config = YAML::LoadFile(config_path_);
    time_offset_ = config["offset_time"].as<double>();
  }

  ~Clock() {}

  double GetTime_ms() {
    return GetTimeStamp() + time_offset_;
  }

  double GetTime_ms_without_offset() {
    return GetTimeStamp();
  }

 private:
  std::string config_path_;
  double time_offset_;
};