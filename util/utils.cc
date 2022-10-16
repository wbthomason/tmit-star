#include "utils.hh"

#include <bullet/btBulletCollisionCommon.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace utils {
std::string
make_grounded_name(const std::string& base_name, const std::vector<std::string>& argument_names) {
  std::ostringstream name_stream;
  name_stream << base_name << '(';
  for (size_t i = 0; i < argument_names.size(); ++i) {
    if (i > 0) {
      name_stream << ", ";
    }

    name_stream << argument_names[i];
  }

  name_stream << ')';

  return name_stream.str();
}

std::optional<std::string> read_file_to_string(const std::filesystem::path file_path) {
  std::ifstream file(file_path, std::ios::in | std::ios::binary);
  if (file) {
    std::ostringstream file_contents;
    file_contents << file.rdbuf();
    file.close();
    return file_contents.str();
  }

  return std::nullopt;
}

btTransform transform3_to_btTransform(const planner::geometric::Transform3<>& tf) {
  const Eigen::Quaterniond rotation(tf.linear());
  const auto& translation = tf.translation();
  return btTransform(btQuaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()),
                     btVector3(translation.x(), translation.y(), translation.z()));
}

std::shared_ptr<spdlog::logger> get_logger(const std::string& logger_name) {
  auto result = spdlog::get(logger_name);
  if (result == nullptr) {
    result = spdlog::stdout_color_st(logger_name);
  }

  return result;
}
}  // namespace utils
