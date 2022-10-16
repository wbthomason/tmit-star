#ifndef UTILS_HH
#define UTILS_HH

#include <bullet/btBulletCollisionCommon.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"
#include "state.hh"

#define ENSURE_SPEC_FIELD(spec, key, msg) \
  if (!spec.contains(key)) {              \
    log->error(msg);                      \
    return std::nullopt;                  \
  }

#define GET_FIELD_WITH_DEFAULT(field_type, name, default_value, spec, type_check) \
  field_type name = default_value;                                                \
  if (spec.contains(#name)) {                                                     \
    if (spec[#name].type_check()) {                                               \
      name = spec[#name].get<field_type>();                                       \
    } else {                                                                      \
      log->error("Wrong type for field {}: wanted {} but got {}",                 \
                 #name,                                                           \
                 #field_type,                                                     \
                 spec[#name].type_name());                                        \
    }                                                                             \
  }

namespace utils {
std::string
make_grounded_name(const std::string& base_name, const std::vector<std::string>& argument_names);
std::optional<std::string> read_file_to_string(const std::filesystem::path file_path);
btTransform transform3_to_btTransform(const planner::geometric::Transform3<>& tf);

std::shared_ptr<spdlog::logger> get_logger(const std::string& logger_name);
}  // namespace utils
#endif
