#include "instrumentation.hh"

#include <chrono>
#include <fstream>
#include <memory>

#include "nlohmann/json.hpp"
#include "robin_hood.h"

namespace utils {
void to_json(nlohmann::json& j, const std::unique_ptr<Instruments::Event>& e) { e->make_json(j); }

void to_json(nlohmann::json& j, const Instruments& i) { j = i.events; }

void Instruments::Event::make_json(nlohmann::json& j) const {
  j = std::chrono::duration_cast<std::chrono::nanoseconds>(time - *instruments.first_time).count();
}

void Instruments::serialize(const std::filesystem::path& output_path) const {
#ifndef NDEBUG
  std::ofstream output_file(output_path);
  output_file << nlohmann::json(*this);
  output_file.close();
#endif
}

void Instruments::log_event(const std::string& name) {
#ifndef NDEBUG
  const auto time = std::chrono::system_clock::now();
  if (!first_time) {
    first_time = time;
  }

  events[name].emplace_back(std::make_unique<Instruments::Event>(time, *this));
#endif
}
}  // namespace utils
