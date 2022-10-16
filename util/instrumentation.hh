#ifndef INSTRUMENTATION_HH
#define INSTRUMENTATION_HH

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"
#include "robin_hood.h"

namespace utils {
class Instruments {
  struct Event {
    Event(const std::chrono::system_clock::time_point& time, const Instruments& instruments)
    : time(time), instruments(instruments) {}
    std::chrono::system_clock::time_point time;
    const Instruments& instruments;
    virtual void make_json(nlohmann::json& j) const;
    virtual ~Event() = default;
  };

  std::unordered_map<std::string, std::vector<std::unique_ptr<Event>>> events;
  std::optional<std::chrono::system_clock::time_point> first_time;

 public:
  template <typename ValueT = unsigned long> struct Counter {
    Counter(const std::string& name, ValueT initial_value, Instruments& instruments)
    : value(initial_value), name(name), instruments(instruments) {
      update_value(initial_value);
    }

    struct CounterEvent : public Event {
      CounterEvent(const std::chrono::system_clock::time_point& time,
                   ValueT value,
                   const Instruments& instruments)
      : Event(time, instruments), value(value) {}
      ValueT value;
      void make_json(nlohmann::json& j) const override {
        j = nlohmann::json{
        std::chrono::duration_cast<std::chrono::nanoseconds>(time - *instruments.first_time).count(),
        value};
      }
    };

    void add_to_value(ValueT add) { update_value(value + add); }
    void increment() { add_to_value(static_cast<ValueT>(1)); }
    void decrement() { subtract_from_value(static_cast<ValueT>(1)); }
    void subtract_from_value(ValueT sub) { update_value(value - sub); }
    void update_value(ValueT new_value) {
#ifndef NDEBUG
      const auto time = std::chrono::system_clock::now();
      this->value     = new_value;
      if (!instruments.first_time) {
        instruments.first_time = time;
      }

      instruments.events[name].emplace_back(
      std::make_unique<CounterEvent>(time, new_value, instruments));
#endif
    }


   private:
    ValueT value;
    const std::string name;
    Instruments& instruments;
  };

  void log_event(const std::string& name);
  void serialize(const std::filesystem::path& output_path) const;
  friend void to_json(nlohmann::json& j, const Instruments& i);
  friend void to_json(nlohmann::json& j, const std::unique_ptr<Instruments::Event>& e);
  template <typename ValueT = unsigned long>
  Counter<ValueT> add_counter(const std::string& name, ValueT initial_value) {
    return Counter(name, initial_value, *this);
  }
};
}  // namespace utils

#endif
