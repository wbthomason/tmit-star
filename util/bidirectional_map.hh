#ifndef BIDIRECTIONAL_MAP_HH
#define BIDIRECTIONAL_MAP_HH
#include <robin_hood.h>

#include <utility>
#include <vector>

namespace utils {
template <typename KeyT1, typename KeyT2> struct BidirectionalMap {
  template <typename K1, typename K2> void insert(K1&& k1, K2&& k2) {
    const auto t1_idx = t1_values.size();
    const auto t2_idx = t2_values.size();
    t1_index.emplace(k1, t2_idx);
    t2_index.emplace(k2, t1_idx);
    t1_values.emplace_back(std::forward<K1>(k1));
    t2_values.emplace_back(std::forward<K2>(k2));
  }

  KeyT1& get(const KeyT2& key) { return t1_values[t2_index.at(key)]; }
  KeyT2& get(const KeyT1& key) { return t2_values[t1_index.at(key)]; }

  KeyT1& operator[](const KeyT2& key) { return get(key); }
  KeyT2& operator[](const KeyT1& key) { return get(key); }

  const KeyT1& get(const KeyT2& key) const { return t1_values[t2_index.at(key)]; }
  const KeyT2& get(const KeyT1& key) const { return t2_values[t1_index.at(key)]; }

  const KeyT1& operator[](const KeyT2& key) const { return get(key); }
  const KeyT2& operator[](const KeyT1& key) const { return get(key); }

  bool contains(const KeyT1& key) const { return t1_index.contains(key); }
  bool contains(const KeyT2& key) const { return t2_index.contains(key); }

  size_t size() const { return t1_values.size(); }

  typename std::vector<KeyT1>::iterator find(const KeyT2& key) {
    auto result = t2_index.find(key);
    if (result == t2_index.end()) {
      return end_left();
    }

    return t1_values.begin() + result->second;
  }

  typename std::vector<KeyT2>::iterator find(const KeyT1& key) {
    auto result = t1_index.find(key);
    if (result == t1_index.end()) {
      return end_right();
    }

    return t2_values.begin() + result->second;
  }

  typename std::vector<KeyT1>::iterator end_left() { return t1_values.end(); }
  typename std::vector<KeyT2>::iterator end_right() { return t2_values.end(); }

  void reserve(size_t capacity) {
    t1_values.reserve(capacity);
    t2_values.reserve(capacity);
    t1_index.reserve(capacity);
    t2_index.reserve(capacity);
  }

  struct BidirectionalMapIterator {
    BidirectionalMapIterator(const std::vector<KeyT1>& t1_values, const std::vector<KeyT2>& t2_values, size_t idx)
    : t1_values(t1_values), t2_values(t2_values), idx(idx) {}

    std::pair<const KeyT1&, const KeyT2&> operator*() { return {t1_values[idx], t2_values[idx]}; }
    bool operator!=(const BidirectionalMapIterator& rhs) { return idx != rhs.idx; }
    void operator++() { ++idx; }

   protected:
    const typename std::vector<KeyT1>& t1_values;
    const typename std::vector<KeyT2>& t2_values;
    size_t idx{0};
  };

  auto begin() const { return BidirectionalMapIterator(t1_values, t2_values, 0); }
  auto end() const { return BidirectionalMapIterator(t1_values, t2_values, t1_values.size()); }

  robin_hood::unordered_map<KeyT1, size_t> t1_index;
  robin_hood::unordered_map<KeyT2, size_t> t2_index;
  std::vector<KeyT1> t1_values;
  std::vector<KeyT2> t2_values;
};
}  // namespace utils
#endif
