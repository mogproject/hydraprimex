#pragma once

#include <unordered_map>
#include <vector>

#include "util/Random.hpp"

namespace ds {
namespace map {

/**
 * @brief Bi-directional map.
 *
 * @tparam T type of mapped values
 */
template <typename T>
class Bimap {
 private:
  std::vector<T> right_;
  std::unordered_map<T, int> left_;

 public:
  Bimap(std::vector<T> const& xs = {}) : right_(xs) {
    for (std::size_t i = 0; i < right_.size(); ++i) left_[right_[i]] = i;
  }

  friend bool operator==(Bimap<T> const& lhs, Bimap<T> const& rhs) { return lhs.right_ == rhs.right_; }
  friend bool operator!=(Bimap<T> const& lhs, Bimap<T> const& rhs) { return !(lhs.right_ == rhs.right_); }

  /**
   * @brief Clears the entire map.
   */
  void clear() {
    right_.clear();
    left_.clear();
  }

  /**
   * @brief Returns the size of the map.
   *
   * @return std::size_t map size
   */
  std::size_t size() const { return right_.size(); }

  /**
   * @brief Returns the forward map.
   *
   * @return std::vector<T> const& forward map
   */
  std::vector<T> const& right() const { return right_; }

  /**
   * @brief Returns the backward (reverse) map.
   *
   * @return std::vector<T> const& backward map
   */
  std::unordered_map<T, int> const& left() const { return left_; }

  /**
   * @brief Forward lookup.
   *
   * @param i index
   * @return T value of f(i)
   */
  T f(int i) const { return right_[i]; }

  /**
   * @brief Backward lookup.
   *
   * @param x element
   * @return int value of f^{-1}(x)
   */
  int g(T x) const { return left_.at(x); }

  /**
   * @brief Randomly shuffles the map.
   *
   * @param rand util::Random instance
   */
  void shuffle(util::Random& rand) {
    rand.shuffle(right_);
    for (std::size_t i = 0; i < right_.size(); ++i) left_[right_[i]] = i;
  }
};
}  // namespace map
}  // namespace ds
