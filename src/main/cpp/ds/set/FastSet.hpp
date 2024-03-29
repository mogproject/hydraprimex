#pragma once

#include <stdexcept>
#include <vector>

namespace ds {
namespace set {
/**
 * @brief Fixed-size, constant-time reusable integer set representation.
 */
class FastSet {
 public:
  FastSet(std::size_t size = 0);

  friend bool operator==(FastSet const& lhs, FastSet const& rhs);
  friend bool operator!=(FastSet const& lhs, FastSet const& rhs);

  /**
   * @brief Returns the capacity of the set.
   *
   * @return capacity
   */
  std::size_t capacity() const;

  /**
   * @brief Returns the current cardinality of the set.
   *
   * @return size
   */
  std::size_t size() const;

  /**
   * @brief Clears the set.
   *
   * O(1)
   */
  void clear();

  /**
   * @brief Initializes the set with a new capcity.
   *
   * O(size)
   */
  void initialize(std::size_t size);

  /**
   * @brief Resizes the capacity of the set.
   *
   * O(size)
   */
  void resize(std::size_t size);

  /**
   * @brief Inserts one element to the set.
   *
   * O(1)
   */
  void set(int x);

  /**
   * @brief Removes one element from the set.
   *
   * O(1)
   */
  void reset(int x);

  /**
   * @brief Checks if the given element is in the set.
   *
   * O(1)
   */
  bool get(int x) const;

  // O(n)
  std::vector<int> to_vector() const;

 private:
  int generation_;
  std::vector<int> data_;
};
}  // namespace set
}  // namespace ds
