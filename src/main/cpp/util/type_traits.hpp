#pragma once

#include <string>
#include <type_traits>

namespace util {

/**
 * @brief Checks if the given type parameter is std::map or std::unordered_map.
 *
 * @tparam T type parameter to check
 */
// clang-format off
template <typename T> struct is_map : std::false_type {};
template <typename K, typename V> struct is_map<std::map<K, V>> : std::true_type {};
template <typename K, typename V> struct is_map<std::unordered_map<K, V>> : std::true_type {};
// clang-format on

/**
 * @brief Checks if the given type parameter is std::set or std::unordered_set.
 *
 * @tparam T type parameter to check
 */
// clang-format off
template <typename T> struct is_set : std::false_type {};
template <typename U> struct is_set<std::set<U>> : std::true_type {};
template <typename U> struct is_set<std::unordered_set<U>> : std::true_type {};
// clang-format on

/**
 * @brief Checks if the given type parameter is a char array.
 *
 * @tparam T type parameter to check
 */
template <typename T>
struct is_char_array {
  static constexpr bool value = std::is_array<T>::value && std::is_same<typename std::remove_extent<T>::type, char>::value;
};

/**
 * @brief Checks if the given type parameter is a char array or std::string.
 *
 * @tparam T type parameter to check
 */
template <typename T>
struct is_string {
  static constexpr bool value = std::is_base_of<std::string, T>::value || util::is_char_array<T>::value;
};
}  // namespace util
