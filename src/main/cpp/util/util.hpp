#pragma once

#include <algorithm>
#include <cassert>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "type_traits.hpp"

namespace util {

//==================================================================================================
// STL Extensions (assuming SFINAE)
//==================================================================================================
/** Implementation specialized for map and set. */
template <typename T, typename U>
typename std::enable_if<util::is_map<T>::value || util::is_set<T>::value, bool>::type contains(T const& col, U const& x) {
  return col.find(x) != col.end();
}

/**
 * @brief Checks if string a contains string b.
 *
 * @param a first string
 * @param b second string
 * @return true a contains b
 * @return false a does not contain b
 */
template <typename T, typename U>
typename std::enable_if<util::is_string<U>::value, bool>::type contains(T const& a, U const& b) {
  return std::string(a).find(b) != std::string::npos;
}

/** Otherwise, use std::find to search for the element */
template <typename T, typename U>
typename std::enable_if<!util::is_string<U>::value && !(util::is_map<T>::value || util::is_set<T>::value), bool>::type contains(
    T const& col, U const& x) {
  return std::find(col.begin(), col.end(), x) != col.end();
}

template <typename T>
inline void extend(std::vector<T>& col, std::vector<T> const& xs) {
  for (auto& x : xs) col.push_back(x);
}

template <typename T>
inline std::vector<T> sorted(std::vector<T> const& col) {
  std::vector<T> ret(col);
  std::sort(ret.begin(), ret.end());
  return ret;
}

/**
 * @brief Returns a vector of the integers in the range [start,end).
 *
 * @tparam T integer type
 * @param start starting index
 * @param end ending index
 * @return std::vector<int> vector of integers
 */
template <typename T>
std::vector<T> range_to_vec(T start, T end) {
  std::vector<T> ret;
  for (T i = start; i < end; ++i) ret.push_back(i);
  return ret;
}

/**
 * @brief Returns a vector of the integers in the range [0,n).
 *
 * @tparam T integer type
 * @param n vector size
 * @return std::vector<int> vector of integers
 */
template <typename T>
std::vector<T> range_to_vec(T n) {
  return range_to_vec(0, n);
}

/**
 * @brief Creates an inverse map from a vector.
 *
 * @tparam T element type
 * @param xs vector of distinct elements
 * @return std::unordered_map<T, int> inverse map from element to index
 */
template <typename T>
std::unordered_map<T, int> inverse_map(std::vector<T> const& xs) {
  std::unordered_map<T, int> ret;
  for (std::size_t i = 0; i < xs.size(); ++i) ret[xs[i]] = i;
  return ret;
}

//==================================================================================================
// String Utils
//==================================================================================================

/**
 * @brief Splits a string into tokens.
 *
 * @param s string
 * @param delimiter delimiter string
 * @return std::vector<std::string> list of tokens
 */
std::vector<std::string> split(std::string const& s, std::string const& delimiter = " ");

/**
 * @brief Safer alternative to sprintf.
 *
 * @param fmt
 * @param ...
 * @return formatted string
 */
std::string format(char const* fmt, ...);

//==================================================================================================
// I/O Support
//==================================================================================================

// printers for STL types
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::vector<T> const& v);

template <typename A, typename B>
std::ostream& operator<<(std::ostream& stream, std::pair<A, B> const& p) {
  return stream << "(" << p.first << ", " << p.second << ")";
}
template <typename A, typename B>
std::ostream& operator<<(std::ostream& stream, std::map<A, B> const& p) {
  stream << "{";
  for (auto i = p.begin(); i != p.end(); ++i) stream << ((i == p.begin()) ? "" : ", ") << i->first << ": " << i->second;
  return stream << "}";
}
template <typename A, typename B>
std::ostream& operator<<(std::ostream& stream, std::unordered_map<A, B> const& p) {
  stream << "{";
  for (auto i = p.begin(); i != p.end(); ++i) stream << ((i == p.begin()) ? "" : ", ") << i->first << ": " << i->second;
  return stream << "}";
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::vector<T> const& v) {
  stream << "[";
  for (auto i = v.begin(); i != v.end(); ++i) stream << ((i == v.begin()) ? "" : ", ") << *i;
  return stream << "]";
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::list<T> const& v) {
  stream << "[";
  for (auto i = v.begin(); i != v.end(); ++i) stream << ((i == v.begin()) ? "" : ", ") << *i;
  return stream << "]";
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::set<T> const& s) {
  stream << "{";
  for (auto i = s.begin(); i != s.end(); ++i) stream << ((i == s.begin()) ? "" : ", ") << *i;
  return stream << "}";
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::unordered_set<T> const& s) {
  stream << "{";
  for (auto i = s.begin(); i != s.end(); ++i) stream << ((i == s.begin()) ? "" : ", ") << *i;
  return stream << "}";
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::queue<T>& q) {
  std::vector<T> v;
  for (; !q.empty(); q.pop()) v.push_back(q.front());
  for (auto& x : v) q.push(x);
  return stream << v;
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::stack<T>& s) {
  std::vector<T> v;
  for (; !s.empty(); s.pop()) v.push_back(s.top());
  for (auto it = v.rbegin(); it != v.rend(); ++it) s.push(*it);
  return stream << v;
}
template <typename T>
std::ostream& operator<<(std::ostream& stream, std::priority_queue<T>& q) {
  std::vector<T> v;
  for (; !q.empty(); q.pop()) v.push_back(q.top());
  for (auto& x : v) q.push(x);
  return stream << v;
}

/**
 * @brief Converts an instance of any type to a string.
 *
 * @tparam T type
 * @param x object
 * @return std::string string representation
 */
template <typename T>
std::string to_string(T const& x) {
  std::stringstream ss;
  ss << x;
  return ss.str();
}

}  // namespace util

/** Utility macro for debugging. */
#define cstr(x) util::to_string(x).c_str()
