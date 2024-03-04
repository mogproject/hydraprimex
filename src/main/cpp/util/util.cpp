#include "util.hpp"
#include <cstdarg>

namespace util {
//==================================================================================================
// String Utilities
//==================================================================================================
/**
 * @brief Splits a string into tokens.
 *
 * @param s string
 * @param delimiter delimiter string
 * @return std::vector<std::string> list of tokens
 */
std::vector<std::string> split(std::string const& s, std::string const& delimiter) {
  std::vector<std::string> ret;
  size_t p = 0, q, d = delimiter.size();
  while ((q = s.find(delimiter, p)) != std::string::npos) {
    ret.push_back(s.substr(p, q - p));
    p = q + d;
  }
  ret.push_back(s.substr(p));
  return ret;
}

/**
 * @brief Safer alternative to sprintf.
 *
 * @param fmt
 * @param ...
 * @return formatted string
 */
std::string format(char const* fmt, ...) {
  va_list marker;

  // first pass: find the length
  va_start(marker, fmt);
  auto length = std::vsnprintf(nullptr, 0, fmt, marker) + 1;
  va_end(marker);

  // second pass: create string
  char* str = new char[length];
  va_start(marker, fmt);
  std::vsnprintf(str, length, fmt, marker);
  va_end(marker);

  std::string ret(str);
  delete[] str;
  return ret;
}
}  // namespace util
