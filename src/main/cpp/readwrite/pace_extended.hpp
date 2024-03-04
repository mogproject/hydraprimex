#pragma once

#include <iostream>

#include "ds/ProblemInstance.hpp"

namespace readwrite {
ds::ProblemInstance read_pace_extended(std::istream &is);

ds::ProblemInstance load_pace_extended(char const *path);
}  // namespace readwrite
