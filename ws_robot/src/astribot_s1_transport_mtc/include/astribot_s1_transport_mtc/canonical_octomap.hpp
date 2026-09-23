#pragma once
#include <string>

namespace astribot::transport {
// Collision semantics, not probability bytes. Throws on unsupported or corrupt input.
std::string canonical_octomap(const std::string& data, bool binary,
                              double resolution, const std::string& tree_id);
}
