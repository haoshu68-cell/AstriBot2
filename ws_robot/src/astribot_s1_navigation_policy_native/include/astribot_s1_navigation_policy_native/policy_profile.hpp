#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>

namespace astribot::navigation {
// Same resolution and admission contract as navigation_policy.profile.Profile.
// Called only at startup; the control loop retains typed, validated values.
nlohmann::json load_policy_profile(const std::filesystem::path& path, bool use_sim_time);
}  // namespace astribot::navigation
