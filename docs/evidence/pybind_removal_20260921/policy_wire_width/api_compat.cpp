#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include <type_traits>
using namespace astribot::navigation::policy;
static_assert(std::is_convertible_v<std::optional<std::int64_t>,std::optional<Version::Counter>>);
static_assert(std::is_convertible_v<std::optional<std::uint64_t>,std::optional<Version::Counter>>);
static_assert(std::is_constructible_v<Version,std::string,int,int,unsigned long long>);
static_assert(std::is_same_v<decltype(Version("g",0,0,0).envelope_epoch),const std::uint64_t>);
static_assert(std::is_same_v<decltype(std::declval<CameraCalibration>().image_width_px),const std::int64_t>);
int main() {}
