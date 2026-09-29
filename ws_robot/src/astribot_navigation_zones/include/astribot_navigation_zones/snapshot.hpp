#pragma once
#include "astribot_navigation_zones/geometry.hpp"
#include <memory>
#include <mutex>
namespace astribot_navigation_zones {
struct Snapshot {std::string boot,context,token;uint64_t revision{0};std::vector<Region> regions;bool valid{false};};
class SnapshotCache {
 mutable std::mutex mutex_;std::shared_ptr<const Snapshot> snapshot_;
 double received_{-1e30},stamp_{-1e30};uint64_t sequence_{0};std::string body_;
public:
 bool receive(const Json & wire,double ros_now,double steady_now);
 std::shared_ptr<const Snapshot> get(double ros_now,double steady_now,size_t publishers=1)const;
 void invalidate();
};
}
