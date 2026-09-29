#include "astribot_navigation_zones/snapshot.hpp"
namespace astribot_navigation_zones {
bool SnapshotCache::receive(const Json&w,double ros,double steady){
 std::lock_guard<std::mutex> lock(mutex_);
 try{
 if(w.at("schema_version")!=1||w.at("frame")!="map")throw std::runtime_error("ZONES.SCHEMA");
 const double stamp=w.at("stamp");const auto seq=w.at("sequence").get<uint64_t>();
 if(!std::isfinite(stamp)||stamp>ros+.1||ros-stamp>2.||!std::isfinite(steady))throw std::runtime_error("ZONES.STALE");
 Snapshot s;s.boot=w.at("boot_id");s.context=w.at("context_id");s.revision=w.at("revision");s.valid=w.at("valid");
 if(s.boot.empty()||s.boot.size()>128||s.context.size()>256||(s.valid&&s.context.empty()))throw std::runtime_error("ZONES.CONTEXT");
 s.token=s.boot+":"+s.context+":"+std::to_string(s.revision);const auto body=w.at("regions").dump();
 if(snapshot_&&snapshot_->boot==s.boot&&seq<=sequence_)return false;
 if(snapshot_&&snapshot_->boot==s.boot&&snapshot_->context==s.context&&s.revision<snapshot_->revision)throw std::runtime_error("ZONES.REVISION_ROLLBACK");
 if(snapshot_&&snapshot_->token==s.token&&body!=body_)throw std::runtime_error("ZONES.CONTENT_CONFLICT");
 if(snapshot_&&snapshot_->token==s.token)s.regions=snapshot_->regions;else s.regions=parseRegions(w.at("regions"));
 snapshot_=std::make_shared<Snapshot>(std::move(s));body_=body;received_=steady;stamp_=stamp;sequence_=seq;return true;
 }catch(...){received_=-1e30;return false;}
}
std::shared_ptr<const Snapshot> SnapshotCache::get(double ros,double steady,size_t publishers)const{
 std::lock_guard<std::mutex> lock(mutex_);if(publishers!=1||!snapshot_||!snapshot_->valid||ros<stamp_-.1||ros-stamp_>2.||steady<received_||steady-received_>2.)return nullptr;return snapshot_;
}
void SnapshotCache::invalidate(){std::lock_guard<std::mutex> lock(mutex_);received_=-1e30;}
}
