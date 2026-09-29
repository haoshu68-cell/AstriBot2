#include <gtest/gtest.h>
#include "astribot_sensor_sync/sync_core.hpp"
using namespace astribot::sync;
static Trigger edge(uint64_t id=1) { return {"bench", "epoch1", "clock1", id, 1000000000, true}; }
static Frame frame() { return {"head_rgbd/depth", "device1", "bench", "epoch1", "clock1", 1, 1, 1000000100, 1000, true, true, true}; }
static Config config() { return {{"head_rgbd/depth", "imu"}, 2000000, 1000000, 250000000, 4, true, "clock1"}; }
TEST(Sync, HealthySimulatedIsNeverHardware) { Monitor m(config()); m.trigger(edge()); auto r=m.frame(frame(),1000000200); EXPECT_TRUE(r.valid); EXPECT_FALSE(r.hardware); }
TEST(Sync, HardwareRequiresRealReadbackAndMetadata) { Monitor m(config()); auto t=edge();t.simulated=false;m.trigger(t);auto f=frame();f.simulated=false;auto r=m.frame(f,1000000200);EXPECT_TRUE(r.valid);EXPECT_TRUE(r.hardware);f.sequence=2;f.capture_ns+=1;f.hardware_associated=false;EXPECT_EQ(m.frame(f,1000000300).reason,"NO_HARDWARE_ASSOCIATION"); }
TEST(Sync, MissingAndDelayedEdge) { Monitor m(config());EXPECT_EQ(m.frame(frame(),1000000200).reason,"TRIGGER_UNKNOWN");m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid); }
TEST(Sync, ExpiryNotHiddenByAssociation) { Monitor m(config());m.trigger(edge());EXPECT_EQ(m.frame(frame(),1400000000).reason,"STALE_SAMPLE"); }
TEST(Sync, FutureRejected) { Monitor m(config());m.trigger(edge());EXPECT_EQ(m.frame(frame(),999000000).reason,"FUTURE_SAMPLE"); }
TEST(Sync, DuplicateRejected) { Monitor m(config());m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid);EXPECT_EQ(m.frame(frame(),1000000200).reason,"NON_MONOTONIC_FRAME"); }
TEST(Sync, SkewAndUncertainty) { Monitor m(config());m.trigger(edge());auto f=frame();f.capture_ns+=3000000;EXPECT_EQ(m.frame(f,1004000000).reason,"EXPOSURE_SKEW");f=frame();f.uncertainty_ns=1000001;EXPECT_EQ(m.frame(f,1004000000).reason,"CLOCK_UNCERTAIN"); }
TEST(Sync, ClockMismatchAndUnprovenAssociation) { Monitor m(config());m.trigger(edge());auto f=frame();f.clock_epoch="other";EXPECT_EQ(m.frame(f,1000000200).reason,"CLOCK_EPOCH_MISMATCH");f=frame();f.hardware_associated=false;EXPECT_EQ(m.frame(f,1000000200).reason,"NO_HARDWARE_ASSOCIATION"); }
TEST(Sync, BoundedHistory) { Monitor m(config());for(int i=1;i<=8;++i)m.trigger(edge(i));EXPECT_EQ(m.trigger_count(),4u);EXPECT_EQ(m.frame(frame(),1000000200).reason,"TRIGGER_UNKNOWN"); }
TEST(Sync, ClockRollbackLatchesUntilExplicitReset) { Monitor m(config());m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid);auto f=frame();f.sequence=2;EXPECT_EQ(m.frame(f,900000000).reason,"CLOCK_ROLLBACK");m.trigger(edge(2));EXPECT_EQ(m.frame(f,1000000300).reason,"CLOCK_ROLLBACK");m.reset();m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid); }
TEST(Sync, RejectUnknownSourceAndInvalidConfig) { Monitor m(config());auto f=frame();f.source="unknown";EXPECT_EQ(m.frame(f,1000000200).reason,"UNKNOWN_SOURCE");auto c=config();c.history=0;EXPECT_THROW(Monitor{c},std::invalid_argument); }
TEST(Sync, SimulationForbiddenInHardwareMode) {auto c=config();c.allow_simulated=false;Monitor m(c);m.trigger(edge());EXPECT_EQ(m.frame(frame(),1000000200).reason,"SIMULATED_EVIDENCE");}
TEST(Sync, TriggerConflictRevokesOldEvidence) {Monitor m(config());m.trigger(edge());auto t=edge();t.stamp_ns+=1;EXPECT_FALSE(m.trigger(t));EXPECT_EQ(m.frame(frame(),1000000200).reason,"TRIGGER_CONFLICT");}
TEST(Sync, TimeOnlySensorDoesNotClaimExposureSync) {Monitor m(config());auto f=frame();f.source="imu";f.trigger_sequence=0;f.hardware_associated=false;auto r=m.frame(f,1000000200);EXPECT_TRUE(r.valid);EXPECT_FALSE(r.hardware);EXPECT_EQ(r.reason,"CLOCK_ONLY");}
TEST(Sync, SameTriggerCannotCreateTwoFrames) {Monitor m(config());m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid);auto f=frame();f.sequence=2;f.capture_ns+=1;EXPECT_EQ(m.frame(f,1000000300).reason,"TRIGGER_REUSED");}
TEST(Sync, TimeOnlyMustShareReferenceClock) {Monitor m(config());auto f=frame();f.source="imu";f.clock_epoch="unrelated";EXPECT_EQ(m.frame(f,1000000200).reason,"CLOCK_EPOCH_MISMATCH");}
TEST(Sync, SourceRestartNeedsExplicitReset) {Monitor m(config());m.trigger(edge());EXPECT_TRUE(m.frame(frame(),1000000200).valid);auto f=frame();f.source_epoch="device2";EXPECT_EQ(m.frame(f,1000000200).reason,"SOURCE_EPOCH_CHANGED");}
TEST(Sync, LockLossRejected) {Monitor m(config());auto f=frame();f.clock_locked=false;EXPECT_EQ(m.frame(f,1000000200).reason,"CLOCK_UNLOCKED");}

TEST(Sync, DefaultMetadataDoesNotClaimClockLock) { Frame f; EXPECT_FALSE(f.clock_locked); }

TEST(Sync, UnconfiguredClockNeverAcceptsMatchingPlaceholder) {auto c=config();c.reference_clock_epoch="UNCONFIGURED";Monitor m(c);auto f=frame();f.clock_epoch="UNCONFIGURED";EXPECT_EQ(m.frame(f,1000000200).reason,"REFERENCE_CLOCK_UNCONFIGURED");}
