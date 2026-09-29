// Tests the exact policy header carried by the pinned renderer patch.
#include "RenderingWorkerCount.hh"
#include <cassert>
#include <stdexcept>
int main() {
  using astribot_rendering::workerCount;
  assert(workerCount(nullptr,28)==28);
  assert(workerCount(nullptr,0)==1);
  assert(workerCount("1",28)==1);
  assert(workerCount("4",28)==4);
  assert(workerCount("64",28)==64);
  for (const auto* value : {"", "0", "65", "-1", "+4", " 4", "4 ", "4.0", "abc", "999999999999999999999"}) {
    bool rejected=false;
    try { (void)workerCount(value,28); } catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
  }
}
