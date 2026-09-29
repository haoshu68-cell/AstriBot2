#include <cassert>
#include "astribot_s1_task_arbiter_native/ownership.hpp"
using astribot_s1_task_arbiter_native::Ownership;
int main() {
  Ownership owner;
  assert(owner.reserve(50));
  assert(!owner.reserve(100));  // goal callback reserves until accepted callback
  owner.accept("old", 50);
  assert(!owner.reserve(100));
  assert(owner.acquire("old"));
  assert(!owner.reserve(10));
  assert(owner.reserve(50));  // equal priority replaces
  owner.accept("new", 50);
  assert(!owner.acquire("new"));
  assert(!owner.reserve(100));
  owner.release("unrelated");
  assert(owner.active()->id == "old");
  owner.release("new");  // pending timeout/cancel does not release active
  assert(owner.active()->id == "old");
  assert(!owner.reserve(10));
  assert(owner.reserve(100));
  owner.accept("next", 100);
  owner.release("old");  // only terminal invokes release
  assert(owner.acquire("next"));
  owner.release("old");  // late terminal cannot release new generation
  assert(owner.active()->id == "next");
  owner.release("next");
  assert(owner.reserve(10));
}
