#include "astribot_s1_transport_mtc/canonical_octomap.hpp"
#include <octomap/OcTree.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace astribot::transport {
namespace {
constexpr std::size_t max_bytes = 32U * 1024U * 1024U;
constexpr std::size_t max_nodes = 2U * 1024U * 1024U;
void reject(const char* reason) { throw std::invalid_argument(reason); }

// Old liboctomap readers do not stop recursive descent on short input. Validate
// its serialized topology before allowing the library to allocate/read nodes.
class Validator {
 public:
  explicit Validator(const std::string& bytes): data(bytes) {}
  void full(unsigned depth = 0) {
    node(); need(sizeof(float) + 1);
    float value; std::memcpy(&value, data.data()+offset, sizeof(value));
    if (!std::isfinite(value)) reject("OCTOMAP_NONFINITE_LOG_ODDS");
    offset += sizeof(value);
    const auto mask = static_cast<unsigned char>(data[offset++]);
    if (depth >= 16 && mask) reject("OCTOMAP_DEPTH_EXCEEDED");
    for (unsigned i=0; i<8; ++i) if (mask & (1U<<i)) full(depth+1);
  }
  void binary(unsigned depth = 0, bool required_children = false) {
    node(); need(2);
    const auto a=static_cast<unsigned char>(data[offset++]);
    const auto b=static_cast<unsigned char>(data[offset++]);
    if (depth >= 16 && (a || b)) reject("OCTOMAP_DEPTH_EXCEEDED");
    if (required_children && !a && !b) reject("OCTOMAP_EMPTY_INNER_NODE");
    for (unsigned i=0; i<8; ++i) {
      const auto kind=((i<4 ? a:b) >> ((i%4)*2)) & 3U;
      if (kind == 3) binary(depth+1, true);
      else if (kind) node();
    }
  }
  void finish() const { if(offset != data.size()) reject("OCTOMAP_TRAILING_BYTES"); }
 private:
  void need(std::size_t n) const { if(data.size()-offset<n) reject("OCTOMAP_TRUNCATED"); }
  void node() { if(++nodes>max_nodes) reject("OCTOMAP_NODE_BUDGET_EXCEEDED"); }
  const std::string& data; std::size_t offset=0, nodes=0;
};
void append16(std::string& output, unsigned value) {
  output.push_back(static_cast<char>(value & 255U));
  output.push_back(static_cast<char>((value>>8U) & 255U));
}
}

std::string canonical_octomap(const std::string& data, bool binary,
                              double resolution, const std::string& tree_id) {
  if(tree_id!="OcTree") reject("OCTOMAP_TYPE_UNSUPPORTED");
  if(!std::isfinite(resolution) || resolution<=0) reject("OCTOMAP_RESOLUTION_INVALID");
  if(data.empty() || data.size()>max_bytes) reject("OCTOMAP_DATA_SIZE_INVALID");
  static_assert(sizeof(float)==4, "Octomap full data requires 32-bit float");
  Validator validator(data);
  if(binary) validator.binary(); else validator.full();
  validator.finish();
  octomap::OcTree tree(resolution);
  std::istringstream stream(data, std::ios::binary);
  if(binary) tree.readBinaryData(stream); else tree.readData(stream);
  if(!stream || stream.peek()!=std::char_traits<char>::eof()) reject("OCTOMAP_READ_FAILED");
  // FCL uses this tree's occupancy threshold and a free threshold of log-odds 0.
  // Both bits are retained at the boundary (0 may satisfy both predicates).
  // Include inner nodes: FCL can prune on their classification before leaf tests.
  std::vector<std::array<unsigned, 6>> nodes;
  nodes.reserve(tree.size());
  for(auto it=tree.begin_tree();it!=tree.end_tree();++it) {
    const auto key=it.getKey();
    nodes.push_back({key[0],key[1],key[2],it.getDepth(),it.isLeaf()?1U:0U,
      (tree.isNodeOccupied(*it)?1U:0U) | (it->getLogOdds()<=0.f?2U:0U)});
  }
  std::sort(nodes.begin(),nodes.end());
  std::string output="astribot.octomap.collision/1\n";
  // The actual library threshold belongs to the canonical contract as well.
  const auto threshold=tree.getOccupancyThresLog();
  output.append(reinterpret_cast<const char*>(&threshold),sizeof(threshold));
  for(const auto& node:nodes) for(auto value:node) append16(output,value);
  return output;
}
}
