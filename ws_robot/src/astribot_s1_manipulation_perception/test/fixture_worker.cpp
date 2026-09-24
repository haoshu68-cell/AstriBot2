// Contract-test fixture ONLY. Never installed or used as a model backend.
#include <chrono>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
int main(int argc, char **argv) {
  std::string model, output;
  bool grasp = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string key = argv[i];
    if (key == "--model")
      model = argv[i + 1];
    if (key == "--output")
      output = argv[i + 1];
    if (key == "--device")
      grasp = true;
  }
  std::ifstream in(model);
  std::string behavior;
  in >> behavior;
  if (behavior == "slow")
    std::this_thread::sleep_for(std::chrono::seconds(2));
  if (grasp) {
    float row[17] = {.8f, .05f, .02f, .03f, 1, 0, 0, 0, 1,
                     0,   0,    0,    1,    0, 0, 1, -1};
    if (behavior == "nan")
      row[0] = std::numeric_limits<float>::quiet_NaN();
    if (behavior == "raw_score")
      row[0] = 1.4f;
    if (behavior == "negative_only")
      row[0] = -.02f;
    std::ofstream out(output, std::ios::binary);
    out.write(reinterpret_cast<const char *>(row), sizeof(row));
    if (behavior == "mixed_negative") {
      row[0] = -.01f;
      out.write(reinterpret_cast<const char *>(row), sizeof(row));
      row[0] = .7f;
      row[1] = 0;
      out.write(reinterpret_cast<const char *>(row), sizeof(row));
    }
  } else {
    std::ofstream out(output);
    if (behavior == "reject") {
      out << R"({"schema":"astribot.object_pose/1","success":false,"reason":"UNOBSERVABLE_GEOMETRY"})";
      return 2;
    }
    out << R"({"schema":"astribot.object_pose/1","success":true,"camera_from_object":[[1,0,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]],"model_coverage":0.7,"visible_model_coverage":null,"scene_coverage":0.9,"rmse_m":0.002,"symmetry_equivalent":false,"ambiguous":false})";
  }
  return 0;
}
