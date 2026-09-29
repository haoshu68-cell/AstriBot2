#define main object_pose_register_main
#include "../src/register_cli.cpp"
#undef main
#include <algorithm>
#include <chrono>
#include <limits>

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
cv::Mat boxVertices(const cv::Vec3d& size) {
  cv::Mat model(8, 6, CV_32F, cv::Scalar(0));
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 3; ++j)
      model.at<float>(i, j) = ((i >> j) & 1 ? .5 : -.5) * size[j];
  return model;
}
void rejects(const cv::Mat& model,
             const std::vector<astribot::object_pose::BoxPrimitive>& boxes) {
  bool rejected = false;
  try { (void)squarePrismZSymmetry(model, boxes); }
  catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "invalid square-prism declaration accepted");
}
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "one test case required");
    const std::string test = argv[1];
    astribot::object_pose::BoxPrimitive box;
    box.center_m = {0, 0, 0}; box.size_m = {.06, .06, .12};
    const auto model = boxVertices(box.size_m);
    if (test == "group") {
      auto group = squarePrismZSymmetry(model, {box});
      require(group.size() == 7, "seven nonidentity rotations required");
      const auto identity = cv::Matx33d::eye();
      for (const auto& r : group)
        require(cv::norm(cv::Mat(r - identity)) > 1, "identity must be implicit");
      group.push_back(identity);
      bool quarter_turn = false, flips_z = false;
      for (std::size_t i = 0; i < group.size(); ++i) {
        const auto& r = group[i];
        require(std::abs(cv::determinant(cv::Mat(r)) - 1) < 1e-12, "improper rotation");
        require(cv::norm(cv::Mat(r.t()*r - identity)) < 1e-12, "nonorthogonal rotation");
        quarter_turn |= std::abs(r(1, 0)) == 1 && r(2, 2) == 1;
        flips_z |= r(2, 2) == -1;
        for (std::size_t j = i + 1; j < group.size(); ++j)
          require(cv::norm(cv::Mat(r - group[j])) > 1, "duplicate group member");
        for (const auto& s : group) {
          const auto product = r * s;
          require(std::any_of(group.begin(), group.end(), [&](const auto& member) {
            return cv::norm(cv::Mat(product - member)) < 1e-12;
          }), "group not closed under composition");
        }
        for (int row = 0; row < model.rows; ++row) {
          const cv::Vec3d vertex(model.at<float>(row, 0), model.at<float>(row, 1), model.at<float>(row, 2));
          const auto rotated = r * vertex;
          for (int axis = 0; axis < 3; ++axis)
            require(std::abs(std::abs(rotated[axis]) - box.size_m[axis]/2) < 1e-8,
                    "rotation does not preserve box vertices");
        }
      }
      require(quarter_turn && flips_z, "missing quarter turn or end exchange");
    } else if (test == "wrong_geometry") {
      rejects(model, {}); rejects(model, {box, box});
      auto bad = box; bad.size_m[1] = .07; rejects(boxVertices(bad.size_m), {bad});
      bad = box; bad.size_m[2] = .06; rejects(boxVertices(bad.size_m), {bad});
      bad = box; bad.size_m[0] = 0; rejects(model, {bad});
      bad = box; bad.center_m[0] = .001; rejects(model, {bad});
      bad = box; bad.size_m[2] = std::numeric_limits<double>::quiet_NaN(); rejects(model, {bad});
    } else if (test == "cad_mismatch") {
      auto wrong = boxVertices({.06, .07, .12}); rejects(wrong, {box});
      wrong = model.clone(); wrong.col(0) += .001; rejects(wrong, {box});
      wrong = model.clone(); wrong.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN(); rejects(wrong, {box});
      rejects(cv::Mat(), {box});
    } else if (test == "cli") {
      const auto directory = std::filesystem::temp_directory_path() /
          ("square_prism_cli_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
      std::filesystem::create_directory(directory);
      const auto cad = (directory / "model.xyz").string();
      const auto scene = (directory / "empty.xyz").string();
      const auto visibility = (directory / "visibility.json").string();
      const auto output = (directory / "result.json").string();
      std::ofstream(cad) << "-.03 -.03 -.06 1 0 0\n.03 .03 .06 1 0 0\n";
      std::ofstream(scene).close();
      std::vector<std::string> args = {"object_pose_register", "--model", cad, "--scene", scene,
          "--visibility-model", visibility, "--symmetry", "square_prism_z", "--output", output};
      std::vector<char*> raw;
      for (auto& arg : args) raw.push_back(arg.data());
      for (bool cube : {false, true}) {
        std::ofstream(visibility) << "{\"schema\":\"astribot.box_union/1\",\"units\":\"m\",\"boxes\":["
            "{\"center_m\":[0,0,0],\"size_m\":[0.06,0.06," << (cube ? .06 : .12) << "]}]}";
        require(object_pose_register_main(static_cast<int>(raw.size()), raw.data()) == 2,
                "CLI input rejection must exit 2");
        // OpenCV FileStorage cannot parse JSON null in the public failure result.
        // JSON syntax is covered by cli_test.py; check the exact reason field here.
        std::ifstream result(output);
        std::ostringstream payload; payload << result.rdbuf();
        const std::string reason = cube ? "invalid_square_prism_z_model" : "insufficient_model_points";
        require(payload.str().find("\"reason\":\"" + reason + "\"") != std::string::npos,
                "CLI declaration did not preserve the expected rejection reason");
      }
      std::filesystem::remove_all(directory);
    } else { throw std::runtime_error("unknown test case"); }
    std::cout << test << " PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
