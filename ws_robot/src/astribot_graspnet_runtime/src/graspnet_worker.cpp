#include "astribot_graspnet_runtime/cloud_io.hpp"
#include <torch/script.h>
#include <ATen/CPUGeneratorImpl.h>
#include <ATen/Context.h>
#include <ATen/Parallel.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc) throw std::runtime_error("arguments require values");
      const std::string name(argv[i]);
      if (name != "--model" && name != "--input" && name != "--output" && name != "--device")
        throw std::runtime_error("unknown argument: " + name);
      if (!args.emplace(name, argv[i+1]).second) throw std::runtime_error("duplicate argument");
    }
    for (auto name : {"--model", "--input", "--output"})
      if (args.count(name) == 0) throw std::runtime_error(std::string("missing ") + name);
    const auto start = std::chrono::steady_clock::now();
    setenv("CUBLAS_WORKSPACE_CONFIG", ":4096:8", 0);
    at::set_num_threads(1);
    at::globalContext().setAllowTF32CuDNN(false);
    at::globalContext().setAllowTF32CuBLAS(false);
    at::globalContext().setBenchmarkCuDNN(false);
    at::globalContext().setDeterministicAlgorithms(true, false);
    auto cloud = astribot_graspnet_runtime::read_cloud(args.at("--input"));
    // Upstream samples existing points randomly. Use a fixed CPU generator for
    // reproducibility across Python preparation and C++ worker processes.
    constexpr std::size_t samples = 20000;
    const auto count = cloud.size() / 3;
    std::vector<float> sampled(samples * 3);
    auto generator = at::make_generator<at::CPUGeneratorImpl>(0);
    auto options = torch::TensorOptions().dtype(torch::kLong).device(torch::kCPU);
    auto indices = count >= samples ? torch::randperm(count, generator, options).slice(0, 0, samples)
      : torch::cat({torch::arange(count, options), torch::randint(count, {static_cast<long>(samples-count)}, generator, options)});
    const auto* selected = indices.data_ptr<std::int64_t>();
    for (std::size_t i = 0; i < samples; ++i) {
      const auto source = selected[i];
      std::copy_n(cloud.data() + 3 * source, 3, sampled.data() + 3 * i);
    }
    torch::NoGradGuard no_grad;
    const auto device_name = args.count("--device") ? args.at("--device") : "cuda";
    if (device_name != "cuda" && device_name != "cpu") throw std::runtime_error("device must be cuda or cpu");
    const torch::Device device(device_name);
    torch::jit::ExtraFilesMap metadata{{"provenance.json", ""}};
    auto module = torch::jit::load(args.at("--model"), device, metadata);
    if (metadata.at("provenance.json").empty()) throw std::runtime_error("missing model provenance");
    module.eval();
    auto input = torch::from_blob(sampled.data(), {1, static_cast<long>(samples), 3}, torch::kFloat32).to(device);
    const auto loaded = std::chrono::steady_clock::now();
    auto output = module.forward({input}).toTensor().to(torch::kCPU).contiguous();
    const auto completed = std::chrono::steady_clock::now();
    if (output.scalar_type() != torch::kFloat32 || output.dim() != 2 || output.size(1) != 17 || output.size(0) > 1024)
      throw std::runtime_error("model returned incompatible grasp tensor");
    if (!torch::isfinite(output).all().item<bool>()) throw std::runtime_error("model returned nonfinite grasps");
    const auto temp_path = args.at("--output") + ".tmp";
    {
      std::ofstream stream(temp_path, std::ios::binary | std::ios::trunc);
      if (!stream || !stream.write(reinterpret_cast<const char*>(output.data_ptr<float>()), output.numel() * sizeof(float)))
        throw std::runtime_error("cannot write grasp output");
      stream.close();
      if (!stream) throw std::runtime_error("cannot complete grasp output");
    }
    std::filesystem::rename(temp_path, args.at("--output"));
    const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b-a).count(); };
    std::cout << "{\"backend\":\"libtorch_cpp\",\"input_points\":" << count
      << ",\"sampled_points\":" << samples << ",\"candidates\":" << output.size(0)
      << ",\"load_ms\":" << ms(start, loaded) << ",\"inference_ms\":" << ms(loaded, completed)
      << ",\"collision_checked\":false,\"provenance\":" << metadata.at("provenance.json") << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "graspnet_worker: " << error.what() << '\n';
    return 2;
  }
}
