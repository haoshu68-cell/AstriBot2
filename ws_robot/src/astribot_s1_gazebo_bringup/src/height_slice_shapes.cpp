#include "astribot_s1_gazebo_bringup/height_slice_shapes.hpp"

#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/Util.hh>
#include <ignition/gazebo/Util.hh>
#include <sdf/Box.hh>
#include <sdf/Mesh.hh>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace astribot::simulation {
namespace {

std::string mesh_file(const sdf::Mesh & shape)
{
  namespace fs = std::filesystem;
  const auto & uri = shape.Uri();
  const auto full_path = ignition::gazebo::asFullPath(uri, shape.FilePath());
  if (fs::is_regular_file(full_path)) return full_path;

  auto relative = uri;
  for (const auto * scheme : {"model://", "package://", "file://"}) {
    if (relative.rfind(scheme, 0) == 0) {
      relative.erase(0, std::string(scheme).size());
      break;
    }
  }
  const fs::path path(relative);
  if (path.is_absolute() && fs::is_regular_file(path)) return path.string();

  std::vector<fs::path> roots;
  if (!shape.FilePath().empty()) {
    const auto model_dir = fs::path(shape.FilePath()).parent_path();
    roots.push_back(model_dir);
    // Warehouse file://models/... is relative to the package share root,
    // whereas FilePath names models/<model>/model.sdf.
    if (model_dir.parent_path().filename() == "models")
      roots.push_back(model_dir.parent_path().parent_path());
  }
  for (const auto & resource : ignition::gazebo::resourcePaths()) {
    const fs::path root(resource);
    roots.push_back(root);
    // Gazebo's warehouse launch exports <share>/models and <share>/worlds.
    // Do not accidentally resolve file://models/... as models/models/....
    if (root.filename() == "models" || root.filename() == "worlds")
      roots.push_back(root.parent_path());
  }
  for (const auto & root : roots) {
    const auto candidate = (root / path).lexically_normal();
    if (fs::is_regular_file(candidate)) return candidate.string();
  }
  // Consult Gazebo's registered URI callbacks after local candidates, because
  // common4 logs failed URI resolution even when FindFile is non-verbose.
  const auto file = ignition::common::findFile(full_path, false);
  if (!file.empty() && fs::is_regular_file(file)) return file;
  throw std::runtime_error(
    "height slice: cannot resolve collision mesh '" + uri +
    "' from SDF '" + shape.FilePath() + "'");
}

SliceVertex transformed(const ignition::math::Vector3d & vertex,
                        const ignition::math::Pose3d & pose)
{
  const auto point = pose.Pos() + pose.Rot().RotateVector(vertex);
  if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) ||
      !std::isfinite(point.Z()))
    throw std::runtime_error("height slice: non-finite collision vertex");
  return {point.X(), point.Y(), point.Z()};
}

}  // namespace

std::vector<SliceTriangle> collision_triangles(
  const sdf::Geometry & geometry,
  const ignition::math::Pose3d & map_from_collision)
{
  std::vector<SliceTriangle> triangles;
  if (geometry.Type() == sdf::GeometryType::BOX) {
    const auto half = geometry.BoxShape()->Size() * 0.5;
    std::array<SliceVertex, 8> vertices;
    for (std::size_t i = 0; i < vertices.size(); ++i)
      vertices[i] = transformed(
        {(i & 1) ? half.X() : -half.X(),
         (i & 2) ? half.Y() : -half.Y(),
         (i & 4) ? half.Z() : -half.Z()}, map_from_collision);
    constexpr std::array<std::array<std::size_t, 3>, 12> faces{{
      {{0, 2, 3}}, {{0, 3, 1}}, {{4, 5, 7}}, {{4, 7, 6}},
      {{0, 1, 5}}, {{0, 5, 4}}, {{2, 6, 7}}, {{2, 7, 3}},
      {{0, 4, 6}}, {{0, 6, 2}}, {{1, 3, 7}}, {{1, 7, 5}}}};
    for (const auto & face : faces)
      triangles.push_back({vertices[face[0]], vertices[face[1]], vertices[face[2]]});
    return triangles;
  }
  if (geometry.Type() != sdf::GeometryType::MESH)
    throw std::runtime_error("height slice: unsupported collision geometry type " +
                             std::to_string(static_cast<int>(geometry.Type())));

  const auto & shape = *geometry.MeshShape();
  if (!shape.Submesh().empty())
    throw std::runtime_error("height slice: submesh selection is unsupported: " +
                             shape.Submesh());
  const auto file = mesh_file(shape);
  const auto * mesh = ignition::common::MeshManager::Instance()->Load(file);
  if (!mesh)
    throw std::runtime_error("height slice: failed to load collision mesh: " + file);
  for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i) {
    const auto submesh = mesh->SubMeshByIndex(i).lock();
    if (!submesh || submesh->SubMeshPrimitiveType() != ignition::common::SubMesh::TRIANGLES ||
        submesh->IndexCount() % 3 != 0)
      throw std::runtime_error("height slice: invalid triangle submesh in " + file);
    for (unsigned int j = 0; j < submesh->IndexCount(); j += 3) {
      SliceTriangle triangle;
      for (unsigned int k = 0; k < 3; ++k) {
        const auto index = submesh->Index(j + k);
        if (index < 0 || static_cast<unsigned int>(index) >= submesh->VertexCount())
          throw std::runtime_error("height slice: invalid mesh vertex index in " + file);
        // The importer already applied DAE units, up-axis, and node transforms.
        triangle[k] = transformed(submesh->Vertex(index) * shape.Scale(), map_from_collision);
      }
      triangles.push_back(triangle);
    }
  }
  if (triangles.empty())
    throw std::runtime_error("height slice: collision mesh has no triangles: " + file);
  return triangles;
}

}  // namespace astribot::simulation
