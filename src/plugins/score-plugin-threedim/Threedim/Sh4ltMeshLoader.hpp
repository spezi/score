#pragma once
#include <Threedim/TinyObj.hpp>
#include <halp/controls.hpp>
#include <halp/geometry.hpp>
#include <halp/meta.hpp>
#include <ossia/detail/mutex.hpp>

#include <sh4lt/follower.hpp>
#include <sh4lt/logger/console.hpp>
#include <sh4lt/shtype/shtype.hpp>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

namespace Threedim
{

/**
 * @brief Sh4ltMeshLoader - Loads 3D mesh data from a sh4lt socket
 *
 * This node receives mesh data via sh4lt from Blender.
 * 
 * Expected ShType: "application/x-polymesh"
 * 
 * Binary data format:
 *   Header:
 *     - vert_count: int32
 *     - poly_count: int32
 *   Vertex data (vert_count times):
 *     - position: 3 floats (x, y, z)
 *     - texcoord: 2 floats (u, v)
 *     - normal: 3 floats (nx, ny, nz)
 *   Polygon data (poly_count times):
 *     - num_verts: int32
 *     - indices: num_verts * int32
 */
class Sh4ltMeshLoader
{
public:
  halp_meta(name, "Sh4lt Mesh Loader")
  halp_meta(category, "Visuals/3D")
  halp_meta(c_name, "sh4lt_mesh_loader")
  halp_meta(author, "ossia team")
  halp_meta(manual_url, "https://ossia.io/score-docs/processes/meshes.html#sh4lt-mesh-loader")
  halp_meta(uuid, "a7f3b2c1-8d4e-5f6a-9b0c-1d2e3f4a5b6c")

  struct ins
  {
    struct : halp::lineedit<"Sh4lt Label", "">
    {
      halp_meta(description, "sh4lt label (e.g., mesh_Cube)");
      void update(Sh4ltMeshLoader& self) { self.on_label_changed(); }
    } label;

    PositionControl position;
    RotationControl rotation;
    ScaleControl scale;
  } inputs;

  struct
  {
    struct : halp::mesh
    {
      halp_meta(name, "Geometry");
      std::vector<halp::dynamic_geometry> mesh;
    } geometry;
  } outputs;

  Sh4ltMeshLoader();
  ~Sh4ltMeshLoader();

  void operator()();

private:
  void on_label_changed();
  void start_receiver();
  void stop_receiver();
  void setup_format(const sh4lt::ShType& shtype);
  void on_data(void* data, std::size_t size);
  void rebuild_geometry();
  void apply_pending_data();

  // Sh4lt receiver
  std::shared_ptr<sh4lt::logger::Console> m_logger;
  std::optional<sh4lt::Follower> m_receiver;
  std::string m_current_label;

  // Thread-safe data exchange
  std::mutex m_mutex;
  float_vec m_pending_complete;
  mesh m_pending_meshinfo{};
  std::atomic<bool> m_has_pending_data{false};
  bool m_format_valid = false;

  // Current mesh data
  std::vector<mesh> m_meshinfo;
  float_vec m_complete;
};

} // namespace Threedim
