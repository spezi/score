#include "Sh4ltMeshLoader.hpp"

#include <QDebug>
#include <QString>

#include <cstring>
#include <vector>

namespace Threedim
{

Sh4ltMeshLoader::Sh4ltMeshLoader()
    : m_logger{std::make_shared<sh4lt::logger::Console>()}
{
}

Sh4ltMeshLoader::~Sh4ltMeshLoader()
{
  stop_receiver();
}

void Sh4ltMeshLoader::operator()()
{
  // Check for pending data and apply it
  apply_pending_data();
}

void Sh4ltMeshLoader::on_label_changed()
{
  std::string new_label = inputs.label.value;

  if(new_label == m_current_label)
    return;

  stop_receiver();

  if(!new_label.empty())
  {
    m_current_label = new_label;
    start_receiver();
  }
}

void Sh4ltMeshLoader::start_receiver()
{
  if(m_current_label.empty())
    return;

  try
  {
    const auto path = sh4lt::ShType::get_path(m_current_label, sh4lt::ShType::default_group());

    m_receiver.emplace(
        path,
        [this](void* data, size_t size, const sh4lt::Time::info_t*) { on_data(data, size); },
        [this](const sh4lt::ShType& shtype) { setup_format(shtype); },
        []() { /* on disconnect */ },
        m_logger);

    qDebug() << "Sh4ltMeshLoader: Started receiving from label" 
             << QString::fromStdString(m_current_label);
  }
  catch(const std::exception& e)
  {
    qWarning() << "Sh4ltMeshLoader: Failed to start receiver:" << e.what();
  }
}

void Sh4ltMeshLoader::stop_receiver()
{
  m_receiver.reset();
  m_current_label.clear();
  m_format_valid = false;
}

void Sh4ltMeshLoader::setup_format(const sh4lt::ShType& shtype)
{
  const auto media_type = shtype.media();
  qDebug() << "Sh4ltMeshLoader: Received format:" << QString::fromStdString(media_type);

  // Check if this is polymesh data from Blender
  if(media_type.find("application/x-polymesh") != std::string::npos)
  {
    m_format_valid = true;
    qDebug() << "Sh4ltMeshLoader: Valid polymesh format detected";
  }
  else
  {
    m_format_valid = false;
    qWarning() << "Sh4ltMeshLoader: Unknown format, expected application/x-polymesh";
  }
}

void Sh4ltMeshLoader::on_data(void* data, std::size_t size)
{
  if(!m_format_valid)
    return;

  // Minimum size: 2 ints (header)
  if(size < 2 * sizeof(int32_t))
  {
    qWarning() << "Sh4ltMeshLoader: Data too small";
    return;
  }

  const uint8_t* ptr = static_cast<const uint8_t*>(data);
  const uint8_t* end = ptr + size;

  // Read header
  int32_t vert_count, poly_count;
  std::memcpy(&vert_count, ptr, sizeof(int32_t));
  ptr += sizeof(int32_t);
  std::memcpy(&poly_count, ptr, sizeof(int32_t));
  ptr += sizeof(int32_t);

  if(vert_count <= 0 || poly_count <= 0)
    return;

  // Vertex data: 8 floats per vertex (x, y, z, u, v, nx, ny, nz)
  const std::size_t vertex_data_size = vert_count * 8 * sizeof(float);
  if(ptr + vertex_data_size > end)
  {
    qWarning() << "Sh4ltMeshLoader: Not enough data for vertices";
    return;
  }

  const float* vertices = reinterpret_cast<const float*>(ptr);
  ptr += vertex_data_size;

  // Read polygon data and count triangles needed
  const uint8_t* poly_ptr = ptr;
  int32_t total_triangles = 0;
  
  for(int32_t p = 0; p < poly_count; p++)
  {
    if(poly_ptr + sizeof(int32_t) > end)
    {
      qWarning() << "Sh4ltMeshLoader: Polygon data truncated";
      return;
    }
    
    int32_t num_verts;
    std::memcpy(&num_verts, poly_ptr, sizeof(int32_t));
    poly_ptr += sizeof(int32_t);
    
    if(num_verts >= 3)
      total_triangles += (num_verts - 2);  // Fan triangulation
    
    // Skip vertex indices
    poly_ptr += num_verts * sizeof(int32_t);
  }

  if(total_triangles <= 0)
    return;

  // Now build triangulated mesh
  // Output format: [all positions][all texcoords][all normals]
  const int32_t tri_vert_count = total_triangles * 3;
  const int64_t total_floats = tri_vert_count * (3 + 2 + 3);

  {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_pending_complete.resize(total_floats);

    float* dst_pos = m_pending_complete.data();
    float* dst_tex = dst_pos + tri_vert_count * 3;
    float* dst_norm = dst_tex + tri_vert_count * 2;

    // Reset polygon pointer
    poly_ptr = ptr;
    int32_t out_vert = 0;

    for(int32_t p = 0; p < poly_count; p++)
    {
      int32_t num_verts;
      std::memcpy(&num_verts, poly_ptr, sizeof(int32_t));
      poly_ptr += sizeof(int32_t);

      // Read all vertex indices for this polygon
      std::vector<int32_t> indices(num_verts);
      for(int32_t i = 0; i < num_verts; i++)
      {
        std::memcpy(&indices[i], poly_ptr, sizeof(int32_t));
        poly_ptr += sizeof(int32_t);
      }

      // Fan triangulation: for polygon with vertices 0,1,2,3,4
      // triangles are: (0,1,2), (0,2,3), (0,3,4)
      for(int32_t t = 0; t < num_verts - 2; t++)
      {
        int32_t tri_indices[3] = { indices[0], indices[t + 1], indices[t + 2] };

        for(int k = 0; k < 3; k++)
        {
          int32_t idx = tri_indices[k];
          if(idx < 0 || idx >= vert_count)
            continue;

          const float* v = vertices + idx * 8;

          // Position (x, y, z) - Transform from Blender Z-up to ossia Y-up
          // Blender: (x, y, z) -> ossia: (x, z, -y)
          dst_pos[out_vert * 3 + 0] = v[0];
          dst_pos[out_vert * 3 + 1] = v[2];
          dst_pos[out_vert * 3 + 2] = -v[1];

          // Texcoord (u, v) - unchanged
          dst_tex[out_vert * 2 + 0] = v[3];
          dst_tex[out_vert * 2 + 1] = v[4];

          // Normal (nx, ny, nz) - same transformation as position
          dst_norm[out_vert * 3 + 0] = v[5];
          dst_norm[out_vert * 3 + 1] = v[7];
          dst_norm[out_vert * 3 + 2] = -v[6];

          out_vert++;
        }
      }
    }

    // Build mesh info
    m_pending_meshinfo = mesh{};
    m_pending_meshinfo.vertices = out_vert;
    m_pending_meshinfo.pos_offset = 0;
    m_pending_meshinfo.texcoord = true;
    m_pending_meshinfo.texcoord_offset = tri_vert_count * 3;
    m_pending_meshinfo.normals = true;
    m_pending_meshinfo.normal_offset = tri_vert_count * 3 + tri_vert_count * 2;
    m_pending_meshinfo.colors = false;
    m_pending_meshinfo.points = false;

    m_has_pending_data.store(true, std::memory_order_release);
  }
}

void Sh4ltMeshLoader::apply_pending_data()
{
  if(!m_has_pending_data.load(std::memory_order_acquire))
    return;

  {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::swap(m_complete, m_pending_complete);
    m_meshinfo.clear();
    m_meshinfo.push_back(m_pending_meshinfo);

    m_has_pending_data.store(false, std::memory_order_release);
  }

  rebuild_geometry();
}

void Sh4ltMeshLoader::rebuild_geometry()
{
  if(m_meshinfo.empty())
    return;

  outputs.geometry.mesh.clear();

  for(const auto& m : m_meshinfo)
  {
    if(m.vertices <= 0)
      continue;

    halp::dynamic_geometry geom;

    geom.buffers.clear();
    geom.bindings.clear();
    geom.attributes.clear();
    geom.input.clear();

    geom.topology = halp::primitive_topology::triangles;
    geom.cull_mode = halp::cull_mode::back;
    geom.front_face = halp::front_face::counter_clockwise;
    geom.index = {};
    geom.vertices = m.vertices;

    geom.buffers.push_back(halp::geometry_cpu_buffer{
        .data = m_complete.data(),
        .size = int64_t(m_complete.size() * sizeof(float)),
        .dirty = true});

    // Bindings - Position
    geom.bindings.push_back(halp::geometry_binding{
        .stride = 3 * sizeof(float),
        .step_rate = 1,
        .classification = halp::binding_classification::per_vertex});

    // Bindings - Texcoord
    if(m.texcoord)
    {
      geom.bindings.push_back(halp::geometry_binding{
          .stride = 2 * sizeof(float),
          .step_rate = 1,
          .classification = halp::binding_classification::per_vertex});
    }

    // Bindings - Normal
    if(m.normals)
    {
      geom.bindings.push_back(halp::geometry_binding{
          .stride = 3 * sizeof(float),
          .step_rate = 1,
          .classification = halp::binding_classification::per_vertex});
    }

    // Attributes - Position
    geom.attributes.push_back(halp::geometry_attribute{
        .binding = 0,
        .location = halp::attribute_location::position,
        .format = halp::attribute_format::float3,
        .offset = 0});

    int binding_idx = 1;

    // Attributes - Texcoord
    if(m.texcoord)
    {
      geom.attributes.push_back(halp::geometry_attribute{
          .binding = binding_idx++,
          .location = halp::attribute_location::tex_coord,
          .format = halp::attribute_format::float2,
          .offset = 0});
    }

    // Attributes - Normal
    if(m.normals)
    {
      geom.attributes.push_back(halp::geometry_attribute{
          .binding = binding_idx++,
          .location = halp::attribute_location::normal,
          .format = halp::attribute_format::float3,
          .offset = 0});
    }

    // Vertex input - Position
    geom.input.push_back(
        halp::geometry_input{.buffer = 0, .offset = m.pos_offset * (int)sizeof(float)});

    // Vertex input - Texcoord
    if(m.texcoord)
    {
      geom.input.push_back(
          halp::geometry_input{.buffer = 0, .offset = m.texcoord_offset * (int)sizeof(float)});
    }

    // Vertex input - Normal
    if(m.normals)
    {
      geom.input.push_back(
          halp::geometry_input{.buffer = 0, .offset = m.normal_offset * (int)sizeof(float)});
    }

    outputs.geometry.mesh.push_back(std::move(geom));
    outputs.geometry.dirty_mesh = true;
  }
}

} // namespace Threedim
