/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include "windowFramework.h"
#include "graphicsOutput.h"
#include "graphicsEngine.h"
#include "graphicsBuffer.h"

BEGIN_VISP_NAMESPACE

const std::string SHADER_VERT_NORMAL_AND_DEPTH_CAMERA =
"#version 330\n"
"in vec3 p3d_Normal;\n"
"in vec4 p3d_Vertex;\n"
"uniform mat3 p3d_NormalMatrix;\n"
"uniform mat4 p3d_ModelViewMatrix;\n"
"uniform mat4 p3d_ModelViewProjectionMatrix;\n"
"out vec3 oNormal;\n"
"out float distToCamera;\n"
"void main()\n"
"{\n"
"  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;\n"
"  oNormal = p3d_NormalMatrix * normalize(p3d_Normal);\n"
"  vec4 cs_position = p3d_ModelViewMatrix * p3d_Vertex;\n"
"  distToCamera = -cs_position.z;\n"
"}\n";

const std::string SHADER_VERT_NORMAL_AND_DEPTH_OBJECT =
"#version 330\n"
"in vec3 p3d_Normal;\n"
"in vec4 p3d_Vertex;\n"
"uniform mat4 p3d_ModelViewMatrix;\n"
"uniform mat4 p3d_ModelViewProjectionMatrix;\n"
"out vec3 oNormal;\n"
"out float distToCamera;\n"
"void main()\n"
"{\n"
"  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;\n"
"  oNormal = vec3(p3d_Normal.x, -p3d_Normal.z, p3d_Normal.y);\n"
"  vec4 cs_position = p3d_ModelViewMatrix * p3d_Vertex;\n"
"  distToCamera = -cs_position.z;\n"
"}\n";

const std::string SHADER_FRAG_NORMAL_AND_DEPTH =
"#version 330\n"
"in vec3 oNormal;\n"
"in float distToCamera;\n"
"out vec4 p3d_FragData;\n"
"void main()\n"
"{\n"
"  p3d_FragData.bgra = vec4(normalize(oNormal), distToCamera);\n"
"}\n";


const std::string SHADER_FRAG_NORMAL_AND_DEPTH_FAST =
R"(#version 330
in vec3 oNormal;
in float distToCamera;
out vec4 p3d_FragData;
uniform float nearV;
uniform float farV;
void main()
{
  //float depthV = distToCamera <= nearV ? 0.0 : (distToCamera - nearV) / (farV - nearV);
  float depthV = distToCamera / farV;
  vec3 n = normalize(oNormal);
  n = (n + 1.0) / 2.0;
  p3d_FragData.bgra = vec4(n, depthV);
}
)";


std::string renderTypeToName(vpPanda3DGeometryRenderer::vpRenderType type)
{
  switch (type) {
  case vpPanda3DGeometryRenderer::vpRenderType::OBJECT_NORMALS:
    return "normals-world";
  case vpPanda3DGeometryRenderer::vpRenderType::CAMERA_NORMALS:
    return "normals-camera";
  default:
    return "";
  }
}

void vpPanda3DGeometryRenderer::beforeFrameRendered()
{
  if (m_fast) {
    m_renderRoot.set_shader_input("nearV", LVector2f(m_renderParameters.getNearClippingDistance()));
    m_renderRoot.set_shader_input("farV", LVector2f(m_renderParameters.getFarClippingDistance()));
  }

}

vpPanda3DGeometryRenderer::vpPanda3DGeometryRenderer(vpRenderType renderType, bool fast) : vpPanda3DBaseRenderer(renderTypeToName(renderType)), m_renderType(renderType), m_fast(fast) { }

void vpPanda3DGeometryRenderer::setupScene()
{
  m_renderRoot = m_window->get_render().attach_new_node(m_name);
  PT(Shader) shader;
  if (m_renderType == OBJECT_NORMALS) {
    shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                          SHADER_VERT_NORMAL_AND_DEPTH_OBJECT,
                          m_fast ? SHADER_FRAG_NORMAL_AND_DEPTH_FAST : SHADER_FRAG_NORMAL_AND_DEPTH);
  }
  else if (m_renderType == CAMERA_NORMALS) {
    shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                          SHADER_VERT_NORMAL_AND_DEPTH_CAMERA,
                          m_fast ? SHADER_FRAG_NORMAL_AND_DEPTH_FAST : SHADER_FRAG_NORMAL_AND_DEPTH);
  }
  m_renderRoot.set_shader(shader);
}

void vpPanda3DGeometryRenderer::setupRenderTarget()
{
  if (m_window == nullptr) {
    throw vpException(vpException::fatalError, "Cannot setup render target when window is null");
  }
  FrameBufferProperties fbp;
  if (!m_fast) {
    fbp.set_rgb_color(true);
    fbp.set_float_depth(false);
    fbp.set_float_color(true);
    fbp.set_depth_bits(16);
    fbp.set_rgba_bits(32, 32, 32, 32);
  }
  else {
    fbp.set_rgb_color(true);
    fbp.set_float_depth(false);
    fbp.set_float_color(false);
    fbp.set_depth_bits(16);
    fbp.set_rgba_bits(8, 8, 8, 8);
  }

  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());
  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window  | GraphicsPipe::BF_resizeable;
  GraphicsOutput *windowOutput = m_window->get_graphics_output();
  GraphicsEngine *engine = windowOutput->get_engine();
  GraphicsPipe *pipe = windowOutput->get_pipe();

  static int id = 0;
  m_normalDepthBuffer = engine->make_output(pipe, renderTypeToName(m_renderType) + std::to_string(id), m_renderOrder, fbp, win_prop, flags,
                                            windowOutput->get_gsg(), windowOutput);
  m_normalDepthTexture = new Texture("geometry texture " + std::to_string(id));
  ++id;
  if (m_normalDepthBuffer == nullptr) {
    throw vpException(vpException::fatalError, "Could not create geometry info buffer");
  }
  // if (!m_normalDepthBuffer->is_valid()) {
  //   throw vpException(vpException::fatalError, "Geometry info buffer is invalid");
  // }
  m_buffers.push_back(m_normalDepthBuffer);
  m_normalDepthBuffer->set_inverted(windowOutput->get_gsg()->get_copy_texture_inverted());
  // fbp.setup_color_texture(m_normalDepthTexture);
  if (!m_fast) {
    m_normalDepthTexture->set_format(Texture::F_rgba32);
  }
  else {
    m_normalDepthTexture->set_component_type(Texture::T_unsigned_byte);
    m_normalDepthTexture->set_format(Texture::F_rgba);
  }
  m_normalDepthBuffer->add_render_texture(m_normalDepthTexture, GraphicsOutput::RenderTextureMode::RTM_copy_texture, GraphicsOutput::RenderTexturePlane::RTP_color);
  m_normalDepthBuffer->set_clear_color(LColor(0.f));
  m_normalDepthBuffer->set_clear_color_active(true);

  DisplayRegion *region = m_normalDepthBuffer->make_display_region();
  if (region == nullptr) {
    throw vpException(vpException::fatalError, "Could not create display region");
  }
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

void vpPanda3DGeometryRenderer::getRender(vpImage<vpRGBf> &normals, vpImage<float> &depth) const
{
  if (m_fast) {
    throw vpException(vpException::notImplementedError, "Cannot extract render with this function when fast option is enabled");
  }
  normals.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());
  depth.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());
  if (m_normalDepthTexture->get_component_type() != Texture::T_float) {
    throw vpException(vpException::badValue, "Unexpected data type in normals texture");
  }

  int rowIncrement = normals.getWidth() * 4;
  float *data = (float *)(&(m_normalDepthTexture->get_ram_image().front()));
  data = data + rowIncrement * (normals.getHeight() - 1);
  rowIncrement = -rowIncrement;

  for (unsigned int i = 0; i < normals.getHeight(); ++i) {
    vpRGBf *normalRow = normals[i];
    float *depthRow = depth[i];
    for (unsigned int j = 0; j < normals.getWidth(); ++j) {
      normalRow[j].R = (data[j * 4]);
      normalRow[j].G = (data[j * 4 + 1]);
      normalRow[j].B = (data[j * 4 + 2]);
      depthRow[j] = (data[j * 4 + 3]);
    }
    data += rowIncrement;
  }
}

void vpPanda3DGeometryRenderer::getRender(vpImage<vpRGBf> &normals, vpImage<float> &depth, const vpRect &bb, unsigned int h, unsigned w) const
{
  normals.resize(h, w);
  // memset(normals.bitmap, 0, normals.getSize() * sizeof(vpRGBf));
  depth.resize(normals.getHeight(), normals.getWidth(), 0.f);
  // memset(depth.bitmap, 0, normals.getSize());

  const unsigned top = static_cast<unsigned int>(std::max(0.0, bb.getTop()));
  const unsigned left = static_cast<unsigned int>(std::max(0.0, bb.getLeft()));
  const unsigned numComponents = m_normalDepthTexture->get_num_components();
  const unsigned rowIncrement = m_renderParameters.getImageWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image


  int image_width = static_cast<int>(m_renderParameters.getImageWidth());
  if (!m_fast) {
    const float *data = (float *)(&(m_normalDepthTexture->get_ram_image().front()));
    // Panda3D stores data upside down
    data += rowIncrement * (m_renderParameters.getImageHeight() - 1);
    if (numComponents != 4) {
      throw vpException(vpException::dimensionError, "Expected panda texture to have 4 components!");
    }
    if (m_normalDepthTexture->get_component_type() != Texture::T_float) {
      throw vpException(vpException::badValue, "Unexpected data type in normals texture");
    }
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < m_renderParameters.getImageHeight(); ++i) {
      const float *const rowData = data - i * rowIncrement;
      vpRGBf *normalRow = normals[top + i];
      float *depthRow = depth[top + i];
      for (int j = 0; j < image_width; ++j) {
        int left_j = left + j;
        int j_4 = j * 4;
        normalRow[left_j].R = (rowData[j_4]);
        normalRow[left_j].G = (rowData[j_4 + 1]);
        normalRow[left_j].B = (rowData[j_4 + 2]);
        depthRow[left_j] = (rowData[j_4 + 3]);
      }
    }
  }
  else {
    float nearV = m_renderParameters.getNearClippingDistance();
    float farV = m_renderParameters.getFarClippingDistance();
    float ratio = 1.f / (farV - nearV);
    using T = uint8_t;
    Texture::ComponentType expectedComponent = Texture::T_unsigned_byte;
    float maxValue = static_cast<float>(std::numeric_limits<T>::max());
    const T *data = (T *)(&(m_normalDepthTexture->get_ram_image().front()));
    // Panda3D stores data upside down
    data += rowIncrement * (m_renderParameters.getImageHeight() - 1);
    if (numComponents != 4) {
      throw vpException(vpException::dimensionError, "Expected panda texture to have 4 components!");
    }
    if (m_normalDepthTexture->get_component_type() != expectedComponent) {
      throw vpException(vpException::badValue, "Unexpected data type in normals texture");
    }

    for (unsigned int i = 0; i < m_renderParameters.getImageHeight(); ++i) {
      const T *const rowData = data - i * rowIncrement;
      vpRGBf *normalRow = normals[top + i];
      float *depthRow = depth[top + i];
      std::array<float, 3> v;
      for (int j = 0; j < image_width; ++j) {
        int left_j = left + j;
        int j_4 = j * 4;
        v[0] = (static_cast<float>(rowData[j_4]) / maxValue) * 2.f - 1.f;
        v[1] = (static_cast<float>(rowData[j_4 + 1]) / maxValue) * 2.f - 1.f;
        v[2] = (static_cast<float>(rowData[j_4 + 2]) / maxValue) * 2.f - 1.f;
        // const double norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

        normalRow[left_j].R = v[0];
        normalRow[left_j].G = v[1];
        normalRow[left_j].B = v[2];
        depthRow[left_j] = (static_cast<float>(rowData[j_4 + 3]) / maxValue) * farV;
      }
    }
  }

}

void vpPanda3DGeometryRenderer::getRender(vpImage<vpRGBf> &normals) const
{
  if (m_fast) {
    throw vpException(vpException::notImplementedError, "Cannot extract render with this function when fast option is enabled");
  }
  normals.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());
  if (m_normalDepthTexture->get_component_type() != Texture::T_float) {
    throw vpException(vpException::badValue, "Unexpected data type in normals texture");
  }

  int rowIncrement = normals.getWidth() * 4;
  float *data = (float *)(&(m_normalDepthTexture->get_ram_image().front()));
  data = data + rowIncrement * (normals.getHeight() - 1);
  rowIncrement = -rowIncrement;
  for (unsigned int i = 0; i < normals.getHeight(); ++i) {
    vpRGBf *normalRow = normals[i];
    for (unsigned int j = 0; j < normals.getWidth(); ++j) {
      normalRow[j].R = (data[j * 4]);
      normalRow[j].G = (data[j * 4 + 1]);
      normalRow[j].B = (data[j * 4 + 2]);
    }
    data += rowIncrement;
  }
}

void vpPanda3DGeometryRenderer::getRender(vpImage<float> &depth) const
{
  if (m_fast) {
    throw vpException(vpException::notImplementedError, "Cannot extract render with this function when fast option is enabled");
  }
  depth.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());

  if (m_normalDepthTexture->get_component_type() != Texture::T_float) {
    throw vpException(vpException::badValue, "Unexpected data type in normals texture");
  }

  int rowIncrement = depth.getWidth() * 4;
  float *data = (float *)(&(m_normalDepthTexture->get_ram_image().front()));
  data = data + rowIncrement * (depth.getHeight() - 1);
  rowIncrement = -rowIncrement;

  for (unsigned int i = 0; i < depth.getHeight(); ++i) {
    float *depthRow = depth[i];
    for (unsigned int j = 0; j < depth.getWidth(); ++j) {
      depthRow[j] = (data[j * 4 + 3]);
    }
    data += rowIncrement;
  }
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpPanda3DGeometryRenderer.cpp.o) has no symbols
void dummy_vpPanda3DGeometryRenderer() { }

#endif
