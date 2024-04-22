/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

const char *vpPanda3DGeometryRenderer::SHADER_VERT_NORMAL_AND_DEPTH_CAMERA = R"shader(
#version 140
in vec3 p3d_Normal;
varying vec3 oNormal;
uniform mat3 p3d_NormalMatrix;
void main()
{
    gl_Position = ftransform();
    // View space is Z-up right handed, flip z and y
    oNormal = p3d_NormalMatrix * normalize(p3d_Normal);
    //oNormal.yz = -oNormal.zy;
}
)shader";

const char *vpPanda3DGeometryRenderer::SHADER_VERT_NORMAL_AND_DEPTH_WORLD = R"shader(
#version 120

varying vec3 oNormal;

void main()
{
    gl_Position = ftransform();
    oNormal = normalize(gl_Normal);
}
)shader";

const char *vpPanda3DGeometryRenderer::SHADER_FRAG_NORMAL_AND_DEPTH = R"shader(
#version 120

varying vec3 oNormal;

void main()
{
  vec3 n = normalize(oNormal);
  //if (!gl_FrontFacing)
      //n = -n;
  float fDepth = gl_FragCoord.z;

  gl_FragColor = vec4(n, fDepth);
}
)shader";

std::string renderTypeToName(vpPanda3DGeometryRenderer::vpRenderType type)
{
  switch (type) {
  case vpPanda3DGeometryRenderer::vpRenderType::WORLD_NORMALS:
    return "normals-world";
  case vpPanda3DGeometryRenderer::vpRenderType::CAMERA_NORMALS:
    return "normals-camera";
  default:
    return "";
  }
}

vpPanda3DGeometryRenderer::vpPanda3DGeometryRenderer(vpRenderType renderType) : vpPanda3DBaseRenderer(renderTypeToName(renderType)), m_renderType(renderType) { }

void vpPanda3DGeometryRenderer::setupScene()
{
  m_renderRoot = m_window->get_render().attach_new_node(m_name);
  PT(Shader) shader;
  if (m_renderType == WORLD_NORMALS) {
    shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                                      SHADER_VERT_NORMAL_AND_DEPTH_WORLD,
                                      SHADER_FRAG_NORMAL_AND_DEPTH);
  }
  else if (m_renderType == CAMERA_NORMALS) {
    shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                                      SHADER_VERT_NORMAL_AND_DEPTH_CAMERA,
                                      SHADER_FRAG_NORMAL_AND_DEPTH);
  }
  m_renderRoot.set_shader(shader);
}

void vpPanda3DGeometryRenderer::setupRenderTarget()
{
  FrameBufferProperties fbp;
  fbp.set_rgb_color(true);
  fbp.set_float_depth(false);
  fbp.set_float_color(true);
  fbp.set_depth_bits(16);
  fbp.set_rgba_bits(32, 32, 32, 32);

  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());

  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window;

  GraphicsEngine *engine = m_window->get_graphics_output()->get_engine();
  GraphicsPipe *pipe = m_window->get_graphics_output()->get_pipe();
  m_normalDepthBuffer = engine->make_output(pipe, "My Buffer", -100, fbp, win_prop, flags,
                                            m_window->get_graphics_output()->get_gsg(),
                                            m_window->get_graphics_output());
  m_normalDepthTexture = new Texture();
  fbp.setup_color_texture(m_normalDepthTexture);
  m_normalDepthBuffer->add_render_texture(m_normalDepthTexture, GraphicsOutput::RenderTextureMode::RTM_copy_ram);
  m_normalDepthBuffer->set_clear_color(LColor(0.f));
  m_normalDepthBuffer->set_clear_color_active(true);
  DisplayRegion *region = m_normalDepthBuffer->make_display_region();
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

void vpPanda3DGeometryRenderer::getRender(vpImage<vpRGBf> &normals, vpImage<float> &depth) const
{
  normals.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());
  depth.resize(m_normalDepthTexture->get_y_size(), m_normalDepthTexture->get_x_size());
  float *data = (float *)(&(m_normalDepthTexture->get_ram_image().front()));

//#pragma omp parallel for simd
  for (unsigned int i = 0; i < normals.getRows() * normals.getCols(); ++i) {
    normals.bitmap[i].B = (data[i * 4]);
    normals.bitmap[i].G = (data[i * 4 + 1]);
    normals.bitmap[i].R = (data[i * 4 + 2]);
    depth.bitmap[i] = (data[i * 4 + 3]);
  }
}

#endif
