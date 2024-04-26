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

#include <visp3/ar/vpPanda3DRGBRenderer.h>

#if defined(VISP_HAVE_PANDA3D)

const char *vpPanda3DRGBRenderer::COOK_TORRANCE_VERT = R"shader(
#version 330

in vec3 p3d_Normal;
in vec4 p3d_Vertex;

out vec3 oNormal;
uniform mat3 p3d_NormalMatrix;
uniform mat4 p3d_ModelViewMatrix;
uniform mat4 p3d_ModelViewProjectionMatrix;

void main()
{
  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
  // View space is Z-up right handed, flip z and y
  oNormal = p3d_NormalMatrix * normalize(p3d_Normal);
  oNormal.yz = oNormal.zy;
  oNormal.y = -oNormal.y;
  vec4 cs_position = p3d_ModelViewMatrix * p3d_Vertex;
}
)shader";

const char *vpPanda3DRGBRenderer::COOK_TORRANCE_FRAG = R"shader(
#version 330

in vec3 oNormal;

out vec4 p3d_FragData;

uniform struct {
  vec4 ambient;
} p3d_LightModel;

uniform struct p3d_LightSourceParameters {
  // Primary light color.
  vec4 color;

  // View-space position.  If w=0, this is a directional light, with the xyz
  // being -direction.
  vec4 position;

  // constant, linear, quadratic attenuation in one vector
  vec3 attenuation;

} p3d_LightSource[4];

void main()
{
  vec3 n = normalize(oNormal);
  p3d_FragData = p3d_LightModel.ambient * vec4(1.f, 0.f, 0.f,0.f);
  for(int i = 0; i < p3d_LightSource.length(); ++i) {
    vec3 aFac = p3d_LightSource[i].attenuation;
    float attenuation = 1.f / (aFac[0] + aFac[1] * dist, aFac[2] * dist * dist);
    //p3d_FragData += p3d_LightSource[i].color;
  }
}
)shader";


void vpPanda3DRGBRenderer::getRender(vpImage<vpRGBa> &I) const
{
  I.resize(m_colorTexture->get_y_size(), m_colorTexture->get_x_size());
  unsigned char *data = (unsigned char *)(&(m_colorTexture->get_ram_image().front()));
  // BGRA order in panda3d
  for (unsigned int i = 0; i < I.getSize(); ++i) {
    I.bitmap[i].B = data[i * 4];
    I.bitmap[i].G = data[i * 4 + 1];
    I.bitmap[i].R = data[i * 4 + 2];
    I.bitmap[i].A = data[i * 4 + 3];
  }
  // memcpy(I.bitmap, data, sizeof(unsigned char) * I.getSize() * 4);
}

void vpPanda3DRGBRenderer::setupScene()
{
  vpPanda3DBaseRenderer::setupScene();
  setLightableScene(m_renderRoot);
  PT(Shader) shader;
  shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                                    COOK_TORRANCE_VERT,
                                    COOK_TORRANCE_FRAG);
  m_renderRoot.set_shader(shader);
  //m_renderRoot.set_shader_auto();
}

void vpPanda3DRGBRenderer::setupRenderTarget()
{
  if (m_window == nullptr) {
    throw vpException(vpException::fatalError, "Cannot setup render target when window is null");
  }
  FrameBufferProperties fbp;
  fbp.set_rgb_color(true);
  fbp.set_float_depth(false);
  fbp.set_float_color(false);
  fbp.set_depth_bits(16);
  fbp.set_rgba_bits(8, 8, 8, 8);


  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());

  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window | GraphicsPipe::BF_resizeable;
  GraphicsOutput *windowOutput = m_window->get_graphics_output();
  GraphicsEngine *engine = windowOutput->get_engine();
  GraphicsStateGuardian *gsg = windowOutput->get_gsg();
  GraphicsPipe *pipe = windowOutput->get_pipe();
  m_colorBuffer = engine->make_output(pipe, "Color Buffer", -100,
                                      fbp, win_prop, flags,
                                      gsg, windowOutput);
  if (m_colorBuffer == nullptr) {
    throw vpException(vpException::fatalError, "Could not create color buffer");
  }
  m_buffers.push_back(m_colorBuffer);
  m_colorBuffer->set_inverted(gsg->get_copy_texture_inverted());
  m_colorTexture = new Texture();
  fbp.setup_color_texture(m_colorTexture);
  m_colorBuffer->add_render_texture(m_colorTexture, GraphicsOutput::RenderTextureMode::RTM_copy_ram);
  m_colorBuffer->set_clear_color(LColor(0.f));
  m_colorBuffer->set_clear_color_active(true);
  DisplayRegion *region = m_colorBuffer->make_display_region();
  if (region == nullptr) {
    throw vpException(vpException::fatalError, "Could not create display region");
  }
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

#endif
