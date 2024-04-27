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
out vec4 viewVertex;

uniform mat3 p3d_NormalMatrix;
uniform mat4 p3d_ModelViewMatrix;
uniform mat4 p3d_ModelViewProjectionMatrix;

in vec2 p3d_MultiTexCoord0;
out vec2 texcoords;


void main()
{
  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
  oNormal = p3d_NormalMatrix * normalize(p3d_Normal);
  viewVertex = p3d_ModelViewMatrix * p3d_Vertex;
  texcoords = p3d_MultiTexCoord0;
}
)shader";

const char *vpPanda3DRGBRenderer::COOK_TORRANCE_FRAG = R"shader(
#version 330

#define M_PI 3.1415926535897932384626433832795

in vec3 oNormal;
in vec4 viewVertex;

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

uniform struct p3d_MaterialParameters {
  vec4 ambient;
  vec4 diffuse;
  vec4 emission;
  vec3 specular;
  float shininess;

  // These properties are new in 1.10.
  vec4 baseColor;
  float roughness;
  float metallic;
  float refractiveIndex;
} p3d_Material;

in vec2 texcoords;
uniform sampler2D p3d_Texture0;


float D(float roughness2, float hn)
{
  return (1.f / (M_PI * roughness2)) * pow(hn, (2.f / roughness2) - 2.f);
}

float G(float hn, float nv, float nl, float vh)
{
  return min(1.0, min((2.f * hn * nv) / vh, (2.f * hn * nl) / vh));
}

float computeF0(float ior)
{
  return pow(ior - 1.0, 2) / pow(ior + 1.0, 2.0);
}
float F(float F0, float vh)
{
  return F0 + (1.f - F0) * pow(1.f - vh, 5);
}


void main()
{
  vec3 n = normalize(oNormal); // normalized normal vector
  vec3 v = normalize(-viewVertex.xyz); // normalized view vector
  float nv = max(0.f, dot(n, v));
  float roughness2 = pow(p3d_Material.roughness, 2);
  float F0 = computeF0(p3d_Material.refractiveIndex);

  p3d_FragData = p3d_LightModel.ambient * p3d_Material.ambient;


  for(int i = 0; i < p3d_LightSource.length(); ++i) {

    vec3 lf = p3d_LightSource[i].position.xyz - (viewVertex.xyz * p3d_LightSource[i].position.w);
    float lightDist = length(lf);
    vec3 l = normalize(lf); // normalized light vector
    vec3 h = normalize(l + v); // half way vector
    float hn = dot(h, n);
    float nl = max(0.f, dot(n, l));
    float vh = max(0.f, dot(v, h));

    vec3 aFac = p3d_LightSource[i].attenuation;
    float attenuation = 1.f / (aFac[0] + aFac[1] * lightDist + aFac[2] * lightDist * lightDist);

    float DV = D(roughness2, hn);
    float GV = G(hn, nv, nl, vh);
    float FV = F(F0, vh);

    float rs = (DV * GV * FV) / (4.f * nl * nv);

    float shininess = p3d_Material.shininess;
    p3d_FragData += (p3d_LightSource[i].color * attenuation) * nl * (p3d_Material.diffuse + shininess * rs * vec4(p3d_Material.specular, 1.f));


    //p3d_FragData.r = attenuation;
    //p3d_FragData = diffuse;

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
