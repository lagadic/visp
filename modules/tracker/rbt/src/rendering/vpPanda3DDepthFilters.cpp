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

#include <visp3/rbt/vpPanda3DDepthFilters.h>

#if defined(VISP_HAVE_PANDA3D)

#include "graphicsOutput.h"
#include "graphicsEngine.h"
#include "windowFramework.h"

#include <visp3/gui/vpDisplayFactory.h>

BEGIN_VISP_NAMESPACE

const std::string vpPanda3DDepthGaussianBlur::FRAGMENT_SHADER =
"#version 330\n"
"\n"
"in vec2 texcoords;\n"
"\n"
"uniform sampler2D p3d_Texture0;\n"
"uniform vec2 dp; // 1 divided by number of pixels\n"
"\n"
"const float kernel[25] = float[25](\n"
"  2, 4, 5, 4, 2,\n"
"  4, 9, 12, 9, 4,\n"
"  5, 12, 15, 12, 5,\n"
"  4, 9, 12, 9, 4,\n"
"  2, 4, 5, 4, 2\n"
");\n"
"const float normalize = 1 / 159.0;\n"
"\n"
"vec2 offset[25] = vec2[25](\n"
"  vec2(-2*dp.x,-2*dp.y),  vec2(-dp.x,-2*dp.y),  vec2(0,-2*dp.y),    vec2(dp.x,-2*dp.y), vec2(2*dp.x,-2*dp.y),\n"
"  vec2(-2*dp.x,-dp.y),    vec2(-dp.x, -dp.y),   vec2(0.0, -dp.y),   vec2(dp.x, -dp.y),  vec2(2*dp.x,-dp.y),\n"
"  vec2(-2*dp.x,0.0),      vec2(-dp.x, 0.0),     vec2(0.0, 0.0),     vec2(dp.x, 0.0),    vec2(2*dp.x,0.0),\n"
"  vec2(-2*dp.x, dp.y),    vec2(-dp.x, dp.y),    vec2(0.0, dp.y),    vec2(dp.x, dp.y),   vec2(2*dp.x, dp.y),\n"
"  vec2(-2*dp.x, 2*dp.y),  vec2(-dp.x, 2*dp.y),  vec2(0.0, 2*dp.y),  vec2(dp.x, 2*dp.y), vec2(2*dp.x, 2*dp.y)\n"
");\n"
"\n"
"out vec4 p3d_FragData;\n"
"\n"
"void main() {\n"
"  float v = 0.f;\n"
"\n"
"  for(int i = 0; i < 25; ++i) {\n"
"    v += kernel[i] * texture(p3d_Texture0, texcoords + offset[i]).a;\n"
"  }\n"
"  p3d_FragData.a = v * normalize;\n"
"}\n"
")\n";

vpPanda3DDepthGaussianBlur::vpPanda3DDepthGaussianBlur(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, vpPanda3DDepthGaussianBlur::FRAGMENT_SHADER)
{ }

FrameBufferProperties vpPanda3DDepthGaussianBlur::getBufferProperties() const
{
  FrameBufferProperties fbp;
  fbp.set_depth_bits(0);
  fbp.set_rgba_bits(0, 0, 0, 32);
  fbp.set_float_color(true);
  return fbp;
}

void vpPanda3DDepthGaussianBlur::getRender(vpImage<unsigned char> &I) const
{
  vpPanda3DPostProcessFilter::getRenderBasic(I);
}

const std::string vpPanda3DDepthCannyFilter::FRAGMENT_SHADER =
"#version 330\n"
"#define PI 3.1415926538\n"
"\n"
"in vec2 texcoords;\n"
"\n"
"uniform sampler2D p3d_Texture0;\n"
"uniform vec2 dp; // 1 divided by number of pixels\n"
"uniform float edgeThreshold;\n"
"\n"
"const float kernel[9] = float[9](\n"
"  0.0, 1.0, 0.0,\n"
"  1.0,-4.0, 1.0,\n"
"  0.0, 1.0, 0.0\n"
");\n"
"\n"
"const float kernel_h[9] = float[9](\n"
"  -1.0, 0.0, 1.0,\n"
"  -2.0, 0.0, 2.0,\n"
"  -1.0, 0.0, 1.0\n"
");\n"
"\n"
"const float kernel_v[9] = float[9](\n"
"  -1.0, -2.0, -1.0,\n"
"  0.0, 0.0, 0.0,\n"
"  1.0, 2.0, 1.0\n"
");\n"
"\n"
"vec2 offset[9] = vec2[9](\n"
"  vec2(-dp.x, -dp.y), vec2(0.0, -dp.y), vec2(dp.x, -dp.y),\n"
"  vec2(-dp.x, 0.0),   vec2(0.0, 0.0),   vec2(dp.x, 0.0),\n"
"  vec2(-dp.x, dp.y),  vec2(0.0, dp.y),  vec2(dp.x, dp.y)\n"
");\n"
"\n"
"float textureValues[9];\n"
"\n"
"out vec3 p3d_FragData;\n"
"\n"
"void main() {\n"
"  if(texture(p3d_Texture0, texcoords).a == 0) {\n"
"    p3d_FragData = vec3(0.f, 0.f, 0.f);\n"
"  } else {\n"
"    float sum = 0.f;\n"
"    for(int i = 0; i < 9; ++i) {\n"
"      float pix = texture(p3d_Texture0, texcoords + offset[i]).a;\n"
"      pix = (pix < 1e-5f ? 1000.f: pix);\n"
"      textureValues[i] = pix;\n"
"      sum += pix * kernel[i];\n"
"    }\n"
"    if(abs(sum) > edgeThreshold) {\n"
"      float sum_h = 0.f;\n"
"      float sum_v = 0.f;\n"
"      for(int i = 0; i < 9; ++i) {\n"
"        float pix = textureValues[i];\n"
"        sum_h += pix * kernel_h[i];\n"
"        sum_v += pix * kernel_v[i];\n"
"      }\n"
"      vec2 orientationAndValid = (sum_h != 0.f) ? vec2(atan(sum_v, -sum_h), 1.f) : vec2(0.f, 0.f);\n"
"      float orientation = (orientationAndValid.x + PI) / (PI * 2);\n"
"      p3d_FragData = vec3(orientation, orientationAndValid.y, orientation);\n"
"    } else {\n"
"      p3d_FragData = vec3(0.f, 0.f, 0.f);\n"
"    }\n"
"  }\n"
"}\n";

vpPanda3DDepthCannyFilter::vpPanda3DDepthCannyFilter(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, float edgeThreshold)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, vpPanda3DDepthCannyFilter::FRAGMENT_SHADER), m_edgeThreshold(edgeThreshold)
{ }

void vpPanda3DDepthCannyFilter::setupScene()
{
  vpPanda3DPostProcessFilter::setupScene();
  m_renderRoot.set_shader_input("edgeThreshold", LVector2f(m_edgeThreshold));
}

void vpPanda3DDepthCannyFilter::setEdgeThreshold(float edgeThreshold)
{
  m_edgeThreshold = edgeThreshold;
  m_renderRoot.set_shader_input("edgeThreshold", LVector2f(m_edgeThreshold));
}

FrameBufferProperties vpPanda3DDepthCannyFilter::getBufferProperties() const
{
  FrameBufferProperties fbp;
  fbp.set_depth_bits(0);
  fbp.set_rgba_bits(8, 8, 8, 0);
  fbp.set_alpha_bits(0);

  fbp.set_rgb_color(false);

  fbp.set_float_color(false);
  return fbp;
}

PointerTo<Texture> vpPanda3DDepthCannyFilter::setupTexture(const FrameBufferProperties &fbp) const
{
  PointerTo<Texture> tex = new Texture();
  fbp.setup_color_texture(tex);
  tex->set_format(Texture::F_rgb);
  tex->set_component_type(Texture::T_unsigned_byte);
  return tex;
}

void vpPanda3DDepthCannyFilter::getRender(vpImage<float> &I, vpImage<unsigned char> &valid) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());

  valid.resize(I.getHeight(), I.getWidth());
  const unsigned numComponents = m_texture->get_num_components();
  int rowIncrement = I.getWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image
  uint8_t *data = (uint8_t *)(&(m_texture->get_ram_image().front()));
  // Panda3D stores data upside down
  data += rowIncrement * (I.getHeight() - 1);
  rowIncrement = -rowIncrement;
  if (numComponents != 3) {
    throw;
  }
  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    float *colorRow = I[i];
    unsigned char *validRow = valid[i];
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      colorRow[j] = static_cast<float>(data[j * numComponents]) / std::numeric_limits<uint8_t>::max() * M_PI * 2 - M_PI;
      validRow[j] = static_cast<unsigned char>((data[j * numComponents + 1] > 0) * 255);
    }
    data += rowIncrement;
  }
}

void vpPanda3DDepthCannyFilter::getRender(vpImage<float> &I, vpImage<unsigned char> &valid, const vpRect &bb, unsigned int h, unsigned w) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(h, w, 0.f);
  valid.resize(I.getHeight(), I.getWidth(), 0);

  const unsigned top = static_cast<unsigned int>(std::max(0.0, bb.getTop()));
  const unsigned left = static_cast<unsigned int>(std::max(0.0, bb.getLeft()));
  const unsigned numComponents = m_texture->get_num_components();
  const unsigned rowIncrement = m_renderParameters.getImageWidth() * numComponents;

  const uint8_t *data = (uint8_t *)(&(m_texture->get_ram_image().front()));
  data += rowIncrement * (m_renderParameters.getImageHeight() - 1);
  if (numComponents != 3) {
    throw vpException(vpException::dimensionError, "Expected panda texture to have 3 components!");
  }
  for (unsigned int i = 0; i < m_renderParameters.getImageHeight(); ++i) {
    const uint8_t *rowData = data - i * rowIncrement;
    float *colorRow = I[top + i];
    unsigned char *validRow = valid[top + i];
    for (unsigned int j = 0; j < m_renderParameters.getImageWidth(); ++j) {
      colorRow[left + j] = static_cast<float>(rowData[j * numComponents]) / std::numeric_limits<uint8_t>::max() * M_PI * 2 - M_PI;
      validRow[left + j] = static_cast<unsigned char>((rowData[j * numComponents + 1] > 0) * 255);
    }
  }
}

END_VISP_NAMESPACE

#endif
