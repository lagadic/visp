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

BEGIN_VISP_NAMESPACE

const char *vpPanda3DDepthGaussianBlur::FRAGMENT_SHADER = R"shader(
#version 330

in vec2 texcoords;

uniform sampler2D p3d_Texture0;
uniform vec2 dp; // 1 divided by number of pixels

const float kernel[25] = float[25](
  2, 4, 5, 4, 2,
  4, 9, 12, 9, 4,
  5, 12, 15, 12, 5,
  4, 9, 12, 9, 4,
  2, 4, 5, 4, 2
);
const float normalize = 1 / 159.0;

vec2 offset[25] = vec2[25](
  vec2(-2*dp.x,-2*dp.y),  vec2(-dp.x,-2*dp.y),  vec2(0,-2*dp.y),    vec2(dp.x,-2*dp.y), vec2(2*dp.x,-2*dp.y),
  vec2(-2*dp.x,-dp.y),    vec2(-dp.x, -dp.y),   vec2(0.0, -dp.y),   vec2(dp.x, -dp.y),  vec2(2*dp.x,-dp.y),
  vec2(-2*dp.x,0.0),      vec2(-dp.x, 0.0),     vec2(0.0, 0.0),     vec2(dp.x, 0.0),    vec2(2*dp.x,0.0),
  vec2(-2*dp.x, dp.y),    vec2(-dp.x, dp.y),    vec2(0.0, dp.y),    vec2(dp.x, dp.y),   vec2(2*dp.x, dp.y),
  vec2(-2*dp.x, 2*dp.y),  vec2(-dp.x, 2*dp.y),  vec2(0.0, 2*dp.y),  vec2(dp.x, 2*dp.y), vec2(2*dp.x, 2*dp.y)
);

out vec4 p3d_FragData;

void main() {
  float v = 0.f;

  for(int i = 0; i < 25; ++i) {
    v += kernel[i] * texture(p3d_Texture0, texcoords + offset[i]).a;
  }
  p3d_FragData.a = v * normalize;
}
)shader";

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

const char *vpPanda3DDepthCannyFilter::FRAGMENT_SHADER = R"shader(
#version 330

in vec2 texcoords;

uniform sampler2D p3d_Texture0;
uniform vec2 dp; // 1 divided by number of pixels
uniform float edgeThreshold;


const float kernel[9] = float[9](
  0.0, 1.0, 0.0,
  1.0,-4.0, 1.0,
  0.0, 1.0, 0.0
);

const float kernel_h[9] = float[9](
  -1.0, 0.0, 1.0,
  -2.0, 0.0, 2.0,
  -1.0, 0.0, 1.0
);

const float kernel_v[9] = float[9](
  -1.0, -2.0, -1.0,
  0.0, 0.0, 0.0,
  1.0, 2.0, 1.0
);

vec2 offset[9] = vec2[9](
  vec2(-dp.x, -dp.y), vec2(0.0, -dp.y), vec2(dp.x, -dp.y),
  vec2(-dp.x, 0.0),   vec2(0.0, 0.0),   vec2(dp.x, 0.0),
  vec2(-dp.x, dp.y),  vec2(0.0, dp.y),  vec2(dp.x, dp.y)
);

float textureValues[9];


out vec4 p3d_FragData;

void main() {
  if(texture(p3d_Texture0, texcoords).a == 0) {
    p3d_FragData = vec4(0.f, 0.f, 0.f, 0.f);
  } else {

    float sum = 0.f;
    for(int i = 0; i < 9; ++i) {
      float pix = texture(p3d_Texture0, texcoords + offset[i]).a;
      pix = (pix < 1e-5f ? 1000.f: pix);
      textureValues[i] = pix;
      sum += pix * kernel[i];
    }
    if(abs(sum) > edgeThreshold) {
      float sum_h = 0.f;
      float sum_v = 0.f;
      for(int i = 0; i < 9; ++i) {
        float pix = textureValues[i];
        sum_h += pix * kernel_h[i];
        sum_v += pix * kernel_v[i];
      }
      float norm = sqrt(sum_v * sum_v + sum_h * sum_h);
      vec2 orientationAndValid = (sum_h != 0.f) ? vec2(atan(sum_v, -sum_h), 1.f) : vec2(0.f, 0.f);
      p3d_FragData.bgra = vec4(sum_h, sum_v, orientationAndValid.x, orientationAndValid.y);
    } else {
      p3d_FragData = vec4(0.f, 0.f, 0.f, 0.f);
    }
  }
}
)shader";

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
  fbp.set_rgba_bits(32, 32, 32, 32);
  fbp.set_float_color(true);
  return fbp;
}

void vpPanda3DDepthCannyFilter::getRender(vpImage<vpRGBf> &I, vpImage<unsigned char> &valid) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());

  valid.resize(I.getHeight(), I.getWidth());
  const unsigned numComponents = m_texture->get_num_components();
  int rowIncrement = I.getWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image
  float *data = (float *)(&(m_texture->get_ram_image().front()));
  // Panda3D stores data upside down
  data += rowIncrement * (I.getHeight() - 1);
  rowIncrement = -rowIncrement;
  if (numComponents != 4) {
    throw;
  }
  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    vpRGBf *colorRow = I[i];
    unsigned char *validRow = valid[i];
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      colorRow[j].R = data[j * numComponents];
      colorRow[j].G = data[j * numComponents + 1];
      colorRow[j].B = data[j * numComponents + 2];
      validRow[j] = static_cast<unsigned char>(data[j * numComponents + 3]);
    }
    data += rowIncrement;
  }
}

void vpPanda3DDepthCannyFilter::getRender(vpImage<vpRGBf> &I, vpImage<unsigned char> &valid, const vpRect &bb, unsigned int h, unsigned w) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(h, w, 0.f);
  valid.resize(I.getHeight(), I.getWidth(), 0);

  const unsigned top = static_cast<unsigned int>(std::max(0.0, bb.getTop()));
  const unsigned left = static_cast<unsigned int>(std::max(0.0, bb.getLeft()));
  const unsigned numComponents = m_texture->get_num_components();
  const unsigned rowIncrement = m_renderParameters.getImageWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image

  const float *data = (float *)(&(m_texture->get_ram_image().front()));
  data += rowIncrement * (m_renderParameters.getImageHeight() - 1);
  if (numComponents != 4) {
    throw vpException(vpException::dimensionError, "Expected panda texture to have 4 components!");
  }
  for (unsigned int i = 0; i < m_renderParameters.getImageHeight(); ++i) {
    const float *rowData = data - i * rowIncrement;
    vpRGBf *colorRow = I[top + i];
    unsigned char *validRow = valid[top + i];
    for (unsigned int j = 0; j < m_renderParameters.getImageWidth(); ++j) {
      colorRow[left + j].R = rowData[j * numComponents];
      colorRow[left + j].G = rowData[j * numComponents + 1];
      colorRow[left + j].B = rowData[j * numComponents + 2];
      validRow[left + j] = static_cast<unsigned char>(rowData[j * numComponents + 3]);
    }
  }
}

END_VISP_NAMESPACE

#endif
