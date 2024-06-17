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

#include <visp3/ar/vpPanda3DPostProcessFilter.h>

#if defined(VISP_HAVE_PANDA3D)

#include <lightRampAttrib.h>

BEGIN_VISP_NAMESPACE
const char *vpPanda3DPostProcessFilter::FILTER_VERTEX_SHADER = R"shader(
#version 330
in vec4 p3d_Vertex;
uniform mat4 p3d_ModelViewProjectionMatrix;
in vec2 p3d_MultiTexCoord0;
out vec2 texcoords;

void main()
{
  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
  texcoords = p3d_MultiTexCoord0;
}
)shader";

void vpPanda3DPostProcessFilter::setupScene()
{
  CardMaker cm("cm");
  cm.set_frame_fullscreen_quad();
  m_renderRoot = NodePath(cm.generate()); // Render root is a 2D rectangle
  m_renderRoot.set_depth_test(false);
  m_renderRoot.set_depth_write(false);
  GraphicsOutput *buffer = m_inputRenderer->getMainOutputBuffer();
  if (buffer == nullptr) {
    throw vpException(vpException::fatalError,
    "Cannot add a postprocess filter to a renderer that does not define getMainOutputBuffer()");
  }
  m_shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                        FILTER_VERTEX_SHADER,
                        m_fragmentShader);
  m_renderRoot.set_shader(m_shader);
  m_renderRoot.set_shader_input("dp", LVector2f(1.0 / buffer->get_texture()->get_x_size(), 1.0 / buffer->get_texture()->get_y_size()));
  std::cout << m_fragmentShader << std::endl;
  m_renderRoot.set_texture(buffer->get_texture());
  m_renderRoot.set_attrib(LightRampAttrib::make_identity());
}

void vpPanda3DPostProcessFilter::setupCamera()
{
  m_cameraPath = m_window->make_camera();
  m_camera = (Camera *)m_cameraPath.node();
  PT(OrthographicLens) lens = new OrthographicLens();
  lens->set_film_size(2, 2);
  lens->set_film_offset(0, 0);
  lens->set_near_far(-1000, 1000);
  m_camera->set_lens(lens);
  m_cameraPath = m_renderRoot.attach_new_node(m_camera);
  m_camera->set_scene(m_renderRoot);
}

void vpPanda3DPostProcessFilter::setupRenderTarget()
{

  if (m_window == nullptr) {
    throw vpException(vpException::fatalError, "Cannot setup render target when window is null");
  }
  FrameBufferProperties fbp = getBufferProperties();
  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());

  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window | GraphicsPipe::BF_resizeable;
  GraphicsOutput *windowOutput = m_window->get_graphics_output();
  GraphicsEngine *engine = windowOutput->get_engine();
  GraphicsStateGuardian *gsg = windowOutput->get_gsg();
  GraphicsPipe *pipe = windowOutput->get_pipe();
  m_buffer = engine->make_output(pipe, m_name, m_renderOrder,
                                      fbp, win_prop, flags,
                                      gsg, windowOutput);
  if (m_buffer == nullptr) {
    throw vpException(vpException::fatalError, "Could not create buffer");
  }
  m_buffers.push_back(m_buffer);
  //m_buffer->set_inverted(true);
  m_texture = new Texture();
  fbp.setup_color_texture(m_texture);
  m_buffer->add_render_texture(m_texture, m_isOutput ? GraphicsOutput::RenderTextureMode::RTM_copy_ram : GraphicsOutput::RenderTextureMode::RTM_copy_texture);
  m_buffer->set_clear_color(LColor(0.f));
  m_buffer->set_clear_color_active(true);
  DisplayRegion *region = m_buffer->make_display_region();
  if (region == nullptr) {
    throw vpException(vpException::fatalError, "Could not create display region");
  }
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

void vpPanda3DPostProcessFilter::setRenderParameters(const vpPanda3DRenderParameters &params)
{
  m_renderParameters = params;
  unsigned int previousH = m_renderParameters.getImageHeight(), previousW = m_renderParameters.getImageWidth();
  bool resize = previousH != params.getImageHeight() || previousW != params.getImageWidth();

  m_renderParameters = params;
  if (m_window != nullptr) {
    GraphicsOutput *buffer = m_inputRenderer->getMainOutputBuffer();
    m_renderRoot.set_shader_input("dp", LVector2f(1.0 / buffer->get_texture()->get_x_size(), 1.0 / buffer->get_texture()->get_y_size()));
  }
  if (resize) {
    for (GraphicsOutput *buffer: m_buffers) {
      //buffer->get_type().is_derived_from()
      GraphicsBuffer *buf = dynamic_cast<GraphicsBuffer *>(buffer);
      if (buf == nullptr) {
        throw vpException(vpException::fatalError, "Panda3D: could not cast to GraphicsBuffer when rendering.");
      }
      else {
        buf->set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());
      }
    }
  }
}

void vpPanda3DPostProcessFilter::getRenderBasic(vpImage<unsigned char> &I) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());
  const unsigned numComponents = m_texture->get_num_components();
  int rowIncrement = I.getWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image
  unsigned char *data = (unsigned char *)(&(m_texture->get_ram_image().front()));
  // Panda3D stores data upside down
  data += rowIncrement * (I.getHeight() - 1);
  rowIncrement = -rowIncrement;

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    data += rowIncrement;
    unsigned char *colorRow = I[i];
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      colorRow[j] = data[j * numComponents];
    }
  }
}

void vpPanda3DPostProcessFilter::getRenderBasic(vpImage<vpRGBf> &I) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }

  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());
  const unsigned numComponents = m_texture->get_num_components();
  int rowIncrement = I.getWidth() * numComponents; // we ask for only 8 bits image, but we may get an rgb image
  float *data = (float *)(&(m_texture->get_ram_image().front()));
  // Panda3D stores data upside down
  data += rowIncrement * (I.getHeight() - 1);
  rowIncrement = -rowIncrement;

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    data += rowIncrement;
    vpRGBf *colorRow = I[i];
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      colorRow[j].B = data[j * numComponents];
      colorRow[j].G = data[j * numComponents + 1];
      colorRow[j].R = data[j * numComponents + 2];
    }
  }
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpPanda3DPostProcessFilter.cpp.o) has no symbols
void dummy_vpPanda3DPostProcessFilter() { };

#endif
