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

#include <visp3/core/vpConfig.h>                 // for BEGIN_VISP_NAMESPACE

#if defined(VISP_HAVE_PANDA3D)

#include <vector>                                // for vector
#include <string.h>                              // for memcpy
#include <sstream>                               // for basic_ostream, basic...
#include <string>                                // for basic_string, string

#include <cardMaker.h>                           // for CardMaker
#include <orthographicLens.h>                    // for OrthographicLens


#include <visp3/ar/vpPanda3DRGBRenderer.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>      // for vpPanda3DBaseRenderer
#include <visp3/ar/vpPanda3DRenderParameters.h>  // for vpPanda3DRenderParam...
#include <visp3/core/vpException.h>              // for vpException
#include <visp3/core/vpHomogeneousMatrix.h>      // for vpHomogeneousMatrix
#include <visp3/core/vpImage.h>                  // for vpImage
#include <visp3/core/vpRGBa.h>                   // for vpRGBa

BEGIN_VISP_NAMESPACE

const char *vpPanda3DRGBRenderer::COOK_TORRANCE_VERT =
"#version 330\n"
"in vec3 p3d_Normal;\n"
"in vec4 p3d_Vertex;\n"
"out vec3 oNormal;\n"
"out vec4 viewVertex;\n"
"uniform mat3 p3d_NormalMatrix;\n"
"uniform mat4 p3d_ModelViewMatrix;\n"
"uniform mat4 p3d_ModelViewProjectionMatrix;\n"
"in vec2 p3d_MultiTexCoord0;\n"
"out vec2 texcoords;\n"
"out vec3 F0;\n"
"uniform struct p3d_MaterialParameters {\n"
"  vec4 ambient;\n"
"  vec4 diffuse;\n"
"  vec4 emission;\n"
"  vec3 specular;\n"
"  float shininess;\n"
"  // These properties are new in 1.10.\n"
"  vec4 baseColor;\n"
"  float roughness;\n"
"  float metallic;\n"
"  float refractiveIndex;\n"
"} p3d_Material;\n"
"vec3 computeF0(float ior, float metallic, vec3 baseColor)\n"
"{\n"
"  float F0f = pow(abs((1.0 - ior) / (1.0 + ior)), 2.0);\n"
"  vec3 F0 = vec3(F0f, F0f, F0f);\n"
"  return mix(F0, baseColor, metallic);\n"
"}\n"
"void main()\n"
"{\n"
"  gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;\n"
"  oNormal = p3d_NormalMatrix * normalize(p3d_Normal);\n"
"  viewVertex = p3d_ModelViewMatrix * p3d_Vertex;\n"
"  texcoords = p3d_MultiTexCoord0;\n"
"  F0 = computeF0(p3d_Material.refractiveIndex, p3d_Material.metallic, p3d_Material.baseColor.xyz);\n"
"}\n";

const char *vpPanda3DRGBRenderer::COOK_TORRANCE_FRAG =
"// Version 330, specified when generating shader\n"
"#define M_PI 3.1415926535897932384626433832795\n"
"in vec3 oNormal;\n"
"in vec4 viewVertex;\n"
"in vec3 F0;\n"
"out vec4 p3d_FragData;\n"
"uniform struct {\n"
"  vec4 ambient;\n"
"} p3d_LightModel;\n"
"uniform struct p3d_LightSourceParameters {\n"
"  // Primary light color.\n"
"  vec4 color;\n"
"  // View-space position.  If w=0, this is a directional light, with the xyz\n"
"  // being -direction.\n"
"  vec4 position;\n"
"  // constant, linear, quadratic attenuation in one vector\n"
"  vec3 attenuation;\n"
"} p3d_LightSource[4];\n"
"uniform struct p3d_MaterialParameters {\n"
"  vec4 ambient;\n"
"  vec4 diffuse;\n"
"  vec4 emission;\n"
"  vec3 specular;\n"
"  float shininess;\n"
"  // These properties are new in 1.10.\n"
"  vec4 baseColor;\n"
"  float roughness;\n"
"  float metallic;\n"
"  float refractiveIndex;\n"
"} p3d_Material;\n"
"in vec2 texcoords;\n"
"#ifdef HAS_TEXTURE\n"
"uniform sampler2D p3d_Texture0;\n"
"#endif\n"
"float D(float roughness2, float hn)\n"
"{\n"
"  return (1.f / (M_PI * roughness2)) * pow(hn, (2.f / roughness2) - 2.f);\n"
"}\n"
"float G(float hn, float nv, float nl, float vh)\n"
"{\n"
"  return min(1.0, min((2.f * hn * nv) / vh, (2.f * hn * nl) / vh));\n"
"}\n"
"vec3 F(vec3 F0, float vh)\n"
"{\n"
"  return F0 + (vec3(1.f, 1.f, 1.f) - F0) * pow(1.f - vh, 5);\n"
"}\n"
"void main()\n"
"{\n"
"  vec3 n = normalize(oNormal); // normalized normal vector\n"
"  vec3 v = normalize(-viewVertex.xyz); // normalized view vector\n"
"  float nv = max(0.f, dot(n, v));\n"
"  float roughness2 = clamp(pow(p3d_Material.roughness, 2), 0.01, 0.99);\n"

"  #ifdef HAS_TEXTURE\n"
"    vec4 baseColor = texture(p3d_Texture0, texcoords);\n"
"   vec4 ambientColor = baseColor;\n"
"  #else\n"
"    vec4 ambientColor = p3d_Material.ambient;\n"
"    vec4 baseColor = p3d_Material.baseColor;\n"
"  #endif\n"
"  p3d_FragData = p3d_LightModel.ambient * baseColor;\n"
"  for(int i = 0; i < p3d_LightSource.length(); ++i) {\n"
"    vec3 lf = p3d_LightSource[i].position.xyz - (viewVertex.xyz * p3d_LightSource[i].position.w);\n"
"    float lightDist = length(lf);\n"
"    vec3 l = normalize(lf); // normalized light vector\n"
"    vec3 h = normalize(l + v); // half way vector\n"
"    float hn = dot(h, n);\n"
"    float nl = max(0.f, dot(n, l));\n"
"    float vh = max(0.f, dot(v, h));\n"
"    vec3 aFac = p3d_LightSource[i].attenuation;\n"
"    float attenuation = 1.f / (aFac[0] + aFac[1] * lightDist + aFac[2] * lightDist * lightDist);\n"
"    vec3 FV = F(F0, vh);\n"
"    vec3 kd = (1.f - p3d_Material.metallic) *  (1.f - FV) * (1.f / M_PI);\n"
"    #ifdef SPECULAR\n"
"      vec3 specularColor = vec3(0.f, 0.f, 0.f);\n"
"      if(nl > 0.f && nv > 0.f) {\n"
"        float DV = D(roughness2, hn);\n"
"        float GV = G(hn, nv, nl, vh);\n"
"        vec3 rs = (DV * GV * FV) / (4.f * nl * nv);\n"
"        specularColor = rs * p3d_Material.specular;\n"
"      }\n"
"    #else\n"
"      vec3 specularColor = vec3(0.0, 0.0, 0.0);\n"
"    #endif\n"
"    p3d_FragData += (p3d_LightSource[i].color * attenuation) * nl * (baseColor * vec4(kd, 1.f) + vec4(specularColor, 1.f));\n"
"  }\n"
"  p3d_FragData.bgra = p3d_FragData;\n"
"}\n";

std::string vpPanda3DRGBRenderer::makeFragmentShader(bool hasTexture, bool specular)
{
  std::stringstream ss;
  ss << "#version 330" << std::endl;
  if (hasTexture) {
    ss << "#define HAS_TEXTURE 1" << std::endl;
  }
  if (specular) {
    ss << "#define SPECULAR 1" << std::endl;
  }
  else {
    ss << "#undef SPECULAR" << std::endl;
  }
  ss << vpPanda3DRGBRenderer::COOK_TORRANCE_FRAG;
  return ss.str();
}

void vpPanda3DRGBRenderer::addNodeToScene(const NodePath &object)
{
  NodePath objectInScene = object.copy_to(m_renderRoot);
  objectInScene.set_name(object.get_name());
  TextureCollection txs = objectInScene.find_all_textures();
  bool hasTexture = (static_cast<unsigned int>(txs.size()) > 0);
  // gltf2bam and other tools may store some fallback textures. We shouldnt use them as they whiten the result
  if (hasTexture) {
    std::vector<std::string> fallbackNames { "pbr-fallback", "normal-fallback", "emission-fallback" };
    unsigned int numMatches = 0;
    for (const std::string &fallbackName: fallbackNames) {
      numMatches += static_cast<int>(txs.find_texture(fallbackName) != nullptr);
    }
    hasTexture = (static_cast<unsigned int>(txs.size()) > numMatches); // Some textures are not fallback textures
  }

  PT(Shader) shader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                                    COOK_TORRANCE_VERT,
                                    makeFragmentShader(hasTexture, m_showSpeculars));

  objectInScene.set_shader(shader);

  setNodePose(objectInScene, vpHomogeneousMatrix());
}

void vpPanda3DRGBRenderer::setBackgroundImage(const vpImage<vpRGBa> &background)
{

  if (m_display2d == nullptr) {
    CardMaker cm("card");
    cm.set_frame_fullscreen_quad();

    NodePath myCamera2d(new Camera("myCam2d"));
    PT(OrthographicLens) lens = new OrthographicLens();
    lens->set_film_size(2, 2);
    lens->set_near_far(-1000, 1000);
    lens->set_film_offset(0, 0);
    ((Camera *)myCamera2d.node())->set_lens(lens);

    NodePath myRender2d("myRender2d");
    myRender2d.set_depth_test(false);
    myRender2d.set_depth_write(false);
    myCamera2d.reparent_to(myRender2d);
    m_backgroundImage = myRender2d.attach_new_node(cm.generate());

    m_display2d = m_colorBuffer->make_display_region();
    m_display2d->set_sort(-100);
    m_display2d->set_camera(myCamera2d);
  }
  if (m_backgroundTexture == nullptr) {
    m_backgroundTexture = new Texture();
  }
  m_backgroundImage.set_texture(m_backgroundTexture);
  m_backgroundTexture->setup_2d_texture(background.getWidth(), background.getHeight(),
                                    Texture::ComponentType::T_unsigned_byte,
                                    Texture::Format::F_rgba8);
  //m_backgroundTexture = TexturePool::load_texture("/home/sfelton/IMG_20230221_165330430.jpg");
  unsigned char *data = (unsigned char *)m_backgroundTexture->modify_ram_image();

  for (unsigned int i = 0; i < background.getHeight(); ++i) {
    const vpRGBa *srcRow = background[background.getHeight() - (i + 1)];
    unsigned char *destRow = data + i * background.getWidth() * 4;
    for (unsigned int j = 0; j < background.getWidth(); ++j) {
      destRow[j * 4] = srcRow[j].B;
      destRow[j * 4 + 1] = srcRow[j].G;
      destRow[j * 4 + 2] = srcRow[j].R;
      destRow[j * 4 + 3] = srcRow[j].A;
    }
  }
}

void vpPanda3DRGBRenderer::getRender(vpImage<vpRGBa> &I) const
{
  I.resize(m_colorTexture->get_y_size(), m_colorTexture->get_x_size());
  unsigned char *data = (unsigned char *)(&(m_colorTexture->get_ram_image().front()));
  int rowIncrement = I.getWidth() * 4;
  // Panda3D stores the image using the OpenGL convention (origin is bottom left),
  // while we store data with origin as upper left. We copy with a flip
  data = data + rowIncrement * (I.getHeight() - 1);
  rowIncrement = -rowIncrement;

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    vpRGBa *colorRow = I[i];

    memcpy((unsigned char *)(colorRow), data, sizeof(unsigned char) * 4 * I.getWidth());
    // for (unsigned int j = 0; j < I.getWidth(); ++j) {
    //   // BGRA order in panda3d
    //   colorRow[j].R = data[j * 4];
    //   colorRow[j].G = data[j * 4 + 1];
    //   colorRow[j].B = data[j * 4 + 2];
    //   colorRow[j].A = data[j * 4 + 3];
    // }
    data += rowIncrement;
  }
}

void vpPanda3DRGBRenderer::setupScene()
{
  vpPanda3DBaseRenderer::setupScene();
  setLightableScene(m_renderRoot);
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
  fbp.set_srgb_color(true);

  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());

  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window | GraphicsPipe::BF_resizeable;
  GraphicsOutput *windowOutput = m_window->get_graphics_output();
  GraphicsEngine *engine = windowOutput->get_engine();
  GraphicsStateGuardian *gsg = windowOutput->get_gsg();
  GraphicsPipe *pipe = windowOutput->get_pipe();
  m_colorBuffer = engine->make_output(pipe, "Color Buffer", m_renderOrder,
                                      fbp, win_prop, flags,
                                      gsg, windowOutput);
  if (m_colorBuffer == nullptr) {
    throw vpException(vpException::fatalError, "Could not create color buffer");
  }
  m_buffers.push_back(m_colorBuffer);
  //m_colorBuffer->set_inverted(gsg->get_copy_texture_inverted());
  m_colorTexture = new Texture();
  fbp.setup_color_texture(m_colorTexture);
  //m_colorTexture->set_format(Texture::Format::F_srgb_alpha);
  m_colorBuffer->add_render_texture(m_colorTexture, GraphicsOutput::RenderTextureMode::RTM_copy_texture);
  m_colorBuffer->set_clear_color(LColor(0.f));
  m_colorBuffer->set_clear_color_active(true);
  DisplayRegion *region = m_colorBuffer->make_display_region();
  if (region == nullptr) {
    throw vpException(vpException::fatalError, "Could not create display region");
  }
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpPanda3DRGBRenderer.cpp.o) has no symbols
void dummy_vpPanda3DRGBRenderer() { };

#endif
