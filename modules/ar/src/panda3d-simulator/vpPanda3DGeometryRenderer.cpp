#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DGeometryRenderer.h>


const char *vpPanda3DGeometryRenderer::SHADER_VERT_NORMAL_AND_DEPTH = R"shader(
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


void vpPanda3DGeometryRenderer::setupScene()
{
  m_renderRoot = m_window->get_render().attach_new_node(m_name);
  m_normalDepthShader = Shader::make(Shader::ShaderLanguage::SL_GLSL,
                                    SHADER_VERT_NORMAL_AND_DEPTH,
                                    SHADER_FRAG_NORMAL_AND_DEPTH);
  m_renderRoot.set_shader(m_normalDepthShader);


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

#pragma omp parallel for simd
  for (unsigned int i = 0; i < normals.getRows() * normals.getCols(); ++i) {
    normals.bitmap[i].R = (data[i * 4]);
    normals.bitmap[i].G = (data[i * 4 + 1]);
    normals.bitmap[i].B = (data[i * 4 + 2]);
    depth.bitmap[i] = (data[i * 4 + 3]);
  }
}

#endif
