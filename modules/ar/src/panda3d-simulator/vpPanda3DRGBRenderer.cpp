#include <visp3/ar/vpPanda3DRGBRenderer.h>

#if defined(VISP_HAVE_PANDA3D)


void vpPanda3DRGBRenderer::getRender(vpImage<vpRGBa> &I) const
{
  I.resize(m_colorTexture->get_y_size(), m_colorTexture->get_x_size());
  std::cout << "Before request" << std::endl;
  unsigned char *data = (unsigned char *)(&(m_colorTexture->get_ram_image().front()));
  std::cout << m_colorTexture->get_format() << std::endl;
  // CPTA_uchar p((const unsigned char *)I.bitmap, ((const unsigned char *)I.bitmap) + I.getSize() * 4);
  // m_colorTexture->set_ram_image_as(p, "rgba");
  std::cout << "After request" << std::endl;
  // BGRA order in panda3d
  for (unsigned int i = 0; i < I.getSize(); ++i) {
    I.bitmap[i].B = data[i * 4];
    I.bitmap[i].G = data[i * 4 + 1];
    I.bitmap[i].R = data[i * 4 + 2];
    I.bitmap[i].A = data[i * 4 + 3];
  }
  // memcpy(I.bitmap, data, sizeof(unsigned char) * I.getSize() * 4);
}


void vpPanda3DRGBRenderer::setupRenderTarget()
{
  FrameBufferProperties fbp;
  fbp.set_rgb_color(true);
  fbp.set_float_depth(false);
  fbp.set_float_color(false);
  fbp.set_depth_bits(16);
  fbp.set_rgba_bits(8, 8, 8, 8);

  WindowProperties win_prop;
  win_prop.set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());

  // Don't open a window - force it to be an offscreen buffer.
  int flags = GraphicsPipe::BF_refuse_window;

  GraphicsEngine *engine = m_window->get_graphics_output()->get_engine();
  GraphicsPipe *pipe = m_window->get_graphics_output()->get_pipe();
  m_colorBuffer = engine->make_output(pipe, "Color Buffer", -100, fbp, win_prop, flags,
                                            m_window->get_graphics_output()->get_gsg(),
                                            m_window->get_graphics_output());
  m_colorTexture = new Texture();
  fbp.setup_color_texture(m_colorTexture);
  m_colorBuffer->add_render_texture(m_colorTexture, GraphicsOutput::RenderTextureMode::RTM_copy_ram);
  m_colorBuffer->set_clear_color(LColor(0.f));
  m_colorBuffer->set_clear_color_active(true);
  DisplayRegion *region = m_colorBuffer->make_display_region();
  region->set_camera(m_cameraPath);
  region->set_clear_color(LColor(0.f));
}

#endif
