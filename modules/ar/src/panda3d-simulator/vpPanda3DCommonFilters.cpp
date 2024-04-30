#include <visp3/ar/vpPanda3DCommonFilters.h>
#include <visp3/ar/vpPanda3DRGBRenderer.h>

#if defined(VISP_HAVE_PANDA3D)

const char *vpPanda3DLuminanceFilter::FRAGMENT_SHADER = R"shader(
#version 330

in vec2 texcoords;

uniform sampler2D p3d_Texture0;

out vec4 p3d_FragData;

void main() {
  vec4 v = texture(p3d_Texture0, texcoords);
  p3d_FragData.b = 0.299 * v.r + 0.587 * v.g + 0.114 * v.b;
}

)shader";


vpPanda3DLuminanceFilter::vpPanda3DLuminanceFilter(const std::string &name, std::shared_ptr<vpPanda3DRGBRenderer> &inputRenderer, bool isOutput)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, std::string(vpPanda3DLuminanceFilter::FRAGMENT_SHADER))
{

}
FrameBufferProperties vpPanda3DLuminanceFilter::getBufferProperties() const
{
  FrameBufferProperties fbp;
  fbp.set_depth_bits(0);
  fbp.set_rgba_bits(0, 0, 8, 0);
  fbp.set_float_color(false);
  return fbp;
}
void vpPanda3DLuminanceFilter::getRender(vpImage<unsigned char> &I) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }
  unsigned indexMultiplier = m_texture->get_num_components(); // we ask for only 8 bits image, but we may get an rgb image
  I.resize(m_texture->get_y_size(), m_texture->get_x_size());
  unsigned char *data = (unsigned char *)(&(m_texture->get_ram_image().front()));
  if (indexMultiplier != 1) {
    for (unsigned int i = 0; i < I.getSize(); ++i) {
      I.bitmap[i] = data[i * indexMultiplier];
    }
  }
  else {
    memcpy(I.bitmap, &data[0], I.getSize() * sizeof(unsigned char));
  }
}



#endif
