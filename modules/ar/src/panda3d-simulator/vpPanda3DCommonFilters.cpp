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


vpPanda3DLuminanceFilter::vpPanda3DLuminanceFilter(const std::string &name, std::shared_ptr<vpPanda3DRGBRenderer> inputRenderer, bool isOutput)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, std::string(vpPanda3DLuminanceFilter::FRAGMENT_SHADER))
{ }
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
  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());
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


const char *vpPanda3DGaussianBlur::FRAGMENT_SHADER = R"shader(
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
    v += kernel[i] * texture(p3d_Texture0, texcoords + offset[i]).b ;
  }
  p3d_FragData.b = v * normalize;
}
)shader";

vpPanda3DGaussianBlur::vpPanda3DGaussianBlur(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, vpPanda3DGaussianBlur::FRAGMENT_SHADER)
{ }

FrameBufferProperties vpPanda3DGaussianBlur::getBufferProperties() const
{
  FrameBufferProperties fbp;
  fbp.set_depth_bits(0);
  fbp.set_rgba_bits(0, 0, 8, 0);
  fbp.set_float_color(false);
  return fbp;
}

void vpPanda3DGaussianBlur::getRender(vpImage<unsigned char> &I) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }
  unsigned indexMultiplier = m_texture->get_num_components(); // we ask for only 8 bits image, but we may get an rgb image
  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());
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

const char *vpPanda3DCanny::FRAGMENT_SHADER = R"shader(
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


out vec4 p3d_FragData;

void main() {
  float sum = 0.f;
  for(int i = 0; i < 9; ++i) {
    float pix = texture(p3d_Texture0, texcoords + offset[i]).b;
    sum += pix * kernel[i];
  }
  if(abs(sum * 255.f) > edgeThreshold) {
    float sum_h = 0.f;
    float sum_v = 0.f;
    for(int i = 0; i < 9; ++i) {
      float pix = texture(p3d_Texture0, texcoords + offset[i]).b;
      sum_h += pix * kernel_h[i];
      sum_v += pix * kernel_v[i];
    }

    vec2 orientationAndValid = sum_h * sum_h + sum_v *  sum_v > 0 ? vec2(atan(sum_v/sum_h), 1.f) : vec2(0.f, 0.f);
    p3d_FragData = vec4(sum_h, sum_v, orientationAndValid.x, orientationAndValid.y);
  } else {
    p3d_FragData = vec4(0.f, 0.f, 0.f, 0.f);
  }
}
)shader";

vpPanda3DCanny::vpPanda3DCanny(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, float edgeThreshold)
  : vpPanda3DPostProcessFilter(name, inputRenderer, isOutput, vpPanda3DCanny::FRAGMENT_SHADER), m_edgeThreshold(edgeThreshold)
{ }

void vpPanda3DCanny::setupScene()
{
  vpPanda3DPostProcessFilter::setupScene();
  m_renderRoot.set_shader_input("edgeThreshold", LVector2f(m_edgeThreshold));
}

void vpPanda3DCanny::setEdgeThreshold(float edgeThreshold)
{
  m_edgeThreshold = edgeThreshold;
  m_renderRoot.set_shader_input("edgeThreshold", LVector2f(m_edgeThreshold));
}


FrameBufferProperties vpPanda3DCanny::getBufferProperties() const
{
  FrameBufferProperties fbp;
  fbp.set_depth_bits(0);
  fbp.set_rgba_bits(32, 32, 32, 32);
  fbp.set_float_color(true);
  return fbp;
}

void vpPanda3DCanny::getRender(vpImage<vpRGBf> &I) const
{
  if (!m_isOutput) {
    throw vpException(vpException::fatalError, "Tried to fetch output of a postprocessing filter that was configured as an intermediate output");
  }
  unsigned indexMultiplier = m_texture->get_num_components(); // we ask for only 8 bits image, but we may get an rgb image
  I.resize(m_renderParameters.getImageHeight(), m_renderParameters.getImageWidth());
  float *data = (float *)(&(m_texture->get_ram_image().front()));
  for (unsigned int i = 0; i < I.getSize(); ++i) {
    I.bitmap[i].B = (data[i * 4]);
    I.bitmap[i].G = (data[i * 4 + 1]);
    I.bitmap[i].R = (data[i * 4 + 2]);
  }
}

#endif
