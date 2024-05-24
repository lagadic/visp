#ifndef vpPanda3DPostProcessFilter_h
#define vpPanda3DPostProcessFilter_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include "cardMaker.h"
#include "orthographicLens.h"


/**
 * \ingroup group_ar_renderer_panda3d_filters
 * \brief Base class for postprocessing filters that map the result of a vpPanda3DBaseRenderer to a new image.
 *
 * Unlike 3D renderers, implementations of this class do not have access to 3D information
 * (except if it is the result of the processed image).
 *
 * Implementation wise, the process is the following:
 *
 * - The output texture (retrieved through vpPanda3DBaseRenderer::getMainOutputBuffer) is blitted on a quad,
 * that is placed perfectly in front of the camera.
 * - A shader (given as an argument to the constructor) is applied to this quad.
 * - The result is copied back to ram if required.
 */
class VISP_EXPORT vpPanda3DPostProcessFilter : public vpPanda3DBaseRenderer
{
public:
  vpPanda3DPostProcessFilter(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, std::string fragmentShader)
    : vpPanda3DBaseRenderer(name), m_inputRenderer(inputRenderer), m_isOutput(isOutput), m_fragmentShader(fragmentShader)
  {
    m_renderOrder = m_inputRenderer->getRenderOrder() + 1;
  }

  bool isRendering3DScene() const vp_override
  {
    return false;
  }

  GraphicsOutput *getMainOutputBuffer() vp_override { return m_buffer; }

protected:
  virtual void setupScene() vp_override;

  void setupCamera() vp_override;

  void setupRenderTarget() vp_override;

  void setRenderParameters(const vpPanda3DRenderParameters &params) vp_override;

  void getRenderBasic(vpImage<unsigned char> &I) const;
  void getRenderBasic(vpImage<vpRGBf> &I) const;


  virtual FrameBufferProperties getBufferProperties() const = 0;

  std::shared_ptr<vpPanda3DBaseRenderer> m_inputRenderer;
  bool m_isOutput; //! Whether this filter is an output to be used and should be copied to ram
  std::string m_fragmentShader;
  PT(Shader) m_shader;
  Texture *m_texture;
  GraphicsOutput *m_buffer;

  static const char *FILTER_VERTEX_SHADER;
};

#endif
#endif