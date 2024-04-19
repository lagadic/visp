#ifndef vpPanda3DRGBRenderer_h
#define vpPanda3DRGBRenderer_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/core/vpImage.h>

#include <directionalLight.h>

class VISP_EXPORT vpPanda3DRGBRenderer : public vpPanda3DBaseRenderer
{
public:
  vpPanda3DRGBRenderer() : vpPanda3DBaseRenderer("RGB") { }

  /**
   * @brief Store the render resulting from calling renderFrame() into a vpImage.
   *
   * If the image does not have the correct dimensions, it is resized.
   *
   * @param I The image in which to store the render.
   */
  void getRender(vpImage<vpRGBa> &I) const;

protected:

  void setupRenderTarget() vp_override;

private:
  Texture *m_colorTexture;
  GraphicsOutput *m_colorBuffer;

};


#endif //VISP_HAVE_PANDA3D
#endif
