#ifndef vpPanda3DCommonFilters_h
#define vpPanda3DCommonFilters_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DPostProcessFilter.h>

class vpPanda3DRGBRenderer;

/**
 * @brief Class that implements an RGB to grayscale conversion.
 *
 */
class VISP_EXPORT vpPanda3DLuminanceFilter : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DLuminanceFilter(const std::string &name, std::shared_ptr<vpPanda3DRGBRenderer> inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const vp_override;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const char *FRAGMENT_SHADER;
};

/**
 * @brief Class that implements a gaussian filter on a grayscale image.
 * The grayscale image should be contained in the blue channel of the image.
 *
 */
class VISP_EXPORT vpPanda3DGaussianBlur : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DGaussianBlur(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const vp_override;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const char *FRAGMENT_SHADER;
};


class VISP_EXPORT vpPanda3DCanny : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DCanny(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, float edgeThreshold);
  FrameBufferProperties getBufferProperties() const vp_override;
  void getRender(vpImage<vpRGBf> &I) const;
  void setEdgeThreshold(float edgeThreshold);

protected:
  void setupScene() vp_override;

private:
  static const char *FRAGMENT_SHADER;
  float m_edgeThreshold;
};


#endif
#endif
