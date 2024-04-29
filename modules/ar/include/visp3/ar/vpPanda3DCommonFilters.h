#ifndef vpPanda3DCommonFilters_h
#define vpPanda3DCommonFilters_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DPostProcessFilter.h>

class vpPanda3DRGBRenderer;

class VISP_EXPORT vpPanda3DLuminanceFilter : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DLuminanceFilter(const std::string &name, std::shared_ptr<vpPanda3DRGBRenderer> &inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const vp_override;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const char *FRAGMENT_SHADER;

};
#endif
#endif
