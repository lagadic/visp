#ifndef vpPanda3DGeometryRenderer_h
#define vpPanda3DGeometryRenderer_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>

/**
 * @brief Renderer that outputs object normals (in world frame) as well as depth
 *
 */
class VISP_EXPORT vpPanda3DGeometryRenderer : public vpPanda3DBaseRenderer
{
public:
  vpPanda3DGeometryRenderer(const std::string &rendererName) : vpPanda3DBaseRenderer(rendererName) { }
  ~vpPanda3DGeometryRenderer() { }

  void setupScene() vp_override;
  void setupRenderTarget() vp_override;


  void getRender(vpImage<vpRGBf> &normals, vpImage<float> &depth) const;


  static const char *SHADER_VERT_NORMAL_AND_DEPTH_WORLD;
  static const char *SHADER_VERT_NORMAL_AND_DEPTH_CAMERA;
  static const char *SHADER_FRAG_NORMAL_AND_DEPTH;

private:
  PT(Shader) m_normalDepthShader;
  Texture *m_normalDepthTexture;
  GraphicsOutput *m_normalDepthBuffer;

};


#endif //VISP_HAVE_PANDA3D
#endif
