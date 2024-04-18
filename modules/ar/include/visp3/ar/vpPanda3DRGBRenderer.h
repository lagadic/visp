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

  void setupScene() vp_override
  {
    vpPanda3DBaseRenderer::setupScene();
    m_renderRoot.set_shader_auto();
    // PT(DirectionalLight) d_light;
    // d_light = new DirectionalLight("my d_light");
    // d_light->set_color(LColor(0.8, 0.8, 0.5, 1));
    // NodePath dlnp = m_renderRoot.attach_new_node(d_light);
    // dlnp.set_hpr(-30, -60, 0);
    // m_renderRoot.set_light(dlnp);
  }

  void setupRenderTarget() vp_override;

  void getRender(vpImage<vpRGBa> &I) const;


  void addNodeToScene(const NodePath &object) vp_override
  {
    NodePath objectInScene = object.copy_to(m_renderRoot);
    objectInScene.set_name(object.get_name());
    objectInScene.set_shader_auto();
    PT(Material) mat = new Material();
    mat->set_diffuse(LColor(1.0, 1.0, 0.0, 1.0));
    mat->set_metallic(0.5);
    mat->set_specular(0.5);

    objectInScene.set_material(mat);
    m_renderRoot.ls();
  }
  // {
  //   // NodePath objectInScene = m_renderRoot.attach_new_node(object.node());
  //   // objectInScene.set_name(object.get_name());
  //   //objectInScene.set_shader_auto();
  //   PT(Material) mat = new Material();
  //   // mat->set_diffuse(LColor(1.0, 1.0, 0.0, 1.0));
  //   // mat->set_metallic(0.5);
  //   // mat->set_specular(0.5);

  //   // objectInScene.set_material(mat);
  // }

private:
  PT(Shader) m_colorShader;
  Texture *m_colorTexture;
  GraphicsOutput *m_colorBuffer;

};


#endif //VISP_HAVE_PANDA3D
#endif
