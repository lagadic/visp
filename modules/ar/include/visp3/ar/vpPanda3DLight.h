#ifndef vpPand3DLight_h
#define vpPand3DLight_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <string>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRGBf.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include "nodePath.h"
#include "ambientLight.h"
#include "directionalLight.h"
#include "pointLight.h"



class VISP_EXPORT vpPanda3DLight
{
public:
  vpPanda3DLight(const std::string &name, const vpRGBf &color) : m_name(name), m_color(color) { }

  const std::string &getName() const { return m_name; }
  const vpRGBf &getColor() const { return m_color; }
  virtual void addToScene(NodePath &scene) const = 0;

protected:
  std::string m_name;
  vpRGBf m_color;
};

class VISP_EXPORT vpPanda3DAmbientLight : public vpPanda3DLight
{
public:
  vpPanda3DAmbientLight(const std::string &name, const vpRGBf &color) : vpPanda3DLight(name, color) { }
  void addToScene(NodePath &scene) const vp_override
  {
    PT(AmbientLight) light = new AmbientLight(m_name);
    light->set_color(LColor(m_color.R, m_color.G, m_color.B, 1));
    NodePath alnp = scene.attach_new_node(light);
    scene.set_light(alnp);
  }
};

class VISP_EXPORT vpPanda3DPointLight : public vpPanda3DLight
{
public:
  vpPanda3DPointLight(const std::string &name, const vpRGBf &color, const vpColVector &position) : vpPanda3DLight(name, color), m_position(position)
  {
    if (position.size() != 3) {
      throw vpException(vpException::dimensionError, "Point light position must be a 3 dimensional vector");
    }
  }
  void addToScene(NodePath &scene) const vp_override
  {
    PT(PointLight) light = new PointLight(m_name);
    light->set_color(LColor(m_color.R, m_color.G, m_color.B, 1));
    NodePath np = scene.attach_new_node(light);
    vpPoint posPanda = vpPanda3DBaseRenderer::vispPointToPanda(m_position);
    np.set_pos(posPanda.get_X(), posPanda.get_Y(), posPanda.get_Z());
    scene.set_light(np);
  }
private:
  const vpPoint m_position;
};

class VISP_EXPORT vpPanda3DLightable
{
public:
  virtual ~vpPanda3DLightable() = default;
  virtual void addLight(const vpPanda3DLight &light) = 0;
};

class VISP_EXPORT vpPanda3DLightableScene : public vpPanda3DLightable
{
public:
  vpPanda3DLightableScene() : vpPanda3DLightable()
  { }

  vpPanda3DLightableScene(NodePath &scene) : vpPanda3DLightable(), m_lightableScene(scene)
  { }



  void addLight(const vpPanda3DLight &light) vp_override
  {
    if (m_lightableScene.is_empty()) {
      throw vpException(vpException::notInitialized, "Tried to add a light to a scene that is not initialized.");
    }
    light.addToScene(m_lightableScene);
  }
protected:
  void setLightableScene(NodePath &scene) { m_lightableScene = scene; }
private:
  NodePath m_lightableScene;
};



#endif
#endif
