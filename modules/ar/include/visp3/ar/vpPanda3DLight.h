/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef VP_PANDA3D_LIGHT_H
#define VP_PANDA3D_LIGHT_H

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
#include "directionalLight.h"

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d_lighting
 *
 * \brief Base class for a Light that can be added to a Panda3D scene.
 *
 * Note that modifying any object that inherits from this class
 * after the method addToScene has been called * will not update the rendered light.
 *
 * \see https://docs.panda3d.org/1.10/cpp/programming/render-attributes/lighting
 *
*/
class VISP_EXPORT vpPanda3DLight
{
public:
  /**
   * \brief Build a new Panda3D light, given a unique name and an RGB color.
   *
   *
   * \param name the name of the light: should be unique in the scene where the light will be added.
   * \param color The color of the light: Each R,G,B component is unbounded and can exceed a value of 1 to increase its intensity.
   */
  vpPanda3DLight(const std::string &name, const vpRGBf &color) : m_name(name), m_color(color) { }

  /**
   * \brief Get the name of the light.
   *
   * This name should be unique and will be required when interacting with Panda3D to fetch the node.
   */
  const std::string &getName() const { return m_name; }
  /**
   * \brief Get the light's color
   *
   * \return const vpRGBf&
   */
  const vpRGBf &getColor() const { return m_color; }

  /**
   * \brief Add the light to the scene.
   *
   * \param scene Scene where the light should be added.
   */
  virtual void addToScene(NodePath &scene) const = 0;

protected:
  std::string m_name; //! Name of the light. Should be unique in the scene
  vpRGBf m_color; //! RGB Color of the light. Can exceed 1 for each component.
};

/**
 *
 * \ingroup group_ar_renderer_panda3d_lighting
 *
 * \brief Class representing an ambient light.
 *
 * Ambient light are not physically possible, but are used to emulate light coming from all directions.
 * They do not generate speculars or reflections.
 *
 * \see https://docs.panda3d.org/1.10/python/reference/panda3d.core.AmbientLight
 */
class VISP_EXPORT vpPanda3DAmbientLight : public vpPanda3DLight
{
public:
  vpPanda3DAmbientLight(const std::string &name, const vpRGBf &color) : vpPanda3DLight(name, color) { }

  void addToScene(NodePath &scene) const VP_OVERRIDE
  {
    PT(AmbientLight) light = new AmbientLight(m_name);
    light->set_color(LColor(m_color.R, m_color.G, m_color.B, 1));
    NodePath alnp = scene.attach_new_node(light);
    scene.set_light(alnp);
  }
};

/**
 * \ingroup group_ar_renderer_panda3d_lighting
 *
 * \brief Class representing a Point Light.
 *
 * Point lights emit light all around them, from a single point.
 * Their light can be subject to a distance-based attenuation.
 */
class VISP_EXPORT vpPanda3DPointLight : public vpPanda3DLight
{
public:
  /**
   * \brief Build a new point light.
   *
   * \see vpPanda3DLight constructor.
   *
   * \param name name of the light
   * \param color  color of the light
   * \param position Position in the scene of the light. Uses ViSP coordinates.
   * \param attenuation Attenuation components of the light as a function of distance.
   * Should be a vector of size 3 where the first component is the constant intensity factor (no falloff),
   * the second is a linear falloff coefficient, and the last one is the quadratic falloff component.
   * To follow the inverse square law, set this value vector to [0, 0, 1]
   * To have no falloff, set it to [1, 0, 0].
   */
  vpPanda3DPointLight(const std::string &name, const vpRGBf &color, const vpColVector &position, const vpColVector &attenuation)
    : vpPanda3DLight(name, color), m_attenuation(attenuation)
  {
    if (position.size() != 3) {
      throw vpException(vpException::dimensionError, "Point light position must be a 3 dimensional vector");
    }
    m_position.resize(4, false);
    m_position.insert(0, position);
    m_position[3] = 1.0;
    if (attenuation.size() != 3) {
      throw vpException(vpException::dimensionError, "Point light attenuation components must be a 3 dimensional vector");
    }
  }

  void addToScene(NodePath &scene) const VP_OVERRIDE
  {
    PT(PointLight) light = new PointLight(m_name);
    light->set_color(LColor(m_color.R, m_color.G, m_color.B, 1));
    light->set_attenuation(LVecBase3(m_attenuation[0], m_attenuation[1], m_attenuation[2]));
    NodePath np = scene.attach_new_node(light);
    //vpColVector posPanda = vpPanda3DBaseRenderer::vispPointToPanda(m_position);
    np.set_pos(m_position[0], m_position[1], m_position[2]);
    scene.set_light(np);
  }

private:
  vpColVector m_position; //! Position of the light, in homogeneous coordinates
  vpColVector m_attenuation; //! Attenuation components: [constant, linear, quadratic]
};
/**
 *
 *
 * \ingroup group_ar_renderer_panda3d_lighting
 * \brief Class representing a directional light
 *
 * A directional light has no origin nor falloff.
 *
 */
class VISP_EXPORT vpPanda3DDirectionalLight : public vpPanda3DLight
{
public:
  /**
   * \brief Build a new directional light.
   *
   * \see vpPanda3DLight constructor.
   *
   * \param name name of the light
   * \param color  color of the light
   * \param direction Position in the scene of the light. Uses ViSP coordinates.
   */
  vpPanda3DDirectionalLight(const std::string &name, const vpRGBf &color, const vpColVector &direction)
    : vpPanda3DLight(name, color), m_direction(direction)
  {
    if (m_direction.size() != 3) {
      throw vpException(vpException::dimensionError, "Direction light direction must be a 3 dimensional vector");
    }
    m_direction.normalize();
  }

  void addToScene(NodePath &scene) const VP_OVERRIDE
  {
    PT(DirectionalLight) light = new DirectionalLight(m_name);
    light->set_color(LColor(m_color.R, m_color.G, m_color.B, 1));
    vpColVector dir = vpPanda3DBaseRenderer::vispVectorToPanda(m_direction);
    std::cout << m_direction << ", " << dir << std::endl;
    light->set_direction(LVector3f(m_direction[0], m_direction[1], m_direction[2]));
    NodePath np = scene.attach_new_node(light);
    scene.set_light(np);
  }

private:
  vpColVector m_direction; //! Direction vector of the light, in scene frame
};

/**
 * \ingroup group_ar_renderer_panda3d_lighting
 * \brief Interface for objects, scenes or other Panda3D related data that can be lit by a vpPanda3DLight.
 *
 */
class VISP_EXPORT vpPanda3DLightable
{
public:
  virtual ~vpPanda3DLightable() = default;
  /**
   * \brief Light this lightable object with a new light
   *
   * \param light
   */
  virtual void addLight(const vpPanda3DLight &light) = 0;
};
/**
 * \ingroup group_ar_renderer_panda3d_lighting
 * \brief Implementation of vpPanda3DLightable for a panda scene with a root node.
 *
 * The root node should be specified with setLightableScene by an inheriting implementation.
 */
class VISP_EXPORT vpPanda3DLightableScene : public vpPanda3DLightable
{
public:
  vpPanda3DLightableScene() : vpPanda3DLightable()
  { }

  vpPanda3DLightableScene(NodePath &scene) : vpPanda3DLightable(), m_lightableScene(scene)
  { }

  /**
   * \brief Add a light to the scene. All of the objects in the scene will be lit.
   *
   * \throws if the scene is not setup (setLightableScene or constructor with NodePath has not been called)
   * \param light light to add
   */
  void addLight(const vpPanda3DLight &light) VP_OVERRIDE
  {
    if (m_lightableScene.is_empty()) {
      throw vpException(vpException::notInitialized, "Tried to add a light to a scene that is not initialized.");
    }
    light.addToScene(m_lightableScene);
  }
protected:
  void setLightableScene(NodePath &scene) { m_lightableScene = scene; }
private:
  NodePath m_lightableScene; //! Scene that should be lit when calling addLight
};

END_VISP_NAMESPACE

#endif
#endif
