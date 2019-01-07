/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 * Description:
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#ifndef vpScene_h
#define vpScene_h

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "vpBound.h"
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpWireFrameSimulatorTypes.h>

// Inventor includes
#if defined(VISP_HAVE_COIN3D)
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/nodes/SoSeparator.h>

#include <list>
#include <vector>

typedef struct indexFaceSet {
  indexFaceSet() : nbPt(0), pt(), nbIndex(0), index(){};
  int nbPt;
  std::vector<vpPoint> pt;
  int nbIndex;
  std::vector<int> index;
} indexFaceSet;

#endif

typedef enum { BND_MODEL, WRL_MODEL, UNKNOWN_MODEL } Model_3D;

Model_3D getExtension(const char *file);
void set_scene_wrl(const char *str, Bound_scene *sc, float factor);
void set_scene(const char *, Bound_scene *, float);
void vp2jlc_matrix(const vpHomogeneousMatrix &, Matrix &);

#if defined(VISP_HAVE_COIN3D)
void extractFaces(SoVRMLIndexedFaceSet *face_set, indexFaceSet *ifs);
void ifsToBound(Bound *, std::list<indexFaceSet *> &);
void destroyIfs(std::list<indexFaceSet *> &);
#endif

#endif
#endif
