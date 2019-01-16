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

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <cmath>
#include <limits>

#include "vpKeyword.h"
#include "vpLex.h"
#include "vpParser.h"
#include "vpScene.h"

#include <visp3/core/vpException.h>
#include <visp3/core/vpPoint.h>

/*
  Get the extension of the file and return it
*/
Model_3D getExtension(const char *file)
{
  std::string sfilename(file);

  size_t bnd = sfilename.find("bnd");
  size_t BND = sfilename.find("BND");
  size_t wrl = sfilename.find("wrl");
  size_t WRL = sfilename.find("WRL");

  size_t size = sfilename.size();

  if ((bnd > 0 && bnd < size) || (BND > 0 && BND < size))
    return BND_MODEL;
  else if ((wrl > 0 && wrl < size) || (WRL > 0 && WRL < size)) {
#if defined(VISP_HAVE_COIN3D)
    return WRL_MODEL;
#else
    std::cout << "Coin not installed, cannot read VRML files" << std::endl;
    throw std::string("Coin not installed, cannot read VRML files");
#endif
  }
  return UNKNOWN_MODEL;
}

/*
   Enable to initialize the scene
*/
void set_scene(const char *str, Bound_scene *sc, float factor)
{
  FILE *fd;

  // if ((fd = fopen (str, 0)) == -1)
  if ((fd = fopen(str, "r")) == NULL) {
    std::string error = "The file " + std::string(str) + " can not be opened";

    throw(vpException(vpException::ioError, error.c_str()));
  }
  open_keyword(keyword_tbl);
  open_lex();
  open_source(fd, str);
  malloc_Bound_scene(sc, str, (Index)BOUND_NBR);
  parser(sc);

  // if (factor != 1)
  if (std::fabs(factor) > std::numeric_limits<double>::epsilon()) {
    for (int i = 0; i < sc->bound.nbr; i++) {
      for (int j = 0; j < sc->bound.ptr[i].point.nbr; j++) {
        sc->bound.ptr[i].point.ptr[j].x = sc->bound.ptr[i].point.ptr[j].x * factor;
        sc->bound.ptr[i].point.ptr[j].y = sc->bound.ptr[i].point.ptr[j].y * factor;
        sc->bound.ptr[i].point.ptr[j].z = sc->bound.ptr[i].point.ptr[j].z * factor;
      }
    }
  }

  close_source();
  close_lex();
  close_keyword();
  fclose(fd);
}

#if defined(VISP_HAVE_COIN3D)

void set_scene_wrl(const char *str, Bound_scene *sc, float factor)
{
  // Load the sceneGraph
  SoDB::init();
  SoInput in;
  SbBool ok = in.openFile(str);
  SoVRMLGroup *sceneGraphVRML2;

  if (!ok) {
    throw(vpException(vpException::fatalError, "Can't open file \"%s\". Please check the Marker_Less.ini file", str));
  }

  if (!in.isFileVRML2()) {
    SoSeparator *sceneGraph = SoDB::readAll(&in);
    if (sceneGraph == NULL) { /*return -1;*/
    }
    sceneGraph->ref();

    SoToVRML2Action tovrml2;
    tovrml2.apply(sceneGraph);
    sceneGraphVRML2 = tovrml2.getVRML2SceneGraph();
    sceneGraphVRML2->ref();
    sceneGraph->unref();
  } else {
    sceneGraphVRML2 = SoDB::readAllVRML(&in);
    if (sceneGraphVRML2 == NULL) {
      /*return -1;*/
      throw(vpException(vpException::notInitialized, "Cannot read VRML file"));
    }
    sceneGraphVRML2->ref();
  }

  in.closeFile();

  int nbShapes = sceneGraphVRML2->getNumChildren();

  SoNode *child;

  malloc_Bound_scene(sc, str, (Index)BOUND_NBR);

  int iterShapes = 0;
  for (int i = 0; i < nbShapes; i++) {
    child = sceneGraphVRML2->getChild(i);
    if (child->getTypeId() == SoVRMLShape::getClassTypeId()) {
      int nbFaces = 0;
      std::list<indexFaceSet *> ifs_list;
      SoChildList *child2list = child->getChildren();
      for (int j = 0; j < child2list->getLength(); j++) {
        if (((SoNode *)child2list->get(j))->getTypeId() == SoVRMLIndexedFaceSet::getClassTypeId()) {
          indexFaceSet *ifs = new indexFaceSet;
          SoVRMLIndexedFaceSet *face_set;
          face_set = (SoVRMLIndexedFaceSet *)child2list->get(j);
          extractFaces(face_set, ifs);
          ifs_list.push_back(ifs);
          nbFaces++;
        }
        //         if (((SoNode*)child2list->get(j))->getTypeId() ==
        //         SoVRMLIndexedLineSet::getClassTypeId())
        //         {
        //           std::cout << "> We found a line" << std::endl;
        //           SoVRMLIndexedLineSet * line_set;
        //           line_set = (SoVRMLIndexedLineSet*)child2list->get(j);
        //           extractLines(line_set);
        //         }
      }
      sc->bound.nbr++;
      ifsToBound(&(sc->bound.ptr[iterShapes]), ifs_list);
      destroyIfs(ifs_list);
      iterShapes++;
    }
  }

  // if (factor != 1)
  if (std::fabs(factor) > std::numeric_limits<double>::epsilon()) {
    for (int i = 0; i < sc->bound.nbr; i++) {
      for (int j = 0; j < sc->bound.ptr[i].point.nbr; j++) {
        sc->bound.ptr[i].point.ptr[j].x = sc->bound.ptr[i].point.ptr[j].x * factor;
        sc->bound.ptr[i].point.ptr[j].y = sc->bound.ptr[i].point.ptr[j].y * factor;
        sc->bound.ptr[i].point.ptr[j].z = sc->bound.ptr[i].point.ptr[j].z * factor;
      }
    }
  }
}

void extractFaces(SoVRMLIndexedFaceSet *face_set, indexFaceSet *ifs)
{
  //   vpList<vpPoint> pointList;
  //   pointList.kill();
  SoVRMLCoordinate *coord = (SoVRMLCoordinate *)(face_set->coord.getValue());
  int coordSize = coord->point.getNum();

  ifs->nbPt = coordSize;
  for (int i = 0; i < coordSize; i++) {
    SbVec3f point(0, 0, 0);
    point[0] = coord->point[i].getValue()[0];
    point[1] = coord->point[i].getValue()[1];
    point[2] = coord->point[i].getValue()[2];
    vpPoint pt(point[0], point[1], point[2]);
    ifs->pt.push_back(pt);
  }

  SoMFInt32 indexList = face_set->coordIndex;
  int indexListSize = indexList.getNum();

  ifs->nbIndex = indexListSize;
  for (int i = 0; i < indexListSize; i++) {
    int index = face_set->coordIndex[i];
    ifs->index.push_back(index);
  }
}

void ifsToBound(Bound *bptr, std::list<indexFaceSet *> &ifs_list)
{
  int nbPt = 0;
  for (std::list<indexFaceSet *>::const_iterator it = ifs_list.begin(); it != ifs_list.end(); ++it) {
    nbPt += (*it)->nbPt;
  }
  bptr->point.nbr = (Index)nbPt;
  bptr->point.ptr = (Point3f *)malloc((unsigned int)nbPt * sizeof(Point3f));

  ifs_list.front();
  unsigned int iter = 0;
  for (std::list<indexFaceSet *>::const_iterator it = ifs_list.begin(); it != ifs_list.end(); ++it) {
    indexFaceSet *ifs = *it;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbPt; j++) {
      bptr->point.ptr[iter].x = (float)ifs->pt[j].get_oX();
      bptr->point.ptr[iter].y = (float)ifs->pt[j].get_oY();
      bptr->point.ptr[iter].z = (float)ifs->pt[j].get_oZ();
      iter++;
    }
  }

  unsigned int nbFace = 0;
  ifs_list.front();
  std::list<int> indSize;
  int indice = 0;
  for (std::list<indexFaceSet *>::const_iterator it = ifs_list.begin(); it != ifs_list.end(); ++it) {
    indexFaceSet *ifs = *it;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbIndex; j++) {
      if (ifs->index[j] == -1) {
        nbFace++;
        indSize.push_back(indice);
        indice = 0;
      } else
        indice++;
    }
  }

  bptr->face.nbr = (Index)nbFace;
  bptr->face.ptr = (Face *)malloc(nbFace * sizeof(Face));

  std::list<int>::const_iterator iter_indSize = indSize.begin();
  for (unsigned int i = 0; i < indSize.size(); i++) {
    bptr->face.ptr[i].vertex.nbr = (Index)*iter_indSize;
    bptr->face.ptr[i].vertex.ptr = (Index *)malloc((unsigned int)*iter_indSize * sizeof(Index));
    ++iter_indSize;
  }

  int offset = 0;
  indice = 0;
  for (std::list<indexFaceSet *>::const_iterator it = ifs_list.begin(); it != ifs_list.end(); ++it) {
    indexFaceSet *ifs = *it;
    iter = 0;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbIndex; j++) {
      if (ifs->index[j] != -1) {
        bptr->face.ptr[indice].vertex.ptr[iter] = (Index)(ifs->index[j] + offset);
        iter++;
      } else {
        iter = 0;
        indice++;
      }
    }
    offset += ifs->nbPt;
  }
}

void destroyIfs(std::list<indexFaceSet *> &ifs_list)
{
  for (std::list<indexFaceSet *>::const_iterator it = ifs_list.begin(); it != ifs_list.end(); ++it) {
    delete *it;
  }
  ifs_list.clear();
}
#else
void set_scene_wrl(const char * /*str*/, Bound_scene * /*sc*/, float /*factor*/) {}
#endif

/*
  Convert the matrix format to deal with the one in the simulator
*/
void vp2jlc_matrix(const vpHomogeneousMatrix &vpM, Matrix &jlcM)
{
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 0; j < 4; j++)
      jlcM[j][i] = (float)vpM[i][j];
  }
}

#endif
