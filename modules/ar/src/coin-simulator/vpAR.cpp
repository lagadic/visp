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
 * Use to display an image behind the internal view of the simulator
 * used for augmented reality application
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpAR.cpp
  \brief class used to display an image behind the internal view of the
  simulator. Used for augmented reality applications.
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI

#include <visp3/ar/vpAR.h>
#include <visp3/core/vpTime.h>

/* Objets OIV. */
#include <Inventor/nodes/SoCone.h>           /* Objet cone.                            */
#include <Inventor/nodes/SoCoordinate3.h>    /* Liste de points.                */
#include <Inventor/nodes/SoCylinder.h>       /* Objet cylindre.                    */
#include <Inventor/nodes/SoIndexedFaceSet.h> /* Liste de face.               */
#include <Inventor/nodes/SoPointLight.h>     /* Objet lumiere ponctuelle.        */
#include <Inventor/nodes/SoRotationXYZ.h>    /* Transfo rotation simple.       */
#include <Inventor/nodes/SoScale.h>          /* Trasnfo mise a l'echelle.             */
#include <Inventor/nodes/SoTranslation.h>    /* Trasnfo translation.            */

#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoDirectionalLight.h> /* Objet lumiere directionnelle*/
#include <Inventor/nodes/SoDrawStyle.h>        /* Style de rendu.                  */
#include <Inventor/nodes/SoEnvironment.h>      /* Eclairage ambiant.              */
#include <Inventor/nodes/SoGroup.h>            /* Groupement de noeuds (sans separation)*/
#include <Inventor/nodes/SoMaterial.h>         /* Matiere (couleur) des objets.     */

/*!
        Basic Destructor that calls the kill() method of the vpSimulator
   class.
*/
vpAR::~vpAR() { kill(); }

/*!
        Initialisation of the internal view of the simulator.

        \param width : Width of the internal view.
        \param height : Height of the internal view.
        \param type : Type of background image ie gray scaled or color.
*/
void vpAR::initInternalViewer(const unsigned int width, const unsigned int height, vpImageType type)
{

  vpSimulator::initInternalViewer(width, height);

  // no image is loaded
  background = false;

  if (image_background != NULL) {
    free(image_background);
    image_background = NULL;
  }

  typeImage = type;
  if (typeImage == grayImage)
    image_background = (GLubyte *)malloc(internal_width * internal_height * sizeof(GLubyte));
  else
    image_background = (GLubyte *)malloc(3 * internal_width * internal_height * sizeof(GLubyte));
}

/*!
        Set the background image and turn it to deal with the frame of OpenGL.

        \param I : Gray scaled image for the background.
*/
// Grey pictures SetBackGroundImage
void vpAR::setImage(vpImage<unsigned char> &I)
{

  if ((internal_width != I.getWidth()) || (internal_height != I.getHeight())) {
    vpERROR_TRACE("The image size is different from the view size ");
    throw(vpException(vpException::dimensionError), "The image size is different from the view size");
  }

  background = true;

  for (unsigned int i = 0; i < I.getHeight(); i++)
    for (unsigned int j = 0; j < I.getWidth(); j++)
      // le repere image open GL est en bas a gauche donc l'image serait
      // inverse
      image_background[i * I.getWidth() + j] = I[I.getHeight() - i - 1][j];
}

/*!
        Set the background image and turn it to deal with the frame of OpenGL.

        \param I : Color image for the background.
*/
// Color pictures SetBackGroundImage
void vpAR::setImage(vpImage<vpRGBa> &I)
{

  if ((internal_width != I.getWidth()) || (internal_height != I.getHeight())) {
    vpERROR_TRACE("The image size is different from the view size ");
    throw(vpException(vpException::dimensionError), "The image size is different from the view size");
  }

  background = true;

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    unsigned int k = 0;
    for (unsigned int j = 0; j < I.getWidth(); j++)
    // le repere image open GL est en bas a gauche donc l'image serait inverse
    {
      image_background[i * I.getWidth() * 3 + k + 0] = I[I.getHeight() - i - 1][j].R;
      image_background[i * I.getWidth() * 3 + k + 1] = I[I.getHeight() - i - 1][j].G;
      image_background[i * I.getWidth() * 3 + k + 2] = I[I.getHeight() - i - 1][j].B;
      k += 3;
    }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_ar.a(vpAR.cpp.o) has no symbols
void dummy_vpAR(){};
#endif
