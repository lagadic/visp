/****************************************************************************
 *
 * $Id: vpRA.cpp,v 1.1 2007-03-02 10:45:38 marchand Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Simulator based on SoQt.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT

#include <visp/vpRA.h>
#include <visp/vpTime.h>


/* Objets OIV. */
#include <Inventor/nodes/SoCone.h> /* Objet cone.                            */
#include <Inventor/nodes/SoCylinder.h> /* Objet cylindre.                    */
#include <Inventor/nodes/SoPointLight.h> /* Objet lumiere ponctuelle.        */
#include <Inventor/nodes/SoCoordinate3.h> /* Liste de points.                */
#include <Inventor/nodes/SoIndexedFaceSet.h> /* Liste de face.               */
#include <Inventor/nodes/SoTranslation.h> /* Trasnfo translation.            */
#include <Inventor/nodes/SoScale.h> /* Trasnfo mise a l'echelle.             */
#include <Inventor/nodes/SoRotationXYZ.h> /* Transfo rotation simple.       */

#include <Inventor/nodes/SoDirectionalLight.h> /* Objet lumiere directionnelle*/
#include <Inventor/nodes/SoMaterial.h> /* Matiere (couleur) des objets.     */
#include <Inventor/nodes/SoDrawStyle.h> /* Style de rendu.                  */
#include <Inventor/nodes/SoEnvironment.h> /* Eclairage ambiant.              */
#include <Inventor/nodes/SoGroup.h> /* Groupement de noeuds (sans separation)*/
#include <Inventor/actions/SoWriteAction.h>



vpRA::~vpRA()
{
  kill() ;
}

void
vpRA::initInternalViewer(int width, int height)
{

  vpSimulator::initInternalViewer(width,height) ;
  /*internal_width = width;
  internal_height = height;

  if (mainWindowInitialized==false)
  {
    initSoQt() ;
    initSceneGraph() ;
  }

  internalView = new vpViewerRA(mainWindow, this);

  // set the scene to render from this view
  internalView->setSceneGraph(internalRoot);

  // set the title
  internalView->setTitle("Internal camera view") ;

  //If the view mode is on, user events will be caught and used to influence
  //the camera position / orientation. in this viewer we do not want that,
  //we set it to false
  internalView->setViewing(false);

  // Turn the viewer decorations
  internalView->setDecoration(false) ;

  internalView->resize(width, height) ;
  */

  // no image is loaded
  background = false ;
  image_background = NULL ;

  if ( image_background != NULL)
  {
    free(image_background) ;
    image_background = NULL ;
  }

  image_background =(GLubyte *)
    malloc(internal_width*internal_height*sizeof(GLubyte)) ;

  // open the window
  // internalView->show();

}



// Grey pictures SetBackGroundImage
void
vpRA::setImage(vpImage<unsigned char> &I)
{
  typeImage= vpRA::grayImage ;

  if ((internal_width != I.getWidth()) ||
      (internal_height != I.getHeight()))
	{
	  cout << "The image size is different from the view size " << endl ;
	  cout << "return an ERROR -1 " << endl ;
	  //	  return ERROR ;
	}


  background = true ;


  for (int i=0 ; i < I.getHeight() ; i++)
    for (int j=0 ; j < I.getWidth() ; j++)
      //le repere image open GL est en bas a gauche donc l'image serait inverse
      image_background[i*I.getWidth()+j] = I[I.getHeight()-i-1][j] ;

}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
