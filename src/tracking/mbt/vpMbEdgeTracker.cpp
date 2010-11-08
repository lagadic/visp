/****************************************************************************
 *
 * $Id: vpMbEdgeTracker.cpp 2807 2010-09-14 10:14:54Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbEdgeTracker.cpp
  \brief Make the complete tracking of an object by using its CAD model.
*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageIo.h>
#include <visp/vpRobust.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMatrixException.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>

#include <visp/vpMbEdgeTracker.h>
#include <visp/vpMbtDistanceLine.h>
#include <visp/vpMbtXmlParser.h>

#include <string>


/*!
  Basic constructor
*/
vpMbEdgeTracker::vpMbEdgeTracker()
{
  index_polygon =0;
  compute_interaction=1;
  nline = 0;
  lambda = 1;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
//  percentageTtPt = 0.3;
  displayMe = false;

  Lline.kill();
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbEdgeTracker::~vpMbEdgeTracker()
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l!=NULL) delete l ;
    l = NULL ;
    Lline.next() ;
  }
  Lline.kill() ;
}

/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbEdgeTracker::setMovingEdge(const vpMe &_me)
{
  this->me = _me;

  Lline.front() ;
  vpMbtDistanceLine *l ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    l->setMovingEdge(&me) ;
    Lline.next();
  }
}


/*!
  Compute the visual servoing loop to get the pose of the feature set.
  
  \exception vpTrackingException::notEnoughPointError if the number of detected 
  feature is equal to zero. 
  
  \param _I : The current image. 
 */
void
vpMbEdgeTracker::computeVVS(const vpImage<unsigned char>& _I)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtDistanceLine *l ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  int iter = 0;

  //Nombre de moving edges
  int nbrow  = 0;
  
  Lline.front();
  while (!Lline.outside())
  {
    l = Lline.value() ;
    nbrow += l->nbFeature ;
    l->initInteractionMatrixError() ;
    Lline.next() ;
  }
  
  if (nbrow==0)
  {
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }
  
  vpMatrix L(nbrow,6), Lp;

  // compute the error vector
  vpColVector error(nbrow);
  int nerror = error.getRows();
  vpColVector v ;

  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.
  
  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;
  
  /*** First phase ***/

  while ( reloop == true && iter<10)
  {
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }
    
    count = 0;
    
    Lline.front();
    int n = 0;
    reloop = false;
    while (!Lline.outside())
    {
      l = Lline.value();
      l->computeInteractionMatrixError(cMo);
      
      double fac = 1;
      if (iter == 0)
      {
	      l->Lindex_polygon.front();
	      while (!l->Lindex_polygon.outside())
	      {
	        int index = l->Lindex_polygon.value();
	        if (l->hiddenface->isAppearing(index))
	        {
	          fac = 0.2;
	          break;
	        }
	        if(l->closeToImageBorder(_I, 10))
	        {
	          fac = 0.1;
	          break;
	        }
	        l->Lindex_polygon.next() ;
	      }
      }
      
      if (iter == 0 && l->meline != NULL)
        l->meline->list.front();
      
      for (int i=0 ; i < l->nbFeature ; i++)
      {
        for (int j=0; j < 6 ; j++)
        {
          L[n+i][j] = l->L[i][j]; //On remplit la matrice d'interaction globale
        }
        error[n+i] = l->error[i]; //On remplit la matrice d'erreur
	
        if (error[n+i] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n+i] = 0;

        if (iter == 0)
        {
          factor[n+i] = fac;
          vpMeSite site = l->meline->list.value();
          if (site.suppress != 0) factor[n+i] = 0.2;
          l->meline->list.next();
        }

        //If pour la premiere extremite des moving edges
        if (i == 0)
        {
          e_cur = l->error[0];
	        if (l->nbFeature > 1)
	        {
            e_next = l->error[1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
            }
            e_prev = e_cur;
	        }
	        else w[n+i] = 1;
        }

        //If pour la derniere extremite des moving edges
        else if(i == l->nbFeature-1)
        {
          e_cur = l->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          }
        }

        else
        {
          e_cur = l->error[i];
          e_next = l->error[i+1];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n+i] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n+i] += 0.5;
          }
          e_prev = e_cur;
        }
      }
      
      n+= l->nbFeature ;
      Lline.next() ;
    }
    
    count = count / (double)nbrow;
    if (count < 0.85)
    {
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi ; double eri ;
    for(int i = 0; i < nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;

      weighted_error[i] =  wi*eri ;
    }

    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j] ;
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -0.7*LTL.pseudoInverse()*LTR; 
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
//   cout << "\t First minimization in " << iter << " iteration " << endl ;
  
/*** Second phase ***/

  vpRobust robust(nerror);
  robust.setIteration(0) ;
  iter = 0;
  //vpColVector error_px(nerror);

  while ( ((int)((residu_1 - r)*1e8) !=0 )  && (iter<30))
  {
    Lline.front() ;
    int n = 0 ;
    while (!Lline.outside())
    {
      l = Lline.value();
      l->computeInteractionMatrixError(cMo) ;
      for (int i=0 ; i < l->nbFeature ; i++)
      {
        for (int j=0; j < 6 ; j++)
        {
          L[n+i][j] = l->L[i][j] ;
          error[n+i] = l->error[i] ;
	  //error_px[n+i] = l->error[i] * cam.get_px();
        }
      }
      n+= l->nbFeature ;
      Lline.next() ;
    }
    
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 1;
	
       robust.setThreshold(2/cam.get_px()); // limite en metre
       robust.MEstimator(vpRobust::TUKEY, error,w);
       //robust.setThreshold(2); // limite en pixel
       //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }
    else
    {
      robust.setIteration(iter);
      robust.MEstimator(vpRobust::TUKEY, error,w);
      //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }

    residu_1 = r;

    double num=0;
    double den=0;
    double wi;
    double eri;
    for(int i=0; i<nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi;

      weighted_error[i] =  wi*eri ;
    }
    
    r = sqrt(num/den); //Le critere d'arret prend en compte le poids

    if((iter==0)|| compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j];
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -lambda*LTL.pseudoInverse()*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }

  Lline.front() ;
  int n =0 ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    {
      double wmean = 0 ;
      if (l->nbFeature > 0) l->meline->list.front();
      
      for (int i=0 ; i < l->nbFeature ; i++)
      {
        wmean += w[n+i] ;
	vpMeSite p = l->meline->list.value() ;
	if (w[n+i] < 0.5)
	{
	  p.suppress = 4 ;
	  l->meline->list.modify(p) ;
	}
	
	l->meline->list.next();
      }
      n+= l->nbFeature ;
      
      if (l->nbFeature!=0)
        wmean /= l->nbFeature ;
      else
        wmean = 1;
            
      l->setMeanWeight(wmean);

      if (wmean < 0.8)
        l->Reinit = true;
    }
    Lline.next() ;
  }
  
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;

}

/*!
  Check if the tracking failed.
  
  \throw vpTrackingException::fatalError if the test fails. 
*/
void
vpMbEdgeTracker::testTracking()
{
  int nbExpectedPoint = 0;
  int nbGoodPoint = 0;
  int nbBadPoint = 0;
  
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l->isVisible() && l->meline != NULL)
    {
      nbExpectedPoint += (int)l->meline->expecteddensity;
      l->meline->list.front();
      while (!l->meline->list.outside())
      {
	vpMeSite pix = l->meline->list.value();
	if (pix.suppress == 0) nbGoodPoint++;
	else nbBadPoint++;
	l->meline->list.next();
      }
    }
    Lline.next();
  }
  
  if (nbGoodPoint < percentageGdPt *(nbGoodPoint+nbBadPoint) || nbExpectedPoint < 2)
  {
    std::cout << "nbGoodPoint :" << nbGoodPoint << std::endl;
    std::cout << "nbBadPoint :" << nbBadPoint << std::endl;
    std::cout << "nbExpectedPoint :" << nbExpectedPoint << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
  }
}


/*!
  Compute each state of the tracking procedure for all the feature sets.
  
  If the tracking is considered as failed an exception is thrown.
  
  \param I : The image.
 */
void
vpMbEdgeTracker::track(const vpImage<unsigned char> &I)
{
  try
  {  
    trackMovingEdge(I);
  }
  catch(...)
  {
    vpTRACE("Error in moving edge tracking") ;
    throw ;
  }

  // initialize the vector that contains the error and the matrix that contains
  // the interaction matrix
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l->isVisible())
    {
      l->initInteractionMatrixError() ;
    }

    Lline.next() ;
  }

  try
  {
    computeVVS(I);
  }
  catch(...)
  {
    vpTRACE("Error in computeVVS") ;
    throw std::string("Error in computeVVS");
  }
  
  try
  {
    testTracking();
  }
  catch(...)
  {
    throw std::string("test Tracking fail");
  }
  
  if (displayMe)
  {
    Lline.front() ;
    while (!Lline.outside())
    {
      l = Lline.value() ;
      if (l->isVisible())
      {
        l->displayMovingEdges(I);
      }

      Lline.next() ;
    }
  }
  
  try
  {
    updateMovingEdge(I);
  }
  catch(...)
  {
    vpTRACE("Error in moving edge updating") ;
    throw ;
  }
  
  // Looking for new visible face
  bool newvisibleface = false ;
  visibleFace(cMo, newvisibleface) ;
  initMovingEdge(I,cMo) ;

  // Reinit the moving edge for the lines which need it.
  reinitMovingEdge(I,cMo);

}


/*!
 Initialize the tracking thanks to the initial pose of the camera.
 
 \param I : The image.
 \param _cMo : The initial pose used to initialize the tracking.
*/
void
vpMbEdgeTracker::init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo)
{
  this->cMo = _cMo;
  bool a = false;
  visibleFace(_cMo, a);
  initMovingEdge(I,_cMo);
}

/*!
  Load the xml configuration file.
  Write the parameters in the corresponding objects (Ecm, camera).

  \param _filename : full name of the xml file.
*/
void 
vpMbEdgeTracker::loadConfigFile(const std::string& _filename)
{
  loadConfigFile(_filename.c_str());
}

/*!
  Load the xml configuration file.
  Write the parameters in the corresponding objects (Ecm, camera).
  
  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param filename : full name of the xml file.
*/
void
vpMbEdgeTracker::loadConfigFile(const char* filename)
{
#ifdef VISP_HAVE_XML2
  vpMbtXmlParser xmlp;
  
  try{
    xmlp.parse(filename);
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ",filename);
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  vpMe meParser;
  xmlp.getCameraParameters(camera);
  xmlp.getMe(meParser);
  setCameraParameters(camera);
  setMovingEdge(meParser);
#else
	vpTRACE("You need the libXML2 to read the config file %s", filename);
#endif
}


/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbEdgeTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness)
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    l->display(I,_cMo, cam, col, thickness) ;
    Lline.next() ;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbEdgeTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness)
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    l->display(I,_cMo, cam, col, thickness) ;
    Lline.next() ;
  }
}


/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the lines.
  
  \param I : The image.
  \param _cMo : The pose of the camera used to initialize the moving edges.
*/
void
vpMbEdgeTracker::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtDistanceLine *l ;

  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;

    bool isvisible = false ;

    l->Lindex_polygon.front() ;
    while (!l->Lindex_polygon.outside())
    {
      int index = l->Lindex_polygon.value() ;
      if (index ==-1) isvisible =true ;
      else
      {
        if (l->hiddenface->isVisible(index)) isvisible = true ;
      }
      l->Lindex_polygon.next() ;
    }

    //Si la ligne n'appartient a aucune face elle est tout le temps visible
    if (l->Lindex_polygon.nbElements() == 0) isvisible = true;

    if (isvisible)
    {
      l->setVisible(true) ;
      if (l->meline==NULL)
      {
        //cout << "init me line "<< l->getIndex() <<endl ;
        l->initMovingEdge(I,_cMo) ;
      }
    }
    else
    {
      l->setVisible(false) ;
      if (l->meline!=NULL) delete l->meline;
      l->meline=NULL;
    }
    Lline.next() ;
  }
}


/*!
  Track the moving edges in the image.
  
  \param I : the image.
*/
void
vpMbEdgeTracker::trackMovingEdge(const vpImage<unsigned char> &I)
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value();
    if(l->isVisible() == true){
      if(l->meline == NULL){
        l->initMovingEdge(I, cMo);
      }
      l->trackMovingEdge(I, cMo) ;
    }
    Lline.next() ;
  }
}


/*!
  Update the moving edges at the end of the virtual visual servoing.
  
  \param I : the image.
*/
void
vpMbEdgeTracker::updateMovingEdge(const vpImage<unsigned char> &I)
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    l->updateMovingEdge(I, cMo) ;
    if (l->nbFeature == 0 && l->isVisible()) l->Reinit = true;
    Lline.next() ;
  }
}


/*!
  Reinitialize the lines if it is required.
  
  A line is reinitialized if the 2D line do not match enough with the projected 3D line.
  
  \param I : the image.
  \param _cMo : the pose of the used to re-initialise the moving edges
*/
void
vpMbEdgeTracker::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l->Reinit == true  && l->isVisible() == true)
      l->reinitMovingEdge(I, _cMo);
    Lline.next() ;
  }
}


/*!
  Check if two vpPoints are similar.
  
  To be similar : \f$ (X_1 - X_2)^2 + (Y_1 - Y_2)^2 + (Z_1 - Z_2)^2 < threshold \f$.
  
  \param P1 : The first point to compare
  \param P2 : The second point to compare
  \param threshold : The threshold  used to decide if the points are similar or not.
*/
bool samePoint(const vpPoint &P1, const vpPoint &P2, double threshold=1e-5)
{
  double d = vpMath::sqr(P1.get_oX() - P2.get_oX())+
  vpMath::sqr(P1.get_oY() - P2.get_oY())+
  vpMath::sqr(P1.get_oZ() - P2.get_oZ()) ;
  if (d  < threshold)
    return true ;
  else
    return false ;
}


/*!
  Add a line belonging to the \f$ index \f$ th polygone to the list of lines. It is defined by its two extremities.
  
  If the line already exists, the ploygone's index is added to the list of polygon to which it belongs.
  
  \param P1 : The first extremity of the line.
  \param P2 : The second extremity of the line.
  \param polygone : The index of the polygon to which the line belongs.
  \param name : the optionnal name of the line 
*/
void
vpMbEdgeTracker::addLine(vpPoint &P1, vpPoint &P2, int polygone, std::string name)
{
  //suppress line already in the model
  
  bool already_here = false ;
  vpMbtDistanceLine *l ;

  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (samePoint(*(l->p1),P1))
    {
      if (samePoint(*(l->p2),P2))
      {
        already_here = true ;
        l->Lindex_polygon += polygone ;
        l->hiddenface = &faces ;
      }
    }
    
    if (samePoint(*(l->p1),P2))
    {
      if (samePoint(*(l->p2),P1))
      {
        l->Lindex_polygon += polygone ;
        already_here = true ;
        l->hiddenface = &faces ;
      }
    }
    Lline.next() ;
  }

  if (!already_here)
  {
    l = new vpMbtDistanceLine ;

    l->setCameraParameters(&cam) ;
    l->buildFrom(P1,P2) ;
    l->Lindex_polygon += polygone ;
    l->setMovingEdge(&me) ;
    l->hiddenface = &faces ;
    l->setIndex(nline) ;
    l->setName(name);
    nline +=1 ;
    Lline += l ;
  }
  else
  {
    //vpTRACE("This line is already in the model") ;
  }
}

/*!
  Remove a line using its name. 
  
  \param name : The name of the line to remove. 
*/
void
vpMbEdgeTracker::removeLine(const std::string& name)
{
  vpMbtDistanceLine *l;
  Lline.front();
  while (!Lline.outside())
  {
    l = Lline.value();
    if (name.compare(l->getName()) == 0)
    {
      Lline.suppress();
      break;
    }
    Lline.next();
  }
}


/*!
  Add a polygon to the list of polygons.
  
  \param p : The polygon to add.
*/
void
vpMbEdgeTracker::addPolygon(vpMbtPolygon &p)
{
  p.setIndex(index_polygon) ;
  faces.addPolygon(&p) ;

  int nbpt = p.getNbPoint() ;
  for (int i=0 ; i < nbpt-1 ; i++)
    addLine(p.p[i],p.p[i+1],index_polygon) ;
  addLine(p.p[nbpt-1],p.p[0],index_polygon) ;

  index_polygon++ ;
}


/*!
  Detect the visible faces in the image and says if a new one appeared.
  
  \param cMo : The pose of the camera used to project the 3D model into the image.
  \param newvisibleline : This parameter is set to true if a new face appeared.
*/
void
vpMbEdgeTracker::visibleFace(const vpHomogeneousMatrix &cMo, bool &newvisibleline)
{
  int n ;

  n = faces.setVisible(cMo) ;
//  cout << "visible face " << n << endl ;
  if (n > nbvisiblepolygone)
  {
    //cout << "une nouvelle face est visible " << endl ;
    newvisibleline = true ;
  }
  else
    newvisibleline = false ;

  nbvisiblepolygone= n ;
}


/*!
  Load a 3D model contained in a file.
  
  \param file : Path to the file containing the 3D model description.
*/
void
vpMbEdgeTracker::loadModel(const char* file)
{
  std::string model(file);
  vpMbTracker::loadModel(model);
}


/*!
  Add a face to track from its corners (in the object frame). This method is
  called from the loadModel() one to add a face of the object to track. 
  The initialisation of the face depends on the primitive to track.
  
  \param _corners : The vector of corners representing the face.
  \param _indexFace : The index of the face.
*/
void 
vpMbEdgeTracker::initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace)
{
  vpMbtPolygon *polygon = NULL;
  polygon = new vpMbtPolygon;
  polygon->setNbPoint(_corners.size());
  polygon->setIndex(_indexFace);
  for(unsigned int j = 0; j < _corners.size(); j++) {
    polygon->addPoint(j, _corners[j]);
  }
  addPolygon(*polygon);

  delete polygon;
  polygon = NULL;
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose. 
  
*/
void
vpMbEdgeTracker::resetTracker()
{
  this->cMo.setIdentity();
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l!=NULL) delete l ;
    l = NULL ;
    Lline.next() ;
  }
  Lline.kill() ;
  
  faces.reset();
  
  index_polygon =0;
  compute_interaction=1;
  nline = 0;
  lambda = 1;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
}



/*!
  Re-initialise the model used by the tracker.  
  
  \param _I : The image containing the object to initialize.
  \param _cad_name : Path to the file containing the 3D model description.
  \param _cMo : The new vpHomogeneousMatrix between the camera and the new model
*/
void
vpMbEdgeTracker::reInitModel(const vpImage<unsigned char>& _I, const char* _cad_name, const vpHomogeneousMatrix& _cMo)
{
  resetTracker();
  loadModel(_cad_name);
  init(_I, _cMo);
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a 
  vpMeSite with its flag "suppress" equal to 0. Only these points are used 
  during the virtual visual servoing stage. 
  
  \return the number of good points. 
*/
unsigned int 
vpMbEdgeTracker::getNbPoints()
{
  unsigned int nbGoodPoints = 0;
  vpMbtDistanceLine *l ;
  Lline.front() ;
  while (!Lline.outside())
  {
    l = Lline.value() ;
    if (l->isVisible() && l->meline != NULL)
    {
      l->meline->list.front();
      while (!l->meline->list.outside())
      {
	      if (l->meline->list.value().suppress == 0) nbGoodPoints++;
	      l->meline->list.next();
      }
    }
    Lline.next();
  }
  return nbGoodPoints;
}


/*!
  Return the polygon (face) "index".
  
  \exception vpException::dimensionError if index does not represent a good 
  polygon.
  
  \param _index : Index of the polygon to return.
  \return Pointer to the polygon index.
*/
vpMbtPolygon* 
vpMbEdgeTracker::getPolygon(const unsigned int _index)
{
  if(_index >= static_cast<unsigned int>(faces.getPolygon().nb) ){
    throw vpException(vpException::dimensionError, "index out of range");
  }
  faces.getPolygon().front();
  for(unsigned int i=0; i<_index; i++){
    faces.getPolygon().next();
  }
  return faces.getPolygon().value();
}

/*!
  Get the number of polygon (face) representing the object to track.
  
  \return Number of polygon.
*/
unsigned int 
vpMbEdgeTracker::getNbPolygon() 
{
  return static_cast<unsigned int>(faces.getPolygon().nb);
}

