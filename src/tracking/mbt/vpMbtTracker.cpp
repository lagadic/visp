/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbtTracker.cpp
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

#include <visp/vpMbtTracker.h>
#include <visp/vpMbtDistanceLine.h>
#include <visp/vpMbtXmlParser.h>

#include <string>


/*!
  Basic constructor
*/
vpMbtTracker::vpMbtTracker()
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

  caoPolygonPoint = NULL;
  caoPolygonLine = NULL;
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtTracker::~vpMbtTracker()
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
vpMbtTracker::setMovingEdge(vpMe &_me)
{
  me.setThreshold (_me.threshold);
  me.setPointsToTrack (_me.points_to_track);
  me.setAngleStep (_me.anglestep);
  me.setRange (_me.range);
  me.setMu1 (_me.mu1);
  me.setMu2 (_me.mu2);
  me.setMaskNumber (_me.n_mask);
  me.setMaskSign (_me.mask_sign);
  me.setMaskSize (_me.mask_size);
  me.setSampleStep (_me.sample_step);
  me.setStrip (_me.strip);
  me.setMinSamplestep (_me.min_samplestep);
  me.setAberration (_me.aberration);
  me.setInitAberration (_me.init_aberration);

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
 */
void
vpMbtTracker::computeVVS()
{
  double residu_1 =1e3;
  double r =1e3-1;

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
    vpERROR_TRACE("\n\t\t Erreur-> plus de primitive...") ;
    throw std::string("\n\t\t Erreur-> plus de primitive...");
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

      L.pseudoInverse(Lp,1e-6) ;
    }

    v = -0.7*Lp*weighted_error;
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

      L.pseudoInverse(Lp,1e-6) ;
    }

    v = -lambda*Lp*weighted_error;
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
*/
void
vpMbtTracker::testTracking()
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
    throw  std::string("test Tracking fail");;
  }
}


/*!
  Compute each state of the tracking procedure for all the feature sets.
  
  If the tracking is considered as failed an exception is thrown.
  
  \param I : The image.
 */
void
vpMbtTracker::track(vpImage<unsigned char> &I)
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
    computeVVS();
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
 Initialize the tracking by clicking on several points used to compute the initial pose.
 The 3D coordinates of the points have to be stored in a file.
 The order of the points is given by the order of the 3D coordinates in the file.
 
 \param I : the image containing the object to initialize.
 \param filename : Path to the file containing the 3D coordinates.
 \param displayHelp : If this flag is true, then an image (name : 'filename'.ppm) is displayed to show where to click
*/
void
vpMbtTracker::initClick(vpImage<unsigned char>& I, const char *filename, bool displayHelp)
{
  vpHomogeneousMatrix last_cMo;
  vpPoseVector init_pos;

  // Load the last poses from files
  std::fstream finitpos ;
  std::fstream finit ;
  char s[FILENAME_MAX];

  sprintf(s,"%s.0.pos",modelFileName.c_str());
  finitpos.open(s,std::ios::in) ;
  if(finitpos.fail() ){
  	std::cout << "cannot read " << s << std::endl << "cMo set to identity" << std::endl;
  	last_cMo.setIdentity();
  }
  else{
    finitpos >> init_pos[0];
    finitpos >> init_pos[1];
    finitpos >> init_pos[2];
    finitpos >> init_pos[3];
    finitpos >> init_pos[4];
    finitpos >> init_pos[5];

    finitpos.close();
    last_cMo.buildFrom(init_pos) ;
  }
  std::cout <<"last_cMo : "<<std::endl << last_cMo <<std::endl;

  display(I, last_cMo, cam, vpColor::green);
  vpDisplay::displayFrame(I, last_cMo, cam, 0.05, vpColor::green);
  vpDisplay::flush(I);

  std::cout << "No modification : left click " << std::endl;
  std::cout << "Modify initial pose : right click " << std::endl ;

  vpDisplay::displayCharString(I, 15, 10,
			       "left click to validate, right click to modify initial pose",
			       vpColor::red);

  vpDisplay::flush(I) ;

  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  while (!vpDisplay::getClick(I, ip, button)) ;


  if (button == vpMouseButton::button1)
  cMo = last_cMo ;
  else
  {
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    vpPose pose ;

    pose.clearPoint() ;

    // lecture du fichier
    // nom de l'image
    // nombre de points
    // X Y Z
    // X Y Z

    double X,Y,Z ;
    int i ;
    sprintf(s,"%s.init",filename);
    std::cout << "filename " << s << std::endl ;
    finit.open(s,std::ios::in) ;
    if (finit.fail())
    {
      std::cout << "cannot read " << s << "enter a character to continue" << std::endl;
      throw std::string("cannot read init file");
    }

    sprintf(s,"%s.ppm",filename);

    vpImage<vpRGBa> Iref ;
    //Display window creation and initialistation
#if defined VISP_HAVE_X11
    vpDisplayX d;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d;
#endif
    try{
      if(displayHelp){
        vpImageIo::readPPM(Iref,s) ;
	#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
        d.init(Iref,10,500, "Where to initialize...")  ;
	  	  vpDisplay::display(Iref) ;
	  	  vpDisplay::flush(Iref);
	#endif
	  	}
    }
    catch(...){}

    int n ;
    finit >> n ;
    std::cout << "number of points  " << n << std::endl ;
    vpPoint *P = new vpPoint [n]  ;
    for (i=0 ; i < n ; i++)
    {
      finit >> X ;
      finit >> Y ;
      finit >> Z ;
      P[i].setWorldCoordinates(X,Y,Z) ; // (X,Y,Z)
    }

    finit.close();

////////////////////////////////
    bool isWellInit = false;
    while(!isWellInit)
    {
////////////////////////////////
      for(int i=0 ; i< n ; i++)
      {
        std::cout << "Click on point " << i+1 << std::endl ;
        double x=0,y=0;
        vpDisplay::getClick(I, ip) ;
        vpDisplay::displayCross(I, ip, 5,vpColor::green) ;
        vpDisplay::flush(I) ;
        vpPixelMeterConversion::convertPoint(cam, ip, x, y);
        P[i].set_x(x);
        P[i].set_y(y);

        std::cout << "click sur point " << ip << std::endl;

        P[i].display(I,cam,vpColor::green) ;  //display target point
        pose.addPoint(P[i]) ; // and added to the pose computation point list
      }
      vpDisplay::flush(I) ;

      vpHomogeneousMatrix cMo1, cMo2;
      pose.computePose(vpPose::LAGRANGE, cMo1) ;
      double d1 = pose.computeResidual(cMo1);
      pose.computePose(vpPose::DEMENTHON, cMo2) ;
      double d2 = pose.computeResidual(cMo2);
      
      if(d1 < d2){
        cMo = cMo1;
      }
      else{
        cMo = cMo2;
      }
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      std::cout << "cMo:" << std::endl << cMo << std::endl;

      display(I, cMo, cam, vpColor::green);
      vpDisplay::displayCharString(I, 15, 10,
				 "left click to validate, right click to re initialize object",
				 vpColor::red);

      vpDisplay::flush(I) ;

      vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
      while (!vpDisplay::getClick(I, ip, button)) ;


      if (button == vpMouseButton::button1)
      {
        isWellInit = true;
      }
      else
      {
        pose.clearPoint() ;
        vpDisplay::display(I) ;
        vpDisplay::flush(I) ;
      }
    }
////////////////////////////////////

    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::red);

    delete [] P;

	//save the pose into file
//	sprintf(s,"%s.0.pos",filename);
  sprintf(s,"%s.0.pos",modelFileName.c_str());
	finitpos.open(s,std::ios::out) ;
	init_pos.buildFrom(cMo);
	finitpos << init_pos;
	finitpos.close();
  }

  //save the pose into file
  sprintf(s,"%s.0.pos",filename);
  finitpos.open(s,std::ios::out) ;
  init_pos.buildFrom(cMo);
  finitpos << init_pos;
  finitpos.close();

  std::cout <<"cMo : "<<std::endl << cMo <<std::endl;

  bool bo ;
  visibleFace(cMo, bo) ;


  initMovingEdge(I,cMo) ;
}


/*!
 Initialize the tracking thanks to the initial pose of the camera.
 
 \param I :The image.
 \param _cMo : The initial pose used to initialize the tracking.
*/
void
vpMbtTracker::init(vpImage<unsigned char>& I, vpHomogeneousMatrix &_cMo)
{
  this->cMo = _cMo;
  bool a = false;
  visibleFace(_cMo, a);
  initMovingEdge(I,_cMo);
}


/*!
  Load the xml configuration file.
  Write the parameters in the corresponding objects ( Ecm,vpMbtTracker )

  \param filename : full name of the xml file.
*/
void
vpMbtTracker::loadConfigFile(const char* filename)
{
#ifdef VISP_HAVE_XML2
  vpMbtXmlParser xmlp;

  // remove the caracter ->" ( 34 in ascii code)
  int i=0;
  int j=0;
  char str[FILENAME_MAX];

  for(i=0;i<(int)strlen(filename);i++)
  {
    while(filename[i]==34)
      i++;
    str[j]=filename[i];
    j++;
  }
  str[j]='\0';

//  printf("try to load : %s\n",str);

  if(xmlp.Parse(filename)!=vpMbtXmlParser::SEQUENCE_OK)
  {
    vpERROR_TRACE("Can't open XML file \"%s\"\n ",filename);
    exit(-1);
  }
//  vpTRACE(" ");
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
  Display the 3D model from a given position of the camera

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbtTracker::display(vpImage<unsigned char>& I, vpHomogeneousMatrix &_cMo, vpCameraParameters &cam,
											vpColor col,
											unsigned int thickness)
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
vpMbtTracker::initMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &_cMo)
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
vpMbtTracker::trackMovingEdge(vpImage<unsigned char> &I)
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
vpMbtTracker::updateMovingEdge(vpImage<unsigned char> &I)
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
*/
void
vpMbtTracker::reinitMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &_cMo)
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
bool samePoint(vpPoint &P1, vpPoint &P2, double threshold=1e-5)
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
*/
void
vpMbtTracker::addLine(vpPoint &P1, vpPoint &P2, int polygone, std::string name)
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

void
vpMbtTracker::removeLine(std::string name)
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
  Add a polygon to the list of polygons
  
  \param p : The polygon to add.
*/
void
vpMbtTracker::addPolygon(vpMbtPolygon &p)
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
vpMbtTracker::visibleFace(vpHomogeneousMatrix &cMo, bool &newvisibleline)
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
vpMbtTracker::loadModel(const char* file)
{
  std::ifstream infile;

  std::string str (file);
  std::string::iterator it;

  infile.open (file, std::ifstream::in);
  if(infile.fail() )
  {
    std::cout << "cannot read model file" << file << std::endl;
    throw std::string("cannot read model file");
  }

  it = str.end();

  if( *(it-1) == 'o' && *(it-2) == 'a' && *(it-3) == 'c' && *(it-4) == '.')
  {
    loadCAOModel(infile);
  }

  if( *(it-1) == 'l' && *(it-2) == 'r' && *(it-3) == 'w' && *(it-4) == '.')
  {
#if defined(VISP_HAVE_COIN) || defined(NMBT_HAVE_COIN)
    loadVRMLModel(file);
#else
    std::cout << "Coin not installed, cannot read VRML files" << std::endl;
    throw std::string("Coin not installed, cannot read VRML files");
#endif
  }
  
  modelFileName = file;
  modelFileName = modelFileName.substr(0, modelFileName.size() - 4);

  infile.close();

}


/*!
  Load a 3D model contained in a .cao file.
  
  \param file : Path to the .cao file containing the 3D model description.
*/
void
vpMbtTracker::loadCAOModel(std::ifstream &file_id)
{
  //On nettoie d'abord
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

  char c;
  int k;
  //On regarde la version
  while( (file_id.get(c)!=NULL)&&(c == '#')) file_id.ignore(256,'\n');
  file_id.unget();

  int caoVersion;
  file_id.get(c);
  if(c=='V')
  {
    file_id >> caoVersion;
  }
  else
  {
    std::cout <<"in mbtCadModel::Load -> Bad parameter header file : use V0, V1, ...";
    throw;
  }

//   while( (file_id.get(c)!=NULL)&&(c!='\n'));
  while( (file_id.get(c)!=NULL)&&(c!='\n')) ;
  while( (file_id.get(c)!=NULL)&&(c == '#')) file_id.ignore(256,'\n') ;
  file_id.unget();

  //Read the points
  int caoNbrPoint;
  file_id >> caoNbrPoint;
  std::cout << "> " << caoNbrPoint << " points" << std::endl;
  vpPoint *caoPoints = NULL;
  if (caoNbrPoint > 0)
    caoPoints = new vpPoint[caoNbrPoint];

  double x ; // 3D coordinates
  double y ;
  double z ;

  int i ;    // image coordinate (used for matching)
  int j ;


  for (k=0; k < caoNbrPoint; k++)
  {
    file_id >> x ;
    file_id >> y ;
    file_id >> z ;
    if (caoVersion == 2)
    {
      file_id >> i ;
      file_id >> j ;
    }

    caoPoints[k].setWorldCoordinates(x, y, z) ;
  }

  while( (file_id.get(c)!=NULL)&&(c!='\n')) ;
  while( (file_id.get(c)!=NULL)&&(c == '#')) file_id.ignore(256,'\n');
  file_id.unget();


  //Read the lines
  int caoNbrLine;
  file_id >> caoNbrLine;
  int *caoLinePoints = NULL;
  std::cout << "> " << caoNbrLine<< " lines" << std::endl;
  if (caoNbrLine > 0)
    caoLinePoints = new int[2*caoNbrLine];

  int index1, index2;

  for (k=0; k < caoNbrLine ; k++)
  {
    file_id >> index1 ;
    file_id >> index2 ;

    caoLinePoints[2*k] = index1;
    caoLinePoints[2*k+1] = index2;
    addLine(caoPoints[index1], caoPoints[index2]);
    file_id >> index1 ;
    file_id >> index2 ;
  }

  while( (file_id.get(c)!=NULL)&&(c!='\n')) ;
  while( (file_id.get(c)!=NULL)&&(c == '#')) file_id.ignore(256,'\n');
  file_id.unget();


  int caoNbrPolygonLine;
  file_id >> caoNbrPolygonLine;
  std::cout << "> " << caoNbrPolygonLine << " polygon line" << std::endl;
  if (caoNbrPolygonLine > 0)
    caoPolygonLine = new vpMbtPolygon[caoNbrPolygonLine];

  int index;
  for (k = 0;k < caoNbrPolygonLine; k++)
  {
    int nbLinePol;
    file_id >> nbLinePol;
    caoPolygonLine[k].setNbPoint(nbLinePol);
    for(int i = 0; i < nbLinePol; i++)
    {
      file_id >> index;
      caoPolygonLine[k].addPoint(i,caoPoints[caoLinePoints[2*index]]);
    }
    addPolygon(caoPolygonLine[k]);
  }

  //delete[] caoPolygonLine;

  while( (file_id.get(c)!=NULL)&&(c!='\n')) ;
  while( (file_id.get(c)!=NULL)&&(c == '#')) file_id.ignore(256,'\n');
  file_id.unget();


  int caoNbrPolygonPoint;
  file_id >> caoNbrPolygonPoint;
  std::cout << "> " << caoNbrPolygonPoint << " polygon point" << std::endl;
  if (caoNbrPolygonPoint > 0)
    caoPolygonPoint = new vpMbtPolygon[caoNbrPolygonPoint];

  for (k = 0;k < caoNbrPolygonPoint; k++)
  {
    int nbPointPol;
    file_id >> nbPointPol;
    caoPolygonPoint[k].setNbPoint(nbPointPol);
    for(int i = 0; i < nbPointPol; i++)
    {
      file_id >> index;
      caoPolygonPoint[k].addPoint(i,caoPoints[index]);
    }
    file_id >> index;
    addPolygon(caoPolygonPoint[k]);
  }
  //delete[] caoPolygonPoint;
}

#if defined(VISP_HAVE_COIN) || defined(NMBT_HAVE_COIN)
/*!
  Load a 3D model contained in a .wrl file.
  
  \param file : Path to the .wrl file containing the 3D model description.
*/
void
vpMbtTracker::loadVRMLModel(const char* file_id)
{
  //Load the sceneGraph
  SoDB::init();
  SoInput in;
  SbBool ok = in.openFile(file_id);
  SoSeparator  *sceneGraph;
  SoVRMLGroup  *sceneGraphVRML2;

  if (!ok) {
    vpERROR_TRACE("can't open file \"%s\" \n Please check the Marker_Less.ini file", file_id);
    exit(1);
  }

  if(!in.isFileVRML2())
  {
    sceneGraph = SoDB::readAll(&in);
    if (sceneGraph == NULL) { /*return -1;*/ }
    sceneGraph->ref();

    SoToVRML2Action tovrml2;
    tovrml2.apply(sceneGraph);
    sceneGraphVRML2 =tovrml2.getVRML2SceneGraph();
    sceneGraphVRML2->ref();
    sceneGraph->unref();
  }
  else
  {
    sceneGraphVRML2	= SoDB::readAllVRML(&in);
    if (sceneGraphVRML2 == NULL) { /*return -1;*/ }
    sceneGraphVRML2->ref();
  }

  in.closeFile();

  int nbShapes = sceneGraphVRML2->getNumChildren();

  SoNode * child;

  for (int i = 0; i < nbShapes; i++)
  {
    child = sceneGraphVRML2->getChild(i);
    if (child->getTypeId() == SoVRMLShape::getClassTypeId())
    {
      SoChildList * child2list = child->getChildren();
      for (int j = 0; j < child2list->getLength(); j++)
      {
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedFaceSet::getClassTypeId())
        {
          SoVRMLIndexedFaceSet * face_set;
          face_set = (SoVRMLIndexedFaceSet*)child2list->get(j);
          extractFaces(face_set);
        }
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedLineSet::getClassTypeId())
        {
          SoVRMLIndexedLineSet * line_set;
          line_set = (SoVRMLIndexedLineSet*)child2list->get(j);
          extractLines(line_set);
        }
      }
    }
  }
}



void
vpMbtTracker::extractFaces(SoVRMLIndexedFaceSet* face_set)
{
  vpList<vpPoint> pointList;
  pointList.kill();
  SoMFInt32 indexList = face_set->coordIndex;
  int indexListSize = indexList.getNum();

  SbVec3f point(0,0,0);
  vpPoint pt;
  SoVRMLCoordinate *coord;

  vpMbtPolygon *polygon = NULL;


  for (int i = 0; i < indexListSize; i++)
  {
    if (face_set->coordIndex[i] == -1)
    {
      if(pointList.nbElements() > 0)
      {
        polygon = new vpMbtPolygon;
        polygon->setNbPoint(pointList.nbElements());
        pointList.front();
        for(int j = 0; j < pointList.nbElements(); j++)
        {
          polygon->addPoint(j, pointList.value());
          pointList.next();
        }
        addPolygon(*polygon);
	
        delete polygon;
        polygon = NULL;
        pointList.kill();
      }
    }
    else
    {
      coord = (SoVRMLCoordinate *)(face_set->coord.getValue());
      int index = face_set->coordIndex[i];
      point[0]=coord->point[index].getValue()[0];
      point[1]=coord->point[index].getValue()[1];
      point[2]=coord->point[index].getValue()[2];

      pt.setWorldCoordinates(point[0],point[1],point[2]);
      pointList.addRight(pt);
    }
  }
}



void
vpMbtTracker::extractLines(SoVRMLIndexedLineSet* line_set)
{
  vpList<vpPoint> pointList;
  pointList.kill();
  SoMFInt32 indexList = line_set->coordIndex;
  int indexListSize = indexList.getNum();

  SbVec3f point(0,0,0);
  vpPoint pt;
  SoVRMLCoordinate *coord;

  vpMbtPolygon *polygon = NULL;


  for (int i = 0; i < indexListSize; i++)
  {
    if (line_set->coordIndex[i] == -1)
    {
      if(pointList.nbElements() > 1)
      {
        polygon = new vpMbtPolygon;
        polygon->setNbPoint(pointList.nbElements());
        pointList.front();
        for(int j = 0; j < pointList.nbElements(); j++)
        {
          polygon->addPoint(j, pointList.value());
          pointList.next();
        }
        addPolygon(*polygon);
	
        delete polygon;
        polygon = NULL;
        pointList.kill();
      }
    }
    else
    {
      coord = (SoVRMLCoordinate *)(line_set->coord.getValue());
      int index = line_set->coordIndex[i];
      point[0]=coord->point[index].getValue()[0];
      point[1]=coord->point[index].getValue()[1];
      point[2]=coord->point[index].getValue()[2];

      pt.setWorldCoordinates(point[0],point[1],point[2]);
      pointList.addRight(pt);
    }
  }
}
#endif

/*!
  reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose. 
  
*/
void
vpMbtTracker::resetTracker()
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
  if(caoPolygonPoint != NULL){
    delete[] caoPolygonPoint;
    caoPolygonPoint = NULL;
  }
  if(caoPolygonLine != NULL){
    delete[] caoPolygonLine;
    caoPolygonLine = NULL;
  }
  
  faces.reset();
  
  index_polygon =0;
  compute_interaction=1;
  nline = 0;
  lambda = 1;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
}



/*!
  re-init the model used by the tracker.  
  
 \param I : the image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param _cMo : the new vpHomogeneousMatrix between the camera and the new model
*/
void
vpMbtTracker::reInitModel(vpImage<unsigned char>& I, const char* cad_name, vpHomogeneousMatrix& _cMo)
{
  resetTracker();
  loadModel(cad_name);
  init(I, _cMo);
}


