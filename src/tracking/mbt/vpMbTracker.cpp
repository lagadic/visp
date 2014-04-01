/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Generic model based tracker
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.cpp
  \brief Generic model based tracker
*/


#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpColVector.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpColor.h>
#include <visp/vpIoTools.h>
#include <visp/vpException.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbTracker.h>
#include <visp/vpMatrixException.h>
#include <visp/vpIoTools.h>
#include <iostream>
#include <limits>
#ifdef VISP_HAVE_COIN
//Inventor includes
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#endif


/*!
  Basic constructor.
  Set default values.

*/
vpMbTracker::vpMbTracker()
  : cam(), cMo(), modelFileName(), modelInitialised(false),
    poseSavingFilename(), computeCovariance(false), covarianceMatrix(), displayFeatures(false)
{
}

/*!
  Basic destructor that doest nothing.
*/
vpMbTracker::~vpMbTracker()
{
}


/*!
  Initialise the tracking by clicking on the image points corresponding to the 
  3D points (object frame) in the file initFile. The structure of this file
  is (without the comments):
  \code
  4 // Number of points in the file (minimum is four)
  0.01 0.01 0.01    //  \
  ...               //  | 3D coordinates in the object basis
  0.01 -0.01 -0.01  // /
  \endcode

  \param I : Input image
  \param initFile : File containing the points where to click
  \param displayHelp : Optionnal display of an image ( 'initFile.ppm' ). This
    image may be used to show where to click. 

  \exception vpException::ioError : The file specified in initFile doesn't exist.

  \sa setPathNamePoseSaving()

*/
void
vpMbTracker::initClick(const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp)
{
  vpHomogeneousMatrix last_cMo;
  vpPoseVector init_pos;
	vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  
	std::string ext = ".init";
	std::string str_pose = "";
  size_t pos =  (unsigned int)initFile.rfind(ext);

  // Load the last poses from files
  std::fstream finitpos ;
  std::fstream finit ;
  char s[FILENAME_MAX];
  if(poseSavingFilename.empty()){
    if( pos == initFile.size()-ext.size() && pos != 0)
      str_pose = initFile.substr(0,pos) + ".0.pos";
		else
      str_pose =  initFile + ".0.pos";
		
    finitpos.open(str_pose.c_str() ,std::ios::in) ;
		sprintf(s, "%s", str_pose.c_str());
  }else{
    finitpos.open(poseSavingFilename.c_str() ,std::ios::in) ;
    sprintf(s, "%s", poseSavingFilename.c_str());
  }
  if(finitpos.fail() ){
  	std::cout << "cannot read " << s << std::endl << "cMo set to identity" << std::endl;
  	last_cMo.setIdentity();
  }
  else{
    for (unsigned int i = 0; i < 6; i += 1){
      finitpos >> init_pos[i];
    }

    finitpos.close();
    last_cMo.buildFrom(init_pos) ;
    
    std::cout <<"last_cMo : "<<std::endl << last_cMo <<std::endl;

    vpDisplay::display(I);
    display(I, last_cMo, cam, vpColor::green, 1, true);
    vpDisplay::displayFrame(I, last_cMo, cam, 0.05, vpColor::green);
    vpDisplay::flush(I);

    std::cout << "No modification : left click " << std::endl;
    std::cout << "Modify initial pose : right click " << std::endl ;

    vpDisplay::displayCharString(I, 15, 10,
              "left click to validate, right click to modify initial pose",
              vpColor::red);

    vpDisplay::flush(I) ;

    while (!vpDisplay::getClick(I, ip, button)) ;
  }


  if (!finitpos.fail() && button == vpMouseButton::button1){
    cMo = last_cMo ;
  }
  else
  {
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    vpPose pose ;

    pose.clearPoint() ;

    // file parser
    // number of points
    // X Y Z
    // X Y Z

    double X,Y,Z ;
    
    if( pos == initFile.size()-ext.size() && pos != 0)
      sprintf(s,"%s", initFile.c_str());
		else
      sprintf(s,"%s.init", initFile.c_str());
	
    std::cout << "filename " << s << std::endl ;
    finit.open(s,std::ios::in) ;
    if (finit.fail()){
      std::cout << "cannot read " << s << std::endl;
	    throw vpException(vpException::ioError, "cannot read init file");
    }

		std::string dispF;
    if( pos == initFile.size()-ext.size() && pos != 0)
      dispF = initFile.substr(0,pos) + ".ppm";
		else
      dispF = initFile + ".ppm";
    
		sprintf(s, "%s", dispF.c_str());

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
        vpImageIo::read(Iref,s) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
        d.init(Iref,10,500, "Where to initialize...")  ;
    	  vpDisplay::display(Iref) ;
    	  vpDisplay::flush(Iref);
#endif
	  	}
    }
    catch(...){}

    unsigned int n ;
    finit >> n ;
    std::cout << "number of points  " << n << std::endl ;
    if (n > 100000) {
      throw vpException(vpException::badValue,
        "Exceed the max number of points.");
    }

    vpPoint *P = new vpPoint [n]  ;
    for (unsigned int i=0 ; i < n ; i++){
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
      for(unsigned int i=0 ; i< n ; i++)
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

        vpDisplay::displayPoint (I, ip, vpColor::green); //display target point
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

      display(I, cMo, cam, vpColor::green, 1, true);
      vpDisplay::displayCharString(I, 15, 10,
				 "left click to validate, right click to re initialize object",
				 vpColor::red);

      vpDisplay::flush(I) ;

      button = vpMouseButton::button1;
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
		if(poseSavingFilename.empty())
			savePose(str_pose);
		else
			savePose(poseSavingFilename);
	}

  std::cout <<"cMo : "<<std::endl << cMo <<std::endl;

  init(I);
}

/*!
  Initialise the tracking by clicking on the image points corresponding to the 
  3D points (object frame) in the list points3D_list. 
  
  \param I : Input image
  \param points3D_list : List of the 3D points (object frame).
  \param displayFile : Path to the image used to display the help. 
*/
void vpMbTracker::initClick(const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
                            const std::string &displayFile)
{
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

	vpPose pose ;
  vpPoint *P = NULL; P = new vpPoint [points3D_list.size()]  ;
	for (unsigned int i=0 ; i < points3D_list.size() ; i++)
		P[i].setWorldCoordinates(points3D_list[i].get_oX(),points3D_list[i].get_oY(),points3D_list[i].get_oZ()) ; 
  
	vpImage<vpRGBa> Iref ;
	//Display window creation and initialistation
	#if defined VISP_HAVE_X11
		vpDisplayX d;
	#elif defined VISP_HAVE_GDI
		vpDisplayGDI d;
	#elif defined VISP_HAVE_OPENCV
		vpDisplayOpenCV d;
	#endif
			
	if(displayFile != ""){	
		try{
			std::cout << displayFile.c_str() << std::endl;
      vpImageIo::read(Iref,displayFile.c_str()) ;
			#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
				d.init(Iref,10,500, "Where to initialize...")  ;
				vpDisplay::display(Iref) ;
				vpDisplay::flush(Iref);
			#endif
		}
		catch(...){}
	}
	
	vpImagePoint ip;
	bool isWellInit = false;
	while(!isWellInit)
	{
		for(unsigned int i=0 ; i< points3D_list.size() ; i++)
		{
			std::cout << "Click on point " << i+1 << std::endl ;
			double x=0,y=0;
      vpDisplay::getClick(I, ip) ;
      vpDisplay::displayCross(I, ip, 5,vpColor::green) ;
      vpDisplay::flush(I) ;
			vpPixelMeterConversion::convertPoint(cam, ip, x, y);
			P[i].set_x(x);
			P[i].set_y(y);

			std::cout << "Click on point " << ip << std::endl;

      vpDisplay::displayPoint (I, ip, vpColor::green); //display target point
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

    display(I, cMo, cam, vpColor::green, 1, true);
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

  vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::red);

	delete [] P;

  init(I);
}

/*!
  Initialise the tracking by reading the 3D points (object frame) and the image points
  in initFile. The structure of this file is (without the comments):
  \code
  4 // Number of 3D points in the file (minimum is four)
  0.01 0.01 0.01    //  \
  ...               //  | 3D coordinates in meters in the object frame
  0.01 -0.01 -0.01  // /
  4 // Number of image points in the file (has to be the same as the number of 3D points)
  100 200    //  \
  ...        //  | 2D coordinates in pixel in the image
  50 10  		//  /
  \endcode
  
  \param I : Input image
  \param initFile : Path to the file containing all the points.
*/
void vpMbTracker::initFromPoints( const vpImage<unsigned char>& I, const std::string& initFile )
{
	char s[FILENAME_MAX];
	std::fstream finit ;
	
	std::string ext = ".init";
  size_t pos = initFile.rfind(ext);
	
  if( pos == initFile.size()-ext.size() && pos != 0)
    sprintf(s,"%s", initFile.c_str());
	else
    sprintf(s,"%s.init", initFile.c_str());
	
	std::cout << "filename " << s << std::endl ;
	finit.open(s,std::ios::in) ;
	if (finit.fail()){
		std::cout << "cannot read " << s << std::endl;
		throw vpException(vpException::ioError, "cannot read init file");
	}
    
	unsigned int size;
	double X, Y, Z;
	finit >> size ;
  std::cout << "number of points  " << size << std::endl ;

  if (size > 100000) {
    throw vpException(vpException::badValue,
      "Exceed the max number of points.");
  }

	vpPoint *P = new vpPoint [size]; 
	vpPose pose ;
	
	for(unsigned int i=0 ; i< size ; i++)
	{
		finit >> X ;
		finit >> Y ;
		finit >> Z ;
		P[i].setWorldCoordinates(X,Y,Z) ;
	}
	
	unsigned int size2;
	double x, y;
	vpImagePoint ip;
	finit >> size2 ;
	if(size != size2)
		vpERROR_TRACE( "vpMbTracker::initFromPoints(), Number of 2D points different to the number of 3D points." );
	
	for(unsigned int i=0 ; i< size ; i++)
	{
		finit >> x;
		finit >> y;
		ip = vpImagePoint(x,y);
		vpPixelMeterConversion::convertPoint(cam, ip, x, y);
		P[i].set_x(x);
		P[i].set_y(y);
		pose.addPoint(P[i]);
	}

	vpHomogeneousMatrix cMo1, cMo2;
	pose.computePose(vpPose::LAGRANGE, cMo1) ;
	double d1 = pose.computeResidual(cMo1);
	pose.computePose(vpPose::DEMENTHON, cMo2) ;
	double d2 = pose.computeResidual(cMo2);

	if(d1 < d2)
		cMo = cMo1;
	else
		cMo = cMo2;
	
	pose.computePose(vpPose::VIRTUAL_VS, cMo);

	delete [] P;

  init(I);
}

/*!
  Initialise the tracking with the list of image points (points2D_list) and
  the list of corresponding 3D points (object frame) (points3D_list).
  
  \param I : Input image
  \param points2D_list : List of image points.
  \param points3D_list : List of 3D points (object frame). 
*/
void vpMbTracker::initFromPoints( const vpImage<unsigned char>& I, const std::vector<vpImagePoint> &points2D_list,
                                  const std::vector<vpPoint> &points3D_list )
{
	if(points2D_list.size() != points3D_list.size())
		vpERROR_TRACE( "vpMbTracker::initFromPoints(), Number of 2D points different to the number of 3D points." );
	
	size_t size = points3D_list.size();
	vpPoint *P = new vpPoint [size]; 
	vpPose pose ;
	
	for(size_t i=0 ; i< size ; i++)
	{
		P[i].setWorldCoordinates(points3D_list[i].get_oX(),points3D_list[i].get_oY(),points3D_list[i].get_oZ()) ;
		double x=0,y=0;
		vpPixelMeterConversion::convertPoint(cam, points2D_list[i], x, y);
		P[i].set_x(x);
		P[i].set_y(y);
		pose.addPoint(P[i]);
	}

	vpHomogeneousMatrix cMo1, cMo2;
	pose.computePose(vpPose::LAGRANGE, cMo1) ;
	double d1 = pose.computeResidual(cMo1);
	pose.computePose(vpPose::DEMENTHON, cMo2) ;
	double d2 = pose.computeResidual(cMo2);

	if(d1 < d2)
		cMo = cMo1;
	else
		cMo = cMo2;
	
	pose.computePose(vpPose::VIRTUAL_VS, cMo);

	delete [] P;

  init(I);
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.
  The structure of this file is (without the comments):
  \code
  // The six value of the pose vector
  0.0000    //  \
  0.0000    //  | 
  1.0000    //  | Exemple of value for the pose vector where Z = 1 meter
  0.0000    //  |
  0.0000    //  | 
  0.0000    //  /
  \endcode
  
  Where the three firsts lines refer to the translation and the three last to the rotation in thetaU parametrisation (see vpThetaUVector).
  \param I : Input image
  \param initFile : Path to the file containing the pose.
*/
void vpMbTracker::initFromPose(const vpImage<unsigned char>& I, const std::string &initFile)
{
	char s[FILENAME_MAX];
	std::fstream finit ;
	vpPoseVector init_pos;
	
	std::string ext = ".pos";
  size_t pos =  initFile.rfind(ext);
	
  if( pos == initFile.size()-ext.size() && pos != 0)
    sprintf(s,"%s", initFile.c_str());
	else
    sprintf(s,"%s.pos", initFile.c_str());
	
	std::cout << "filename " << s << std::endl ;
	finit.open(s,std::ios::in) ;
	if (finit.fail()){
		std::cout << "cannot read " << s << std::endl;
		throw vpException(vpException::ioError, "cannot read init file");
	}
	
	for (unsigned int i = 0; i < 6; i += 1){
		finit >> init_pos[i];
	}
	
	cMo.buildFrom(init_pos);
  init(I);
}

/*!
  Initialise the tracking thanks to the pose.
  
  \param I : Input image
  \param cMo_ : Pose matrix.
*/
void vpMbTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_)
{
  this->cMo = cMo_;
  init(I);
}

/*!
  Initialise the tracking thanks to the pose vector.
  
  \param I : Input image
  \param cPo : Pose vector. 
*/
void vpMbTracker::initFromPose (const vpImage<unsigned char>& I, const vpPoseVector &cPo)
{
  vpHomogeneousMatrix _cMo(cPo);
  initFromPose(I, _cMo);
}

/*!
  Save the pose in the given filename
  
  \param filename : Path to the file used to save the pose. 
*/
void vpMbTracker::savePose(const std::string &filename)
{
	vpPoseVector init_pos;
	std::fstream finitpos ;
	char s[FILENAME_MAX];
	
	sprintf(s,"%s", filename.c_str());
	finitpos.open(s, std::ios::out) ;
		
	init_pos.buildFrom(cMo);
	finitpos << init_pos;
	finitpos.close();
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the 
  loadCAOModel() method. 

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#ifdef VISP_HAVE_COIN
  SoDB::finish();
#endif
}
  \endcode


  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao. 

  \param modelFile : the file containing the model.
*/
void
vpMbTracker::loadModel(const std::string& modelFile)
{
  std::string::const_iterator it;
  
  if(vpIoTools::checkFilename(modelFile)){
    it = modelFile.end();
    if((*(it-1) == 'o' && *(it-2) == 'a' && *(it-3) == 'c' && *(it-4) == '.') || 
       (*(it-1) == 'O' && *(it-2) == 'A' && *(it-3) == 'C' && *(it-4) == '.') ){
      loadCAOModel(modelFile);
    }
    else if((*(it-1) == 'l' && *(it-2) == 'r' && *(it-3) == 'w' && *(it-4) == '.') ||
            (*(it-1) == 'L' && *(it-2) == 'R' && *(it-3) == 'W' && *(it-4) == '.') ){
      loadVRMLModel(modelFile);
    }
    else{
      throw vpException(vpException::ioError, "file cannot be open");
    }
  }
  else{
    throw vpException(vpException::ioError, "file cannot be open");
  }
  
  this->modelInitialised = true;
  this->modelFileName = modelFile;
}


/*!
  Load the 3D model of the object from a vrml file. Only LineSet and FaceSet are
  extracted from the vrml file. 

  \warning When this class is called, remember that you have to call Call
  SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#ifdef VISP_HAVE_COIN
  SoDB::finish();
#endif
}
  \endcode

  \warning The cylinders extracted using this method do not use the Cylinder
  keyword of vrml since vrml exporter such as Blender or AC3D consider a
  cylinder as an IndexedFaceSet. To test whether an indexedFaceSet is a cylinder
  or not, the name of the geometry is read. If the name begins with "cyl" then
  the faceset is supposed to be a cylinder. For example, the line
  \code
geometry DEF cyl_cylinder1 IndexedFaceSet
  \endcode
  defines a cylinder named cyl_cylinder1.

  \throw vpException::fatalError if the file cannot be open. 
  
  \param modelFile : The full name of the file containing the 3D model.
*/
void 
vpMbTracker::loadVRMLModel(const std::string& modelFile)
{
#ifdef VISP_HAVE_COIN
  SoDB::init(); // Call SoDD::finish() before ending the program.

  SoInput in;
  SbBool ok = in.openFile(modelFile.c_str());
  SoSeparator  *sceneGraph;
  SoVRMLGroup  *sceneGraphVRML2;

  if (!ok) {
    vpERROR_TRACE("can't open file to load model");
    throw vpException(vpException::fatalError, "can't open file to load model");
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

  vpHomogeneousMatrix transform;
  unsigned int indexFace = 0;
  extractGroup(sceneGraphVRML2, transform, indexFace);
  
  sceneGraphVRML2->unref();
#else
  vpERROR_TRACE("coin not detected with ViSP, cannot load model : %s", modelFile.c_str());
  throw vpException(vpException::fatalError, "coin not detected with ViSP, cannot load model");
#endif
}

/*!
  Load a 3D model contained in a .cao file.
  
  the structure of the file is (without the comments) :
  \code
  V1
  8 // Number of points describing the object
  0.01 0.01 0.01  //  \
  ...             //  | coordinates of the points in the object frame (in m.)
  0.01 0.01 0.01  // /
  3 // Number of lines to track. 
  0 2 //  \
  1 4 //  | Index of the points representing the extremities of the lines
  1 5 // /
  0 // Number of polygon (face) to track using the line previously described
  // Face described as follow : nbLine IndexLine1 indexLine2 ... indexLineN
  3 // Number of polygon (face) to track using the line previously described
  4 0 2 3 4 // Face described as follow : nbPoint IndexPoint1 IndexPoint2 ... IndexPointN
  4 1 3 5 7
  3 1 5 6 
  1 // Number of cylinder
  6 7 0.05 // Index of the limits points on the axis (used to know the 'height' of the cylinder) and radius of the cyclinder (in m.)
  \endcode
  
  \param modelFile : Full name of the .CAO file containing the model.
*/
void 
vpMbTracker::loadCAOModel(const std::string& modelFile)
{
  std::ifstream fileId;
  fileId.exceptions ( std::ifstream::failbit | std::ifstream::eofbit );
  fileId.open (modelFile.c_str(), std::ifstream::in);
  if(fileId.fail()) {
    std::cout << "cannot read CAO model file: " << modelFile << std::endl;
    throw vpException(vpException::ioError, "cannot read CAO model file");
  }

  try{
    char c;
    // Extraction of the version (remove empty line and commented ones (comment
    // line begin with the #)).
    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    fileId.get(c);
    while (!fileId.fail() && (c == '#'))
    {
      fileId.ignore(256, '\n');
      fileId.get(c);
    }
    fileId.unget();

    int caoVersion;
    fileId.get(c);
    if(c=='V'){
      fileId >> caoVersion;
    }
    else{
      std::cout <<"in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ...";
      throw vpException(vpException::badValue,
                        "in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ...");
    }

    //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
    fileId.get(c);
    while (!fileId.fail() && (c != '\n'))
    {
      fileId.get(c);
    }

    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    fileId.get(c);
    while (!fileId.fail() && (c == '#'))
    {
      fileId.ignore(256, '\n');
      fileId.get(c);
    }
    fileId.unget();

    //Read the points
    unsigned int caoNbrPoint;
    fileId >> caoNbrPoint;
    std::cout << "> " << caoNbrPoint << " points" << std::endl;
    if (caoNbrPoint > 100000) {
      throw vpException(vpException::badValue,
                        "Exceed the max number of points in the CAO model.");
    }

    if (caoNbrPoint == 0) {
      throw vpException(vpException::badValue,
                        "in vpMbTracker::loadCAOModel() -> no points are defined");

    }
    vpPoint *caoPoints = new vpPoint[caoNbrPoint];

    double x ; // 3D coordinates
    double y ;
    double z ;

    int i ;    // image coordinate (used for matching)
    int j ;

    for(unsigned int k=0; k < caoNbrPoint; k++){
      fileId >> x ;
      fileId >> y ;
      fileId >> z ;
      if (caoVersion == 2){
        fileId >> i ;
        fileId >> j ;
      }

      if(k != caoNbrPoint-1){// the rest of the line is removed (not the last one due to the need to remove possible comments).
        fileId.ignore(256,'\n');
      }
      caoPoints[k].setWorldCoordinates(x, y, z) ;
    }

    //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
    fileId.get(c);
    while (!fileId.fail() && (c != '\n'))
    {
      fileId.get(c);
    }

    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    fileId.get(c);
    while (!fileId.fail() && (c == '#'))
    {
      fileId.ignore(256, '\n');
      fileId.get(c);
    }
    fileId.unget();

    //Read the lines
    unsigned int caoNbrLine;
    fileId >> caoNbrLine;
    unsigned int *caoLinePoints = NULL;
    std::cout << "> " << caoNbrLine << " lines" << std::endl;

    if (caoNbrLine > 100000) {
      delete [] caoPoints;
      throw vpException(vpException::badValue,
                        "Exceed the max number of lines in the CAO model.");
    }

    if (caoNbrLine > 0)
      caoLinePoints = new unsigned int[2*caoNbrLine];

    unsigned int index1, index2;

    for(unsigned int k=0; k < caoNbrLine ; k++){
      fileId >> index1 ;
      fileId >> index2 ;

      caoLinePoints[2*k] = index1;
      caoLinePoints[2*k+1] = index2;

      if(index1 < caoNbrPoint && index2 < caoNbrPoint){
        std::vector<vpPoint> extremities;
        extremities.push_back(caoPoints[index1]);
        extremities.push_back(caoPoints[index2]);
        initFaceFromCorners(extremities, k);
      }
      else{
        vpTRACE(" line %d has wrong coordinates.", k);
      }

      if(k != caoNbrLine-1){// the rest of the line is removed (not the last one due to the need to remove possible comments).
        fileId.ignore(256,'\n');
      }
    }

    //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
    fileId.get(c);
    while (!fileId.fail() && (c != '\n'))
    {
      fileId.get(c);
    }

    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    fileId.get(c);
    while (!fileId.fail() && (c == '#'))
    {
      fileId.ignore(256, '\n');
      fileId.get(c);
    }
    fileId.unget();

    /* Load polygon from the lines extracted earlier
          (the first point of the line is used)*/
    unsigned int caoNbrPolygonLine;
    fileId >> caoNbrPolygonLine;
    std::cout << "> " << caoNbrPolygonLine << " polygon line" << std::endl;
    if (caoNbrPolygonLine > 100000) {
      delete [] caoPoints;
      delete [] caoLinePoints;
      throw vpException(vpException::badValue, "Exceed the max number of polygon lines.");
    }

    unsigned int index;
    for(unsigned int k = 0;k < caoNbrPolygonLine; k++){
      unsigned int nbLinePol;
      fileId >> nbLinePol;
      std::vector<vpPoint> corners;
      if (nbLinePol > 100000) {
        throw vpException(vpException::badValue, "Exceed the max number of lines.");
      }

      for(unsigned int n = 0; n < nbLinePol; n++){
        fileId >> index;
        if (2*index > 2*caoNbrLine-1) {
          throw vpException(vpException::badValue, "Exceed the max number of lines.");
        }

        corners.push_back(caoPoints[caoLinePoints[2*index]]);
      }
      if(k != caoNbrPolygonLine-1){// the rest of the line is removed (not the last one due to the need to remove possible comments).
        fileId.ignore(256,'\n');
      }
      initFaceFromCorners(corners, k);
    }

    //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
    fileId.get(c);
    while (!fileId.fail() && (c != '\n'))
    {
      fileId.get(c);
    }

    //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
    fileId.get(c);
    while (!fileId.fail() && (c == '#'))
    {
      fileId.ignore(256, '\n');
      fileId.get(c);
    }
    fileId.unget();

    /* Extract the polygon using the point coordinates (top of the file) */
    unsigned int caoNbrPolygonPoint;
    fileId >> caoNbrPolygonPoint;
    std::cout << "> " << caoNbrPolygonPoint << " polygon point" << std::endl;

    if (caoNbrPolygonPoint > 100000) {
      throw vpException(vpException::badValue, "Exceed the max number of polygon point.");
    }

    for(unsigned int k = 0;k < caoNbrPolygonPoint; k++){
      unsigned int nbPointPol;
      fileId >> nbPointPol;
      if (nbPointPol > 100000) {
        throw vpException(vpException::badValue, "Exceed the max number of points.");
      }

      std::vector<vpPoint> corners;
      for(unsigned int n = 0; n < nbPointPol; n++){
        fileId >> index;
        if (index > caoNbrPoint-1) {
          throw vpException(vpException::badValue, "Exceed the max number of points.");
        }
        corners.push_back(caoPoints[index]);
      }
      if(k != caoNbrPolygonPoint-1){// the rest of the line is removed (not the last one due to the need to remove possible comments).
        fileId.ignore(256,'\n');
      }
      initFaceFromCorners(corners, k);
    }

    unsigned int caoNbCylinder;
    try{
      //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
      fileId.get(c);
      while (!fileId.fail() && (c != '\n'))
      {
        fileId.get(c);
      }

      //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
      fileId.get(c);
      while (!fileId.fail() && (c == '#'))
      {
        fileId.ignore(256, '\n');
        fileId.get(c);
      }      fileId.unget();

      if(fileId.eof()){// check if not at the end of the file (for old style files)
        delete[] caoPoints;
        delete[] caoLinePoints;
        return ;
      }

      /* Extract the cylinders */


      fileId >> caoNbCylinder;
      std::cout << "> " << caoNbCylinder << " cylinder" << std::endl;
      if (caoNbCylinder > 100000) {
        throw vpException(vpException::badValue, "Exceed the max number of cylinders.");
      }

      for(unsigned int k=0; k<caoNbCylinder; ++k){
        double radius;
        unsigned int indexP1, indexP2;
        fileId >> indexP1;
        fileId >> indexP2;
        fileId >> radius;
        initCylinder(caoPoints[indexP1], caoPoints[indexP2], radius);
      }

    }catch(...){
      std::cerr << "Cannot get the number of cylinders. Defaulting to zero." << std::endl;
      caoNbCylinder = 0;
    }

    unsigned int caoNbCircle;
    try{
      //while( (fileId.get(c)!=NULL)&&(c!='\n')) ;
      fileId.get(c);
      while (!fileId.fail() && (c != '\n'))
      {
        fileId.get(c);
      }

      //while ((fileId.get(c) != NULL) && (c == '#')) fileId.ignore(256, '\n');
      fileId.get(c);
      while (!fileId.fail() && (c == '#'))
      {
        fileId.ignore(256, '\n');
        fileId.get(c);
      }      fileId.unget();

      if(fileId.eof()){// check if not at the end of the file (for old style files)
        delete[] caoPoints;
        delete[] caoLinePoints;
        return ;
      }

      /* Extract the circles */
      fileId >> caoNbCircle;
      std::cout << "> " << caoNbCircle << " circle" << std::endl;
      if (caoNbCircle > 100000) {
        throw vpException(vpException::badValue, "Exceed the max number of cicles.");
      }

      for(unsigned int k=0; k<caoNbCircle; ++k){
        double radius;
        unsigned int indexP1, indexP2, indexP3;
        fileId >> radius;
        fileId >> indexP1;
        fileId >> indexP2;
        fileId >> indexP3;
//        std::cout << "circle centre: " << caoPoints[indexP1].get_oX() << " " << caoPoints[indexP1].get_oY() << " " << caoPoints[indexP1].get_oZ() << std::endl;
//        std::cout << "circle p2: " << caoPoints[indexP2].get_oX() << " " << caoPoints[indexP2].get_oY() << " " << caoPoints[indexP2].get_oZ() << std::endl;
//        std::cout << "circle p3: " << caoPoints[indexP3].get_oX() << " " << caoPoints[indexP3].get_oY() << " " << caoPoints[indexP3].get_oZ() << std::endl;
//        std::cout << "circle radius: " << radius << std::endl;
        initCircle(caoPoints[indexP1], caoPoints[indexP2], caoPoints[indexP3], radius);
      }

    }catch(...){
      std::cerr << "Cannot get the number of circles. Defaulting to zero." << std::endl;
      caoNbCircle = 0;
    }

    delete[] caoPoints;
    delete[] caoLinePoints;
  }catch(std::ifstream::failure e){
    std::cerr << "Cannot read line!" << std::endl;
    throw vpException(vpException::ioError, "cannot read line");
  }

}

#ifdef VISP_HAVE_COIN
/*!
  Extract a VRML object Group. 
  
  \param sceneGraphVRML2 : Current node (either Transform, or Group node).
  \param transform : Transformation matrix for this group.
  \param indexFace : Index of the face.
*/
void
vpMbTracker::extractGroup(SoVRMLGroup *sceneGraphVRML2, vpHomogeneousMatrix &transform, unsigned int &indexFace)
{ 
  vpHomogeneousMatrix transformCur;
  SoVRMLTransform *sceneGraphVRML2Trasnform = dynamic_cast<SoVRMLTransform *>(sceneGraphVRML2);
  if(sceneGraphVRML2Trasnform){
    float rx, ry, rz, rw;
    sceneGraphVRML2Trasnform->rotation.getValue().getValue(rx,ry,rz,rw);
    vpRotationMatrix rotMat(vpQuaternionVector(rx,ry,rz,rw));
//     std::cout << "Rotation: " << rx << " " << ry << " " << rz << " " << rw << std::endl;
    
    float tx, ty, tz;
    tx = sceneGraphVRML2Trasnform->translation.getValue()[0];
    ty = sceneGraphVRML2Trasnform->translation.getValue()[1];
    tz = sceneGraphVRML2Trasnform->translation.getValue()[2];
    vpTranslationVector transVec(tx,ty,tz);
//     std::cout << "Translation: " << tx << " " << ty << " " << tz << std::endl;
    
    float sx, sy, sz;
    sx = sceneGraphVRML2Trasnform->scale.getValue()[0];
    sy = sceneGraphVRML2Trasnform->scale.getValue()[1];
    sz = sceneGraphVRML2Trasnform->scale.getValue()[2];
//     std::cout << "Scale: " << sx << " " << sy << " " << sz << std::endl;
    
    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[0][i] *= sx;
    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[1][i] *= sy;
    for(unsigned int i = 0 ; i < 3 ; i++)
      rotMat[2][i] *= sz;
    
    transformCur = vpHomogeneousMatrix(transVec,rotMat);
    transform = transform * transformCur;
  }
  
  int nbShapes = sceneGraphVRML2->getNumChildren();
//   std::cout << sceneGraphVRML2->getTypeId().getName().getString() << std::endl;
//   std::cout << "Nb object in VRML : " << nbShapes << std::endl;
  
  SoNode * child;
  
  for (int i = 0; i < nbShapes; i++)
  {
    vpHomogeneousMatrix transform_recursive(transform);
    child = sceneGraphVRML2->getChild(i);
    
    if (child->getTypeId() == SoVRMLGroup::getClassTypeId()){
      extractGroup((SoVRMLGroup*)child, transform_recursive, indexFace);
    }
    
    if (child->getTypeId() == SoVRMLTransform::getClassTypeId()){
      extractGroup((SoVRMLTransform*)child, transform_recursive, indexFace);
    }
    
    if (child->getTypeId() == SoVRMLShape::getClassTypeId()){
      SoChildList * child2list = child->getChildren();
      for (int j = 0; j < child2list->getLength(); j++)
      {
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedFaceSet::getClassTypeId())
        {
          SoVRMLIndexedFaceSet * face_set;
          face_set = (SoVRMLIndexedFaceSet*)child2list->get(j);
          if(!strncmp(face_set->getName().getString(),"cyl",3)){
            extractCylinders(face_set, transform);
          }else{
            extractFaces(face_set, transform, indexFace);
          }
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

/*!
  Extract a face of the object to track from the VMRL model. This method calls
  the initFaceFromCorners() method implemented in the child class. 

  \param face_set : Pointer to the face in the vrml format.
  \param transform : Transformation matrix applied to the face.
  \param indexFace : Face index.
*/
void
vpMbTracker::extractFaces(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, unsigned int &indexFace)
{
  std::vector<vpPoint> corners;
  corners.resize(0);

//  SoMFInt32 indexList = _face_set->coordIndex;
//  int indexListSize = indexList.getNum();
  int indexListSize = face_set->coordIndex.getNum();
  
  vpColVector pointTransformed(4);
  vpPoint pt;
  SoVRMLCoordinate *coord;
  
  for (int i = 0; i < indexListSize; i++)
  {
    if (face_set->coordIndex[i] == -1)
    {
      if(corners.size() > 1)
      {
        initFaceFromCorners(corners, indexFace);
        indexFace++;
        corners.resize(0);
      }
    }
    else
    {
      coord = (SoVRMLCoordinate *)(face_set->coord.getValue());
      int index = face_set->coordIndex[i];
      pointTransformed[0]=coord->point[index].getValue()[0];
      pointTransformed[1]=coord->point[index].getValue()[1];
      pointTransformed[2]=coord->point[index].getValue()[2];
      pointTransformed[3] = 1.0;
      
      pointTransformed = transform * pointTransformed;
      
      pt.setWorldCoordinates(pointTransformed[0],pointTransformed[1],pointTransformed[2]);
      corners.push_back(pt);
    }
  }
}

/*!
  Extract a cylinder  to track from the VMRL model. This method calls
  the initCylinder() method implemented in the child class.

  \warning This method extract cylinder described using an indexed face set not
  a cylinder set since software such as AC3D or blender export a cylinder using
  this data type. the object name is used, if it begins with "cyl" then this
  method is called otherwise the extractFaces() is used.

  \param face_set : Pointer to the cylinder in the vrml format.
  \param transform : Transformation matrix applied to the cylinder.
*/
void
vpMbTracker::extractCylinders(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform)
{
  std::vector<vpPoint> corners_c1, corners_c2;//points belonging to the first circle and to the second one.
  SoVRMLCoordinate* coords = (SoVRMLCoordinate *)face_set->coord.getValue();

  unsigned int indexListSize = (unsigned int)coords->point.getNum();

  if(indexListSize % 2 == 1){
    std::cout << "Not an even number of points when extracting a cylinder." << std::endl;
    throw vpException(vpException::dimensionError, "Not an even number of points when extracting a cylinder.");
  }
  corners_c1.resize(indexListSize / 2);
  corners_c2.resize(indexListSize / 2);
  vpColVector pointTransformed(4);
  vpPoint pt;


  // extract all points and fill the two sets.
  for(int i=0; i<coords->point.getNum(); ++i){   
    pointTransformed[0]=coords->point[i].getValue()[0];
    pointTransformed[1]=coords->point[i].getValue()[1];
    pointTransformed[2]=coords->point[i].getValue()[2];
    pointTransformed[3] = 1.0;
    
    pointTransformed = transform * pointTransformed;
    
    pt.setWorldCoordinates(pointTransformed[0],pointTransformed[1],pointTransformed[2]);

    if(i < (int)corners_c1.size()){
      corners_c1[(unsigned int)i] = pt;
    }else{
      corners_c2[(unsigned int)i-corners_c1.size()] = pt;
    }
  }

  vpPoint p1 = getGravityCenter(corners_c1);
  vpPoint p2 = getGravityCenter(corners_c2);

  vpColVector dist(3);
  dist[0] = p1.get_oX() - corners_c1[0].get_oX();
  dist[1] = p1.get_oY() - corners_c1[0].get_oY();
  dist[2] = p1.get_oZ() - corners_c1[0].get_oZ();
  double radius_c1 = sqrt(dist.sumSquare());
  dist[0] = p2.get_oX() - corners_c2[0].get_oX();
  dist[1] = p2.get_oY() - corners_c2[0].get_oY();
  dist[2] = p2.get_oZ() - corners_c2[0].get_oZ();
  double radius_c2 = sqrt(dist.sumSquare());

  if(std::fabs(radius_c1 - radius_c2) > (std::numeric_limits<double>::epsilon() * vpMath::maximum(radius_c1, radius_c2))){
    std::cout << "Radius from the two circles of the cylinders are different." << std::endl;
    throw vpException(vpException::badValue, "Radius from the two circles of the cylinders are different.");
  }

  initCylinder(p1, p2, radius_c1);

}

/*!
  Compute the center of gravity of a set of point. This is used in the cylinder
  extraction to find the center of the circles.

  \throw vpException::dimensionError if the set is empty.

  \param pts : Set of point to extract the center of gravity.
  \return Center of gravity of the set.
*/
vpPoint
vpMbTracker::getGravityCenter(const std::vector<vpPoint>& pts)
{
  if(pts.empty()){
    std::cout << "Cannot extract center of gravity of empty set." << std::endl;
    throw vpException(vpException::dimensionError, "Cannot extract center of gravity of empty set.");
  }
  double oX = 0;
  double oY = 0;
  double oZ = 0;
  vpPoint G;

  for(unsigned int i=0; i<pts.size(); ++i){
    oX += pts[i].get_oX();
    oY += pts[i].get_oY();
    oZ += pts[i].get_oZ();
  }

  G.setWorldCoordinates(oX/pts.size(), oY/pts.size(), oZ/pts.size());
  return G;
}


/*!
  Extract a line of the object to track from the VMRL model. This method calls
  the initFaceFromCorners() method implemented in the child class. 

  \param line_set : Pointer to the line in the vrml format.
*/
void
vpMbTracker::extractLines(SoVRMLIndexedLineSet* line_set)
{
  std::vector<vpPoint> corners;
  corners.resize(0);

  int indexListSize = line_set->coordIndex.getNum();

  SbVec3f point(0,0,0);
  vpPoint pt;
  SoVRMLCoordinate *coord;
  
  unsigned int indexFace = 0;

  for (int i = 0; i < indexListSize; i++)
  {
    if (line_set->coordIndex[i] == -1)
    {
      if(corners.size() > 1)
      {
        initFaceFromCorners(corners, indexFace);
        indexFace++;
        corners.resize(0);
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
      corners.push_back(pt);
    }
  }
}

#endif

/*!
  Compute \f$ J^T R \f$, with J the interaction matrix and R the vector of 
  residu.
  
  \throw vpMatrixException::incorrectMatrixSizeError if the sizes of the 
  matrices do not allow the computation.
  
  \warning The JTR matrix is resized.
  
  \param interaction : The interaction matrix (size Nx6).
  \param error : The residu vector (size Nx1).
  \param JTR : The resulting JTR matrix (size 6x1).
  
*/
void 
vpMbTracker::computeJTR(const vpMatrix& interaction, const vpColVector& error, vpMatrix& JTR)
{
  if(interaction.getRows() != error.getRows() || interaction.getCols() != 6 ){
    throw vpMatrixException(vpMatrixException::incorrectMatrixSizeError, 
              "Incorrect matrices size in computeJTR.");
  }

  JTR.resize(6, 1);
  const unsigned int N = interaction.getRows();

  for (unsigned int i = 0; i < 6; i += 1){
    double ssum = 0;
    for (unsigned int j = 0; j < N; j += 1){
      ssum += interaction[j][i] * error[j];
    }
    JTR[i][0] = ssum;
  }
}


