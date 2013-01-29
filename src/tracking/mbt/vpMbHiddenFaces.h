/****************************************************************************
 *
 * $Id: vpMbTracker.h 4004 2012-11-23 17:34:44Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Description:
 * Generic model based tracker. This class declares the methods to implement in 
 * order to have a model based tracker. 
 *
 * Authors:
 * Romain Tallonneau
 * Aurélien Yol
 *
 *****************************************************************************/
#pragma once

#ifndef vpMbHiddenFaces_HH
#define vpMbHiddenFaces_HH

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMbtPolygon.h>

#ifdef VISP_HAVE_OGRE
  #include <visp/vpAROgre.h>
#endif

#include <vector>

/*!
  \class vpMbHiddenFaces
  
  \brief Implementation of the polygons management for the model-based trackers.

  \ingroup ModelBasedTracking

 */
template<class PolygonType = vpMbtPolygon>
class vpMbHiddenFaces
{
  private:
  std::vector<PolygonType *> Lpol ;
  //! Boolean specifying if a polygon has to be entirely in front of the camera or not.
  bool depthTest;
  //! Number of visible polygon
  unsigned int nbVisiblePolygon;
  
#ifdef VISP_HAVE_OGRE
  vpImage<vpRGBa> ogreBackground;
  bool ogreInitialised;
  vpAROgre *ogre;
  std::vector< Ogre::ManualObject* > lOgrePolygons;
#endif
  
  unsigned int  setVisiblePrivate(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, 
                           bool &changed, 
                           bool useOgre = false, bool testRoi = false,
                           const vpImage<unsigned char> &_I = vpImage<unsigned char>(),
                           const vpCameraParameters &_cam = vpCameraParameters()
                          ) ;

  
  public :
                    vpMbHiddenFaces() ;
                  ~vpMbHiddenFaces() ;
                
    void          addPolygon(PolygonType *p)  ;

#ifdef VISP_HAVE_OGRE
    void          displayOgre(const vpHomogeneousMatrix &_cMo);
#endif   
 
    /*!
     Get the list of polygons.

      \return Mbt Klt polygons list.
    */
    std::vector<PolygonType*>& getPolygon() {return Lpol;}

#ifdef VISP_HAVE_OGRE
  void            initOgre(vpCameraParameters _cam = vpCameraParameters());
#endif
  
    /*!
      Get the depthTest value.

      \return true if all the points of a polygon has to be in front of the camera, false otherwise.
    */
    bool          getDepthTest(){return depthTest;}
    
    /*!
      get the number of visible polygons.

      \return number of visible polygons.
    */
    unsigned int getNbVisiblePolygon(){return nbVisiblePolygon;}

#ifdef VISP_HAVE_OGRE
    /*!
      Get the Ogre3D Context.

      \return A pointer on a vpAROgre instance.
    */
    vpAROgre*     getOgreContext(){return ogre;}
#endif

    bool          isAppearing(const unsigned int i){ return Lpol[i]->isAppearing(); }
    
    
#ifdef VISP_HAVE_OGRE
  /*!
    Tell whether if Ogre Context is initialised or not.

    \return True if it does, false otherwise.
  */
  bool            isOgreInitialised() { return ogreInitialised; }
#endif
  
    /*!
    Check if the polygon at position i in the list is visible.
    
    \param i : TPosition in the list.
    
    \return Return true if the polygon is visible.
  */
    bool          isVisible(const unsigned int i){ return Lpol[i]->isVisible(); }
    
#ifdef VISP_HAVE_OGRE
    bool          isVisibleOgre(const vpTranslationVector &cameraPos, const unsigned int &index);
#endif
    
    //! operator[] as modifier.
    inline PolygonType*        operator[](const unsigned int i)   { return Lpol[i];}
    //! operator[] as reader.
    inline const PolygonType*  operator[](const unsigned int i) const { return Lpol[i];}

    void          reset();   
    
    /*!
      Set the depthTest value.

      \param d : New value.
    */
    void          setDepthTest(const bool &d){depthTest = d;} 
    unsigned int  setVisible(const vpHomogeneousMatrix &_cMo) ;
    unsigned int  setVisible(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angle, bool &changed) ;
    unsigned int  setVisible(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
    unsigned int  setVisible(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
 
#ifdef VISP_HAVE_OGRE
    unsigned int  setVisibleOgre(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
    unsigned int  setVisibleOgre(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
#endif
  /*!
   Get the number of polygons.
    
    \return Size of the list.
  */
  inline unsigned int            size(){ return Lpol.size(); }
} ;

/*!
  Basic constructor.
*/
template<class PolygonType>
vpMbHiddenFaces<PolygonType>::vpMbHiddenFaces(): depthTest(false), nbVisiblePolygon(0)
{
#ifdef VISP_HAVE_OGRE
  ogreInitialised = false;
  ogre = new vpAROgre();
  ogre->setShowConfigDialog(false);
  ogreBackground = vpImage<vpRGBa>(480, 640);
#endif
}


/*!
  Basic destructor.
*/
template<class PolygonType>
vpMbHiddenFaces<PolygonType>::~vpMbHiddenFaces()
{
  for(unsigned int i = 0 ; i < Lpol.size() ; i++){
    if (Lpol[i]!=NULL){
      delete Lpol[i] ;
    }
    Lpol[i] = NULL ;
  }
  Lpol.resize(0);
  
#ifdef VISP_HAVE_OGRE
  delete ogre;
#endif
}

/*!
  Add a polygon to the list of polygons.
  
  \param p : The polygon to add.
*/
template<class PolygonType>
void
vpMbHiddenFaces<PolygonType>::addPolygon(PolygonType *p)
{
  PolygonType *p_new = new PolygonType;
  p_new->index = p->index;
  p_new->setNbPoint(p->nbpt);
  p_new->isvisible = p->isvisible;
  for(unsigned int i = 0; i < p->nbpt; i++)
    p_new->p[i]= p->p[i];
  Lpol.push_back(p_new);
}

/*!
  Compute the number of visible polygons.
  
  \param _cMo : The pose of the camera
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpHomogeneousMatrix &_cMo)
{
  nbVisiblePolygon = 0 ;
  
  for(unsigned int i = 0 ; i < Lpol.size() ; i++){
    if (Lpol[i]->isVisible(_cMo, depthTest)){
      nbVisiblePolygon++;
    }
  }
  return nbVisiblePolygon ;
}

/*!
  Reset the Hidden faces (remove the list of PolygonType)
*/
template<class PolygonType>
void
vpMbHiddenFaces<PolygonType>::reset()
{
  nbVisiblePolygon = 0;
  for(unsigned int i = 0 ; i < Lpol.size() ; i++){
    if (Lpol[i]!=NULL){
      delete Lpol[i] ;
    }
    Lpol[i] = NULL ;
  }
  Lpol.resize(0);
}

/*!
  Compute the number of visible polygons.
  
  \param _cMo : The pose of the camera
  \param angleAppears : Angle used to test the apparition of a face
  \param angleDisappears : Angle used to test the disparition of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param useOgre : True if a Ogre is used to test the visibility, False otherwise
  \param testRoi : True if a face have to be entirely in the image False otherwise
  \param _I : Image used to test if a face is entirely projected in the image.
  \param _cam : Camera parameters.
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisiblePrivate(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, 
                                               bool &changed, bool useOgre, bool testRoi, 
                                               const vpImage<unsigned char> &_I,
                                               const vpCameraParameters &_cam
                                              )
{  
  nbVisiblePolygon = 0;
  changed = false;
  
  vpTranslationVector cameraPos;
  std::vector<vpImagePoint> roi;
  
  if(useOgre){
#ifdef VISP_HAVE_OGRE
    _cMo.inverse().extract(cameraPos);
    ogre->renderOneFrame(ogreBackground, _cMo);
#else
    vpTRACE("ViSP doesn't have Ogre3D, simple visibility test used");
#endif
  }
  
  for (unsigned int i = 0; i < Lpol.size(); i += 1){ 
    Lpol[i]->changeFrame(_cMo);
    Lpol[i]->isappearing = false;
    if(testRoi)
      roi = Lpol[i]->getRoi(_cam);
    
    if(Lpol[i]->isVisible())
    {
      bool testDisappear = false;
      
      if(testRoi)
       testDisappear = (!vpMbtPolygon::roiInsideImage(_I, roi));
      
      if(!testDisappear){
        if(useOgre)
#ifdef VISP_HAVE_OGRE
          testDisappear = ((!Lpol[i]->isVisible(_cMo, angleDisappears)) || !isVisibleOgre(cameraPos,i));
#else
          testDisappear = (!Lpol[i]->isVisible(_cMo, angleDisappears));
#endif
        else
          testDisappear = (!Lpol[i]->isVisible(_cMo, angleDisappears));
      }
  
      // test if the face is still visible
      if(testDisappear){
//         std::cout << "Face " << i << " disappears" << std::endl;
        changed = true;
        Lpol[i]->isvisible = false;
      }
      else {
        nbVisiblePolygon++;
        Lpol[i]->isvisible = true;
      }
    }
    else
    {
      bool testAppear = true;
      
      if(testRoi)
       testAppear = (vpMbtPolygon::roiInsideImage(_I, roi));
      
      if(testAppear){
        if(useOgre)
#ifdef VISP_HAVE_OGRE
          testAppear = ((Lpol[i]->isVisible(_cMo, angleAppears)) && isVisibleOgre(cameraPos,i));
#else
          testAppear = (Lpol[i]->isVisible(_cMo, angleAppears));
#endif
        else
          testAppear = (Lpol[i]->isVisible(_cMo, angleAppears));
      } 
      
      if(testAppear){   
//         std::cout << "Face " << i << " appears" << std::endl;
        Lpol[i]->isvisible = true;
        changed = true;
        nbVisiblePolygon++;
      }
      else
        Lpol[i]->isvisible = false;
    }
  }
  
  return nbVisiblePolygon;
}

/*!
  Compute the number of visible polygons.
  
  \param _I : Image used to check if the region of interest is inside the image.
  \param _cam : Camera parameters.
  \param _cMo : The pose of the camera.
  \param angle : Angle used to test the apparition and disparition of a face.
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angle, bool &changed)
{
  return setVisible(_I, _cam, _cMo, angle, angle, changed);
}

/*!
  Compute the number of visible polygons.
  
  \param _I : Image used to check if the region of interest is inside the image.
  \param _cam : Camera parameters.
  \param _cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param angleAppears : Angle used to test the apparition of a face
  \param angleDisappears : Angle used to test the disparition of a face
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(_cMo,angleAppears,angleDisappears,changed,false,true,_I,_cam);
}

/*!
  Compute the number of visible polygons.
  
  \param _cMo : The pose of the camera
  \param angleAppears : Angle used to test the apparition of a face
  \param angleDisappears : Angle used to test the disparition of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(_cMo,angleAppears,angleDisappears,changed,false);
}

#ifdef VISP_HAVE_OGRE
/*!
  Initialise the ogre context for face visibility tests.
  
  \param _cam : Camera parameters.
*/
template<class PolygonType>
void 
vpMbHiddenFaces<PolygonType>::initOgre(vpCameraParameters _cam)
{
  ogreInitialised = true;
  ogre->setCameraParameters(_cam);
  ogre->init(ogreBackground, false, true);  
  
  for(unsigned int n = 0 ; n < Lpol.size(); n++){
    Ogre::ManualObject* manual = ogre->getSceneManager()->createManualObject(Ogre::StringConverter::toString(n));
  
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    for(unsigned int i = 0; i < Lpol[n]->nbpt; i++){
      manual->position( (Ogre::Real)Lpol[n]->p[i].get_oX(), (Ogre::Real)Lpol[n]->p[i].get_oY(), (Ogre::Real)Lpol[n]->p[i].get_oZ());
      manual->colour(1.0, 1.0, 1.0);
    }
    
    manual->index(0);
    manual->index(1);
    manual->index(2);
    manual->index(3);
    manual->index(0);
    manual->end();
    
    ogre->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
    
    lOgrePolygons.push_back(manual);
  }
}

/*!
  Update the display in Ogre Window.
  
  \param _cMo : Pose used to display.
*/
template<class PolygonType>
void
vpMbHiddenFaces<PolygonType>::displayOgre(const vpHomogeneousMatrix &_cMo)
{
  if(ogreInitialised && !ogre->isWindowHidden()){
    for(unsigned int i = 0 ; i < Lpol.size() ; i++){
      if(Lpol[i]->isVisible()){
        lOgrePolygons[i]->setVisible(true);
      }
      else
        lOgrePolygons[i]->setVisible(false);
    }
    ogre->display(ogreBackground, _cMo);
  }
}

/*!
  Compute the number of visible polygons through Ogre3D.
  
  \param _I : Image used to check if the region of interest is inside the image.
  \param _cam : Camera parameters.
  \param _cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param angleAppears : Angle used to test the apparition of a face
  \param angleDisappears : Angle used to test the disparition of a face
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpImage<unsigned char>& _I, const vpCameraParameters &_cam, const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(_cMo,angleAppears,angleDisappears,changed,true,true,_I,_cam);
}

/*!
  Compute the number of visible polygons through Ogre3D.
  
  \param _cMo : The pose of the camera
  \param angleAppears : Angle used to test the apparition of a face
  \param angleDisappears : Angle used to test the disparition of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpHomogeneousMatrix &_cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(_cMo,angleAppears,angleDisappears,changed,true);
}

/*!
  Test the visibility of a polygon through Ogre3D via RayCasting.
  
  \param cameraPos : Position of the camera in the 3D world.
  \param index : Index of the polygon.
  
  \return Return true if the polygon is visible, False otherwise.
*/
template<class PolygonType>
bool                           
vpMbHiddenFaces<PolygonType>::isVisibleOgre(const vpTranslationVector &cameraPos, const unsigned int &index)
{ 
//   std::cout << "visible" << std::endl;
  // A line is always visible
  if(Lpol[index]->getNbPoint() <= 2){
    lOgrePolygons[index]->setVisible(true);
    Lpol[index]->isvisible = true;
    return true;
  }
  
  Ogre::Vector3 camera((Ogre::Real)cameraPos[0],(Ogre::Real)cameraPos[1],(Ogre::Real)cameraPos[2]);
  if(!ogre->getCamera()->isVisible(lOgrePolygons[index]->getBoundingBox())){
    lOgrePolygons[index]->setVisible(false);
    Lpol[index]->isvisible = false;
    return false;  
  }
  
  //Get the center of gravity 
  Ogre::Vector3 origin(0,0,0);
  for(unsigned int j = 0 ; j < Lpol[index]->getNbPoint() ; j++){
      Ogre::Vector3 tmp((Ogre::Real)Lpol[index]->getPoint(j).get_oX(), (Ogre::Real)Lpol[index]->getPoint(j).get_oY(), (Ogre::Real)Lpol[index]->getPoint(j).get_oZ());
      origin += tmp;
  }
  origin /= (Ogre::Real)Lpol[index]->getNbPoint();
  Ogre::Vector3 direction = origin - camera;
  
  Ogre::RaySceneQuery *mRaySceneQuery = ogre->getSceneManager()->createRayQuery(Ogre::Ray(camera, direction));
  mRaySceneQuery->setSortByDistance(true);
  
  Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator it = result.begin();
  
  bool visible = false;
  if(it != result.end()){
//     std::cout << it->movable->getName() << "/";
    
    if(it->movable->getName().find("SimpleRenderable") != Ogre::String::npos) //Test if the ogreBackground is intersect in first
      it++;
    
    if(it != result.end()){
//       std::cout << it->movable->getName() << " / ";
      if(it->movable->getName() == Ogre::StringConverter::toString(index)){
        visible = true;
      }
    }
//     std::cout << std::endl;
  }
  
  if(visible){
    lOgrePolygons[index]->setVisible(true);
    Lpol[index]->isvisible = true;
  }
  else{
    lOgrePolygons[index]->setVisible(false);
    Lpol[index]->isvisible = false;
  }
  
  ogre->getSceneManager()->destroyQuery(mRaySceneQuery); 
  
  return Lpol[index]->isvisible;
}
#endif //VISP_HAVE_OGRE

#endif

