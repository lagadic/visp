/****************************************************************************
 *
 * $Id: vpMbTracker.h 4004 2012-11-23 17:34:44Z fspindle $
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
 * Description:
 * Generic model based tracker. This class declares the methods to implement in 
 * order to have a model based tracker. 
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
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
#include <limits>

/*!
  \class vpMbHiddenFaces
  
  \brief Implementation of the polygons management for the model-based trackers.

  \ingroup ModelBasedTracking

 */
template<class PolygonType = vpMbtPolygon>
class vpMbHiddenFaces
{
  private:
  //! List of polygons
  std::vector<PolygonType *> Lpol ;
  //! Number of visible polygon
  unsigned int nbVisiblePolygon;
  
#ifdef VISP_HAVE_OGRE
  vpImage<unsigned char> ogreBackground;
  bool ogreInitialised;
  vpAROgre *ogre;
  std::vector< Ogre::ManualObject* > lOgrePolygons;
#endif
  
  unsigned int  setVisiblePrivate(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                           bool &changed, 
                           bool useOgre = false, bool testRoi = false,
                           const vpImage<unsigned char> &I = vpImage<unsigned char>(),
                           const vpCameraParameters &cam = vpCameraParameters()) ;

  public :
                    vpMbHiddenFaces() ;
                  ~vpMbHiddenFaces() ;
                
    void          addPolygon(PolygonType *p)  ;

    bool computeVisibility(const vpHomogeneousMatrix &cMo,
                           const double &angleAppears, const double &angleDisappears,
                           bool &changed, bool useOgre, bool testRoi,
                           const vpImage<unsigned char> &I,
                           const vpCameraParameters &cam,
                           const vpTranslationVector &cameraPos,
                           unsigned int index);
#ifdef VISP_HAVE_OGRE
    void          displayOgre(const vpHomogeneousMatrix &cMo);
#endif   
 
    /*!
     Get the list of polygons.

      \return Mbt Klt polygons list.
    */
    std::vector<PolygonType*>& getPolygon() {return Lpol;}

#ifdef VISP_HAVE_OGRE
  void            initOgre(const vpCameraParameters &cam = vpCameraParameters());
#endif
    
    /*!
      get the number of visible polygons.

      \return number of visible polygons.
    */
    unsigned int getNbVisiblePolygon() const {return nbVisiblePolygon;}

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
    
#ifdef VISP_HAVE_OGRE
    /*!
      Set the background size (by default it is 640x480). 
      The background size has to match with the size of the image that you are using for the traking.
      
      \warning This function has to be called before initOgre().
      
      \param h : Height of the background
      \param w : Width of the background
    */
    void          setBackgroundSizeOgre(const unsigned int &h, const unsigned int &w) { ogreBackground = vpImage<unsigned char>(h, w, 0); }
#endif
    
    unsigned int  setVisible(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angle, bool &changed) ;
    unsigned int  setVisible(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
    unsigned int  setVisible(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
 
#ifdef VISP_HAVE_OGRE
    unsigned int  setVisibleOgre(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
    unsigned int  setVisibleOgre(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed) ;
#endif
  /*!
   Get the number of polygons.
    
    \return Size of the list.
  */
  inline unsigned int            size() const { return (unsigned int)Lpol.size(); }
  
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  //! Boolean specifying if a polygon has to be entirely in front of the camera or not.
  bool depthTest;
  
  /*!
    @name Deprecated functions
  */
  /*!
    \deprecated This method is deprecated since it is no more used since ViSP 2.7.2. \n 
    
    Get the depthTest value.

    \return true if all the points of a polygon has to be in front of the camera, false otherwise.
  */
  vp_deprecated bool getDepthTest(){return depthTest;}
  /*!
    \deprecated This method is deprecated since it is no more used since ViSP 2.7.2. \n
    
    Set the depthTest value.

    \param d : New value.
  */
  vp_deprecated void setDepthTest(const bool &d){depthTest = d;} 
  unsigned int setVisible(const vpHomogeneousMatrix &cMo) ;
#endif
} ;

/*!
  Basic constructor.
*/
template<class PolygonType>
vpMbHiddenFaces<PolygonType>::vpMbHiddenFaces()
  : Lpol(), nbVisiblePolygon(0)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  , depthTest(false)
#endif

{
#ifdef VISP_HAVE_OGRE
  ogreInitialised = false;
  ogre = new vpAROgre();
  ogre->setShowConfigDialog(false);
  ogreBackground = vpImage<unsigned char>(480, 640, 0);
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
  if(ogre != NULL){
    delete ogre;
    ogre = NULL;
  }

  // This is already done by calling "delete ogre"
//  for(unsigned int i = 0 ; i < lOgrePolygons.size() ; i++){
//    if (lOgrePolygons[i]!=NULL){
//      delete lOgrePolygons[i] ;
//    }
//    lOgrePolygons[i] = NULL ;
//  }

  lOgrePolygons.resize(0);
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
  p_new->useLod = p->useLod;
  p_new->minLineLengthThresh = p->minLineLengthThresh;
  p_new->minPolygonAreaThresh = p->minPolygonAreaThresh;
  p_new->setName(p->name);

  for(unsigned int i = 0; i < p->nbpt; i++)
    p_new->p[i]= p->p[i];
  Lpol.push_back(p_new);
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

#ifdef VISP_HAVE_OGRE
  if(ogre != NULL){
    delete ogre;
    ogre = NULL;
  }

  // This is already done by calling "delete ogre"
//  for(unsigned int i = 0 ; i < lOgrePolygons.size() ; i++){
//    if (lOgrePolygons[i]!=NULL){
//      delete lOgrePolygons[i] ;
//    }
//    lOgrePolygons[i] = NULL ;
//  }

  lOgrePolygons.resize(0);

  ogreInitialised = false;
  ogre = new vpAROgre();
  ogre->setShowConfigDialog(false);
  ogreBackground = vpImage<unsigned char>(480, 640);
#endif
}

/*!
  Compute the number of visible polygons.
  
  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param useOgre : True if a Ogre is used to test the visibility, False otherwise
  \param testRoi : True if a face have to be entirely in the image False otherwise
  \param I : Image used to test if a face is entirely projected in the image.
  \param cam : Camera parameters.
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisiblePrivate(const vpHomogeneousMatrix &cMo,
                                                const double &angleAppears, const double &angleDisappears,
                                                bool &changed, bool useOgre, bool testRoi,
                                                const vpImage<unsigned char> &I,
                                                const vpCameraParameters &cam)
{  
  nbVisiblePolygon = 0;
  changed = false;
  
  vpTranslationVector cameraPos;
  
  if(useOgre){
#ifdef VISP_HAVE_OGRE
    cMo.inverse().extract(cameraPos);
    ogre->renderOneFrame(ogreBackground, cMo);
#else
    vpTRACE("ViSP doesn't have Ogre3D, simple visibility test used");
#endif
  }
  
  for (unsigned int i = 0; i < Lpol.size(); i++){
    if (computeVisibility(cMo, angleAppears, angleDisappears, changed, useOgre, testRoi, I, cam, cameraPos, i))
      nbVisiblePolygon ++;
  }
  return nbVisiblePolygon;
}

/*!
  Compute the visibility of a given face index.

  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param useOgre : True if a Ogre is used to test the visibility, False otherwise
  \param testRoi : True if a face have to be entirely in the image False otherwise
  \param I : Image used to test if a face is entirely projected in the image.
  \param cam : Camera parameters.
  \param cameraPos : Position of the camera. Used only when Ogre is used as 3rd party.
  \param index : Index of the face to consider.

  \return Return true if the face is visible.
*/
template<class PolygonType>
bool
vpMbHiddenFaces<PolygonType>::computeVisibility(const vpHomogeneousMatrix &cMo,
                                                const double &angleAppears, const double &angleDisappears,
                                                bool &changed, bool useOgre, bool /* testRoi */,
                                                const vpImage<unsigned char> & I,
                                                const vpCameraParameters & cam,
                                                const vpTranslationVector &
                                                #ifdef VISP_HAVE_OGRE
                                                cameraPos
                                                #endif
                                                ,
                                                unsigned int index)
{
  unsigned int i = index;
  Lpol[i]->changeFrame(cMo);
  Lpol[i]->isappearing = false;

  //Commented because we need to compute visibility
  // even when dealing with line in level of detail case
  /*if(Lpol[i]->getNbPoint() <= 2)
  {
      Lpol[i]->isvisible = true;
  }
  else*/{
  if(Lpol[i]->isVisible())
  {
    bool testDisappear = false;
    unsigned int nbCornerInsidePrev = 0;

//    if(testRoi){
//      nbCornerInsidePrev = Lpol[i]->getNbCornerInsidePrevImage();
//      if(Lpol[i]->getNbCornerInsideImage(I, cam) == 0)
//        testDisappear = true;
//    }

    if(!testDisappear){
      if(useOgre)
#ifdef VISP_HAVE_OGRE
        testDisappear = ((!Lpol[i]->isVisible(cMo, angleDisappears, true, cam, I)) || !isVisibleOgre(cameraPos,i));
#else
        testDisappear = (!Lpol[i]->isVisible(cMo, angleDisappears, false, cam, I));
#endif
      else
        testDisappear = (!Lpol[i]->isVisible(cMo, angleDisappears, false, cam, I));
    }

    // test if the face is still visible
    if(testDisappear){
//               std::cout << "Face " << i << " disappears" << std::endl;
      changed = true;
      Lpol[i]->isvisible = false;
    }
    else {
      //nbVisiblePolygon++;
      Lpol[i]->isvisible = true;

      if(nbCornerInsidePrev > Lpol[i]->getNbCornerInsidePrevImage())
        changed = true;
    }
  }
  else
  {
    bool testAppear = true;

//    if(testRoi && Lpol[i]->getNbCornerInsideImage(I, cam) == 0)
//      testAppear = false;

    if(testAppear){
      if(useOgre)
#ifdef VISP_HAVE_OGRE
        testAppear = ((Lpol[i]->isVisible(cMo, angleAppears, true, cam, I)) && isVisibleOgre(cameraPos,i));
#else
        testAppear = (Lpol[i]->isVisible(cMo, angleAppears, false, cam, I));
#endif
      else
        testAppear = (Lpol[i]->isVisible(cMo, angleAppears, false, cam, I));
    }

    if(testAppear){
//      std::cout << "Face " << i << " appears" << std::endl;
      Lpol[i]->isvisible = true;
      changed = true;
      //nbVisiblePolygon++;
    }
    else{
//      std::cout << "Problem" << std::endl;
      Lpol[i]->isvisible = false;
    }
  }
  }
  //   std::cout << "Nombre de polygones visibles: " << nbVisiblePolygon << std::endl;
  return Lpol[i]->isvisible;
}

/*!
  Compute the number of visible polygons.
  
  \param I : Image used to check if the region of interest is inside the image.
  \param cam : Camera parameters.
  \param cMo : The pose of the camera.
  \param angle : Angle used to test the appearance and disappearance of a face.
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angle, bool &changed)
{
  return setVisible(I, cam, cMo, angle, angle, changed);
}

/*!
  Compute the number of visible polygons.
  
  \param I : Image used to check if the region of interest is inside the image.
  \param cam : Camera parameters.
  \param cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo,angleAppears,angleDisappears,changed,false,true,I,cam);
}

/*!
  Compute the number of visible polygons.
  
  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo,angleAppears,angleDisappears,changed,false);
}

#ifdef VISP_HAVE_OGRE
/*!
  Initialise the ogre context for face visibility tests.
  
  \param cam : Camera parameters.
*/
template<class PolygonType>
void 
vpMbHiddenFaces<PolygonType>::initOgre(const vpCameraParameters &cam)
{
  ogreInitialised = true;
  ogre->setCameraParameters(cam);
  ogre->init(ogreBackground, false, true);
  
  for(unsigned int n = 0 ; n < Lpol.size(); n++){
    Ogre::ManualObject* manual = ogre->getSceneManager()->createManualObject(Ogre::StringConverter::toString(n));
  
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    for(unsigned int i = 0; i < Lpol[n]->nbpt; i++){
      manual->position( (Ogre::Real)Lpol[n]->p[i].get_oX(), (Ogre::Real)Lpol[n]->p[i].get_oY(), (Ogre::Real)Lpol[n]->p[i].get_oZ());
      manual->colour(1.0, 1.0, 1.0);
      manual->index(i);
    }
    
    manual->index(0);
    manual->end();
    
    ogre->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
    
    lOgrePolygons.push_back(manual);
  }
}

/*!
  Update the display in Ogre Window.
  
  \param cMo : Pose used to display.
*/
template<class PolygonType>
void
vpMbHiddenFaces<PolygonType>::displayOgre(const vpHomogeneousMatrix &cMo)
{
  if(ogreInitialised && !ogre->isWindowHidden()){
    for(unsigned int i = 0 ; i < Lpol.size() ; i++){
      if(Lpol[i]->isVisible()){
        lOgrePolygons[i]->setVisible(true);
      }
      else
        lOgrePolygons[i]->setVisible(false);
    }
    ogre->display(ogreBackground, cMo);
  }
}

/*!
  Compute the number of visible polygons through Ogre3D.
  
  \param I : Image used to check if the region of interest is inside the image.
  \param cam : Camera parameters.
  \param cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpImage<unsigned char>& I, const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo,angleAppears,angleDisappears,changed,true,true,I,cam);
}

/*!
  Compute the number of visible polygons through Ogre3D.
  
  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points have been lost. False otherwise
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo,angleAppears,angleDisappears,changed,true);
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
  double distance, distancePrev;
  if(it != result.end()){
    if(it->movable->getName().find("SimpleRenderable") != Ogre::String::npos) //Test if the ogreBackground is intersect in first
      it++;

    if(it != result.end()){
      distance = it->distance;
      distancePrev = distance;
      if(it->movable->getName() == Ogre::StringConverter::toString(index)){
        visible = true;
      }
      else{
        it++;
        while(!visible && it != result.end()){
          distance = it->distance;
          //if(distance == distancePrev){
          if(std::fabs(distance - distancePrev) < distance * std::numeric_limits<double>::epsilon()){
            if(it->movable->getName() == Ogre::StringConverter::toString(index)){
              visible = true;
              break;
            }
            it++;
            distancePrev = distance;
          }
          else
            break;
        }
      }
    }
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

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated This method is deprecated since it is no more used since ViSP 2.7.2. \n
  
  Compute the number of visible polygons.
  
  \param cMo : The pose of the camera
  
  \return Return the number of visible polygons
*/
template<class PolygonType>
unsigned int
vpMbHiddenFaces<PolygonType>::setVisible(const vpHomogeneousMatrix &cMo)
{
  nbVisiblePolygon = 0 ;
  
  for(unsigned int i = 0 ; i < Lpol.size() ; i++){
    if (Lpol[i]->isVisible(cMo, depthTest)){
      nbVisiblePolygon++;
    }
  }
  return nbVisiblePolygon ;
}
#endif //VISP_BUILD_DEPRECATED_FUNCTIONS

#endif // vpMbHiddenFaces

