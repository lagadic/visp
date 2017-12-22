/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Generic model based tracker. This class declares the methods to implement
 *in order to have a model based tracker.
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/
#pragma once

#ifndef vpMbHiddenFaces_HH
#define vpMbHiddenFaces_HH

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/mbt/vpMbScanLine.h>
#include <visp3/mbt/vpMbtPolygon.h>

#ifdef VISP_HAVE_OGRE
#include <visp3/ar/vpAROgre.h>
#endif

#include <limits>
#include <vector>

template <class PolygonType> class vpMbHiddenFaces;

template <class PolygonType> void swap(vpMbHiddenFaces<PolygonType> &first, vpMbHiddenFaces<PolygonType> &second);

/*!
  \class vpMbHiddenFaces

  \brief Implementation of the polygons management for the model-based
  trackers.

  \ingroup group_mbt_faces
 */
template <class PolygonType = vpMbtPolygon> class vpMbHiddenFaces
{
private:
  //! List of polygons
  std::vector<PolygonType *> Lpol;
  //! Number of visible polygon
  unsigned int nbVisiblePolygon;
  vpMbScanLine scanlineRender;

#ifdef VISP_HAVE_OGRE
  vpImage<unsigned char> ogreBackground;
  bool ogreInitialised;
  unsigned int nbRayAttempts;
  double ratioVisibleRay;
  vpAROgre *ogre;
  std::vector<Ogre::ManualObject *> lOgrePolygons;
  bool ogreShowConfigDialog;
#endif

  unsigned int setVisiblePrivate(const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                 const double &angleDisappears, bool &changed, bool useOgre = false,
                                 bool not_used = false, const vpImage<unsigned char> &I = vpImage<unsigned char>(),
                                 const vpCameraParameters &cam = vpCameraParameters());

public:
  vpMbHiddenFaces();
  ~vpMbHiddenFaces();
  vpMbHiddenFaces(const vpMbHiddenFaces &copy);
  vpMbHiddenFaces &operator=(vpMbHiddenFaces other);
  friend void swap<>(vpMbHiddenFaces &first, vpMbHiddenFaces &second);

  void addPolygon(PolygonType *p);

  bool computeVisibility(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                         bool &changed, bool useOgre, bool not_used, const vpImage<unsigned char> &I,
                         const vpCameraParameters &cam, const vpTranslationVector &cameraPos, unsigned int index);

  void computeClippedPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);

  void computeScanLineRender(const vpCameraParameters &cam, const unsigned int &w, const unsigned int &h);

  void computeScanLineQuery(const vpPoint &a, const vpPoint &b, std::vector<std::pair<vpPoint, vpPoint> > &lines,
                            const bool &displayResults = false);

  vpMbScanLine &getMbScanLineRenderer() { return scanlineRender; }

#ifdef VISP_HAVE_OGRE
  void displayOgre(const vpHomogeneousMatrix &cMo);
#endif

  /*!
   Get the list of polygons.

    \return Mbt Klt polygons list.
  */
  std::vector<PolygonType *> &getPolygon() { return Lpol; }

#ifdef VISP_HAVE_OGRE
  void initOgre(const vpCameraParameters &cam = vpCameraParameters());
#endif

  /*!
    get the number of visible polygons.

    \return number of visible polygons.
  */
  unsigned int getNbVisiblePolygon() const { return nbVisiblePolygon; }

#ifdef VISP_HAVE_OGRE
  /*!
    Get the number of rays that will be sent toward each polygon for
    visibility test. Each ray will go from the optic center of the camera to a
    random point inside the considered polygon.

    \sa getGoodNbRayCastingAttemptsRatio()

    \return Number of rays sent.
  */
  unsigned int getNbRayCastingAttemptsForVisibility() { return nbRayAttempts; }

  /*!
    Get the Ogre3D Context.

    \return A pointer on a vpAROgre instance.
  */
  vpAROgre *getOgreContext() { return ogre; }

  /*!
    Get the ratio of visibility attempts that has to be successful to consider
    a polygon as visible.

    \sa getNbRayCastingAttemptsForVisibility()

    \return Ratio of succesful attempts that has to be considered. Value will
    be between 0.0 (0%) and 1.0 (100%).
  */
  double getGoodNbRayCastingAttemptsRatio() { return ratioVisibleRay; }
#endif

  bool isAppearing(const unsigned int i) { return Lpol[i]->isAppearing(); }

#ifdef VISP_HAVE_OGRE
  /*!
    Tell whether if Ogre Context is initialised or not.

    \return True if it does, false otherwise.
  */
  bool isOgreInitialised() { return ogreInitialised; }
#endif

  /*!
  Check if the polygon at position i in the list is visible.

  \param i : TPosition in the list.

  \return Return true if the polygon is visible.
*/
  bool isVisible(const unsigned int i) { return Lpol[i]->isVisible(); }

#ifdef VISP_HAVE_OGRE
  bool isVisibleOgre(const vpTranslationVector &cameraPos, const unsigned int &index);
#endif

  //! operator[] as modifier.
  inline PolygonType *operator[](const unsigned int i) { return Lpol[i]; }
  //! operator[] as reader.
  inline const PolygonType *operator[](const unsigned int i) const { return Lpol[i]; }

  void reset();

#ifdef VISP_HAVE_OGRE
  /*!
    Set the background size (by default it is 640x480).
    The background size has to match with the size of the image that you are
    using for the traking.

    \warning This function has to be called before initOgre().

    \param h : Height of the background
    \param w : Width of the background
  */
  void setBackgroundSizeOgre(const unsigned int &h, const unsigned int &w)
  {
    ogreBackground = vpImage<unsigned char>(h, w, 0);
  }

  /*!
    Set the number of rays that will be sent toward each polygon for
    visibility test. Each ray will go from the optic center of the camera to a
    random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const double &)

    \param attempts Number of rays to be sent.
  */
  void setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) { nbRayAttempts = attempts; }

  /*!
    Set the ratio of visibility attempts that has to be successful to consider
    a polygon as visible.

    \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

    \param ratio : Ratio of succesful attempts that has to be considered.
    Value has to be between 0.0 (0%) and 1.0 (100%).
  */
  void setGoodNbRayCastingAttemptsRatio(const double &ratio)
  {
    ratioVisibleRay = ratio;
    if (ratioVisibleRay > 1.0)
      ratioVisibleRay = 1.0;
    if (ratioVisibleRay < 0.0)
      ratioVisibleRay = 0.0;
  }
  /*!
    Enable/Disable the appearance of Ogre config dialog on startup.

    \warning This method has only effect when Ogre is used and Ogre visibility
    test is enabled using setOgreVisibilityTest() with true parameter.

    \param showConfigDialog : if true, shows Ogre dialog window (used to set
    Ogre rendering options) when Ogre visibility is enabled. By default, this
    functionality is turned off.
  */
  inline void setOgreShowConfigDialog(const bool showConfigDialog) { ogreShowConfigDialog = showConfigDialog; }
#endif

  unsigned int setVisible(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                          const vpHomogeneousMatrix &cMo, const double &angle, bool &changed);
  unsigned int setVisible(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                          const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                          bool &changed);
  unsigned int setVisible(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                          bool &changed);

#ifdef VISP_HAVE_OGRE
  unsigned int setVisibleOgre(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                              const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                              bool &changed);
  unsigned int setVisibleOgre(const vpHomogeneousMatrix &cMo, const double &angleAppears, const double &angleDisappears,
                              bool &changed);
#endif
  /*!
   Get the number of polygons.

    \return Size of the list.
  */
  inline unsigned int size() const { return (unsigned int)Lpol.size(); }
};

/*!
  Basic constructor.
*/
template <class PolygonType>
vpMbHiddenFaces<PolygonType>::vpMbHiddenFaces() : Lpol(), nbVisiblePolygon(0), scanlineRender()
{
#ifdef VISP_HAVE_OGRE
  ogreInitialised = false;
  nbRayAttempts = 1;
  ratioVisibleRay = 1.0;
  ogreShowConfigDialog = false;
  ogre = new vpAROgre();
  ogreBackground = vpImage<unsigned char>(480, 640, 0);
#endif
}

/*!
  Basic destructor.
*/
template <class PolygonType> vpMbHiddenFaces<PolygonType>::~vpMbHiddenFaces()
{
  for (unsigned int i = 0; i < Lpol.size(); i++) {
    if (Lpol[i] != NULL) {
      delete Lpol[i];
    }
    Lpol[i] = NULL;
  }
  Lpol.resize(0);

#ifdef VISP_HAVE_OGRE
  if (ogre != NULL) {
    delete ogre;
    ogre = NULL;
  }

  // This is already done by calling "delete ogre"
  //  for(unsigned int i = 0 ; i < lOgrePolygons.size() ; i++){
  //    if (lOgrePolygons[i]!=NULL){
  //      delete lOgrePolygons[i];
  //    }
  //    lOgrePolygons[i] = NULL;
  //  }

  lOgrePolygons.resize(0);
#endif
}

/*!
  Copy constructor.
*/
template <class PolygonType>
vpMbHiddenFaces<PolygonType>::vpMbHiddenFaces(const vpMbHiddenFaces<PolygonType> &copy)
  : Lpol(), nbVisiblePolygon(copy.nbVisiblePolygon), scanlineRender(copy.scanlineRender)
#ifdef VISP_HAVE_OGRE
    ,
    ogreBackground(copy.ogreBackground), ogreInitialised(copy.ogreInitialised), nbRayAttempts(copy.nbRayAttempts),
    ratioVisibleRay(copy.ratioVisibleRay), ogre(NULL), lOgrePolygons(), ogreShowConfigDialog(copy.ogreShowConfigDialog)
#endif
{
  // Copy the list of polygons
  for (unsigned int i = 0; i < copy.Lpol.size(); i++) {
    PolygonType *poly = new PolygonType(*copy.Lpol[i]);
    Lpol.push_back(poly);
  }
}

template <class PolygonType> void swap(vpMbHiddenFaces<PolygonType> &first, vpMbHiddenFaces<PolygonType> &second)
{
  using std::swap;
  swap(first.Lpol, second.Lpol);
  swap(first.nbVisiblePolygon, second.nbVisiblePolygon);
  swap(first.scanlineRender, second.scanlineRender);
#ifdef VISP_HAVE_OGRE
  swap(first.ogreInitialised, second.ogreInitialised);
  swap(first.nbRayAttempts, second.nbRayAttempts);
  swap(first.ratioVisibleRay, second.ratioVisibleRay);
  swap(first.ogreShowConfigDialog, second.ogreShowConfigDialog);
  swap(first.ogre, second.ogre);
  swap(first.ogreBackground, second.ogreBackground);
#endif
}

/*!
  Copy assignment operator.
*/
template <class PolygonType>
vpMbHiddenFaces<PolygonType> &vpMbHiddenFaces<PolygonType>::operator=(vpMbHiddenFaces<PolygonType> other)
{
  swap(*this, other);

  return *this;
}

/*!
  Add a polygon to the list of polygons.

  \param p : The polygon to add.
*/
template <class PolygonType> void vpMbHiddenFaces<PolygonType>::addPolygon(PolygonType *p)
{
  PolygonType *p_new = new PolygonType;
  p_new->index = p->index;
  p_new->setNbPoint(p->nbpt);
  p_new->isvisible = p->isvisible;
  p_new->useLod = p->useLod;
  p_new->minLineLengthThresh = p->minLineLengthThresh;
  p_new->minPolygonAreaThresh = p->minPolygonAreaThresh;
  p_new->setName(p->name);
  p_new->hasOrientation = p->hasOrientation;

  for (unsigned int i = 0; i < p->nbpt; i++)
    p_new->p[i] = p->p[i];
  Lpol.push_back(p_new);
}

/*!
  Reset the Hidden faces (remove the list of PolygonType)
*/
template <class PolygonType> void vpMbHiddenFaces<PolygonType>::reset()
{
  nbVisiblePolygon = 0;
  for (unsigned int i = 0; i < Lpol.size(); i++) {
    if (Lpol[i] != NULL) {
      delete Lpol[i];
    }
    Lpol[i] = NULL;
  }
  Lpol.resize(0);

#ifdef VISP_HAVE_OGRE
  if (ogre != NULL) {
    delete ogre;
    ogre = NULL;
  }

  // This is already done by calling "delete ogre"
  //  for(unsigned int i = 0 ; i < lOgrePolygons.size() ; i++){
  //    if (lOgrePolygons[i]!=NULL){
  //      delete lOgrePolygons[i];
  //    }
  //    lOgrePolygons[i] = NULL;
  //  }

  lOgrePolygons.resize(0);

  ogreInitialised = false;
  nbRayAttempts = 1;
  ratioVisibleRay = 1.0;
  ogre = new vpAROgre();
  ogreBackground = vpImage<unsigned char>(480, 640);
#endif
}

/*!
  Compute the clipped points of the polygons that have been added via
  addPolygon().

  \param cMo : Pose that will be used to clip the polygons.
  \param cam : Camera parameters that will be used to clip the polygons.
*/
template <class PolygonType>
void vpMbHiddenFaces<PolygonType>::computeClippedPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
  for (unsigned int i = 0; i < Lpol.size(); i++) {
    // For fast result we could just clip visible polygons.
    // However clipping all of them gives us the possibility to return more
    // information in the scanline visibility results
    //    if(Lpol[i]->isVisible())
    {
      Lpol[i]->changeFrame(cMo);
      Lpol[i]->computePolygonClipped(cam);
    }
  }
}

/*!
  Render the scene in order to perform, later via computeScanLineQuery(),
  visibility tests.

  \param cam : Camera parameters that will be used to render the scene.
  \param w : Width of the render window.
  \param h : Height of the render window.
*/
template <class PolygonType>
void vpMbHiddenFaces<PolygonType>::computeScanLineRender(const vpCameraParameters &cam, const unsigned int &w,
                                                         const unsigned int &h)
{
  std::vector<std::vector<std::pair<vpPoint, unsigned int> > > polyClipped(Lpol.size());
  std::vector<std::vector<std::pair<vpPoint, unsigned int> > *> listPolyClipped;
  std::vector<int> listPolyIndices;

  for (unsigned int i = 0; i < Lpol.size(); i++) {
    // For fast result we could just use visible polygons.
    // However using all of them gives us the possibility to return more
    // information in the scanline visibility results
    //    if(Lpol[i]->isVisible())
    {
      polyClipped[i].clear();
      Lpol[i]->getPolygonClipped(polyClipped[i]);
      if (polyClipped[i].size() != 0) {
        listPolyClipped.push_back(&polyClipped[i]);
        listPolyIndices.push_back(Lpol[i]->getIndex());
      }
    }
  }

  scanlineRender.drawScene(listPolyClipped, listPolyIndices, cam, w, h);
}

/*!
  Compute Scanline visibility results for a line.

  \warning computeScanLineRender() function has to be called before

  \param a : First point of the line.
  \param b : Second point of the line.
  \param lines : Result of the scanline visibility. List of the visible parts
  of the line. \param displayResults : True if the results have to be
  displayed. False otherwise
*/
template <class PolygonType>
void vpMbHiddenFaces<PolygonType>::computeScanLineQuery(const vpPoint &a, const vpPoint &b,
                                                        std::vector<std::pair<vpPoint, vpPoint> > &lines,
                                                        const bool &displayResults)
{
  scanlineRender.queryLineVisibility(a, b, lines, displayResults);
}

/*!
  Compute the number of visible polygons.

  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise \param useOgre : True if a Ogre is used to
  test the visibility, False otherwise \param not_used : Unused parameter.
  \param I : Image used to test if a face is entirely projected in the image.
  \param cam : Camera parameters.

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisiblePrivate(const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                                             const double &angleDisappears, bool &changed, bool useOgre,
                                                             bool not_used, const vpImage<unsigned char> &I,
                                                             const vpCameraParameters &cam)
{
  nbVisiblePolygon = 0;
  changed = false;

  vpTranslationVector cameraPos;

  if (useOgre) {
#ifdef VISP_HAVE_OGRE
    cMo.inverse().extract(cameraPos);
    ogre->renderOneFrame(ogreBackground, cMo);
#else
    vpTRACE("ViSP doesn't have Ogre3D, simple visibility test used");
#endif
  }

  for (unsigned int i = 0; i < Lpol.size(); i++) {
    // std::cout << "Calling poly: " << i << std::endl;
    if (computeVisibility(cMo, angleAppears, angleDisappears, changed, useOgre, not_used, I, cam, cameraPos, i))
      nbVisiblePolygon++;
  }
  return nbVisiblePolygon;
}

/*!
  Compute the visibility of a given face index.

  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise \param useOgre : True if a Ogre is used to
  test the visibility, False otherwise. \param not_used : Unused parameter.
  \param I : Image used to test if a face is entirely projected in the image.
  \param cam : Camera parameters.
  \param cameraPos : Position of the camera. Used only when Ogre is used as
  3rd party. \param index : Index of the face to consider.

  \return Return true if the face is visible.
*/
template <class PolygonType>
bool vpMbHiddenFaces<PolygonType>::computeVisibility(const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                                     const double &angleDisappears, bool &changed, bool useOgre,
                                                     bool not_used, const vpImage<unsigned char> &I,
                                                     const vpCameraParameters &cam,
                                                     const vpTranslationVector &cameraPos, unsigned int index)
{
  (void)not_used;
  unsigned int i = index;
  Lpol[i]->changeFrame(cMo);
  Lpol[i]->isappearing = false;

  // Commented because we need to compute visibility
  // even when dealing with line in level of detail case
  /*if(Lpol[i]->getNbPoint() <= 2)
  {
      Lpol[i]->isvisible = true;
  }
  else*/ {
    if (Lpol[i]->isVisible()) {
      bool testDisappear = false;
      // unsigned int nbCornerInsidePrev = 0;

      if (!testDisappear) {
        if (useOgre)
#ifdef VISP_HAVE_OGRE
          testDisappear = ((!Lpol[i]->isVisible(cMo, angleDisappears, true, cam, I)) || !isVisibleOgre(cameraPos, i));
#else
        {
          (void)cameraPos; // Avoid warning
          testDisappear = (!Lpol[i]->isVisible(cMo, angleDisappears, false, cam, I));
        }
#endif
        else
          testDisappear = (!Lpol[i]->isVisible(cMo, angleDisappears, false, cam, I));
      }

      // test if the face is still visible
      if (testDisappear) {
        //               std::cout << "Face " << i << " disappears" <<
        //               std::endl;
        changed = true;
        Lpol[i]->isvisible = false;
      } else {
        // nbVisiblePolygon++;
        Lpol[i]->isvisible = true;

        // if(nbCornerInsidePrev > Lpol[i]->getNbCornerInsidePrevImage())
        //  changed = true;
      }
    } else {
      bool testAppear = true;

      if (testAppear) {
        if (useOgre)
#ifdef VISP_HAVE_OGRE
          testAppear = ((Lpol[i]->isVisible(cMo, angleAppears, true, cam, I)) && isVisibleOgre(cameraPos, i));
#else
          testAppear = (Lpol[i]->isVisible(cMo, angleAppears, false, cam, I));
#endif
        else
          testAppear = (Lpol[i]->isVisible(cMo, angleAppears, false, cam, I));
      }

      if (testAppear) {
        //      std::cout << "Face " << i << " appears" << std::endl;
        Lpol[i]->isvisible = true;
        changed = true;
        // nbVisiblePolygon++;
      } else {
        //      std::cout << "Problem" << std::endl;
        Lpol[i]->isvisible = false;
      }
    }
  }
  //   std::cout << "Nombre de polygones visibles: " << nbVisiblePolygon <<
  //   std::endl;
  return Lpol[i]->isvisible;
}

/*!
  Compute the number of visible polygons.

  \param I : Image used to check if the region of interest is inside the
  image. \param cam : Camera parameters. \param cMo : The pose of the camera.
  \param angle : Angle used to test the appearance and disappearance of a
  face. \param changed : True if a face appeared, disappeared or too many
  points have been lost. False otherwise

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                                                      const vpHomogeneousMatrix &cMo, const double &angle,
                                                      bool &changed)
{
  return setVisible(I, cam, cMo, angle, angle, changed);
}

/*!
  Compute the number of visible polygons.

  \param I : Image used to check if the region of interest is inside the
  image. \param cam : Camera parameters. \param cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise \param angleAppears : Angle used to test the
  appearance of a face \param angleDisappears : Angle used to test the
  disappearance of a face

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisible(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                                                      const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                                      const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo, angleAppears, angleDisappears, changed, false, true, I, cam);
}

/*!
  Compute the number of visible polygons.

  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisible(const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                                      const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo, angleAppears, angleDisappears, changed, false);
}

#ifdef VISP_HAVE_OGRE
/*!
  Initialise the ogre context for face visibility tests.

  \param cam : Camera parameters.
*/
template <class PolygonType> void vpMbHiddenFaces<PolygonType>::initOgre(const vpCameraParameters &cam)
{
  ogreInitialised = true;
  ogre->setCameraParameters(cam);
  ogre->setShowConfigDialog(ogreShowConfigDialog);
  ogre->init(ogreBackground, false, true);

  for (unsigned int n = 0; n < Lpol.size(); n++) {
    Ogre::ManualObject *manual = ogre->getSceneManager()->createManualObject(Ogre::StringConverter::toString(n));

    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    for (unsigned int i = 0; i < Lpol[n]->nbpt; i++) {
      manual->position((Ogre::Real)Lpol[n]->p[i].get_oX(), (Ogre::Real)Lpol[n]->p[i].get_oY(),
                       (Ogre::Real)Lpol[n]->p[i].get_oZ());
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
template <class PolygonType> void vpMbHiddenFaces<PolygonType>::displayOgre(const vpHomogeneousMatrix &cMo)
{
  if (ogreInitialised && !ogre->isWindowHidden()) {
    for (unsigned int i = 0; i < Lpol.size(); i++) {
      if (Lpol[i]->isVisible()) {
        lOgrePolygons[i]->setVisible(true);
      } else
        lOgrePolygons[i]->setVisible(false);
    }
    ogre->display(ogreBackground, cMo);
  }
}

/*!
  Compute the number of visible polygons through Ogre3D.

  \param I : Image used to check if the region of interest is inside the
  image. \param cam : Camera parameters. \param cMo : The pose of the camera
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise \param angleAppears : Angle used to test the
  appearance of a face \param angleDisappears : Angle used to test the
  disappearance of a face

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpImage<unsigned char> &I,
                                                          const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo,
                                                          const double &angleAppears, const double &angleDisappears,
                                                          bool &changed)
{
  return setVisiblePrivate(cMo, angleAppears, angleDisappears, changed, true, true, I, cam);
}

/*!
  Compute the number of visible polygons through Ogre3D.

  \param cMo : The pose of the camera
  \param angleAppears : Angle used to test the appearance of a face
  \param angleDisappears : Angle used to test the disappearance of a face
  \param changed : True if a face appeared, disappeared or too many points
  have been lost. False otherwise

  \return Return the number of visible polygons
*/
template <class PolygonType>
unsigned int vpMbHiddenFaces<PolygonType>::setVisibleOgre(const vpHomogeneousMatrix &cMo, const double &angleAppears,
                                                          const double &angleDisappears, bool &changed)
{
  return setVisiblePrivate(cMo, angleAppears, angleDisappears, changed, true);
}

/*!
  Test the visibility of a polygon through Ogre3D via RayCasting.

  \param cameraPos : Position of the camera in the 3D world.
  \param index : Index of the polygon.

  \return Return true if the polygon is visible, False otherwise.
*/
template <class PolygonType>
bool vpMbHiddenFaces<PolygonType>::isVisibleOgre(const vpTranslationVector &cameraPos, const unsigned int &index)
{
  Ogre::Vector3 camera((Ogre::Real)cameraPos[0], (Ogre::Real)cameraPos[1], (Ogre::Real)cameraPos[2]);
  if (!ogre->getCamera()->isVisible(lOgrePolygons[index]->getBoundingBox())) {
    lOgrePolygons[index]->setVisible(false);
    Lpol[index]->isvisible = false;
    return false;
  }

  // Get the center of gravity
  bool visible = false;
  unsigned int nbVisible = 0;

  for (unsigned int i = 0; i < nbRayAttempts; i++) {
    Ogre::Vector3 origin(0, 0, 0);
    Ogre::Real totalFactor = 0.0f;

    for (unsigned int j = 0; j < Lpol[index]->getNbPoint(); j++) {
      Ogre::Real factor = 1.0f;

      if (nbRayAttempts > 1) {
        int r = rand() % 101;

        if (r != 0)
          factor = ((Ogre::Real)r) / 100.0f;
      }

      Ogre::Vector3 tmp((Ogre::Real)Lpol[index]->getPoint(j).get_oX(), (Ogre::Real)Lpol[index]->getPoint(j).get_oY(),
                        (Ogre::Real)Lpol[index]->getPoint(j).get_oZ());
      tmp *= factor;
      origin += tmp;
      totalFactor += factor;
    }

    origin /= totalFactor;

    Ogre::Vector3 direction = origin - camera;
    Ogre::Real distanceCollision = direction.length();

    direction.normalise();
    Ogre::RaySceneQuery *mRaySceneQuery = ogre->getSceneManager()->createRayQuery(Ogre::Ray(camera, direction));
    mRaySceneQuery->setSortByDistance(true);

    Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute();
    Ogre::RaySceneQueryResult::iterator it = result.begin();

    //    while(it != result.end()){
    //      std::cout << it->movable->getName() << "(" << it->distance<< ") :
    //      " << std::flush; it++;
    //    }
    //    std::cout << std::endl;
    //    it = result.begin();

    if (it != result.end())
      if (it->movable->getName().find("SimpleRenderable") !=
          Ogre::String::npos) // Test if the ogreBackground is intersect in
                              // first
        ++it;

    double distance;
    // In a case of a two-axis aligned segment, ray collision is not always
    // working.
    if (Lpol[index]->getNbPoint() == 2 &&
        (((std::fabs(Lpol[index]->getPoint(0).get_oX() - Lpol[index]->getPoint(1).get_oX()) <
           std::numeric_limits<double>::epsilon()) +
          (std::fabs(Lpol[index]->getPoint(0).get_oY() - Lpol[index]->getPoint(1).get_oY()) <
           std::numeric_limits<double>::epsilon()) +
          (std::fabs(Lpol[index]->getPoint(0).get_oZ() - Lpol[index]->getPoint(1).get_oZ()) <
           std::numeric_limits<double>::epsilon())) >= 2)) {
      if (it != result.end()) {
        if (it->movable->getName() == Ogre::StringConverter::toString(index)) {
          nbVisible++;
        } else {
          distance = it->distance;
          // Cannot use epsilon for comparison as ray lenght is slightly
          // different from the collision distance returned by
          // Ogre::RaySceneQueryResult.
          if (distance > distanceCollision || std::fabs(distance - distanceCollision) <
                                                  1e-6 /*std::fabs(distance) * std::numeric_limits<double>::epsilon()*/)
            nbVisible++;
        }
      } else
        nbVisible++; // Collision not detected but present.
    } else {
      if (it != result.end()) {
        distance = it->distance;
        double distancePrev = distance;

        // std::cout << "For " << Ogre::StringConverter::toString(index) << ":
        // " << it->movable->getName() << " / " << std::flush;

        if (it->movable->getName() == Ogre::StringConverter::toString(index)) {
          nbVisible++;
        } else {
          ++it;
          while (it != result.end()) {
            distance = it->distance;

            if (std::fabs(distance - distancePrev) <
                1e-6 /*std::fabs(distance) * std::numeric_limits<double>::epsilon()*/) {
              // std::cout << it->movable->getName() << " / " << std::flush;
              if (it->movable->getName() == Ogre::StringConverter::toString(index)) {
                nbVisible++;
                break;
              }
              ++it;
              distancePrev = distance;
            } else
              break;
          }
        }
      }
    }

    ogre->getSceneManager()->destroyQuery(mRaySceneQuery);
  }

  if (((double)nbVisible) / ((double)nbRayAttempts) > ratioVisibleRay ||
      std::fabs(((double)nbVisible) / ((double)nbRayAttempts) - ratioVisibleRay) <
          ratioVisibleRay * std::numeric_limits<double>::epsilon())
    visible = true;
  else
    visible = false;

  if (visible) {
    lOgrePolygons[index]->setVisible(true);
    Lpol[index]->isvisible = true;
  } else {
    lOgrePolygons[index]->setVisible(false);
    Lpol[index]->isvisible = false;
  }

  return Lpol[index]->isvisible;
}
#endif // VISP_HAVE_OGRE

#endif // vpMbHiddenFaces
