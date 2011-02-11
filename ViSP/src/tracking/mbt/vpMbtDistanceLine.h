/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file vpMbtDistanceLine.h
 \brief Make the complete tracking of an object by using its CAD model.
*/

#ifndef vpMbtDistanceLine_HH
#define vpMbtDistanceLine_HH

#include <visp/vpConfig.h>

#include <visp/vpPoint.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpLine.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpMbtHiddenFace.h>

/*!
  \class vpMbtDistanceLine

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtDistanceLine
{
  private :
    std::string name;
    unsigned int index;
    vpCameraParameters *cam;
    vpMe *me;
    double alpha;
    double wmean;
    vpFeatureLine featureline ;
    
  public: 
    //! The moving edge container
    vpMbtMeLine *meline;
    //! The 3D line
    vpLine *line;
    //! The first extremity
    vpPoint *p1;
    //! The second extremity
    vpPoint *p2;
    //! The interaction matrix
    vpMatrix L;
    //! The error vector
    vpColVector error;
    //! The number of moving edges
    unsigned int nbFeature;
    //! Indicates if the line has to be reinitialized
    bool Reinit;
    //! Pointer to the list of faces
    vpMbtHiddenFaces *hiddenface;
    //! Index of the faces which contain the line
    vpList<int> Lindex_polygon;
    //! Indicates if the line is visible or not
    bool isvisible;
    
  public:
    vpMbtDistanceLine() ;
    ~vpMbtDistanceLine() ;

    /*!
     Get the camera paramters.
   
     \param cam : The vpCameraParameters used to store the camera parameters.
    */
    inline void getCameraParameters(vpCameraParameters *cam) {cam = this->cam;}
    
    /*!
     Set the camera paramters.
     \param cam : The camera parameters.
    */
    inline void setCameraParameters(vpCameraParameters *cam) {this->cam = cam;}
    
    /*!
     Get the mean weight of the line. The mean weight is computed thanks to the weight of each moving edge.
     Those weights are computed by the robust estimation method used during the virtual visual servoing.
   
     \return The mean weight of the line.
    */
    inline double getMeanWeight() const {return wmean;}
    
    /*!
     Set the mean weight of the line.
   
     \param wmean : The mean weight of the line.
    */
    inline void setMeanWeight(const double wmean) {this->wmean = wmean;}
    
    /*!
      Set a boolean parameter to indicates if the line is visible in the image or not.
      
      \param _isvisible : Set to true if the line is visible
    */
    inline void setVisible(bool _isvisible) {isvisible = _isvisible ;}
    
    /*!
      Check if the line is visible in the image or not.
      
      \return Return true if the line is visible
    */
    inline bool isVisible() {return isvisible; }
    
    /*!
      Set the index of the line.
      
      \param i : The index number
    */
    inline void setIndex(const unsigned int i) {index = i;} 
    
    /*!
      Get the index of the line.
      
      \return Return the index of the line.
    */
    inline unsigned int getIndex() {return index ;}
    
    /*!
      Get the name of the line.
      
      \return Return the name of the line
    */
    inline std::string getName() const {return name;}
    
    /*!
      Set the name of the line.
      
      \param name : The name of the line.
    */
    inline void setName(const std::string name) {this->name = name;}
    
    /*!
      Set the name of the line.
      
      \param name : The name of the line.
    */
    inline void setName(const char* name) {this->name = name;}

    void setMovingEdge(vpMe *Me);
    
    void buildFrom(vpPoint &_p1, vpPoint &_p2);
    
    void initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
    void trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
    void updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

    void initInteractionMatrixError();
    void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo);
    void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1, const bool displayFullModel = false);
    void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1, const bool displayFullModel = false);
    void displayMovingEdges(const vpImage<unsigned char> &I);

    bool closeToImageBorder(const vpImage<unsigned char>& I, const unsigned int threshold);

    void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  private:
    void project(const vpHomogeneousMatrix &cMo);
    void setFace( vpMbtHiddenFaces *_hiddenface) { hiddenface = _hiddenface ; }
    void belongToPolygon(int index) { Lindex_polygon += index ; }

} ;

#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT vpList<vpMbtDistanceLine *>;
#endif

#endif
#endif

