/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Manage the line of a polygon used in the model-based tracker.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
 \file vpMbtDistanceLine.h
 \brief Manage the line of a polygon used in the model-based tracker.
*/

#ifndef vpMbtDistanceLine_HH
#define vpMbtDistanceLine_HH

#include <visp/vpPoint.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpLine.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpMbHiddenFaces.h>

#include <list>

/*!
  \class vpMbtDistanceLine
  
  \brief Manage the line of a polygon used in the model-based tracker.

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtDistanceLine
{
  private :
    std::string name;
    unsigned int index;
    vpCameraParameters cam;
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
    vpMbHiddenFaces<vpMbtPolygon> *hiddenface;
    //! Index of the faces which contain the line
    std::list<int> Lindex_polygon;
    //! Indicates if the line is visible or not
    bool isvisible;
    
  public:
    vpMbtDistanceLine() ;
    ~vpMbtDistanceLine() ;

    void buildFrom(vpPoint &_p1, vpPoint &_p2);
    
    bool closeToImageBorder(const vpImage<unsigned char>& I, const unsigned int threshold);
    void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo);
    
    void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1, const bool displayFullModel = false);
    void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1, const bool displayFullModel = false);
    void displayMovingEdges(const vpImage<unsigned char> &I);
    
    /*!
     Get the camera paramters.
   
     \param cam : The vpCameraParameters used to store the camera parameters.
    */
    inline void getCameraParameters(vpCameraParameters& cam) {cam = this->cam;}
    
    /*!
      Get the index of the line.
      
      \return Return the index of the line.
    */
    inline unsigned int getIndex() {return index ;}
    
    /*!
     Get the mean weight of the line. The mean weight is computed thanks to the weight of each moving edge.
     Those weights are computed by the robust estimation method used during the virtual visual servoing.
   
     \return The mean weight of the line.
    */
    inline double getMeanWeight() const {return wmean;}
    
    /*!
      Get the name of the line.
      
      \return Return the name of the line
    */
    inline std::string getName() const {return name;}
    
    void initInteractionMatrixError();
    
    void initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
    
    /*!
      Check if the line is visible in the image or not.
      
      \return Return true if the line is visible
    */
    inline bool isVisible() const {return isvisible; }
    
    void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
    
    /*!
     Set the camera paramters.
     \param cam : The camera parameters.
    */
    inline void setCameraParameters(const vpCameraParameters& cam) {this->cam = cam;}
    
    /*!
      Set the index of the line.
      
      \param i : The index number
    */
    inline void setIndex(const unsigned int i) {index = i;} 
    
    /*!
     Set the mean weight of the line.
   
     \param wmean : The mean weight of the line.
    */
    inline void setMeanWeight(const double wmean) {this->wmean = wmean;}
    
    void setMovingEdge(vpMe *Me);
    
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

    /*!
      Set a boolean parameter to indicates if the line is visible in the image or not.
      
      \param _isvisible : Set to true if the line is visible
    */
    inline void setVisible(bool _isvisible) {isvisible = _isvisible ;}
    
    void trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
    
    void updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  private:
    void belongToPolygon(int index) { Lindex_polygon.push_back(index); }
    void project(const vpHomogeneousMatrix &cMo);
    void setFace( vpMbHiddenFaces<vpMbtPolygon> *_hiddenface) { hiddenface = _hiddenface ; }
    

} ;

#endif

