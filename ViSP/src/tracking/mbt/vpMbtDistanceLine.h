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
    int index;
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
    int nbFeature;
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
    inline void setIndex(int i) {index = i;} 
    
    /*!
      Get the index of the line.
      
      \return Return the index of the line.
    */
    inline int getIndex() {return index ;}
    
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
    
    void initMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo);
    void trackMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo);
    void updateMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo);

    void initInteractionMatrixError();
    void computeInteractionMatrixError(vpHomogeneousMatrix &cMo);
    void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, vpColor col, unsigned int thickness = 1);
    void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, vpColor col, unsigned int thickness = 1);
    void displayMovingEdges(vpImage<unsigned char> &I);

    

    void reinitMovingEdge(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo);

  private:
    void project(vpHomogeneousMatrix &cMo);
    void setFace( vpMbtHiddenFaces *_hiddenface) { hiddenface = _hiddenface ; }
    void belongToPolygon(int index) { Lindex_polygon += index ; }

} ;

#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT vpList<vpMbtDistanceLine *>;
#endif

#endif
#endif

