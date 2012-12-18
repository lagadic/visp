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
 * Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

#ifndef vpMbEdgeKltTracker_HH
#define vpMbEdgeKltTracker_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV

#include <visp/vpRobust.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpMbtXmlParser.h>
#include <visp/vpMbTracker.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpPoseVector.h>
#include <visp/vpMbKltTracker.h>

/*!
  \class vpMbEdgeKltTracker
  
  \warning This class is only available if OpenCV is installed, and used.

  \ingroup ModelBasedTracking
*/
class VISP_EXPORT vpMbEdgeKltTracker: public vpMbKltTracker, public vpMbEdgeTracker
{
private:
  //! If true, compute the interaction matrix at each iteration of the minimisation. Otherwise, compute it only on the first iteration.
  bool compute_interaction;
  //! The gain of the virtual visual servoing stage.
  double lambda;
  //! The threshold used in the robust estimation of KLT.
  double thresholdKLT;
  //! The threshold used in the robust estimation of MBT.
  double thresholdMBT;
  //! The maximum iteration of the virtual visual servoing stage.
  unsigned int  maxIter;

public:
  
  vpMbEdgeKltTracker();
  virtual         ~vpMbEdgeKltTracker();

  virtual void    display(const vpImage<unsigned char>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &_cam, const vpColor& col , const unsigned int l=1, const bool displayFullModel = false);
  virtual void    display(const vpImage<vpRGBa>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &_cam, const vpColor& col , const unsigned int l=1, const bool displayFullModel = false);

  /*!
            Get the value of the gain used to compute the control law.

            \return the value for the gain.
          */
  inline  double  getLambda() {return lambda;}

  /*!
            Get the maximum iteration of the virtual visual servoing stage.

            \return the number of iteration
          */
  inline  unsigned int getMaxIter() {return maxIter;}

          void    loadConfigFile(const char* filename);
  virtual void    loadConfigFile(const std::string& _configFile);
  virtual void    loadModel(const std::string& _modelFile);

  virtual void    setCameraParameters(const vpCameraParameters& _cam);

  /*!
            Set the value of the gain used to compute the control law.

            \param lambda : the desired value for the gain.
          */
  inline  void    setLambda(const double lambda) {this->lambda = lambda; vpMbEdgeTracker::setLambda(lambda); vpMbKltTracker::setLambda(lambda);}

  /*!
            Set the maximum iteration of the virtual visual servoing stage.

            \param max : the desired number of iteration
          */
  inline  void    setMaxIter(const unsigned int max) {maxIter = max;}

          void    setPose(const vpHomogeneousMatrix &_cMo) ;
          
          /*!
            Use Ogre3D for visibility tests
            
            \warning This function has to be called before the initialisation of the tracker.
            
            \param v : True to use it, False otherwise
          */
  virtual inline  void    setOgreVisibilityTest(const bool &v) { vpMbKltTracker::setOgreVisibilityTest(v); }

  virtual void    testTracking(){};
  virtual void    track(const vpImage<unsigned char>& _I);

protected:
          void    computeVVS(const vpImage<unsigned char>& _I, const unsigned int &nbInfos, vpColVector &w_mbt, vpColVector &w_klt, const unsigned int lvl=0);

  virtual void    init(const vpImage<unsigned char>& _I);
  virtual void    initCylinder(const vpPoint& , const vpPoint , const double , const unsigned int ){};
  virtual void    initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1);
          int     initMbtTracking(const unsigned int _level=0);

          bool    postTracking(const vpImage<unsigned char>& _I, vpColVector &w_mbt, vpColVector &w_klt, const unsigned int lvl=0);
          void    postTrackingMbt(vpColVector &_w, const unsigned int _level=0);

          int     trackFirstLoop(const vpImage<unsigned char>& _I, vpColVector &factor, const unsigned int _lvl = 0);
          void    trackSecondLoop(vpMatrix &_L, vpColVector &_error, vpHomogeneousMatrix& _cMo, const unsigned int _lvl=0);
};

#endif

#endif //VISP_HAVE_OPENCV
