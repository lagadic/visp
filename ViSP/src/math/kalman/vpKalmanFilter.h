/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2009 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
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
 * Kalman filtering.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpKalmanFilter_h
#define vpKalmanFilter_h

#include <math.h>

#include <visp/vpKalman.h>

/*!
  \file vpKalmanFilter.h
  \brief Implementation of some specific Kalman filters.
*/

/*!
  \class vpKalmanFilter
  This class provides an implementation of some specific filters
*/
class vpKalmanFilter : public vpKalman
{
 public:
  /*!  
    Selector used to set the Kalman filter with a constant
    velocity state model.
  */ 
  typedef enum {
    /*! State as a constant velocity model with colored noise
        measurements as acceleration terms. Measured available are the
        velocities of the target. To know more about this state model,
        see initStateConstVelWithColoredNoise_MeasureVel(). */
    stateConstVelWithColoredNoise_MeasureVel, 
    /*! State as a constant acceleration model with colored noise
        measurements as acceleration terms. Measured available are the
        velocities of the target. To know more about this state model,
        see initStateConstAccWithColoredNoise_MeasureVel(). */
    stateConstAccWithColoredNoise_MeasureVel,
    /*! Used to indicate that the state model is not initialized. */
    unknown 
  } vpStateModel;
  
  /*!
    Default Kalman filter.
    
    By default the state model is unknown and set to
    vpKalmanFilter::unknown.
  */
  vpKalmanFilter() : vpKalman()
    {
      setStateModel(unknown);
    };
  /*!
    Set the Kalman state model. Depending on the state model, we set
    the state vector size and the measure vector size. 

    The example below shows how to use this method and then to get the
    size of the state and measure vectors.

    \code
#include <visp/vpKalmanFilter.h>

int main()
{
  vpKalmanFilter kalman;

  kalman.setStateModel(vpKalmanFilter::StateConstVelWithColoredNoise_MeasureVel);
  std::cout << "State vector size: " << kalman.getStateSize() << std::endl; // Value is 2
  std::cout << "Measure vector size: " << kalman.getMeasureSize() << std::endl; // Value is 1
}  
    \endcode
  */
  inline void setStateModel(vpStateModel model) {
    this->model = model;
    switch(model) {
    case stateConstVelWithColoredNoise_MeasureVel:
      size_state = 2;
      size_measure = 1;
      break;
    case stateConstAccWithColoredNoise_MeasureVel:
      size_state = 3;
      size_measure = 1;
      break;
    case unknown:
      size_state = 0;
      size_measure = 0;
      break;
    }
  }

  /*!
    Return the current state model.
   */
  inline vpStateModel getStateModel() {
    return model;
  }
  void filter(vpColVector &Z);
 
  /*! @name Filter initializer */
  //@{
  void initFilter(int nsignal, vpColVector &sigma_state,
		  vpColVector &sigma_measure, double rho, double dt);
  //@}

  /*! @name Filter initializer with constant velocity models */
  //@{
  void initStateConstVelWithColoredNoise_MeasureVel(int nsignal, 
						    vpColVector &sigma_state,
						    vpColVector &sigma_measure, 
						    double rho);
  //@}

  /*! @name Filters initializer with constant acceleration models */
  //@{
  void initStateConstAccWithColoredNoise_MeasureVel(int nsignal, 
						    vpColVector &sigma_state,
						    vpColVector &sigma_measure, 
						    double rho,
						    double dt);
  //@}

 private:
  vpStateModel model;

} ;



#endif
