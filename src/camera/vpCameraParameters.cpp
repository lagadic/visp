/****************************************************************************
 *
 * $Id$
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
 *
 * Description:
 * Camera intrinsic parameters.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/


/*!
  \file vpCameraParameters.cpp
  \brief Definition of the vpCameraParameters class member functions.
  Class vpCameraParameters define the camera intrinsic parameters

*/

#include <visp/vpCameraParameters.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>
#include <cmath>
#include <limits>

const double vpCameraParameters::DEFAULT_PX_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_PY_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_U0_PARAMETER = 192.0;
const double vpCameraParameters::DEFAULT_V0_PARAMETER = 144.0;
const double vpCameraParameters::DEFAULT_KUD_PARAMETER = 0.0;
const double vpCameraParameters::DEFAULT_KDU_PARAMETER = 0.0;
const vpCameraParameters::vpCameraParametersProjType
    vpCameraParameters::DEFAULT_PROJ_TYPE =
    vpCameraParameters::perspectiveProjWithoutDistortion;

/*!
  Default constructor.
  By default, a perspective projection without distortion model is set.

  \sa init()
*/
vpCameraParameters::vpCameraParameters()
{
  init() ;
}

/*!
  Copy constructor
 */
vpCameraParameters::vpCameraParameters(const vpCameraParameters &c)
{
  init(c) ;
}

/*!
  Constructor for perspective projection without distortion model

  \param px,py : pixel size
  \param u0,v0 : principal points

 */
vpCameraParameters::vpCameraParameters(const double px, const double py,
                                       const double u0, const double v0)
{
  initPersProjWithoutDistortion(px,py,u0,v0) ;
}

/*!
  Constructor for perspective projection with distortion model

  \param px,py : pixel size
  \param u0,v0 : principal points
  \param kud : undistorted to distorted radial distortion
  \param kdu : distorted to undistorted radial distortion

 */
vpCameraParameters::vpCameraParameters(const double px, const double py,
                                       const double u0, const double v0,
                                       const double kud, const double kdu)
{
  initPersProjWithDistortion(px,py,u0,v0,kud,kdu) ;
}

/*!
  \brief basic initialization with the default parameters
*/
void
vpCameraParameters::init()
{
  this->projModel = DEFAULT_PROJ_TYPE ;
  
  this->px    = DEFAULT_PX_PARAMETER ;
  this->py    = DEFAULT_PY_PARAMETER ;
  this->u0    = DEFAULT_U0_PARAMETER ;
  this->v0    = DEFAULT_V0_PARAMETER ;
  this->kud   = DEFAULT_KUD_PARAMETER ;
  this->kdu   = DEFAULT_KDU_PARAMETER ;
  
  if (fabs(this->px)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  if (fabs(this->py)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  this->inv_px = 1./this->px;
  this->inv_py = 1./this->py;
}

/*!
  Initialization with specific parameters using perpective projection without
  distortion model.
  \param px,py : pixel size
  \param u0,v0 : principal point
 */
void
vpCameraParameters::initPersProjWithoutDistortion(const double px,
    const double py, const double u0, const double v0)
{
  this->projModel = vpCameraParameters::perspectiveProjWithoutDistortion ;
  
  this->px    = px ;
  this->py    = py ;
  this->u0    = u0 ;
  this->v0    = v0 ;
  this->kud   = 0 ;
  this->kdu   = 0 ;
  
  if (fabs(px)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  if (fabs(py)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  this->inv_px = 1./px;
  this->inv_py = 1./py;
}

/*!
  Initialization with specific parameters using perpective projection with
  distortion model.
  \param px,py : pixel size
  \param u0,v0 : principal points
  \param kud : undistorted to distorted radial distortion
  \param kdu : distorted to undistorted radial distortion
*/
void
vpCameraParameters::initPersProjWithDistortion(const double px, const double py,
                            const double u0, const double v0,
                            const double kud, const double kdu)
{
  this->projModel = vpCameraParameters::perspectiveProjWithDistortion ;

  this->px    = px ;
  this->py    = py ;
  this->u0    = u0 ;
  this->v0    = v0 ;
  this->kud   = kud ;
  this->kdu   = kdu ;
  
  if (fabs(px)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  if (fabs(py)<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  this->inv_px = 1./px;
  this->inv_py = 1./py;
}

/*!
  destructor

  nothing much to destroy...
*/
vpCameraParameters::~vpCameraParameters()
{
}

/*!
  initialization from another vpCameraParameters object
*/
void
vpCameraParameters::init(const vpCameraParameters &c)
{
  *this = c ;
}


/*!
  initialise the camera from a calibration matrix. 
  Using a calibration matrix leads to a camera without distorsion
  
  The K matrix in parameters must be like:
  
  \f$ K = \left(\begin{array}{ccc}
  p_x & 0 & u_0 \\
  0 & p_y & v_0  \\
  0 & 0 & 1
  \end{array} \right) \f$
  
  \param _K : the 3by3 calibration matrix
*/
void
vpCameraParameters::initFromCalibrationMatrix(const vpMatrix& _K)
{
  if(_K.getRows() != 3 || _K.getCols() != 3 ){
    throw vpException(vpException::dimensionError, "bad size for calibration matrix");
  }
  if( std::fabs(_K[2][2] - 1.0) > std::numeric_limits<double>::epsilon()){
    throw vpException(vpException::badValue, "bad value: K[2][2] must be equal to 1");
  }
  initPersProjWithoutDistortion (_K[0][0], _K[1][1], _K[0][2], _K[1][2]);
}


/*!
  copy operator
 */
vpCameraParameters&
    vpCameraParameters::operator=(const vpCameraParameters& cam)
{
  projModel = cam.projModel ;
  px = cam.px ;
  py = cam.py ;
  u0 = cam.u0 ;
  v0 = cam.v0 ;
  kud = cam.kud ;
  kdu = cam.kdu ;
  
  inv_px = cam.inv_px; 
  inv_py = cam.inv_py;
  return *this ;
}

/*!
  return the calibration matrix K

  K is 3x3 matrix given by:

  \f$ K = \left(\begin{array}{ccc}
  p_x & 0 & u_0 \\
  0 & p_y & v_0  \\
  0 & 0 & 1
  \end{array} \right) \f$

  \warning : this function is useful only in the case of perspective
  projection without distortion.
*/
vpMatrix
vpCameraParameters::get_K() const
{
  vpMatrix K;
  switch(projModel){
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      K.resize(3,3) ;
      K = 0.0 ;
      K[0][0] = px ;
      K[1][1] = py ;
      K[0][2] = u0 ;
      K[1][2] = v0 ;
      K[2][2] = 1.0 ;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
    default :
      vpERROR_TRACE("\n\t getting K matrix in the case of projection \
          with distortion has no sense");   
      throw(vpException(vpException::notImplementedError,
            "\n\t getting K matrix in the case of projection \
                  with distortion has no sense"));
  }
  return K; 
}


/*!
  Print the camera parameters on the standard output

  \sa operator <<
*/
void
vpCameraParameters::printParameters()
{
  switch(projModel){
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      std::cout.precision(10);
      std::cout << "Camera parameters for perspective projection without distortion:"
                << std::endl ;
      std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      std::cout.precision(10);
      std::cout << "Camera parameters for perspective projection with distortion:"
                << std::endl ;
      std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      std::cout << "  kud = " << kud << std::endl ;
      std::cout << "  kdu = " << kdu << std::endl ;
      break;
  } 
}
/*!

  Print on the output stream \e os the camera parameters.

  \param os : Output stream.
  \param cam : Camera parameters.
*/
std::ostream & operator << (std::ostream & os,
			    const vpCameraParameters &cam)
{
  switch(cam.get_projModel()){
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      os << "Camera parameters for perspective projection without distortion:"
	 << std::endl ;
      os << "  px = " << cam.get_px() <<"\t py = "<< cam.get_py() 
	 << std::endl ;
      os << "  u0 = " << cam.get_u0() <<"\t v0 = "<< cam.get_v0() 
	 << std::endl ;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      os.precision(10);
      os << "Camera parameters for perspective projection with distortion:"
                << std::endl ;
      os << "  px = " << cam.get_px() <<"\t py = "<< cam.get_py() 
	 << std::endl ;
      os << "  u0 = " << cam.get_u0() <<"\t v0 = "<< cam.get_v0()
	 << std::endl ;
      os << "  kud = " << cam.get_kud() << std::endl ;
      os << "  kdu = " << cam.get_kdu() << std::endl ;
      break;
  } 
  return os;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

