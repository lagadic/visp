/****************************************************************************
 *
 * $Id: vpCameraParameters.cpp,v 1.9 2008-01-31 17:35:51 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
  Initialization with specific parameters using perpective projection without
  distortion model.
  \param px,py : pixel size
  \param u0,v0 : principal points

  \warning : this function is deprecated.
  Use initPersProjWithoutDistortion instead.
*/
void
vpCameraParameters::init(const double px, const double py,
			 const double u0, const double v0)
{
  initPersProjWithoutDistortion(px,py,u0,v0);
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

  \warning : this function is usefull only in the case of perspective
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
  Set the principal point

  \warning : deprecated function. Use an init function instead
*/
void
vpCameraParameters::setPrincipalPoint(double u0, double v0)
{
  this->u0    = u0 ;
  this->v0    = v0 ;
}

/*!
  Set the pixel size

  \warning : deprecated function. Use an init function instead
*/
void
vpCameraParameters::setPixelRatio(double px, double py)
{
  this->px    = px ;
  this->py    = py ;
  
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
  Print the camera parameters on the standard output
*/
void
vpCameraParameters::printParameters()
{
  switch(projModel){
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      std::cout.precision(10);
      std::cout << "Parameters for perspective projection without distortion :"
                << std::endl ;
      std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      std::cout.precision(10);
      std::cout << "Parameters for perspective projection with distortion :"
                << std::endl ;
      std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      std::cout << "  kud = " << kud << std::endl ;
      std::cout << "  kdu = " << kdu << std::endl ;
      break;
  } 
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

