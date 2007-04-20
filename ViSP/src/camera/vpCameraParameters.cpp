/****************************************************************************
 *
 * $Id: vpCameraParameters.cpp,v 1.4 2007-04-20 14:22:15 asaunier Exp $
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
 *
 *****************************************************************************/


/*!
  \file vpCameraParameters.cpp
  \brief Definition of the vpCameraParameters class member functions.
  Class vpCameraParameters define the camera intrinsic parameters

*/

#include <visp/vpCameraParameters.h>

const double vpCameraParameters::DEFAULT_U0_PARAMETER = 192.0;
const double vpCameraParameters::DEFAULT_V0_PARAMETER = 144.0;
const double vpCameraParameters::DEFAULT_PX_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_PY_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_KD_PARAMETER = 0.0;

/*!
  basic constructor

  /sa init()
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
  \brief basic initialization with the default parameters
*/
void
vpCameraParameters::init()
{

  u0 = DEFAULT_U0_PARAMETER ;
  v0 = DEFAULT_V0_PARAMETER ;

  px = DEFAULT_PX_PARAMETER ;
  py = DEFAULT_PY_PARAMETER ;

  kd = DEFAULT_KD_PARAMETER ;

  computeMatrix()  ;
}


vpCameraParameters::vpCameraParameters(const double _px, const double _py,
		     const double _u0, const double _v0,
		     const double _kd)
{
  init(_px,_py,_u0, _v0, _kd) ;
}

void
vpCameraParameters::init(const double _px, const double _py,
			 const double _u0, const double _v0,
			 const double _kd )
{
  setPrincipalPoint(_u0, _v0) ;
  setPixelRatio(_px,_py) ;
  setKd(_kd) ;
  computeMatrix() ;
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
  compute the calibration matrix K

  K is 3x3 matrix given by:

  \f$ K = \left(\begin{array}{ccc}
  p_x & 0 & u_0 \\
  0 & p_y & v_0  \\
  0 & 0 & 1
  \end{array} \right) \f$
*/
void
vpCameraParameters::computeMatrix()
{
  K.resize(3,3) ;
  K = 0.0 ;
  K[0][0] = px ;
  K[1][1] = py ;
  K[0][2] = u0 ;
  K[1][2] = v0 ;
  K[2][2] = 1.0 ;
}


/*!
  copy operator
*/
vpCameraParameters&
vpCameraParameters::operator=(const vpCameraParameters& cam)
{

  u0 = cam.u0 ;
  v0 = cam.v0 ;

  px = cam.px ;
  py = cam.py ;

  kd = cam.kd ;

  computeMatrix()  ;

  return *this ;
}

/*!
  set the principal point

  \sa vpCameraParameters::u0, vpCameraParameters::v0
*/
void
vpCameraParameters::setPrincipalPoint(double x, double y)
{
  u0 = x ;
  v0 = y ;

  computeMatrix()  ;
}


/*!
  set the radial distortion value

  \sa vpCameraParameters::kd
*/
void
vpCameraParameters::setKd(double k)
{
  kd = k ;
  computeMatrix()  ;
}

/*!
  set the pixel size

  \sa vpCameraParameters::px,  vpCameraParameters::py
*/
void
vpCameraParameters::setPixelRatio(double Px,double Py)
{
  px = Px ;
  py = Py ;

  computeMatrix() ;
}



/*!
  Print the camera parameters on the standard output

  \param version (1: all the parameters, 2: only  u0, v0, px, py )

*/
void
vpCameraParameters::printParameters(int version)
{
  if(version==1)
  {
    std::cout << "Center : (" << u0<<","<<v0<<")" << std::endl ;
    std::cout << "px = " << px <<"\t py = "<<py<< std::endl ;
    std::cout << "Kd = " << kd << std::endl ;
  }
  else
  {
    if (version==2)
    {
      std::cout << "Center : (" << u0 <<","<<v0<<")" << std::endl ;
      std::cout << "px = " << px <<"\t py = "<<py<< std::endl ;

    }
    else
    {
      std::cout << "Center : (" << u0 <<","<<v0<<")" << std::endl ;
    }
  }
}


