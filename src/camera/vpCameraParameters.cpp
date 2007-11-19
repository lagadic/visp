/****************************************************************************
 *
 * $Id: vpCameraParameters.cpp,v 1.5 2007-11-19 15:40:58 asaunier Exp $
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
  u0    = DEFAULT_U0_PARAMETER ;
  u0_mp = DEFAULT_U0_PARAMETER ;
  u0_pm = DEFAULT_U0_PARAMETER ;

  v0    = DEFAULT_V0_PARAMETER ;
  v0_mp = DEFAULT_V0_PARAMETER ;
  v0_pm = DEFAULT_V0_PARAMETER ;
  
  px    = DEFAULT_PX_PARAMETER ;
  px_mp = DEFAULT_PX_PARAMETER ;
  px_pm = DEFAULT_PX_PARAMETER ;

  py    = DEFAULT_PY_PARAMETER ;
  py_mp = DEFAULT_PY_PARAMETER ;
  py_pm = DEFAULT_PY_PARAMETER ;

  kd_mp = DEFAULT_KD_PARAMETER ;
  kd_pm = DEFAULT_KD_PARAMETER ;

  computeMatrix()  ;
}


vpCameraParameters::vpCameraParameters(const double px, const double py,
		     const double u0, const double v0)
{
  init(px,py,u0, v0) ;
}

/*!
  initialization with specific parameters
  \param px,py : pixel size
  \param u0,v0 : principal points
*/
void
vpCameraParameters::init(const double px, const double py,
			 const double u0, const double v0)
{
  setPrincipalPoint(u0, v0) ;
  setPixelRatio(px,py) ;

  setPrincipalPoint_mp(u0, v0) ;
  setPixelRatio_mp(px,py) ;
  setKd_mp(0);

  setPixelRatio_pm(px,py) ;
  setPrincipalPoint_pm(u0, v0) ;
  setKd_pm(0);
    
  computeMatrix() ;
}

/*!
  initialization of the meter based model part with specific parameters
  \param px,py : pixel size
  \param u0,v0 : principal points
  \param kd : radial distortion
*/
void
vpCameraParameters::init_mp(const double px, const double py,
       const double u0, const double v0, const double kd)
{
  setPrincipalPoint_mp(u0, v0) ;
  setPixelRatio_mp(px,py) ;
  setKd_mp(kd);
}

/*!
  initialization of the pixel based model part with specific parameters
  \param px,py : pixel size
  \param u0,v0 : principal points
  \param kd : radial distortion
*/
void
vpCameraParameters::init_pm(const double px, const double py,
       const double u0, const double v0, const double kd)
{
  setPrincipalPoint_pm(u0, v0) ;
  setPixelRatio_pm(px,py) ;
  setKd_pm(kd);
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
  
  u0_mp = cam.u0_mp ;
  v0_mp = cam.v0_mp ;
  px_mp = cam.px_mp ;
  py_mp = cam.py_mp ;
  kd_mp = cam.kd_mp ;

  u0_pm = cam.u0_pm ;
  v0_pm = cam.v0_pm ;
  px_pm = cam.px_pm ;
  py_pm = cam.py_pm ;
  kd_pm = cam.kd_pm ;
  
  computeMatrix()  ;

  return *this ;
}

/*!
  set the principal point

  \sa vpCameraParameters::u0_mp, vpCameraParameters::v0_mp,
      vpCameraParameters::u0_pm, vpCameraParameters::v0_pm
*/
void
vpCameraParameters::setPrincipalPoint(double x, double y)
{
  u0    = x ;
  v0    = y ;
  
  if(kd_mp == 0){
    u0_mp = x ;
    v0_mp = y ;  
  }
  
  if(kd_pm == 0){
    u0_pm = x ;
    v0_pm = y ;
  }
  
  computeMatrix()  ;
}

/*!
  set the pixel size

  \sa vpCameraParameters::px_mp,  vpCameraParameters::py_mp,
      vpCameraParameters::px_pm,  vpCameraParameters::py_pm
*/
void
vpCameraParameters::setPixelRatio(double Px,double Py)
{
  px    = Px ;
  py    = Py ;

  if(kd_mp == 0){
    px_mp = Px ;
    py_mp = Py ;
  }

  if(kd_pm == 0){
    px_pm = Px ;
    py_pm = Py ;
  }
  computeMatrix() ;
}

/*!
  set the principal point of the meter based distortion model

  \sa vpCameraParameters::u0_mp, vpCameraParameters::v0_mp
*/
void
vpCameraParameters::setPrincipalPoint_mp(double x, double y)
{
  u0_mp = x ;
  v0_mp = y ;
}


/*!
  set the radial distortion value of the pixel based distortion model

  \sa vpCameraParameters::kd_mp
*/
void
vpCameraParameters::setKd_mp(double k)
{
  kd_mp = k ;
}

/*!
  set the pixel size of the meter based distortion model

  \sa vpCameraParameters::px_mp,  vpCameraParameters::py_mp
*/
void
vpCameraParameters::setPixelRatio_mp(double Px,double Py)
{
  px_mp = Px ;
  py_mp = Py ;
}

/*!
  set the principal point of the pixel based distortion model

  \sa vpCameraParameters::u0_pm, vpCameraParameters::v0_pm
*/
void
vpCameraParameters::setPrincipalPoint_pm(double x, double y)
{
  u0_pm = x ;
  v0_pm = y ;
}


/*!
  set the radial distortion value of the pixel based distortion model

  \sa vpCameraParameters::kd_pm
*/
void
vpCameraParameters::setKd_pm(double k)
{
  kd_pm = k ;
}

/*!
  set the pixel size of the pixel based distortion model

  \sa vpCameraParameters::px_pm,  vpCameraParameters::py_pm
*/
void
vpCameraParameters::setPixelRatio_pm(double Px,double Py)
{
  px_pm = Px ;
  py_pm = Py ;
}



/*!
  Print the camera parameters on the standard output

  \param version (1: all the parameters, 2: only  u0, v0, px, py )

*/
void
vpCameraParameters::printParameters(int version)
{
  std::cout.precision(10);
  if(version==1){
    std::cout << "Parameters for model without distortion :" << std::endl ;
    std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
    std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
    std::cout << "Parameters for meter to pixel distortion model :" << std::endl ;
    std::cout << "  u0 = " << u0_mp <<"\t v0 = "<< v0_mp << std::endl ;
    std::cout << "  px = " << px_mp <<"\t py = "<<py_mp<< std::endl ;
    std::cout << "  Kd = " << kd_mp << std::endl ;
    std::cout << "Parameters for pixel to meter distortion model :" << std::endl ;
    std::cout << "  u0 = " << u0_pm <<"\t v0 = "<< v0_pm << std::endl ;
    std::cout << "  px = " << px_pm <<"\t py = "<< py_pm << std::endl ;
    std::cout << "  Kd = " << kd_pm << std::endl ;
  }
  else{
    if (version==2){
      std::cout << "Parameters for model without distortion :" << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      std::cout << "  px = " << px <<"\t py = "<< py << std::endl ;
      std::cout << "Parameters for meter to pixel distortion model :" << std::endl ;
      std::cout << "  u0 = " << u0_mp <<"\t v0 = "<<v0_mp<< std::endl ;
      std::cout << "  px = " << px_mp <<"\t py = "<<py_mp<< std::endl ;
      std::cout << "Parameters for pixel to meter distortion model :" << std::endl ;
      std::cout << "  u0 = " << u0_pm <<"\t v0 = "<<v0_pm<< std::endl ;
      std::cout << "  px = " << px_pm <<"\t py = "<<py_pm<< std::endl ;
    }
    else {
      std::cout << "Parameters for model without distortion :" << std::endl ;
      std::cout << "  u0 = " << u0 <<"\t v0 = "<< v0 << std::endl ;
      std::cout << "Parameters for meter to pixel distortion model :" << std::endl ;
      std::cout << "  u0 = " << u0_mp <<"\t v0 = "<<v0_mp << std::endl ;
      std::cout << "Parameters for pixel to meter distortion model :" << std::endl ;
      std::cout << "  u0 = " << u0_pm <<"\t v0 = "<<v0_pm << std::endl ;
    }
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

