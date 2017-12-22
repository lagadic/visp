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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpProjectionDisplay_H
#define vpProjectionDisplay_H

/*!
  \file vpProjectionDisplay.h
  \brief interface with the image for feature display
*/

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpForwardProjection.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <list>

/*!
  \class vpProjectionDisplay
  \ingroup group_gui_projection
  \brief interface with the image for feature display
*/
class VISP_EXPORT vpProjectionDisplay
{
private:
  vpImage<unsigned char> Icam;
  vpImage<unsigned char> Iext;

#if defined VISP_HAVE_X11
  vpDisplayX dIcam;
  vpDisplayX dIext;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK dIcam;
  vpDisplayGTK dIext;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI dIcam;
  vpDisplayGDI dIext;
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV dIcam;
  vpDisplayOpenCV dIext;
#elif defined(VISP_HAVE_D3D9)
  vpDisplayD3D dIcam;
  vpDisplayD3D dIext;
#endif
public:
  void init();
  void init(int select);
  void close();
  static int internalView() { return 0x01; }
  static int externalView() { return 0x02; }

  /*! Default constructor. */
  vpProjectionDisplay()
    : Icam(), Iext(),
#if defined(VISP_HAVE_DISPLAY)
      dIcam(), dIext(),
#endif
      listFp(), o(), x(), y(), z(), traj()
  {
    init();
  }
  explicit vpProjectionDisplay(int select)
    : Icam(), Iext(),
#if defined(VISP_HAVE_DISPLAY)
      dIcam(), dIext(),
#endif
      listFp(), o(), x(), y(), z(), traj()
  {
    init(select);
  }

  void insert(vpForwardProjection &fp);
  void display(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cextMo, const vpHomogeneousMatrix &cMo,
               const vpCameraParameters &cam, const vpColor &color, const bool &displayTraj = false,
               const unsigned int thickness = 1);
  void displayCamera(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cextMo, const vpHomogeneousMatrix &cMo,
                     const vpCameraParameters &cam, const unsigned int thickness = 1);

private:
  std::list<vpForwardProjection *> listFp;
  vpPoint o;
  vpPoint x;
  vpPoint y;
  vpPoint z;
  vpMatrix traj;
};

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
