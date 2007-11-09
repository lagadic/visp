/****************************************************************************
 *
 * $Id: vpRA.h,v 1.5 2007-11-09 13:35:11 asaunier Exp $
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
 * Use to display an image behind the internal view of the simulator
 * used for augmented reality application
 *
 * Authors:
 * Eric Marchand (march 2007)
 *
 *****************************************************************************/

#ifndef vpRA_HH
#define vpRA_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_COIN

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

#include <visp/vpSimulator.h>

#include <visp/vpViewer.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpTime.h>

class VISP_EXPORT vpRA : public vpSimulator
{

private:

  bool background;

 public:


  virtual ~vpRA() ;
  void initInternalViewer(int width, int height, vpImageType type = grayImage) ;
  void setImage(vpImage<unsigned char> &I) ;
  void setImage(vpImage<vpRGBa> &I) ;

} ;


#endif
#endif
