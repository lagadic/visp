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
 * Description:
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerTriangle.h
 \brief
*/

#ifndef vpTemplateTrackerTriangle_hh
#define vpTemplateTrackerTriangle_hh

#include <assert.h>

#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpColVector.h>
#include <visp/vpTemplateTrackerHeader.h>

class VISP_EXPORT vpTemplateTrackerTriangle
{
  protected:
    double            minx_temp;
    double            miny_temp;
    vpTemplateTrackerDPoint    S1;
    vpTemplateTrackerDPoint    S2;
    vpTemplateTrackerDPoint    S3;

    bool              sommet_mileu_top;
    double            xp1;
    double            yp2;
    double            yp3;
    double            l_t;
    double            h_t;

    bool              pas_bon;
    double            uvinv00;
    double            uvinv01;
    double            uvinv10;
    double            uvinv11;
    double            marge_triangle;

  private:
    vpColVector getS1() const;
    vpColVector getS2() const;
    vpColVector getS3() const;
    
  public:
    vpTemplateTrackerTriangle();
    vpTemplateTrackerTriangle(const vpTemplateTrackerTriangle& T);
    vpTemplateTrackerTriangle(int x1,int y1, int x2,int y2, int x3,int y3);
    vpTemplateTrackerTriangle(double x1,double y1, double x2,double y2, double x3,double y3);
    
    double getMaxx() const;
    double getMaxy() const;
    double getMinx() const;
    double getMiny() const;
    
    vpTemplateTrackerTriangle getPyramidDown() const;
    void getCorners(vpColVector &rS1,vpColVector &rS2,vpColVector &rS3) const;
    
    /*!
       \param i : Allowed values are 0,1 or 2.
       \return
       - if i = 0, return S1
       - if i = 1, return S2
       - if i = 2, return S3
     */
    vpColVector getCorner(unsigned int i) const {
      assert(i<3);
      if(i==0) return getS1();
      else if(i==1) return getS2();
      else /*if(i==2)*/ return getS3();
    };

    void getSize(double *lt,double *ht) const;
    void getSize(int *lt,int *ht) const;
    
    void init(const vpColVector &S1,const vpColVector &S2,const vpColVector &S3);
    void init(int x1,int y1, int x2,int y2, int x3,int y3);
    void init(double x1,double y1, double x2,double y2, double x3,double y3);
    bool inTriangle(const int &i, const int &j) const;
    bool inTriangle(const double &i,const double &j) const;

    vpTemplateTrackerTriangle & operator=(const vpTemplateTrackerTriangle& T);
    
    void setMinZero();
};
#endif

