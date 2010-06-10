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
 * Defines a 2D triangle.
 *
 * Author:
 * Amaury Dame
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpTriangle_h
#define vpTriangle_h

/*!
  \class vpTriangle
  \brief Defines a 2D triangle.
  
  A triangle is internally represented by three 2D points.
  
  By default the three coordinates in the \f$ (i,j) \f$ frame (see vpImagePoint class documentation for more details about the frame.) are \f$ (0,0) \f$, \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/

#include <visp/vpConfig.h>
#include <visp/vpImagePoint.h>
#include <visp/vpMatrix.h>

class VISP_EXPORT vpTriangle
{
  private:
    bool goodTriange;
    vpImagePoint S1;
    double uvinv00;
    double uvinv01;
    double uvinv10;
    double uvinv11;
    double ptempo0;
    double ptempo1;
    vpImagePoint apex1;
    vpImagePoint apex2;
    vpImagePoint apex3;
    
  public:
    vpTriangle();
    vpTriangle(const vpTriangle &tri);
    virtual ~vpTriangle();
    
    vpTriangle &operator=(const vpTriangle& tri);
    
    vpTriangle(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);
    
    void buildFrom(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);
    
    bool inTriangle(const vpImagePoint &iP, double threshold = 0.00001);
    
    /*!
      Get the apexes of the triangle.
      
      \param iP1 : first apex.
      \param iP2 : second apex.
      \param iP3 : third apex.
    */
    void getTriangleApexes(vpImagePoint &iP1, vpImagePoint &iP2, vpImagePoint &iP3) const {
      iP1 = apex1;
      iP2 = apex2;
      iP3 = apex3;}

  private:
    void init(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3);
    
};

#endif