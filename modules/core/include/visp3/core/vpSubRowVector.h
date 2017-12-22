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
 * Mask on a vpRowVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#ifndef __VP_SUB_ROW_VECTOR__
#define __VP_SUB_ROW_VECTOR__

#include <visp3/core/vpRowVector.h>

/*!
  \file vpSubRowVector.h

  \brief Definition of the vpSubRowVector class
*/

/*!
  \class vpSubRowVector
  \ingroup group_core_matrices
  This class provides a mask on a vpRowVector. It has internally a
  pointer to the parent vpRowVector.
  All properties of vpRowVector are available with
  a vpSubRowVector.

  \author Jean Laneurit (IRISA - INRIA Rennes)

  \sa vpMatrix vpColvector vpRowVector
*/

class VISP_EXPORT vpSubRowVector : public vpRowVector
{

private:
  //! Copy constructor unavaible
  vpSubRowVector(const vpSubRowVector & /* m */);

protected:
  //! Number of row of parent vpColvector at initialization
  unsigned int pColNum;
  //! Parent vpColvector
  vpRowVector *parent;

public:
  vpSubRowVector();
  vpSubRowVector(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols);
  virtual ~vpSubRowVector();

  void checkParentStatus() const;

  void init(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols);

  vpSubRowVector &operator=(const vpSubRowVector &B);
  vpSubRowVector &operator=(const vpRowVector &B);
  vpSubRowVector &operator=(const vpMatrix &B);
  vpSubRowVector &operator=(const double &x);
};

#endif
