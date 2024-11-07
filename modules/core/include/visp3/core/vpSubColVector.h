/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Mask on a vpColVector.
 */

/*!
 * \file vpSubColVector.h
 *
 * \brief Definition of the vpSubColVector class
 */

#ifndef VP_SUBCOL_VECTOR_H
#define VP_SUBCOL_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpSubColVector
 * \ingroup group_core_matrices
 * This class provides a mask on a vpColVector. It has internally a
 * pointer to the parent vpColVector.
 * All properties of vpColVector are available with
 * a vpSubColVector.
 *
 * \sa vpMatrix vpColVector vpRowVector
*/
class VISP_EXPORT vpSubColVector : public vpColVector
{
public:
  vpSubColVector();
  vpSubColVector(vpColVector &v, const unsigned int &offset, const unsigned int &nrows);
  virtual ~vpSubColVector() VP_OVERRIDE;

  void checkParentStatus() const;

  void init(vpColVector &v, const unsigned int &offset, const unsigned int &nrows);

  vpSubColVector &operator=(const vpSubColVector &B);

  vpSubColVector &operator=(const vpPoseVector &p);
  vpSubColVector &operator=(const vpRotationVector &rv);
  vpSubColVector &operator=(const vpTranslationVector &tv);

  vpSubColVector &operator=(const vpColVector &B);
  vpSubColVector &operator=(const vpMatrix &B);
  vpSubColVector &operator=(const double &x);

protected:
  //! Number of row of parent vpColVector at initialization
  unsigned int m_pRowNum;
  //! Parent vpColVector
  vpColVector *m_parent;

private:
  //! Copy constructor unavailable
  vpSubColVector(const vpSubColVector &v /* m */);


};
END_VISP_NAMESPACE
#endif
