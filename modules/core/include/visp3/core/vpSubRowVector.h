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
 * Mask on a vpRowVector.
 */

/*!
 * \file vpSubRowVector.h
 *
 * \brief Definition of the vpSubRowVector class
 */

#ifndef VP_SUB_ROW_VECTOR_H
#define VP_SUB_ROW_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRowVector.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpSubRowVector
 * \ingroup group_core_matrices
 * This class provides a mask on a vpRowVector. It has internally a
 * pointer to the parent vpRowVector.
 * All properties of vpRowVector are available with
 * a vpSubRowVector.
 *
 * \sa vpMatrix vpColVector vpRowVector
*/
class VISP_EXPORT vpSubRowVector : public vpRowVector
{

public:
  vpSubRowVector();
  vpSubRowVector(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols);
  virtual ~vpSubRowVector() VP_OVERRIDE;

  void checkParentStatus() const;

  void init(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols);

  vpSubRowVector &operator=(const vpSubRowVector &B);
  vpSubRowVector &operator=(const vpRowVector &B);
  vpSubRowVector &operator=(const vpMatrix &B);
  vpSubRowVector &operator=(const double &x);

protected:
  //! Number of row of parent vpColVector at initialization
  unsigned int m_pColNum;
  //! Parent vpColVector
  vpRowVector *m_parent;

private:
  //! Copy constructor unavailable
  vpSubRowVector(const vpSubRowVector &m /* m */);
};
END_VISP_NAMESPACE
#endif
