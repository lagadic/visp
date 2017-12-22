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
 * Descriptor for various invariants used to drive space roations around X and
 *Y axis.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentCInvariant.h
  \brief Descriptor for various invariants used to drive space roations around
  X and Y axis.
*/
#ifndef __MOMENTCINVARIANT_H__
#define __MOMENTCINVARIANT_H__

#include <visp3/core/vpMoment.h>
#include <visp3/core/vpMomentDatabase.h>

class vpMomentCentered;
class vpMomentBasic;

/*!
  \class vpMomentCInvariant

  \ingroup group_core_moments

  This class defines several 2D (translation+rotation+scale) invariants for
both symmetric and non-symmetric objects. These moment-based invariants are
described in the following papers \cite Chaumette04a, \cite Tahri05z.

  The descriptions for the invariants \f$C_1\f$ to \f$C_{10}\f$ can be found
in \cite Chaumette04a and for invariants
\f$P_x\f$,\f$P_y\f$,\f$S_x\f$,\f$S_y\f$ in \cite Tahri05z.

  These invariants are classicaly used in visual servoing to control the
out-of-plane rotations. The C-type or P-type invariants are used for
non-symmetric objects whereas the S-type invariants are used for symmetric
objects.

  For most cases of non-symmetric objects, (\f$C_4\f$,\f$C_6\f$) or
(\f$P_x\f$,\f$P_y\f$) couples are widely used to control x and y rotations.
  For symmetric objects \f$S_x\f$ and \f$S_y\f$ are the only choice.

  There are 14 translation+rotation+scale invariants (10 C-type, 2 P-type and
2 S-type) that can be accessed from by vpMomentCInvariant::get or any of the
get shortcuts.

  The example below shows how to retrieve the \f$C_2\f$ invariant:
  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCommon.h>
#include <iostream>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p;

  p.set_x(6); p.set_y(-1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(3); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(0); p.set_y(1.2); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(-7); p.set_y(-4); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);

  vpMomentObject obj(5); // Create an image moment object with 5 as maximum order
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p);

  //initialisation with default values
  vpMomentCommon db(vpMomentCommon::getSurface(obj),vpMomentCommon::getMu3(obj),
                    vpMomentCommon::getAlpha(obj),1.);
  bool success;

  db.updateAll(obj); // Update AND compute all moments

  //get C-invariant
  const vpMomentCInvariant& C
    = static_cast<const vpMomentCInvariant&>(db.get("vpMomentCInvariant",success));
  if(success)
      std::cout << C.get(1) << std:: endl; // print C2 invariant
  else
      std::cout << "vpMomentCInvariant not found." << std::endl;

  return 0;
}
\endcode

vpMomentCInvariant depends on vpMomentCentered (see vpMomentDatabase and
vpMomentCommon).
*/
class VISP_EXPORT vpMomentCInvariant : public vpMoment
{
private:
  std::vector<double> I;
  std::vector<double> II;
  std::vector<double> c;
  std::vector<double> s;
  double K;
  void computeI(const vpMomentCentered &momentCentered, std::vector<double> &I);

  /* To calculate Sx and Sy from normalized moments */
  void calcSxSy(double &sx, double &sy) const;
  void calcSxSyNormalized(double &sx, double &sy) const;
  std::vector<double> cn; // same as s above but calculated from normalized moments
  std::vector<double> sn; // same as c above but calculated from normalized moments
  double In1;             // same as I1 in Sx,Sy formulae but calculated from normalized
                          // moments
  bool flg_sxsynormalization_;

public:
  explicit vpMomentCInvariant(bool flg_sxsynormalization = false);
  virtual ~vpMomentCInvariant(){};

  /*!
    Shorcut for getting the value of \f$C_1\f$.
    */
  double C1() const { return values[0]; }
  /*!
    Shorcut for getting the value of \f$C_2\f$.
    */
  double C2() const { return values[1]; }
  /*!
    Shorcut for getting the value of \f$C_3\f$.
    */
  double C3() const { return values[2]; }
  /*!
    Shorcut for getting the value of \f$C_4\f$.
    */
  double C4() const { return values[3]; }
  /*!
    Shorcut for getting the value of \f$C_5\f$.
    */
  double C5() const { return values[4]; }
  /*!
    Shorcut for getting the value of \f$C_6\f$.
    */
  double C6() const { return values[5]; }
  /*!
    Shorcut for getting the value of \f$C_7\f$.
    */
  double C7() const { return values[6]; }
  /*!
    Shorcut for getting the value of \f$C_8\f$.
    */
  double C8() const { return values[7]; }
  /*!
    Shorcut for getting the value of \f$C_9\f$.
    */
  double C9() const { return values[8]; }
  /*!
    Shorcut for getting the value of \f$C_{10}\f$.
    */
  double C10() const { return values[9]; }

  void compute();

  /*!
    Gets the desired invariant.
    \param i given index. For invariants from C1 to C10 the corresponding
    index is from 0 to 9. For \f$S_x\f$,\f$S_y\f$ the indexes are 10,11 and
    for \f$P_x\f$,\f$P_y\f$ they are 12,13.
    */
  double get(unsigned int i) const { return values[i]; }

  /*!
    Access to partial invariant c (see [2]).
    */
  double getC(unsigned int i) const { return c[i]; }
  /*!
    Access to partial invariants. The index convention is the same as in [1].
    */
  double getI(unsigned int index) const { return I[index]; }

  /*!
    Print the moment invariants used to obtain the actual visual features
   */
  void printInvariants(std::ostream &os) const;

  /*!
    Access to partial invariant I (see [2]).
    */
  double getII(unsigned int i) const { return II[i]; }
  /*!
    Access to partial invariant K (see [2]).
    */
  double getK() const { return K; }

  /*!
    Access to partial invariant S (see [2]).
    */
  double getS(unsigned int i) const { return s[i]; }

  /*!
    Moment name.
    */
  const char *name() const { return "vpMomentCInvariant"; }

  /*!
    Print partial invariant.
    */
  void printI(unsigned int index);

  /*!
    Shorcut for getting the value of \f$P_x\f$.
    */
  double Px() { return values[12]; }
  /*!
    Shorcut for getting the value of\f$P_y\f$.
    */
  double Py() { return values[13]; }

  /*!
    Shorcut for getting the value of \f$S_x\f$.
    */
  double Sx() const { return values[10]; }
  /*!
    Shorcut for getting the value of \f$S_y\f$.
    */
  double Sy() const { return values[11]; }

  /*!
   * Getters for I
   * (calculated from normalized 2nd and 3ord order moments)
   */
  double getIn1() const { return In1; }

  /*!
   * Getter for c
   * (calculated from normalized 2nd and 3ord order moments)
   */
  double getCN(unsigned int i) const { return cn[i]; }

  /*!
   * Getter for s
   * (calculated from normalized 2nd and 3ord order moments)
   */
  double getSN(unsigned int i) const { return sn[i]; }

  /*!
   * To know if Sx and Sy were calculated from normalized moments or not
   */
  bool isSxSyfromNormalizedMoments() const { return flg_sxsynormalization_; };

  /*!
   *  To get all the invariant values as a whole.
   */
  inline const std::vector<double> &getMomentVector() const { return values; }

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentCInvariant &v);
};
#endif
