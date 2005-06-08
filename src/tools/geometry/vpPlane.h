
/*                                                                -*-c++-*-
    Copyright (C) 1998  IRISA-INRIA Rennes Vista Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

    Auteur :
      Eric Marchand

    Creation : 1 octobre 1998
    Revision : 30 avril 2002 - Init modified (Andrew Comport)

*/


#ifndef vpPlane_hh
#define vpPlane_hh




/*!
  \class vpPlane

  \brief  definition of the vpPlane class member functions

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  This class defines the container for a plane geometrical structure

  A plane is given by the equation
  ax + by + cz + d = 0
  where
  (x,y,z) is a point of R^3

*/

#include <visp/vpColVector.h>
#include <visp/vpPoint.h>


class vpPlane
{

public:
  double A,B,C,D ;


public:
  vpPlane(const double a, const double b,const  double c,const  double d) ;
  vpPlane() ;
  vpPlane(vpPlane& P) ;
  vpPlane(vpPoint& P,vpColVector &n) ;
  vpPlane(vpPoint &P, vpPoint &Q, vpPoint &R) ;
  void init(vpPoint& P,   vpPoint& Q,  vpPoint& R) ;
  void init(vpColVector& P,vpColVector &n) ;
  void init(vpPlane& P) ;
  // SET information
public:

  void setA(const double _a) {   A = _a ; }
  void setB(const double _b) {   B = _b ; }
  void setC(const double _c) {   C = _c ; }
  void setD(const double _d) {   D = _d ; }


  vpPlane& operator =(const vpPlane& f) ;

  // GET information

  double getA() const { return A ; }
  double getB() const { return B ; }
  double getC() const { return C ; }
  double getD() const { return D ; }

  vpColVector abcd();

  vpColVector getNormal() const;
  void getNormal(vpColVector &n) const;

public: // Display, Print Member function
  friend ostream& operator<< (ostream& os, vpPlane& p)
  {
    return (os  << "("<<p.getA() << ","<<p.getB()<< ","<<p.getC()<< ","<<p.getD() <<") ") ;
  } ;
  void Print(ostream& os) { os << *this << endl ; }


public: // Operation with  Plane

  void projectionPointOnPlan(const vpPoint& P, vpPoint& p2) ;

  double rayIntersection(const vpPoint &M0,
			 const vpPoint &M1,
			 vpColVector &H )const ;

  double getIntersection(const vpColVector &M1,vpColVector &H )const ;


} ;



#endif
