
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpExponentialMap.cpp,v 1.2 2006-02-22 09:51:39 fspindle Exp $
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#include <visp/vpExponentialMap.h>

#define DEBUG_LEVEL1 0

/*!

  Compute the exponential map. The inverse function is inverse().

  \param v : Instantaneous velocity represented by a 6 dimension vector
  [t,tu]^T where tu is a rotation vector (theta u representation see
  vpThetaUVector) and t is a translation vector: [Tx, Ty, Tz, Tux, Tuy, Tuz]

  \return An homogeneous matrix M computed from an instantaneous velocity v. If
  v is expressed in frame c, M = cMc_new

  \sa inverse()
*/
vpHomogeneousMatrix
vpExponentialMap::direct(const vpColVector &v)
{
  double theta,si,co,sinc,mcosc,msinc;
  vpThetaUVector u ;
  vpRotationMatrix rd ;
  vpTranslationVector dt ;

  u[0] = v[3];
  u[1] = v[4];
  u[2] = v[5];
  rd.buildFrom(u);

  theta = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si,theta);
  mcosc = vpMath::mcosc(co,theta);
  msinc = vpMath::msinc(si,theta);

  dt[0] = v[0]*(sinc + u[0]*u[0]*msinc)
        + v[1]*(u[0]*u[1]*msinc - u[2]*mcosc)
        + v[2]*(u[0]*u[2]*msinc + u[1]*mcosc);

  dt[1] = v[0]*(u[0]*u[1]*msinc + u[2]*mcosc)
        + v[1]*(sinc + u[1]*u[1]*msinc)
        + v[2]*(u[1]*u[2]*msinc - u[0]*mcosc);

  dt[2] = v[0]*(u[0]*u[2]*msinc - u[1]*mcosc)
        + v[1]*(u[1]*u[2]*msinc + u[0]*mcosc)
        + v[2]*(sinc + u[2]*u[2]*msinc);

  vpHomogeneousMatrix Delta ;
  Delta.insert(rd) ;
  Delta.insert(dt) ;



  if (DEBUG_LEVEL1)  // test new version wrt old version
  {
    // old version
    int i,j;

    double sinu,cosi,mcosi,s;
    // double u[3];
    //  vpRotationMatrix rd ;
    //  vpTranslationVector dt ;

    s = sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
    if (s > 1.e-15)
    {
      for (i=0;i<3;i++) u[i] = v[3+i]/s;
      sinu = sin(s);
      cosi = cos(s);
      mcosi = 1-cosi;
      rd[0][0] = cosi + mcosi*u[0]*u[0];
      rd[0][1] = -sinu*u[2] + mcosi*u[0]*u[1];
      rd[0][2] = sinu*u[1] + mcosi*u[0]*u[2];
      rd[1][0] = sinu*u[2] + mcosi*u[1]*u[0];
      rd[1][1] = cosi + mcosi*u[1]*u[1];
      rd[1][2] = -sinu*u[0] + mcosi*u[1]*u[2];
      rd[2][0] = -sinu*u[1] + mcosi*u[2]*u[0];
      rd[2][1] = sinu*u[0] + mcosi*u[2]*u[1];
      rd[2][2] = cosi + mcosi*u[2]*u[2];

      dt[0] = v[0]*(sinu/s + u[0]*u[0]*(1-sinu/s))
            + v[1]*(u[0]*u[1]*(1-sinu/s)-u[2]*mcosi/s)
            + v[2]*(u[0]*u[2]*(1-sinu/s)+u[1]*mcosi/s);

      dt[1] = v[0]*(u[0]*u[1]*(1-sinu/s)+u[2]*mcosi/s)
            + v[1]*(sinu/s + u[1]*u[1]*(1-sinu/s))
            + v[2]*(u[1]*u[2]*(1-sinu/s)-u[0]*mcosi/s);

      dt[2] = v[0]*(u[0]*u[2]*(1-sinu/s)-u[1]*mcosi/s)
            + v[1]*(u[1]*u[2]*(1-sinu/s)+u[0]*mcosi/s)
            + v[2]*(sinu/s + u[2]*u[2]*(1-sinu/s));
    }
    else
    {
      for (i=0;i<3;i++)
      {
        for(j=0;j<3;j++) rd[i][j] = 0.0;
        rd[i][i] = 1.0;
        dt[i] = v[i];
      }
    }
    // end old version

    // Test of the new version
    vpHomogeneousMatrix Delta_old ;
    Delta_old.insert(rd) ;
    Delta_old.insert(dt) ;

    int pb = 0;
    for (i=0;i<4;i++)
    {
      for(j=0;j<4;j++)
        if (fabs(Delta[i][j] - Delta_old[i][j]) > 1.e-5) pb = 1;
    }
    if (pb == 1)
    {
      printf("pb vpHomogeneousMatrix::expMap\n");
      cout << " Delta : " << endl << Delta << endl;
      cout << " Delta_old : " << endl << Delta_old << endl;
    }
    // end of the test
  }


  return Delta ;
}

/*!

  Compute an instantaneous velocity from an homogeneous matrix. The inverse
  function is the exponential map, see direct().

  \param M : An homogeneous matrix corresponding to a displacement.

  \return v : Instantaneous velocity represented by a 6 dimension vector
  [t,tu]^T where tu is a rotation vector (theta u representation see
  vpThetaUVector) and t is a translation vector: [Tx, Ty, Tz, Tux, Tuy, Tuz]

  \sa direct()
*/
vpColVector
vpExponentialMap::inverse(const vpHomogeneousMatrix &M)
{
  vpColVector v(6);
  int i;
  double theta,si,co,sinc,mcosc,msinc,det;
  vpThetaUVector u ;
  vpRotationMatrix Rd,a;
  vpTranslationVector dt ;

  M.extract(Rd);
  u.buildFrom(Rd);
  for (i=0;i<3;i++) v[3+i] = u[i];

  theta = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  si = sin(theta);
  co = cos(theta);
  sinc  = vpMath::sinc(si,theta);
  mcosc = vpMath::mcosc(co,theta);
  msinc = vpMath::msinc(si,theta);

  // a below is not a pure rotation matrix, even if not so far from
  // the Rodrigues formula : sinc I + (1-sinc)/t^2 VV^T + (1-cos)/t^2 [V]_X
  // with V = t.U

  a[0][0] = sinc + u[0]*u[0]*msinc;
  a[0][1] = u[0]*u[1]*msinc - u[2]*mcosc;
  a[0][2] = u[0]*u[2]*msinc + u[1]*mcosc;

  a[1][0] = u[0]*u[1]*msinc + u[2]*mcosc;
  a[1][1] = sinc + u[1]*u[1]*msinc;
  a[1][2] = u[1]*u[2]*msinc - u[0]*mcosc;

  a[2][0] = u[0]*u[2]*msinc - u[1]*mcosc;
  a[2][1] = u[1]*u[2]*msinc + u[0]*mcosc;
  a[2][2] = sinc + u[2]*u[2]*msinc;

  det = a[0][0]*a[1][1]*a[2][2] + a[1][0]*a[2][1]*a[0][2]
      + a[0][1]*a[1][2]*a[2][0] - a[2][0]*a[1][1]*a[0][2]
      - a[1][0]*a[0][1]*a[2][2] - a[0][0]*a[2][1]*a[1][2];

  if (fabs(det) > 1.e-5)
  {
     v[0] =  (M[0][3]*a[1][1]*a[2][2]
           +   M[1][3]*a[2][1]*a[0][2]
           +   M[2][3]*a[0][1]*a[1][2]
           -   M[2][3]*a[1][1]*a[0][2]
           -   M[1][3]*a[0][1]*a[2][2]
           -   M[0][3]*a[2][1]*a[1][2])/det;
     v[1] =  (a[0][0]*M[1][3]*a[2][2]
           +   a[1][0]*M[2][3]*a[0][2]
           +   M[0][3]*a[1][2]*a[2][0]
           -   a[2][0]*M[1][3]*a[0][2]
           -   a[1][0]*M[0][3]*a[2][2]
           -   a[0][0]*M[2][3]*a[1][2])/det;
     v[2] =  (a[0][0]*a[1][1]*M[2][3]
           +   a[1][0]*a[2][1]*M[0][3]
           +   a[0][1]*M[1][3]*a[2][0]
           -   a[2][0]*a[1][1]*M[0][3]
           -   a[1][0]*a[0][1]*M[2][3]
           -   a[0][0]*a[2][1]*M[1][3])/det;
  }
  else
  {
     v[0] = M[0][3];
     v[1] = M[1][3];
     v[2] = M[2][3];
  }
  return(v);

}


#undef DEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
