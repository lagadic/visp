
#include <visp/vpCalibration.h>
#include <visp/vpMath.h>
#include <visp/vpPose.h>

#undef MAX
#undef MIN

void
vpCalibration::calibLagrange(
			     vpCameraParameters &cam, vpHomogeneousMatrix &cMo)
{

  vpMatrix A(2*npt,3) ;
  vpMatrix B(2*npt,9) ;


  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  for (unsigned int i = 0 ; i < npt ; i++)
  {

    double x0 = LoX.value() ;
    double y0 = LoY.value() ;
    double z0 = LoZ.value() ;


    double xi = Lu.value()  ;
    double yi = Lv.value()  ;


    A[2*i][0] = x0*xi;
    A[2*i][1] = y0*xi;
    A[2*i][2] = z0*xi;
    B[2*i][0] = -x0;
    B[2*i][1] = -y0;
    B[2*i][2] = -z0;
    B[2*i][3] = 0.0;
    B[2*i][4] = 0.0;
    B[2*i][5] = 0.0;
    B[2*i][6] = -1.0;
    B[2*i][7] = 0.0;
    B[2*i][8] = xi;
    A[2*i+1][0] = x0*yi;
    A[2*i+1][1] = y0*yi;
    A[2*i+1][2] = z0*yi;
    B[2*i+1][0] = 0.0;
    B[2*i+1][1] = 0.0;
    B[2*i+1][2] = 0.0;
    B[2*i+1][3] = -x0;
    B[2*i+1][4] = -y0;
    B[2*i+1][5] = -z0;
    B[2*i+1][6] = 0.0;
    B[2*i+1][7] = -1.0;
    B[2*i+1][8] = yi;

    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }


  vpMatrix BtB ;              /* compute B^T B	*/
  BtB = B.t() * B ;

  /* compute (B^T B)^(-1)					*/
  /* input : btb  	(dimension 9 x 9) = (B^T B) 		*/
  /* output : btbinv 	(dimension 9 x 9) = (B^T B)^(-1) 	*/

  vpMatrix BtBinv ;
  BtBinv = BtB.pseudoInverse(1e-16) ;

  vpMatrix BtA ;
  BtA = B.t()*A ;       /* compute B^T A	*/


  vpMatrix r ;
  r = BtBinv*BtA ;  /* compute (B^T B)^(-1) B^T A */

  vpMatrix e  ;      /* compute - A^T B (B^T B)^(-1) B^T A*/
  e = -(A.t()*B)*r ;

  vpMatrix AtA ;     /* compute A^T A	*/
  AtA = A.AtA() ;

  e += AtA ;  /* compute E = A^T A - A^T B (B^T B)^(-1) B^T A */

  vpColVector x1(3) ;
  vpColVector x2 ;

  e.svd(x1,AtA) ;// destructive on e
  // eigenvector computation of E corresponding to the min eigenvalue.
  /* SVmax	computation*/
  double  svm = 0.0;
  int imin = 1;
  for (int i=0;i<x1.getRows();i++)
  {
    if (x1[i] > svm) { svm = x1[i]; imin = i; }
  }

  svm *= 0.1;	/* for the rank	*/

  for (int i=0;i<x1.getRows();i++)
  {
    if (x1[i] < x1[imin]) imin = i;
  }

  for (int i=0;i<x1.getRows();i++)
    x1[i] = AtA[i][imin];

  x2 = - (r*x1) ; // X_2 = - (B^T B)^(-1) B^T A X_1


  vpColVector sol(12) ;
  vpColVector resul(7) ;
  for (int i=0;i<3;i++) sol[i] = x1[i];	/* X_1	*/
  for (int i=0;i<9;i++) 			/* X_2 = - (B^T B)^(-1) B^T A X_1 */
  {
    sol[i+3] = x2[i];
  }

  if (sol[11] < 0.0) for (int i=0;i<12;i++) sol[i] = -sol[i];  /* since Z0 > 0 */

  resul[0] = sol[3]*sol[0]+sol[4]*sol[1]+sol[5]*sol[2];		/* u0 */

  resul[1] = sol[6]*sol[0]+sol[7]*sol[1]+sol[8]*sol[2];		/* v0 */

  resul[2] = sqrt(sol[3]*sol[3]+sol[4]*sol[4]+sol[5]*sol[5]	/* px */
		  -resul[0]*resul[0]);
  resul[3] = sqrt(sol[6]*sol[6]+sol[7]*sol[7]+sol[8]*sol[8]	/* py	*/
		  -resul[1]*resul[1]);
  cam.setPrincipalPoint(resul[0],resul[1]) ;
  cam.setPixelRatio(resul[2],resul[3]) ;

  resul[4] = (sol[9]-sol[11]*resul[0])/resul[2];	/* X0	*/
  resul[5] = (sol[10]-sol[11]*resul[1])/resul[3];	/* Y0	*/
  resul[6] = sol[11];					/* Z0	*/

  vpMatrix rd(3,3) ;
  /* fill rotation matrix */
  for (int i=0;i<3;i++) rd[0][i] = (sol[i+3]-sol[i]*resul[0])/resul[2];
  for (int i=0;i<3;i++) rd[1][i] = (sol[i+6]-sol[i]*resul[1])/resul[3];
  for (int i=0;i<3;i++) rd[2][i] = sol[i];

  //  std::cout << "norme X1 " << x1.sumSquare() <<std::endl;
  //  std::cout << rd*rd.t() ;

  for (int i=0 ; i < 3 ; i++)
  {
    for (int j=0 ; j < 3 ; j++)
      cMo[i][j] = rd[i][j];
  }
  for (int i=0 ; i < 3 ; i++) cMo[i][3] = resul[i+4] ;

  this->cMo = cMo ;
  this->cMo_mp = cMo;
  this->cMo_pm = cMo;

  double deviation,deviationPM,deviationMP;
  this->computeStdDeviation(deviation,deviationPM,deviationMP);

}


void
vpCalibration::calibVVS(
			vpCameraParameters &cam,
			vpHomogeneousMatrix &cMo,
			bool verbose)
{
  std::cout.precision(10);
  int   n_points = npt ;

  vpColVector oX(n_points), cX(n_points)  ;
  vpColVector oY(n_points), cY(n_points) ;
  vpColVector oZ(n_points), cZ(n_points) ;
  vpColVector u(n_points) ;
  vpColVector v(n_points) ;

  vpColVector P(2*n_points) ;
  vpColVector Pd(2*n_points) ;


  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  for (int i =0 ; i < n_points ; i++)
  {

    oX[i]  = LoX.value() ;
    oY[i]  = LoY.value() ;
    oZ[i]  = LoZ.value() ;


    u[i] = Lu.value()  ;
    v[i] = Lv.value()  ;


    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }

  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,vpCalibration::threshold) == false && iter < vpCalibration::nbIterMax)
  {

    iter++ ;
    residu_1 = r ;


    r = 0 ;

    for (int i=0 ; i < n_points; i++)
    {
      cX[i] = oX[i]*cMo[0][0]+oY[i]*cMo[0][1]+oZ[i]*cMo[0][2] + cMo[0][3];
      cY[i] = oX[i]*cMo[1][0]+oY[i]*cMo[1][1]+oZ[i]*cMo[1][2] + cMo[1][3];
      cZ[i] = oX[i]*cMo[2][0]+oY[i]*cMo[2][1]+oZ[i]*cMo[2][2] + cMo[2][3];

      Pd[2*i] =   u[i] ;
      Pd[2*i+1] = v[i] ;

      P[2*i] =    cX[i]/cZ[i]*cam.get_px() + cam.get_u0() ;
      P[2*i+1] =  cY[i]/cZ[i]*cam.get_py() + cam.get_v0() ;

      r += ((vpMath::sqr(P[2*i]-Pd[2*i]) + vpMath::sqr(P[2*i+1]-Pd[2*i+1]))) ;
    }


    vpColVector error ;
    error = P-Pd ;
    //r = r/n_points ;

    vpMatrix L(n_points*2,10) ;
    for (int i=0 ; i < n_points; i++)
    {

      double x = cX[i] ;
      double y = cY[i] ;
      double z = cZ[i] ;

      double px = cam.get_px() ;
      double py = cam.get_py() ;

      double X =   x/z ;
      double Y =   y/z ;

      //---------------

      {
	L[2*i][0] =  px * (-1/z) ;
	L[2*i][1] =  0 ;
	L[2*i][2] =  px*(X/z) ;
	L[2*i][3] =  px*X*Y ;
	L[2*i][4] =  -px*(1+X*X) ;
	L[2*i][5] =  px*Y ;
      }
      {
	L[2*i][6]= 1 ;
	L[2*i][7]= 0 ;
	L[2*i][8]= x/z ;
	L[2*i][9]= 0;
      }
      {
	L[2*i+1][0] = 0 ;
	L[2*i+1][1] = py*(-1/z) ;
	L[2*i+1][2] = py*(Y/z) ;
	L[2*i+1][3] = py* (1+Y*Y) ;
	L[2*i+1][4] = -py*X*Y ;
	L[2*i+1][5] = -py*X ;
      }
      {
	L[2*i+1][6]= 0 ;
	L[2*i+1][7]= 1 ;
	L[2*i+1][8]= 0;
	L[2*i+1][9]= y/z ;
      }


    }    // end interaction
    vpMatrix Lp ;
    Lp = L.pseudoInverse(1e-15) ;

    vpColVector e ;
    e = Lp*error ;

    vpColVector Tc, Tc_v(6) ;
    Tc = -e*0.1 ;

    //   Tc_v =0 ;
    for (int i=0 ; i <6 ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint(cam.get_u0()+Tc[6],cam.get_v0()+Tc[7]) ;
    cam.setPixelRatio(cam.get_px()+Tc[8],cam.get_py()+Tc[9]) ;

    //    cam.setKd(get_kd() + Tc[10]) ;

    cMo = vpExponentialMap::direct(Tc_v).inverse()*cMo ;
    if(verbose)
      std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  this->cMo   = cMo;
  this->cMo_mp = cMo;
  this->cMo_pm = cMo;
  this->residual = r;
  this->residual_mp = r;
  this->residual_pm = r;
  if(verbose)
    std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

}

void
vpCalibration::calibVVSMulti(
			     unsigned int nbPose,
			     vpCalibration table_cal[],
			     vpCameraParameters &cam,
			     bool verbose
			     )
{
  std::cout.precision(10);
  int nbPoint[255]; //number of points by image
  int nbPointTotal = 0; //total number of points

  for(unsigned int i=0; i<nbPose ; i++){
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal)  ;
  vpColVector oY(nbPointTotal), cY(nbPointTotal) ;
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal) ;
  vpColVector u(nbPointTotal) ;
  vpColVector v(nbPointTotal) ;

  vpColVector P(2*nbPointTotal) ;
  vpColVector Pd(2*nbPointTotal) ;

  int curPoint = 0 ; //current point indice
  for(unsigned int p=0; p<nbPose ; p++){
    table_cal[p].LoX.front() ;
    table_cal[p].LoY.front() ;
    table_cal[p].LoZ.front() ;
    table_cal[p].Lu.front()  ;
    table_cal[p].Lv.front()  ;

    for (int i =0 ; i < nbPoint[p] ; i++)
    {

      oX[curPoint]  = table_cal[p].LoX.value() ;
      oY[curPoint]  = table_cal[p].LoY.value() ;
      oZ[curPoint]  = table_cal[p].LoZ.value() ;


      u[curPoint] = table_cal[p].Lu.value()  ;
      v[curPoint] = table_cal[p].Lv.value()  ;


      table_cal[p].LoX.next() ;
      table_cal[p].LoY.next() ;
      table_cal[p].LoZ.next() ;
      table_cal[p].Lu.next() ;
      table_cal[p].Lv.next() ;
      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax)
  {

    iter++ ;
    residu_1 = r ;


    r = 0 ;
    curPoint = 0 ; //current point indice
    for(unsigned int p=0; p<nbPose ; p++){
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo;
      for (int i=0 ; i < nbPoint[p]; i++)
      {
        cX[curPoint] = oX[curPoint]*cMoTmp[0][0]+oY[curPoint]*cMoTmp[0][1]
	  +oZ[curPoint]*cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] = oX[curPoint]*cMoTmp[1][0]+oY[curPoint]*cMoTmp[1][1]
	  +oZ[curPoint]*cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] = oX[curPoint]*cMoTmp[2][0]+oY[curPoint]*cMoTmp[2][1]
	  +oZ[curPoint]*cMoTmp[2][2] + cMoTmp[2][3];

        Pd[2*curPoint] =   u[curPoint] ;
        Pd[2*curPoint+1] = v[curPoint] ;

        P[2*curPoint] =    cX[curPoint]/cZ[curPoint]*cam.get_px() + cam.get_u0() ;
        P[2*curPoint+1] =  cY[curPoint]/cZ[curPoint]*cam.get_py() + cam.get_v0() ;

        r += ((vpMath::sqr(P[2*curPoint]-Pd[2*curPoint])
	       + vpMath::sqr(P[2*curPoint+1]-Pd[2*curPoint+1]))) ;
        curPoint++;
      }
    }

    vpColVector error ;
    error = P-Pd ;
    //r = r/nbPointTotal ;

    vpMatrix L(nbPointTotal*2,6*nbPose+4) ;
    curPoint = 0 ; //current point indice
    for(unsigned int p=0; p<nbPose ; p++){
      for (int i=0 ; i < nbPoint[p]; i++)
      {

        double x = cX[curPoint] ;
        double y = cY[curPoint] ;
        double z = cZ[curPoint] ;

        double px = cam.get_px() ;
        double py = cam.get_py() ;

        double X =   x/z ;
        double Y =   y/z ;

        //---------------
        {
          {
            L[2*curPoint][6*p] =  px * (-1/z) ;
            L[2*curPoint][6*p+1] =  0 ;
            L[2*curPoint][6*p+2] =  px*(X/z) ;
            L[2*curPoint][6*p+3] =  px*X*Y ;
            L[2*curPoint][6*p+4] =  -px*(1+X*X) ;
            L[2*curPoint][6*p+5] =  px*Y ;
          }
          {
            L[2*curPoint][6*nbPose]= 1 ;
            L[2*curPoint][6*nbPose+1]= 0 ;
            L[2*curPoint][6*nbPose+2]= X ;
            L[2*curPoint][6*nbPose+3]= 0;
          }
          {
            L[2*curPoint+1][6*p+0] = 0 ;
            L[2*curPoint+1][6*p+1] = py*(-1/z) ;
            L[2*curPoint+1][6*p+2] = py*(Y/z) ;
            L[2*curPoint+1][6*p+3] = py* (1+Y*Y) ;
            L[2*curPoint+1][6*p+4] = -py*X*Y ;
            L[2*curPoint+1][6*p+5] = -py*X ;
          }
          {
            L[2*curPoint+1][6*nbPose]= 0 ;
            L[2*curPoint+1][6*nbPose+1]= 1 ;
            L[2*curPoint+1][6*nbPose+2]= 0;
            L[2*curPoint+1][6*nbPose+3]= Y ;
          }

        }
        curPoint++;
      }    // end interaction
    }
    vpMatrix Lp ;
    Lp = L.pseudoInverse(1e-15) ;

    vpColVector e ;
    e = Lp*error ;

    vpColVector Tc, Tc_v(6*nbPose) ;
    Tc = -e*0.1 ;

    //   Tc_v =0 ;
    for (unsigned int i = 0 ; i < 6*nbPose ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint(cam.get_u0()+Tc[6*nbPose],cam.get_v0()+Tc[6*nbPose+1]) ;
    cam.setPixelRatio(cam.get_px()+Tc[6*nbPose+2],cam.get_py()+Tc[6*nbPose+3]) ;

    //    cam.setKd(get_kd() + Tc[10]) ;
    vpColVector Tc_v_Tmp(6) ;

    for(unsigned int p = 0 ; p < nbPose ; p++){
      for (unsigned int i = 0 ; i < 6 ; i++)
        Tc_v_Tmp[i] = Tc_v[6*p + i];

      table_cal[p].cMo = vpExponentialMap::direct(Tc_v_Tmp,1).inverse()
	* table_cal[p].cMo;
    }
    if(verbose)
      std::cout <<  " std dev " << sqrt(r/nbPointTotal) << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  for(unsigned int p = 0 ; p < nbPose ; p++){
    table_cal[p].cMo_pm = table_cal[p].cMo ;
    table_cal[p].cMo_mp = table_cal[p].cMo ;
    table_cal[p].cam.init(cam.get_px(),cam.get_py(),cam.get_u0(),cam.get_v0());
    double deviation,deviation_pm,deviation_mp;
    table_cal[p].computeStdDeviation(deviation,deviation_pm,deviation_mp);
  }
  if(verbose)
    std::cout <<  " std dev " << sqrt(r/nbPointTotal) << std::endl;
}

void
vpCalibration::calibVVSWithDistortion_pm(
					 vpCameraParameters& cam,
					 vpHomogeneousMatrix& cMo,
					 bool verbose)
{
  std::cout.precision(10);
  unsigned int n_points =npt ;

  vpColVector oX(n_points), cX(n_points)  ;
  vpColVector oY(n_points), cY(n_points) ;
  vpColVector oZ(n_points), cZ(n_points) ;
  vpColVector u(n_points) ;
  vpColVector v(n_points) ;

  vpColVector P(2*n_points) ;
  vpColVector Pd(2*n_points) ;


  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  for (unsigned int i =0 ; i < n_points ; i++)
  {

    oX[i]  = LoX.value() ;
    oY[i]  = LoY.value() ;
    oZ[i]  = LoZ.value() ;


    u[i] = Lu.value()  ;
    v[i] = Lv.value()  ;


    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }

  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,threshold)  == false && iter < nbIterMax)
  {

    iter++ ;
    residu_1 = r ;


    r = 0 ;

    vpMatrix L(n_points*2,11) ;
    for (unsigned int i=0 ; i < n_points; i++)
    {
      cX[i] = oX[i]*cMo[0][0]+oY[i]*cMo[0][1]+oZ[i]*cMo[0][2] + cMo[0][3];
      cY[i] = oX[i]*cMo[1][0]+oY[i]*cMo[1][1]+oZ[i]*cMo[1][2] + cMo[1][3];
      cZ[i] = oX[i]*cMo[2][0]+oY[i]*cMo[2][1]+oZ[i]*cMo[2][2] + cMo[2][3];

      double x = cX[i] ;
      double y = cY[i] ;
      double z = cZ[i] ;

      Pd[2*i] =   u[i] ;
      Pd[2*i+1] = v[i] ;

      double kd = cam.get_kd_pm() ;

      double u0 = cam.get_u0_pm() ;
      double v0 = cam.get_v0_pm() ;

      double px = cam.get_px_pm() ;
      double py = cam.get_py_pm() ;

      double up = u[i] ;
      double vp = v[i] ;

      double X =   x/z ;
      double Y =   y/z ;

      double r2 = (vpMath::sqr((up-u0)/px)+vpMath::sqr((vp-v0)/py)) ;

      P[2*i] =   u0 + px*X - kd *(up-u0)*r2 ;
      P[2*i+1] = v0 + py*Y - kd *(vp-v0)*r2 ;

      r += ((vpMath::sqr(P[2*i]-Pd[2*i]) + vpMath::sqr(P[2*i+1]-Pd[2*i+1]))) ;

      //---------------
      {
	{
	  L[2*i][0] =  px * (-1/z) ;
	  L[2*i][1] =  0 ;
	  L[2*i][2] =  px*(X/z) ;
	  L[2*i][3] =  px*X*Y ;
	  L[2*i][4] =  -px*(1+X*X) ;
	  L[2*i][5] =  px*Y ;
	}
	{
	  L[2*i][6]= 1 + kd*r2 + 2*kd*vpMath::sqr((up-u0)/px)  ;
	  L[2*i][7]= 2*kd*(up-u0)*(vp-v0)/vpMath::sqr(py) ;
	  L[2*i][8]= X + 2*kd*(up -u0)*vpMath::sqr(up-u0)/(px*px*px) ;
	  L[2*i][9]= 2*kd*(up-u0)*vpMath::sqr(vp-v0)/(py*py*py) ;
	  L[2*i][10] = -(up-u0)*(r2) ;
	}
	{
	  L[2*i+1][0] = 0 ;
	  L[2*i+1][1] = py*(-1/z) ;
	  L[2*i+1][2] = py*(Y/z) ;
	  L[2*i+1][3] = py* (1+Y*Y) ;
	  L[2*i+1][4] = -py*X*Y ;
	  L[2*i+1][5] = -py*X ;
	}
	{
	  L[2*i+1][6]= 2*kd*(up-u0)*(vp-v0)/vpMath::sqr(px) ;
	  L[2*i+1][7]= 1 + kd*r2 + 2*kd*vpMath::sqr((vp-v0)/py);
	  L[2*i+1][8]= 2*kd*(vp-v0)*vpMath::sqr(up-u0)/(px*px*px);
	  L[2*i+1][9]= Y + 2*kd*(vp -v0)*vpMath::sqr(vp-v0)/(py*py*py);
	  L[2*i+1][10] = -(vp-v0)*r2 ;
	}

      }
    }    // end interaction

    vpColVector error ;
    error = P-Pd ;
    //r = r/n_points ;

    if (r > residu_1) break ;
    vpMatrix Lp ;
    Lp = L.pseudoInverse(1e-15) ;

    vpColVector e ;
    e = Lp*error ;

    vpColVector Tc, Tc_v(6) ;
    Tc = -e*0.1 ;

    for (int i=0 ; i <6 ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint_pm(cam.get_u0_pm() + Tc[6],cam.get_v0_pm() + Tc[7]) ;
    cam.setPixelRatio_pm(cam.get_px_pm() + Tc[8], cam.get_py_pm() + Tc[9]) ;
    cam.setKd_pm(cam.get_kd_pm() + Tc[10]) ;

    cMo = vpExponentialMap::direct(Tc_v).inverse()*cMo ;
    if(verbose)
      std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  this->residual_pm = r;
  this->cMo_pm = cMo ;
  this->cam.init_pm(cam.get_px_pm(),cam.get_py_pm(),cam.get_u0_pm(),cam.get_v0_pm(),cam.get_kd_pm());

  if(verbose)
    std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

}
void
vpCalibration::calibVVSWithDistortion_mp(
					 vpCameraParameters& cam,
					 vpHomogeneousMatrix& cMo,
					 bool verbose)
{
  std::cout.precision(10);
  unsigned int n_points =npt ;

  vpColVector oX(n_points), cX(n_points)  ;
  vpColVector oY(n_points), cY(n_points) ;
  vpColVector oZ(n_points), cZ(n_points) ;
  vpColVector u(n_points) ;
  vpColVector v(n_points) ;

  vpColVector P(2*n_points) ;
  vpColVector Pd(2*n_points) ;


  LoX.front() ;
  LoY.front() ;
  LoZ.front() ;
  Lu.front() ;
  Lv.front() ;

  for (unsigned int i =0 ; i < n_points ; i++)
  {

    oX[i]  = LoX.value() ;
    oY[i]  = LoY.value() ;
    oZ[i]  = LoZ.value() ;


    u[i] = Lu.value()  ;
    v[i] = Lv.value()  ;


    LoX.next() ;
    LoY.next() ;
    LoZ.next() ;
    Lu.next() ;
    Lv.next() ;
  }

  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,threshold)  == false && iter < nbIterMax)
  {

    iter++ ;
    residu_1 = r ;


    r = 0 ;

    vpMatrix L(n_points*2,11) ;
    for (unsigned int i=0 ; i < n_points; i++)
    {
      cX[i] = oX[i]*cMo[0][0]+oY[i]*cMo[0][1]+oZ[i]*cMo[0][2] + cMo[0][3];
      cY[i] = oX[i]*cMo[1][0]+oY[i]*cMo[1][1]+oZ[i]*cMo[1][2] + cMo[1][3];
      cZ[i] = oX[i]*cMo[2][0]+oY[i]*cMo[2][1]+oZ[i]*cMo[2][2] + cMo[2][3];

      double x = cX[i] ;
      double y = cY[i] ;
      double z = cZ[i] ;

      Pd[2*i] =   u[i] ;
      Pd[2*i+1] = v[i] ;

      double kd = cam.get_kd_mp() ;

      double u0 = cam.get_u0_mp() ;
      double v0 = cam.get_v0_mp() ;

      double px = cam.get_px_mp() ;
      double py = cam.get_py_mp() ;

      double X =   x/z ;
      double Y =   y/z ;

      double r2 = (vpMath::sqr(X)+vpMath::sqr(Y)) ;

      P[2*i] =   u0 + px*X*(1+ kd*r2) ;
      P[2*i+1] = v0 + py*Y*(1+ kd*r2) ;

      r += ((vpMath::sqr(P[2*i]-Pd[2*i]) + vpMath::sqr(P[2*i+1]-Pd[2*i+1]))) ;

      //---------------
      {
        {
          L[2*i][0] = px*(-1/z) ;
          L[2*i][1] = 0 ;
          L[2*i][2] = px*(X/z) ;
          L[2*i][3] = px*X*Y ;
          L[2*i][4] = -px*(1+X*X) ;
          L[2*i][5] = px*Y ;
        }
        {
          L[2*i][6]= 1 ;
          L[2*i][7]= 0 ;
          L[2*i][8]= X*(1+kd*r2) ;
          L[2*i][9]= 0;
          L[2*i][10] = px*X*r2 ;
        }
        {
          L[2*i+1][0] = 0 ;
          L[2*i+1][1] = py*(-1/z) ;
          L[2*i+1][2] = py*(Y/z) ;
          L[2*i+1][3] = py*(1+Y*Y) ;
          L[2*i+1][4] = -py*X*Y ;
          L[2*i+1][5] = -py*X ;
        }
        {
          L[2*i+1][6]= 0 ;
          L[2*i+1][7]= 1;
          L[2*i+1][8]= 0;
          L[2*i+1][9]= Y*(1+kd*r2) ;
          L[2*i+1][10] = py*Y*r2 ;
        }
      }  // end interaction
    }
    vpColVector error ;
    error = P-Pd ;
    //r = r/n_points ;

    if (r > residu_1) break ;
    vpMatrix Lp ;
    Lp = L.pseudoInverse(1e-15) ;

    vpColVector e ;
    e = Lp*error ;

    vpColVector Tc, Tc_v(6) ;
    Tc = -e*0.1 ;

    for (int i=0 ; i <6 ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint_mp(cam.get_u0_mp() + Tc[6],cam.get_v0_mp() + Tc[7]) ;
    cam.setPixelRatio_mp(cam.get_px_mp() + Tc[8], cam.get_py_mp() + Tc[9]) ;
    cam.setKd_mp(cam.get_kd_mp() + Tc[10]) ;

    cMo = vpExponentialMap::direct(Tc_v).inverse()*cMo ;
    if(verbose)
      std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  this->residual_mp = r;
  this->cMo_mp = cMo ;
  this->cam.init_mp(cam.get_px_mp(),cam.get_py_mp(),cam.get_u0_mp(),
		    cam.get_v0_mp(),cam.get_kd_mp());

  if(verbose)
    std::cout <<  " std dev " << sqrt(r/n_points) << std::endl;

}

void
vpCalibration::calibVVSWithDistortionMulti_pm(
					      unsigned int nbPose,
					      vpCalibration table_cal[],
					      vpCameraParameters &cam,
					      bool verbose)
{
  std::cout.precision(10);
  unsigned int nbPoint[255]; //number of points by image
  unsigned int nbPointTotal = 0; //total number of points

  for(unsigned int i=0; i<nbPose ; i++){
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal)  ;
  vpColVector oY(nbPointTotal), cY(nbPointTotal) ;
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal) ;
  vpColVector u(nbPointTotal) ;
  vpColVector v(nbPointTotal) ;

  vpColVector P(2*nbPointTotal) ;
  vpColVector Pd(2*nbPointTotal) ;

  int curPoint = 0 ; //current point indice
  for(unsigned int p=0; p<nbPose ; p++){
    table_cal[p].LoX.front() ;
    table_cal[p].LoY.front() ;
    table_cal[p].LoZ.front() ;
    table_cal[p].Lu.front()  ;
    table_cal[p].Lv.front()  ;

    for (unsigned int i =0 ; i < nbPoint[p] ; i++)
    {

      oX[curPoint]  = table_cal[p].LoX.value() ;
      oY[curPoint]  = table_cal[p].LoY.value() ;
      oZ[curPoint]  = table_cal[p].LoZ.value() ;


      u[curPoint] = table_cal[p].Lu.value()  ;
      v[curPoint] = table_cal[p].Lv.value()  ;


      table_cal[p].LoX.next() ;
      table_cal[p].LoY.next() ;
      table_cal[p].LoZ.next() ;
      table_cal[p].Lu.next() ;
      table_cal[p].Lv.next() ;
      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax){
    iter++ ;
    residu_1 = r ;


    r = 0 ;
    curPoint = 0 ; //current point indice
    for(unsigned int p=0; p<nbPose ; p++){
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo_pm;
      for (unsigned int i=0 ; i < nbPoint[p]; i++)
      {
        cX[curPoint] = oX[curPoint]*cMoTmp[0][0]+oY[curPoint]*cMoTmp[0][1]
	  +oZ[curPoint]*cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] = oX[curPoint]*cMoTmp[1][0]+oY[curPoint]*cMoTmp[1][1]
	  +oZ[curPoint]*cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] = oX[curPoint]*cMoTmp[2][0]+oY[curPoint]*cMoTmp[2][1]
	  +oZ[curPoint]*cMoTmp[2][2] + cMoTmp[2][3];

        curPoint++;
      }
    }


    vpMatrix L(nbPointTotal*2,6*nbPose+5) ;
    curPoint = 0 ; //current point indice
    double kd = cam.get_kd_pm() ;

    double px = cam.get_px_pm() ;
    double py = cam.get_py_pm() ;
    double u0 = cam.get_u0_pm() ;
    double v0 = cam.get_v0_pm() ;

    for(unsigned int p=0; p<nbPose ; p++){
      for (unsigned int i=0 ; i < nbPoint[p]; i++)
      {

        double x = cX[curPoint] ;
        double y = cY[curPoint] ;
        double z = cZ[curPoint] ;

        double X =   x/z ;
        double Y =   y/z ;

        double up = u[curPoint] ;
        double vp = v[curPoint] ;

        double r2 = (vpMath::sqr((up-u0)/px)+vpMath::sqr((vp-v0)/py)) ;

        Pd[2*curPoint] =   u[curPoint] ;
        Pd[2*curPoint+1] = v[curPoint] ;


        P[2*curPoint] =   u0 + px*X - kd *(up-u0)*r2 ;
        P[2*curPoint+1] = v0 + py*Y - kd *(vp-v0)*r2 ;

        r += ((vpMath::sqr(P[2*curPoint]-Pd[2*curPoint]) +
	       vpMath::sqr(P[2*curPoint+1]-Pd[2*curPoint+1]))) ;


        //---------------
        {
          {
            L[2*curPoint][6*p] =  px * (-1/z) ;
            L[2*curPoint][6*p+1] =  0 ;
            L[2*curPoint][6*p+2] =  px*(X/z) ;
            L[2*curPoint][6*p+3] =  px*X*Y ;
            L[2*curPoint][6*p+4] =  -px*(1+X*X) ;
            L[2*curPoint][6*p+5] =  px*Y ;
          }

          {
            L[2*curPoint][6*nbPose]= 1 + kd*r2 + 2*kd*vpMath::sqr((up-u0)/px)  ;
            L[2*curPoint][6*nbPose+1]= 2*kd*(up-u0)*(vp-v0)/vpMath::sqr(py) ;
            L[2*curPoint][6*nbPose+2]= X + 2*kd*(up -u0)*vpMath::sqr(up-u0)/(px*px*px) ;
            L[2*curPoint][6*nbPose+3]= 2*kd*(up-u0)*vpMath::sqr(vp-v0)/(py*py*py) ;
            L[2*curPoint][6*nbPose+4] = -(up-u0)*(r2) ;
          }
          {
            L[2*curPoint+1][6*p] = 0 ;
            L[2*curPoint+1][6*p+1] = py*(-1/z) ;
            L[2*curPoint+1][6*p+2] = py*(Y/z) ;
            L[2*curPoint+1][6*p+3] = py* (1+Y*Y) ;
            L[2*curPoint+1][6*p+4] = -py*X*Y ;
            L[2*curPoint+1][6*p+5] = -py*X ;
          }
          {
            L[2*curPoint+1][6*nbPose]= 2*kd*(up-u0)*(vp-v0)/vpMath::sqr(px) ;
            L[2*curPoint+1][6*nbPose+1]= 1 + kd*r2 + 2*kd*vpMath::sqr((vp-v0)/py);
            L[2*curPoint+1][6*nbPose+2]= 2*kd*(vp-v0)*vpMath::sqr(up-u0)/(px*px*px);
            L[2*curPoint+1][6*nbPose+3]= Y + 2*kd*(vp -v0)*vpMath::sqr(vp-v0)/(py*py*py);
            L[2*curPoint+1][6*nbPose+4] = -(vp-v0)*r2 ;
          }

        }
        curPoint++;
      }    // end interaction
    }

    vpColVector error ;
    error = P-Pd ;
    //r = r/nbPointTotal ;

    vpMatrix Lp ;
    /*double rank =*/ L.pseudoInverse(Lp,1e-10) ;
    vpColVector e ;
    e = Lp*error ;
    vpColVector Tc, Tc_v(6*nbPose) ;
    Tc = -0.1*e ;
    for (unsigned int i = 0 ; i < 6*nbPose ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint_pm(u0+Tc[6*nbPose],v0+Tc[6*nbPose+1]) ;
    cam.setPixelRatio_pm(px+Tc[6*nbPose+2],py+Tc[6*nbPose+3]) ;

    cam.setKd_pm(kd + Tc[6*nbPose+4]) ;

    vpColVector Tc_v_Tmp(6) ;
    for(unsigned int p = 0 ; p < nbPose ; p++){
      for (unsigned int i = 0 ; i < 6 ; i++)
        Tc_v_Tmp[i] = Tc_v[6*p + i];

      table_cal[p].cMo_pm = vpExponentialMap::direct(Tc_v_Tmp).inverse()
	* table_cal[p].cMo_pm;
    }
    if(verbose)
      std::cout <<  " std dev: " << sqrt(r/nbPointTotal) << std::endl;
    //std::cout <<  "   residual: " << r << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  for(unsigned int p = 0 ; p < nbPose ; p++){
    table_cal[p].cam = cam ;
    table_cal[p].computeStdDeviation_pm(table_cal[p].cMo_pm,table_cal[p].cam);
  }
  if(verbose)
    std::cout <<" std dev " << sqrt(r/nbPointTotal) << std::endl;

}

void
vpCalibration::calibVVSWithDistortionMulti_mp(
					      unsigned int nbPose,
					      vpCalibration table_cal[],
					      vpCameraParameters &cam,
					      bool verbose)
{
  std::cout.precision(10);
  unsigned int nbPoint[255]; //number of points by image
  unsigned int nbPointTotal = 0; //total number of points

  for(unsigned int p=0; p<nbPose ; p++){
    nbPoint[p] = table_cal[p].npt;
    nbPointTotal += nbPoint[p];
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal)  ;
  vpColVector oY(nbPointTotal), cY(nbPointTotal) ;
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal) ;
  vpColVector u(nbPointTotal) ;
  vpColVector v(nbPointTotal) ;

  vpColVector P(2*nbPointTotal) ;
  vpColVector Pd(2*nbPointTotal) ;

  int curPoint = 0 ; //current point indice
  for(unsigned int p=0; p<nbPose ; p++) {
    table_cal[p].LoX.front() ;
    table_cal[p].LoY.front() ;
    table_cal[p].LoZ.front() ;
    table_cal[p].Lu.front()  ;
    table_cal[p].Lv.front()  ;

    for (unsigned int i =0 ; i < nbPoint[p] ; i++) {

      oX[curPoint]  = table_cal[p].LoX.value() ;
      oY[curPoint]  = table_cal[p].LoY.value() ;
      oZ[curPoint]  = table_cal[p].LoZ.value() ;


      u[curPoint] = table_cal[p].Lu.value()  ;
      v[curPoint] = table_cal[p].Lv.value()  ;


      table_cal[p].LoX.next() ;
      table_cal[p].LoY.next() ;
      table_cal[p].LoZ.next() ;
      table_cal[p].Lu.next() ;
      table_cal[p].Lv.next() ;
      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0 ;

  double  residu_1 = 1e12 ;
  double r =1e12-1;
  while(vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax){
    iter++ ;
    residu_1 = r ;


    r = 0 ;
    curPoint = 0 ; //current point indice
    for(unsigned int p=0; p<nbPose ; p++) {
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo_mp;
      for (unsigned int i=0 ; i < nbPoint[p]; i++) {
        cX[curPoint] = oX[curPoint]*cMoTmp[0][0]+oY[curPoint]*cMoTmp[0][1]
	  +oZ[curPoint]*cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] = oX[curPoint]*cMoTmp[1][0]+oY[curPoint]*cMoTmp[1][1]
	  +oZ[curPoint]*cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] = oX[curPoint]*cMoTmp[2][0]+oY[curPoint]*cMoTmp[2][1]
	  +oZ[curPoint]*cMoTmp[2][2] + cMoTmp[2][3];

        curPoint++;
      }
    }


    vpMatrix L(nbPointTotal*2,6*nbPose+5) ;
    curPoint = 0 ; //current point indice
    double kd = cam.get_kd_mp() ;

    double px = cam.get_px_mp() ;
    double py = cam.get_py_mp() ;
    double u0 = cam.get_u0_mp() ;
    double v0 = cam.get_v0_mp() ;

    for(unsigned int p=0; p<nbPose ; p++) {
      for (unsigned int i=0 ; i < nbPoint[p]; i++) {

        double x = cX[curPoint] ;
        double y = cY[curPoint] ;
        double z = cZ[curPoint] ;

        double up = u[curPoint] ;
        double vp = v[curPoint] ;

        double X =   x/z ;
        double Y =   y/z ;
        double r2 = (vpMath::sqr(X)+vpMath::sqr(Y)) ;
        double Axx = px*(1 + kd*r2+2*kd*X*X);
        double Axy = px*2*kd*X*Y;
        double Ayy = py*(1 + kd*r2+2*kd*Y*Y);
        double Ayx = py*2*kd*X*Y;

        Pd[2*curPoint] =   up ;
        Pd[2*curPoint+1] = vp ;


        P[2*curPoint] =   u0 + px*X*(1 + kd*r2) ;
        P[2*curPoint+1] = v0 + py*Y*(1 + kd*r2) ;

        r += vpMath::sqr(P[2*curPoint]-Pd[2*curPoint]) +
	  vpMath::sqr(P[2*curPoint+1]-Pd[2*curPoint+1]) ;

        //---------------
        {
          {
            L[2*curPoint][6*p] = Axx*(-1/z) ;
            L[2*curPoint][6*p+1] = Axy*(-1/z) ;
            L[2*curPoint][6*p+2] = Axx*(X/z) + Axy*(Y/z) ;
            L[2*curPoint][6*p+3] = Axx*X*Y +  Axy*(1+Y*Y);
            L[2*curPoint][6*p+4] = -Axx*(1+X*X) - Axy*X*Y;
            L[2*curPoint][6*p+5] = Axx*Y -Axy*X;
          }
          {
            L[2*curPoint][6*nbPose]= 1 ;
            L[2*curPoint][6*nbPose+1]= 0 ;
            L[2*curPoint][6*nbPose+2]= X*(1+kd*r2) ;
            L[2*curPoint][6*nbPose+3]= 0;
            L[2*curPoint][6*nbPose+4] = px*X*r2 ;
          }
          {
            L[2*curPoint+1][6*p] = Ayx*(-1/z) ;
            L[2*curPoint+1][6*p+1] = Ayy*(-1/z) ;
            L[2*curPoint+1][6*p+2] = Ayx*(X/z) + Ayy*(Y/z) ;
            L[2*curPoint+1][6*p+3] = Ayx*X*Y + Ayy*(1+Y*Y) ;
            L[2*curPoint+1][6*p+4] = -Ayx*(1+X*X) -Ayy*X*Y ;
            L[2*curPoint+1][6*p+5] = Ayx*Y -Ayy*X;
          }
          {
            L[2*curPoint+1][6*nbPose]= 0 ;
            L[2*curPoint+1][6*nbPose+1]= 1;
            L[2*curPoint+1][6*nbPose+2]= 0;
            L[2*curPoint+1][6*nbPose+3]= Y*(1+kd*r2) ;
            L[2*curPoint+1][6*nbPose+4] = py*Y*r2 ;
          }
        }
        curPoint++;
      }    // fin interaction
    }

    vpColVector error ;
    error = P-Pd ;
    //r = r/nbPointTotal ;

    vpMatrix Lp ;
    /*double rank =*/ L.pseudoInverse(Lp,1e-10) ;
    vpColVector e ;
    e = Lp*error ;
    vpColVector Tc, Tc_v(6*nbPose) ;
    Tc = -0.1*e ;
    //    std::cout<<"T = "<<Tc[24]<<std::endl;

    for (unsigned int i = 0 ; i < 6*nbPose ; i++)
      Tc_v[i] = Tc[i] ;

    cam.setPrincipalPoint_mp(u0+Tc[6*nbPose],v0+Tc[6*nbPose+1]) ;
    cam.setPixelRatio_mp(px+Tc[6*nbPose+2],py+Tc[6*nbPose+3]) ;

    cam.setKd_mp(kd + Tc[6*nbPose+4]) ;

    vpColVector Tc_v_Tmp(6) ;
    for(unsigned int p = 0 ; p < nbPose ; p++){
      for (unsigned int i = 0 ; i < 6 ; i++)
        Tc_v_Tmp[i] = Tc_v[6*p + i];
      table_cal[p].cMo_mp = vpExponentialMap::direct(Tc_v_Tmp).inverse()
	* table_cal[p].cMo_mp;
    }
    if(verbose)
      std::cout <<  " std dev: " << sqrt(r/nbPointTotal) << std::endl;
    //    std::cout <<  " residual: " << r << std::endl;
    //    std::cout <<  " delta residual: " << residu_1 - r << std::endl;

  }
  if(iter == nbIterMax){
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError,
				 "Maximum number of iterations reached")) ;
  }
  for(unsigned int p = 0 ; p < nbPose ; p++){
    table_cal[p].cam = cam ;
    table_cal[p].computeStdDeviation_mp(table_cal[p].cMo_mp,table_cal[p].cam);
  }

  if(verbose)
    std::cout <<" std dev " << sqrt(r/nbPointTotal) << std::endl;

}

/*!
  \brief calibration method of effector-camera from R. Tsai and R. Lorenz

  R. Tsai, R. Lenz. -- A new technique for fully autonomous and efficient 3D
  robotics hand/eye calibration. -- IEEE Transactions on Robotics and
  Automation, 5(3):345--358, June 1989.

  \param nbPose : number of different positions (input)
  \param cMo : table of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param wMe : table of homogeneous matrices representing the transformation
  between the effector and the world coordinates (base of the
  manipulator) (input)
  \param eMc : homogeneous matrix representing the transformation
  between the effector and the camera (output)
*/
void
vpCalibration::calibrationTsai(unsigned int nbPose,
			       vpHomogeneousMatrix cMo[],
			       vpHomogeneousMatrix wMe[],
			       vpHomogeneousMatrix &eMc)
{
  vpColVector x ;
  {
    vpMatrix A ;
    vpColVector B ;
    unsigned int k = 0 ;
    // for all couples ij
    for (unsigned int i=0 ; i < nbPose ; i++)
    {
      vpRotationMatrix wRei, ciRo ;
      wMe[i].extract(wRei) ;
      cMo[i].extract(ciRo) ;

      for (unsigned int j=0 ; j < nbPose ; j++)
      {
        if (j>i) // we don't use two times same couples...
        {
          vpRotationMatrix wRej, cjRo ;
          wMe[j].extract(wRej) ;
          cMo[j].extract(cjRo) ;

          vpRotationMatrix wReij = wRej.t() * wRei ;
          vpRotationMatrix cijRo = cjRo * ciRo.t() ;

          vpThetaUVector wPeij(wReij);
          double theta = sqrt(wPeij[0]*wPeij[0] + wPeij[1]*wPeij[1]
                              + wPeij[2]*wPeij[2]);
          for(int m=0;m<3;m++) wPeij[m] = wPeij[m] * vpMath::sinc(theta/2);

          vpThetaUVector cijPo(cijRo) ;
          theta = sqrt(cijPo[0]*cijPo[0] + cijPo[1]*cijPo[1]
		       + cijPo[2]*cijPo[2]);
          for(int m=0;m<3;m++) cijPo[m] = cijPo[m] * vpMath::sinc(theta/2);

          vpMatrix As;
          vpColVector b(3) ;

          As = vpColVector::skew(vpColVector(wPeij) + vpColVector(cijPo)) ;

          b =  (vpColVector)cijPo - (vpColVector)wPeij ;           // A.40

          if (k==0)
          {
            A = As ;
            B = b ;
          }
          else
          {
            A = vpMatrix::stackMatrices(A,As) ;
            B = vpMatrix::stackMatrices(B,b) ;
          }
          k++ ;
        }
      }
    }

    // the linear system is defined
    // x = AtA^-1AtB is solved
    vpMatrix AtA = A.AtA() ;


    vpMatrix Ap ;
    AtA.pseudoInverse(Ap, 1e-6) ; // rank 3
    x = Ap*A.t()*B ;

    // extraction of theta and U
    double theta ;
    double   d=x.sumSquare() ;
    for (int i=0 ; i < 3 ; i++) x[i] = 2*x[i]/sqrt(1+d) ;
    theta = sqrt(x.sumSquare())/2 ;
    theta = 2*asin(theta) ;
    if (theta !=0)
    {
      for (int i=0 ; i < 3 ; i++) x[i] *= theta/(2*sin(theta/2)) ;
    }
    else
      x = 0 ;
    // Building of the rotation matrix eRc
  }
  vpThetaUVector xP(x[0],x[1],x[2]);
  vpRotationMatrix eRc(xP);
  {
    vpMatrix A ;
    vpMatrix B ;
    // Building of the system for the translation estimation
    // for all couples ij
    vpRotationMatrix I3 ;
    I3.setIdentity() ;
    int k = 0 ;
    for (unsigned int i=0 ; i < nbPose ; i++)
    {
      vpRotationMatrix wRei, ciRo ;
      vpTranslationVector wTei, ciTo ;
      wMe[i].extract(wRei) ;
      cMo[i].extract(ciRo) ;
      wMe[i].extract(wTei) ;
      cMo[i].extract(ciTo) ;


      for (unsigned int j=0 ; j < nbPose ; j++)
      {
        if (j>i) // we don't use two times same couples...
        {

          vpRotationMatrix wRej, cjRo ;
          wMe[j].extract(wRej) ;
          cMo[j].extract(cjRo) ;

          vpTranslationVector wTej, cjTo ;
          wMe[j].extract(wTej) ;
          cMo[j].extract(cjTo) ;

          vpRotationMatrix wReij = wRej.t() * wRei ;

          vpTranslationVector wTeij = wTej+ (-wTei);

          wTeij = wRej.t()*wTeij ;

          vpTranslationVector cijTo = cjTo + (-(cjRo*ciRo.t()*ciTo)) ;

          vpMatrix a ;
          a = (vpMatrix)wReij - (vpMatrix)I3 ;

          vpTranslationVector b ;
          b =  eRc*cijTo + wTeij ;

          if (k==0)
          {
            A = a ;
            B = b ;
          }
          else
          {
            A = vpMatrix::stackMatrices(A,a) ;
            B = vpMatrix::stackMatrices(B,(vpColVector)b) ;
          }
          k++ ;
        }
      }
    }

    // the linear system is solved
    // x = AtA^-1AtB is solved
    vpMatrix AtA = A.AtA() ;
    vpMatrix Ap ;
    vpColVector AeTc ;
    AtA.pseudoInverse(Ap, 1e-6) ;
    AeTc = Ap*A.t()*B ;
    vpTranslationVector eTc(AeTc[0],AeTc[1],AeTc[2]);

    eMc.insert(eTc) ;
    eMc.insert(eRc) ;
  }
}
