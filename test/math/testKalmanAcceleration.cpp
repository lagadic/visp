/****************************************************************************
 *
 * $Id: testTwistMatrix.cpp,v 1.5 2008-06-17 08:08:29 asaunier Exp $
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
 * This file is part of the ViSP toolkit
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
 * Tests some vpKalman functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testKalmanAcceleration.cpp

  \brief Test some vpKalman functionalities with constant acceleration
  state model.
*/
#include <iostream>
#include <fstream>

#include <visp/vpKalmanFilter.h>

typedef struct {
		double ro;
		double Vk[6];
		double Sk[6];
		double Wk[6][3];
		double Qk[6][2];
		double Rk[6];
		double Pest[6][6];
		double Ppre[6][6];
		double wpre[6];
	} st_filtre;	/* struct for a complete implemented KM filter */


void filtc_reinit(double tabfiltc[4], st_filtre *fcp, double Tvo)
{

  short i,j;
  /* init. a zero de tous les elements du filtre */
  for(i=0;i<6;i++) {
    fcp->Vk[i] = 0.0;
    fcp->Sk[i] = 0.0;
    fcp->wpre[i] = 0.0;
    for(j=0;j<3;j++)	fcp->Wk[i][j] = 0.0;
  }

  fcp->ro = tabfiltc[3];  /* coefficient de correlation */

  /**********************************************/
  /* init. des matrices de covariance du filtre */
  /**********************************************/

  /* bruit etat sur vitesse */
  for (i=0;i<3;i++)	fcp->Qk[i][0] = tabfiltc[0];
  for (i=3;i<6;i++)	fcp->Qk[i][0] = tabfiltc[0]*M_PI/180.0*M_PI/180.0;

  /* bruit etat sur acc. */
  for (i=0;i<6;i++)	fcp->Qk[i][1] = tabfiltc[1];//fcp->Qk[i][0]/(Tvo*Tvo);

  /* bruit mesure vitesse */
  fcp->Rk[0] = tabfiltc[2];
  fcp->Rk[1] = 1.0*0.01*0.01;
  fcp->Rk[2] = 0.04*0.01*0.01;
  fcp->Rk[3] = 4.0*(M_PI/180.0)*(M_PI/180.0);
  fcp->Rk[4] = 4.0*(M_PI/180.0)*(M_PI/180.0);
  fcp->Rk[5] = 0.065*(M_PI/180.0)*(M_PI/180.0);

  /* covariance modele d'etat */
  for (i=0;i<6;i++) {
    fcp->Pest[i][0] = fcp->Rk[i];
    fcp->Pest[i][1] = 0.0;
    fcp->Pest[i][2] = fcp->Qk[i][0]/(1-(fcp->ro*fcp->ro));
    fcp->Pest[i][3] = fcp->Rk[i]/Tvo;
    fcp->Pest[i][4] = -(fcp->ro)*fcp->Qk[i][0]/((1-(fcp->ro*fcp->ro))*Tvo);
    fcp->Pest[i][5] = ((2.0*fcp->Rk[i]+(fcp->Qk[i][0]/(1-(fcp->ro*fcp->ro))))
		       /(Tvo*Tvo))+fcp->Qk[i][1];
  }
}

/*************************************************************************/
/*       saisie et mise a jour des parametres du filtre                  */
/*************************************************************************/
void par_filtc_init(st_filtre *fcp, double Tvo)

{

  printf("passe par la: \n");
  double tabfiltc[4];

  // orig
  tabfiltc[0] = 0.0000001; // bruit etat vitesse
  tabfiltc[1] = 0.0000001/(Tvo*Tvo); // bruit etat acc
  tabfiltc[2] = 0.3;
  tabfiltc[2] = 0.3;

/*   printf("tabfiltc[0]: %f\n", tabfiltc[0]); */
/*   scanf("%f", &tabfiltc[0]); */
/*   printf("tabfiltc[2]: %f\n", tabfiltc[2]); */
/*   scanf("%f", &tabfiltc[2]); */
  // tentative de modif le 22/11/2007
  tabfiltc[0] = 0.7;
  tabfiltc[2] = 0.01;

  filtc_reinit(tabfiltc,fcp, Tvo);
}


/*************************************************************/
/* filtre de KALMAN augmente - vit cte - bruit etat + mesure */
/*************************************************************/
void kalmc_a_V(int naxe, int iter, double X_mes[6], st_filtre *fcp,
	       double X_est[6], double X_pre[6])

{

  double w_est[6];

  switch (iter)
  {

  case 0 :
    fcp->wpre[naxe] = 0.0;
    X_est[naxe] = X_mes[naxe];
    X_pre[naxe] = X_mes[naxe];
    fcp->Ppre[naxe][0] = fcp->Pest[naxe][0]+2.0*fcp->Pest[naxe][1]
      +fcp->Pest[naxe][2];
    fcp->Ppre[naxe][1] = fcp->ro*(fcp->Pest[naxe][1]+fcp->Pest[naxe][2]);
    fcp->Ppre[naxe][2] = fcp->ro*fcp->ro*fcp->Pest[naxe][2]+fcp->Qk[naxe][0];

    break;

  default :
    fcp->Vk[naxe] = X_mes[naxe]-X_pre[naxe];

    fcp->Sk[naxe] = fcp->Ppre[naxe][0]+fcp->Rk[naxe];
    if (fcp->Sk[naxe] == 0.0)
      fcp->Sk[naxe] = 0.00000000001;
    fcp->Wk[naxe][0] = fcp->Ppre[naxe][0]/fcp->Sk[naxe];
    fcp->Wk[naxe][1] = fcp->Ppre[naxe][1]/fcp->Sk[naxe];

    X_est[naxe] = X_pre[naxe]+(fcp->Wk[naxe][0]*fcp->Vk[naxe]);
    w_est[naxe] = fcp->wpre[naxe]+(fcp->Wk[naxe][1]*fcp->Vk[naxe]);
    fcp->Pest[naxe][0] = (1.0-fcp->Wk[naxe][0])*fcp->Ppre[naxe][0];
    fcp->Pest[naxe][1] = (1.0-fcp->Wk[naxe][0])*fcp->Ppre[naxe][1];
    fcp->Pest[naxe][2] = fcp->Ppre[naxe][2]
      -fcp->Wk[naxe][1]*fcp->Ppre[naxe][1];

    X_pre[naxe] = X_est[naxe]+w_est[naxe];
    fcp->wpre[naxe] = fcp->ro*w_est[naxe];
    fcp->Ppre[naxe][0] = fcp->Pest[naxe][0]+2.0*fcp->Pest[naxe][1]
      +fcp->Pest[naxe][2];
    fcp->Ppre[naxe][1] = fcp->ro*(fcp->Pest[naxe][1]+fcp->Pest[naxe][2]);
    fcp->Ppre[naxe][2] = fcp->ro*fcp->ro*fcp->Pest[naxe][2]+fcp->Qk[naxe][0];

    break;
  }
}

void kalmc_a_A(int naxe, int iter, double Tv, double [], double X_mes[],
	       st_filtre *fcp, double X_est[], double X_pre[], double Xp_pre[])
{

  double w_est[6],Xp_est[6];

  switch (iter)
  {

//   case 0 :
//     X_est[naxe] = X_mes[naxe];
//     X_pre[naxe] = X_mes[naxe];
//     Xp_pre[naxe] = 0.0;
//     std::cout << "\nPpre old: " ;
//     for(int i=0; i < 6; i++)
//       std::cout << fcp->Ppre[naxe][i] << " ";
//     std::cout << std::endl;

//     break;

  case 0 :
    fcp->wpre[naxe] = 0.0;
    X_est[naxe] = X_mes[naxe];
    X_pre[naxe] = X_mes[naxe];
    Xp_pre[naxe] = 0.0;
    fcp->Ppre[naxe][0] = fcp->Pest[naxe][0]+2.0*fcp->Pest[naxe][1]
      +fcp->Pest[naxe][2]+2.0*Tv*(fcp->Pest[naxe][3]+fcp->Pest[naxe][4])
      +Tv*Tv*fcp->Pest[naxe][5];
    fcp->Ppre[naxe][1] = fcp->ro*(fcp->Pest[naxe][1]+fcp->Pest[naxe][2]
				  +Tv*fcp->Pest[naxe][4]);
    fcp->Ppre[naxe][2] = fcp->ro*fcp->ro*fcp->Pest[naxe][2]+fcp->Qk[naxe][0];
    fcp->Ppre[naxe][3] = fcp->Pest[naxe][3]+fcp->Pest[naxe][4]
      +Tv*fcp->Pest[naxe][5];
    fcp->Ppre[naxe][4] = fcp->ro*fcp->Pest[naxe][4];
    fcp->Ppre[naxe][5] = fcp->Pest[naxe][5]+fcp->Qk[naxe][1];

    std::cout << "\nPpre old: " ;
    for(int i=0; i < 6; i++)
      std::cout << fcp->Ppre[naxe][i] << " ";
    std::cout << std::endl;
    break;

  default :
    fcp->Vk[naxe] = X_mes[naxe]-X_pre[naxe];

    fcp->Sk[naxe] = fcp->Ppre[naxe][0]+fcp->Rk[naxe];
    if (fcp->Sk[naxe] == 0.0)
      fcp->Sk[naxe] = 0.00000000001;
    fcp->Wk[naxe][0] = fcp->Ppre[naxe][0]/fcp->Sk[naxe];
    fcp->Wk[naxe][1] = fcp->Ppre[naxe][1]/fcp->Sk[naxe];
    fcp->Wk[naxe][2] = fcp->Ppre[naxe][3]/fcp->Sk[naxe];

    std::cout << "\nW old: " ;
    for(int i=0; i < 3; i++)
      std::cout << fcp->Wk[naxe][i] << " ";
    std::cout << std::endl;
  

    X_est[naxe] = X_pre[naxe]+(fcp->Wk[naxe][0]*fcp->Vk[naxe]);
    w_est[naxe] = fcp->wpre[naxe]+(fcp->Wk[naxe][1]*fcp->Vk[naxe]);
    Xp_est[naxe] = Xp_pre[naxe]+(fcp->Wk[naxe][2]*fcp->Vk[naxe]);
    fcp->Pest[naxe][0] = (1.0-fcp->Wk[naxe][0])*fcp->Ppre[naxe][0];
    fcp->Pest[naxe][1] = (1.0-fcp->Wk[naxe][0])*fcp->Ppre[naxe][1];
    fcp->Pest[naxe][3] = (1.0-fcp->Wk[naxe][0])*fcp->Ppre[naxe][3];
    fcp->Pest[naxe][2] = fcp->Ppre[naxe][2]
      -fcp->Ppre[naxe][1]*fcp->Wk[naxe][1];
    fcp->Pest[naxe][4] = fcp->Ppre[naxe][4]
      -fcp->Ppre[naxe][3]*fcp->Wk[naxe][1];
    fcp->Pest[naxe][5] = fcp->Ppre[naxe][5]
      -fcp->Ppre[naxe][3]*fcp->Wk[naxe][2];

    X_pre[naxe] = X_est[naxe]+w_est[naxe]+Tv*Xp_est[naxe];
    fcp->wpre[naxe] = fcp->ro*w_est[naxe];
    Xp_pre[naxe] = Xp_est[naxe];
    fcp->Ppre[naxe][0] = fcp->Pest[naxe][0]+2.0*fcp->Pest[naxe][1]
      +fcp->Pest[naxe][2]+2.0*Tv*(fcp->Pest[naxe][3]+fcp->Pest[naxe][4])
      +Tv*Tv*fcp->Pest[naxe][5];
    fcp->Ppre[naxe][1] = fcp->ro*(fcp->Pest[naxe][1]+fcp->Pest[naxe][2]
				  +Tv*fcp->Pest[naxe][4]);
    fcp->Ppre[naxe][2] = fcp->ro*fcp->ro*fcp->Pest[naxe][2]+fcp->Qk[naxe][0];
    fcp->Ppre[naxe][3] = fcp->Pest[naxe][3]+fcp->Pest[naxe][4]
      +Tv*fcp->Pest[naxe][5];
    fcp->Ppre[naxe][4] = fcp->ro*fcp->Pest[naxe][4];
    fcp->Ppre[naxe][5] = fcp->Pest[naxe][5]+fcp->Qk[naxe][1];

    std::cout << "\nPpre old: " ;
    for(int i=0; i < 6; i++)
      std::cout << fcp->Ppre[naxe][i] << " ";
    std::cout << std::endl;
 
    break;
  }
}

int
main()
{
  int nsignal = 1; // Number of signal to filter
  int niter = 100;

  std::string filename = "/tmp/log.dat";
  std::ofstream flog(filename.c_str());

  vpKalmanFilter kalman;
 
  vpKalmanFilter::vpStateModel model;
  model = vpKalmanFilter::stateConstAccWithColoredNoise_MeasureVel;
  kalman.setStateModel(model);

  int size_state_vector = kalman.getStateSize()*nsignal;
  int size_measure_vector = kalman.getMeasureSize()*nsignal;

  vpColVector sigma_measure(size_measure_vector);
  for (int signal=0; signal < nsignal; signal ++) 
    sigma_measure = 0.0001;
  vpColVector sigma_state(size_state_vector);
  for (int signal=0; signal < nsignal; signal ++) { 
    sigma_state[3*signal] = 0.; // not used
    sigma_state[3*signal+1] = 0.000001;
    sigma_state[3*signal+2] = 0.000001;
  }
  
  vpColVector velocity_measure(size_measure_vector);

  double rho = 0.9; // correlation
  double dt = 0.2; // sampling period

  for (int signal=0; signal < nsignal; signal ++) 
    velocity_measure[signal] = 3+2*signal;

  kalman.verbose(false);
  #define OLD
#ifdef OLD
  double tabfiltc[4];
  tabfiltc[0] = sigma_state[1];
  tabfiltc[1] = sigma_state[2];
  tabfiltc[2] = sigma_measure[0];
  tabfiltc[3] = rho;
  
  st_filtre filtre;
  double X_mes[6], X_est[6], X_pre[6];
  X_mes[0] = velocity_measure[0];
  filtc_reinit(tabfiltc, &filtre, dt);
#endif

  kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dt);


  for (int iter=0; iter <= niter; iter++) {
    std::cout << "-------- iter " << iter << " ------------" << std::endl;
    for (int signal=0; signal < nsignal; signal ++) {
      velocity_measure[signal] = 3+2*signal + 0.3*sin(vpMath::rad(360./niter*iter));
    }
    std::cout << "measure : " << velocity_measure.t() << std::endl;

    flog << velocity_measure.t();

    //    kalman.prediction();
    kalman.filter(velocity_measure);
    flog << kalman.Xest.t();
    flog << kalman.Xpre.t();

#ifdef OLD
    X_mes[0] = velocity_measure[0];
    double dummy = 0;
    kalmc_a_A(0, iter, dt, &dummy, X_mes, &filtre, X_est, X_pre, &dummy);
#endif
    std::cout << "Xest: " << kalman.Xest.t() 
#ifdef OLD
	      << " old " << X_est[0] 
#endif
      ;
#ifdef OLD
    flog << " " << X_est[0] ;
#endif
    std::cout << "Xpre: " << kalman.Xpre.t() 
#ifdef OLD
	      << " old " << X_pre[0] 
#endif
	      << std::endl;
    flog <<  std::endl;
  }

  flog.close();
  return 0;
}
