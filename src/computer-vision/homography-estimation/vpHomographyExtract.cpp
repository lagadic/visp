#include <visp/vpHomography.h>
#include <visp/vpMath.h>
#include <math.h>

//#define DEBUG_Homographie 0


#define SEUIL_SING  0.0001

/**********************************************************************
* Nom         : Homographie_EstimationDeplacementCamera
* Description : Estimation du deplacement de la camera entre 2 images
*
*          H = U * D * V^T
*	   D = d' * R' + t' * n'^T
*
*      avec R = s.U * R' * V^T
*           r = U * t'
*	    n = V * n'
*	    d = s * d'
*	    s = det(U)*det(V)
***********************************************************************
* Entres : Matrice d'homographie : H[3x3]
*          Vecteur de la normale desiree : normaleDesiree
* Sortie : Matrice de rotation : aRb
*          Vecteur de translation : aTb
*	   Vecteur de normale : n
**********************************************************************/
void
vpHomography::computeDisplacement (const vpHomography &H,
				   vpRotationMatrix &aRb,
				   vpTranslationVector &aTb,
				   vpColVector &n)
{
  /**** Déclarations des variables ****/

  vpMatrix aRbint(3,3) ;
  vpColVector svTemp(3), sv(3);
  vpMatrix mX(3,2) ;
  vpColVector aTbp(3), normaleEstimee(3);
  double distanceFictive;
  double sinusTheta, cosinusTheta, signeSinus= 1;
  double cosinusDesireeEstimee, cosinusAncien;
  double s, determinantU, determinantV;
  int	 i, j, k, w;
  int    vOrdre[3];

  vpColVector normaleDesiree(3) ;
  normaleDesiree[0]=0;normaleDesiree[1]=0;normaleDesiree[2]=1;

  /**** Corps de la focntion ****/
#ifdef DEBUG_Homographie
  printf ("debut : Homographie_EstimationDeplacementCamera\n");
#endif

  /* Allocation des matrices  */
  vpMatrix mTempU(3,3) ;
  vpMatrix mTempV(3,3) ;
  vpMatrix mU(3,3) ;
  vpMatrix mV(3,3) ;
  vpMatrix aRbp(3,3) ;


  vpMatrix mH(3,3) ;
  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3 ; j++) mH[i][j] = H[i][j];

  /* Preparation au calcul de la SVD */
  mTempU = mH ;

  /*****
	Remarque : mTempU, svTemp et mTempV sont modifies par svd
	Il est necessaire apres de les trier  dans l'ordre decroissant
	des valeurs singulieres
  *****/
  mTempU.svd(svTemp,mTempV) ;

  /* On va mettre les valeurs singulieres en ordre decroissant : */

  /* Determination de l'ordre des valeurs */
  if (svTemp[0] >= svTemp[1]) {
    if (svTemp[0] >= svTemp[2]) {
      if (svTemp[1] > svTemp[2]) {
	vOrdre[0] = 0; 	vOrdre[1] = 1; 	vOrdre[2] = 2;
      } else {
	vOrdre[0] = 0; 	vOrdre[1] = 2; 	vOrdre[2] = 1;
      }
    } else {
      vOrdre[0] = 2;    vOrdre[1] = 0;  vOrdre[2] = 1;
    }
  } else {
    if (svTemp[1] >= svTemp[2]){
      if (svTemp[0] > svTemp[2])
      {	vOrdre[0] = 1; 	vOrdre[1] = 0; 	vOrdre[2] = 2; }
      else
      {	vOrdre[0] = 1; 	vOrdre[1] = 2; 	vOrdre[2] = 0; }
    } else {
      vOrdre[0] = 2;    vOrdre[1] = 1;  vOrdre[2] = 0;
    }
  }
  /*****
	Tri decroissant des matrices U, V, sv
	en fonction des valeurs singulieres car
	hypothese : sv[0]>=sv[1]>=sv[2]>=0
  *****/


  for (i = 0; i < 3; i++) {
    sv[i] = svTemp[vOrdre[i]];
    for (j = 0; j < 3; j++) {
      mU[i][j] = mTempU[i][vOrdre[j]];
      mV[i][j] = mTempV[i][vOrdre[j]];
    }
  }

#ifdef DEBUG_Homographie
  printf("U : \n") ; cout << mU << endl ;
  printf("V : \n") ; cout << mV << endl ;
  printf("Valeurs singulieres : ") ; cout << sv.t() ;
#endif

  /* A verifier si necessaire!!! */
  determinantV = vpMatrix::det33(mV);
  determinantU = vpMatrix::det33(mU);

  s = determinantU * determinantV;

#ifdef DEBUG_Homographie
  printf ("s = det(U) * det(V) = %f * %f = %f\n",determinantU,determinantV,s);
#endif
  if (s < 0)   mV *=-1 ;


  /* d' = d2 */
  distanceFictive = sv[1];
#ifdef DEBUG_Homographie
  printf ("d = %f\n",distanceFictive);
#endif
  n.resize(3) ;

  if (((sv[0] - sv[1]) < SEUIL_SING) && (sv[0] - sv[2]) < SEUIL_SING)
  {
    //#ifdef DEBUG_Homographie
    printf ("\nCas rotation pure\n");
    //#endif
    /*****
	  Cas ou le deplacement est une rotation pure
	  et la normale reste indefini
	  sv[0] = sv[1]= sv[2]
    *****/
    aTbp[0] = 0;
    aTbp[1] = 0;
    aTbp[2] = 0;

    n[0] = normaleDesiree[0];
    n[1] = normaleDesiree[1];
    n[2] = normaleDesiree[2];
  }
  else
  {
#ifdef DEBUG_Homographie
    printf("\nCas general\n");
#endif
    /* Cas general */

    /*****
	  test pour determiner quelle est la bonne solution on teste
	  d'abord quelle solution est plus proche de la perpendiculaire
	  au plan de la cible constuction de la normale n
    *****/

    /*****
	  Calcul de la normale au plan : n' = [ esp1*x1 , x2=0 , esp3*x3 ]
	  dans l'ordre : cas (esp1=+1, esp3=+1) et (esp1=-1, esp3=+1)
    *****/
    mX[0][0] = sqrt ((sv[0] * sv[0] - sv[1] * sv[1])
		     / (sv[0] * sv[0] - sv[2] * sv[2]));
    mX[1][0] = 0.0;
    mX[2][0] = sqrt ((sv[1] * sv[1] - sv[2] * sv[2])
		     / (sv[0] * sv[0] - sv[2] * sv[2]));

    mX[0][1] = -mX[0][0];
    mX[1][1] =  mX[1][0];
    mX[2][1] =  mX[2][0];

    /* Il y a 4 solutions pour n : 2 par cas =>  n1, -n1, n2, -n2 */
    cosinusAncien = 0.0;
    for (w = 0; w < 2; w++) { /* Pour les 2 cas */
      for (k = 0; k < 2; k++) { /* Pour le signe */

	/* Calcul de la normale estimee : n = V.n' */
	for (i = 0; i < 3; i++) {
	  normaleEstimee[i] = 0.0;
	  for (j = 0; j < 3; j++) {
	    normaleEstimee[i] += (2.0 * k - 1.0) * mV[i][j] * mX[j][w];
	  }
	}

	/* Calcul du cosinus de l'angle entre la normale reelle et desire */
	cosinusDesireeEstimee = 0.0;
	for (i = 0; i < 3; i++)
	  cosinusDesireeEstimee += normaleEstimee[i] * normaleDesiree[i];

	/*****
	      Si la solution est meilleur
	      Remarque : On ne teste pas le cas oppose (cos<0)
	*****/
	if (cosinusDesireeEstimee > cosinusAncien)
	{
	  cosinusAncien = cosinusDesireeEstimee;

	  /* Affectation de la normale qui est retourner */
	  for (j = 0; j < 3; j++)
	    n[j] = normaleEstimee[j];

	  /* Construction du vecteur t'= +/- (d1-d3).[x1, 0, -x3] */
	  aTbp[0] =  (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[0][w];
	  aTbp[1] =  (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[1][w];
	  aTbp[2] = -(2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[2][w];

	  /* si c'est la deuxieme solution */
	  if (w == 1)
	    signeSinus = -1; /* car esp1*esp3 = -1  */
	  else
	    signeSinus = 1;
	} /* fin if (cosinusDesireeEstimee > cosinusAncien) */
      } /* fin k */
    } /* fin w */
  } /* fin else */


  /* Calcul du vecteur de translation qui est retourner : t = (U * t') / d */
  for (i = 0; i < 3; i++) {
    aTb[i] = 0.0;
    for (j = 0; j < 3; j++) {
      aTb[i] +=  mU[i][j] * aTbp[j];
    }
    aTb[i] /=  distanceFictive;
  }

#ifdef DEBUG_Homographie
  printf("t'  : ") ; cout << aTbp.t() ;
  printf("t/d : ") ; cout << aTb.t() ;
  printf("n   : ") ; cout << n.t() ;
#endif


  /* Calcul de la matrice de rotation R */

  /*****
	Calcul du sinus(theta) et du cosinus(theta)
	Remarque : sinus(theta) pourra changer de signe en fonction
	de la solution retenue (cf. ci-dessous)
  *****/
  sinusTheta =    signeSinus*sqrt((sv[0]*sv[0] -sv[1]*sv[1])
				  *(sv[1]*sv[1] -sv[2]*sv[2]))
    / ((sv[0] + sv[2]) * sv[1]);

  cosinusTheta =  ( sv[1] * sv[1] + sv[0] * sv[2] )
    / ((sv[0] + sv[2]) * sv[1]);

  /* construction de la matrice de rotation R' */
  aRbp[0][0] = cosinusTheta; aRbp[0][1] = 0;  aRbp[0][2] = -sinusTheta;
  aRbp[1][0] = 0;            aRbp[1][1] = 1;  aRbp[1][2] = 0;
  aRbp[2][0] = sinusTheta;   aRbp[2][1] = 0;  aRbp[2][2] = cosinusTheta;



  /* multiplication Rint = U R' */
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      aRbint[i][j] = 0.0;
      for (k = 0; k < 3; k++) {
	aRbint[i][j] += mU[i][k] * aRbp[k][j];
      }
    }
  }

  /* multiplication R = Rint . V^T */
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      aRb[i][j] = 0.0;
      for (k = 0; k < 3; k++) {
	aRb[i][j] += aRbint[i][k] * mV[j][k];
      }
    }
  }
  /*transpose_carre(aRb,3); */
#ifdef DEBUG_Homographie
  printf("R : %d\n",aRb.isARotationMatrix() ) ; cout << aRb << endl ;
#endif

}


/*!
  Compute the camera displacement aMb associated with an homography aHb


  implicit parameter: Homography aHb (this)
  \param Rotation matrix aRb
  \param Translation vector aTb
  \param Normale: n
 */

void
vpHomography::computeDisplacement(vpRotationMatrix &aRb,
				  vpTranslationVector &aTb,
				  vpColVector &n)
{


  vpColVector nd(3) ;
  nd[0]=0;nd[1]=0;nd[2]=1;

  computeDisplacement(*this,aRb,aTb,n);

}
