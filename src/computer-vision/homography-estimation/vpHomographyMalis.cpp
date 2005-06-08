
/*
  #----------------------------------------------------------------------------
  #  Copyright (C) 2002  IRISA-INRIA Rennes Vista Project
  #  All Rights Reserved.
  #
  #    Contact:
  #       IRISA-INRIA Rennes
  #       Campus Universitaire de Beaulieu
  #       35042 Rennes Cedex
  #       France
  #
  #    email: marchand@irisa.fr
  #    www  : http://www.irisa.fr/lagadic
  #
  #----------------------------------------------------------------------------
*/

#define DEBUG_Homographie

/*!
  \file vpHomographyMalis.cpp

  This file implements the fonctions related with the homography
  estimation using the Malis algorithm
*/
#include <visp/vpHomography.h>

const double eps = 0.0000000001 ;



/**************************************************************************
* NOM :
* Homographie_CrvHofChRepEucl
*
* DESCRIPTION :
* Changement de repere Euclidien.
*
****************************************************************************
* ENTREES :
* int pts_ref[4]	: Definit quels sont les points de reference, ils ne
*			  seront pas affectes par le changement de repere
* int nb_pts		: nombre de points a changer de repere
* double **point_des	: La matrice des coordonnees des points desires
* double **point_cour	: La matrice des coordonnees des points courants
*
*
* SORTIES :
*
* double **pt_des_nr 	: La matrice des coordonnees des points desires
*			  dans le nouveau repere.
* double **pt_cour_nr	: La matrice des coordonnees des points courants
*			  dans le nouveau repere
* double **M_cour	: ??
* double **pinv_M_des	: pseudo inverse de M  ..
*
*
****************************************************************************
* AUTEUR : BOSSARD Nicolas.  INSA Rennes 5eme annee.
*
* DATE DE FIN DE CREATION : 15/12/98
*
* DATES DE MISE A JOUR :
*
****************************************************************************/


void ChRepEucl(int *pts_ref,
	       int nb_pts,
	       vpMatrix &point_des, vpMatrix &point_cour,
	       vpMatrix &pt_des_nr, vpMatrix &pt_cour_nr,
	       vpMatrix &M_cour, vpMatrix &pinv_M_des)
{



  int i,j,k ;
  int cont_pts;		/* */
  double lamb_des[3];	/* */
  double lamb_cour[3] ; /* */



  /* Construction des matrices de changement de repere */
  vpMatrix M_des(3,3) ;
  vpMatrix pinv_M_cour(3,3) ;

  for (i=0;i<3;i++) {
    for (j=0;j<3;j++) {
      M_cour[j][i] = point_cour[pts_ref[i]][j] ;
      M_des[j][i]  = point_des[pts_ref[i]][j]  ;
    }
  }

  /*calcul de la pseudo inverse  */
  pinv_M_cour= M_cour.pseudoInverse(1e-16) ;
  pinv_M_des = M_des.pseudoInverse(1e-16) ;

  if (pts_ref[3] > 0) {
    for (i=0;i<3;i++) {
      for (j=0;j<3;j++) {
	lamb_cour[i] = pinv_M_cour[i][j]*point_cour[pts_ref[3]][j] ;
	lamb_des[i]  = pinv_M_des[i][j]*point_des[pts_ref[3]][j] ;
      }
    }

    for (i=0;i<3;i++) {
      for (j=0;j<3;j++) {
	M_cour[i][j] = M_cour[i][j]*lamb_cour[j] ;
	M_des[i][j] = M_des[i][j]*lamb_des[j] ;
      }
    }

    pinv_M_des = M_des.pseudoInverse(1e-16);
  }


  /* changement de repere pour tous les points autres que
     les trois points de reference */

  cont_pts = 0 ;
  for (k=0;k<nb_pts;k++) {
    if ((pts_ref[0] != k) && (pts_ref[1] != k) && (pts_ref[2] != k)) {
      for (i=0;i<3;i++) {
	pt_cour_nr[cont_pts][i] = 0.0 ;
	pt_des_nr[cont_pts][i]  = 0.0 ;
	for (j=0;j<3;j++) {
	  pt_cour_nr[cont_pts][i] = pt_cour_nr[cont_pts][i] + pinv_M_cour[i][j]*point_cour[k][j] ;
	  pt_des_nr[cont_pts][i]  = pt_des_nr[cont_pts][i]  + pinv_M_des[i][j]*point_des[k][j]   ;
	}
      }
      cont_pts = cont_pts + 1;
    }
  }



}


/**************************************************************************
* NOM :
* Homographie_CrvMafEstHomoPointsCible2D
*
* DESCRIPTION :
* Calcul de l'homographie entre une image courante et une image desiree dans le
* cas particulier d'une cible planaire de points (cible pi).
* Cette procedure est appellee par "Homographie_CrvMafCalculHomographie".
*
****************************************************************************
* ENTREES :
* int 	Nb_pts : nombre de points
* double	**point_des : tableau des coordonnees des points desires
* couble	**point_cour : tableau des coordonnees des points courants
*
* SORTIES :
*
* double **H 			matrice d homographie
*
****************************************************************************
* AUTEUR : BOSSARD Nicolas.  INSA Rennes 5eme annee.
*
* DATE DE CREATION : 02/12/98
*
* DATES DE MISE A JOUR :
*
****************************************************************************/
void
ezio2D(int nb_pts,
       vpMatrix &points_des,
       vpMatrix &points_cour,
       vpMatrix &H)
{
  int i,j ;

  double  vals_inf ;
  int  contZeros, vect;

#ifdef DEBUG_Homographie
  printf ("debut : Homographie_CrvMafEstHomoPointsCible2D\n");
#endif


  /** allocation des matrices utilisees uniquement dans la procedure **/
   vpMatrix M(3*nb_pts,9) ;
  vpMatrix V(9,9) ;
  vpColVector sv(9) ;



  /** construction de la matrice M des coefficients dans le cas general **/
  for (j=0; j<nb_pts ;j++) {
    M[3*j][0] =  0 ;
    M[3*j][1] =  0 ;
    M[3*j][2] =  0 ;
    M[3*j][3] = -points_des[j][0]*points_cour[j][2] ;
    M[3*j][4] = -points_des[j][1]*points_cour[j][2] ;
    M[3*j][5] = -points_des[j][2]*points_cour[j][2] ;
    M[3*j][6] =  points_des[j][0]*points_cour[j][1] ;
    M[3*j][7] =  points_des[j][1]*points_cour[j][1] ;
    M[3*j][8] =  points_des[j][2]*points_cour[j][1] ;

    M[3*j+1][0] =  points_des[j][0]*points_cour[j][2] ;
    M[3*j+1][1] =  points_des[j][1]*points_cour[j][2] ;
    M[3*j+1][2] =  points_des[j][2]*points_cour[j][2] ;
    M[3*j+1][3] =  0 ;
    M[3*j+1][4] =  0 ;
    M[3*j+1][5] =  0 ;
    M[3*j+1][6] = -points_des[j][0]*points_cour[j][0] ;
    M[3*j+1][7] = -points_des[j][1]*points_cour[j][0] ;
    M[3*j+1][8] = -points_des[j][2]*points_cour[j][0] ;

    M[3*j+2][0] = -points_des[j][0]*points_cour[j][1] ;
    M[3*j+2][1] = -points_des[j][1]*points_cour[j][1] ;
    M[3*j+2][2] = -points_des[j][2]*points_cour[j][1] ;
    M[3*j+2][3] =  points_des[j][0]*points_cour[j][0] ;
    M[3*j+2][4] =  points_des[j][1]*points_cour[j][0] ;
    M[3*j+2][5] =  points_des[j][2]*points_cour[j][0] ;
    M[3*j+2][6] =  0 ;
    M[3*j+2][7] =  0 ;
    M[3*j+2][8] =  0 ;
  }

#ifdef DEBUG_Homographie
  printf("Affichage de M\n");
  cout << M << endl ;
#endif
  /** calcul de la pseudo-inverse V de M et des valeurs singulières **/
  M.svd(sv,V);

#ifdef DEBUG_Homographie
  printf("Affichage de V\n");
  cout << V << endl ;
#endif
  /*****
	La meilleure solution est le vecteur de V associe
	a la valeur singuliere la plus petite en valeur	absolu.
	Pour cela on parcourt la matrice des valeurs singulieres
	et on repère la plus petite valeur singulière, on en profite
	pour effectuer un controle sur le rang de la matrice : pas plus
	de 2 valeurs singulières quasi=0
  *****/
  vals_inf = fabs(sv[0]) ;
  vect = 0 ;
  contZeros = 0;
  if (fabs(sv[0]) < eps) {
    contZeros = contZeros + 1 ;
  }
  for (j=1; j<9; j++) {
    if (fabs(sv[j]) < vals_inf) {
      vals_inf = fabs(sv[j]);
      vect = j ;
    }
    if (fabs(sv[j]) < eps) {
      contZeros = contZeros + 1 ;
    }
  }


  /** cas d'erreur : plus de 2 valeurs singulières =0 **/
  if (contZeros > 2) {
    printf("erreur dans le rang de la matrice \r\n");
  }

  H.resize(3,3) ;
  /** construction de la matrice H **/
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++){
      H[i][j] = V[3*i+j][vect];
    }
  }

#ifdef DEBUG_Homographie
  printf("H :\n"); cout << H << endl ;
#endif

}


/**************************************************************************
* NOM :
* Homographie_CrvMafEstHomoPointsC3DEzio
*
* DESCRIPTION :
* Calcul de l'homographie entre une image courante et une image desiree dans le
* cas particulier d'une cible non planaire de points (cible pi).
* Cette procedure est appellee par "Homographie_CrvMafCalculHomographie".
*
*
****************************************************************************
* ENTREES :
* int 	Nb_pts : nombre de points
* double	**point_des : tableau des coordonnees des points desires
* couble	**point_cour : tableau des coordonnees des points courants
*
* SORTIES :
*
* double **H 			matrice d'homographie
* double epipole[3]		epipole
*
****************************************************************************
* AUTEUR : BOSSARD Nicolas.  INSA Rennes 5eme annee.
*
* DATE DE FIN CREATION : 15/12/98
*
* DATES DE MISE A JOUR :
*
****************************************************************************/
void
ezio3D(int nb_pts,
       vpMatrix &point_des,
       vpMatrix & point_cour,
       vpMatrix &H,
       vpColVector &epipole)
{
   TRACE(" ") ;

  int i,j,k,ii,jj ;
  int cont_pts;			/* Pour compter le nombre de points dans l'image */
  int nl;			/*** Nombre de lignes ***/
  int nc ;			/*** Nombre de colonnes ***/
  int  pts_ref[4];		/*** définit lesquels des points de
				     l'image sont les points de référence***/
 	/***  ***/
  int perm;			/***  Compte le nombre de permutations, quand le nombre
				     de permutations =0 arret de l'ordonnancement **/
  int  cont_zeros;		/*** pour compter les valeurs quasi= a zero	***/
  int  cont;
  int  vect;

  int 	 prob;

  /***** Corps de la fonction	*****/
#ifdef DEBUG_Homographie
  printf ("debut : Homographie_CrvMafEstHomoPointsC3DEzio\n");
#endif

  /* allocation des matrices utilisees uniquement dans la procedure */
  prob=0;

  vpMatrix M_cour(3,3) ;
  vpMatrix pinv_M_des(3,3) ;
  vpMatrix c(8,2) ; // matrice des coeff C

  vpColVector d(8) ;
  vpMatrix pinv_c(2,8) ; //matrice pseudo-inverse de C


  vpMatrix H_int(3,3) ;
  vpMatrix pt_cnr((nb_pts-3),3) ; //points courant nouveau repère


  vpMatrix pt_dnr((nb_pts-3),3) ; //points dérivés nouveau repère

  /* preparation du changement de repere */
  /****
       comme plan de reference on choisit pour le moment
       arbitrairement le plan contenant les points 1,2,3 du cinq
  ****/
  pts_ref[0] = 0 ; pts_ref[1] = 1 ; pts_ref[2] = 2 ; pts_ref[3] = -1 ;

  /* changement de repere pour tous les points autres que les trois points de reference */

  ChRepEucl(pts_ref,nb_pts,point_des,point_cour,pt_dnr,pt_cnr,M_cour,pinv_M_des);


  cont_pts = nb_pts - 3 ;

  if (cont_pts < 5) printf(" attention !! pas assez de points ... \r\n");

  nl = cont_pts*(cont_pts-1)*(cont_pts-2)/6 ;
  nc = 7  ;

  /* Allocation matrice MTM */
  vpMatrix MTM(nc,nc) ;

  /* Initialisation matrice MTM */
  for (ii=0;ii<nc;ii++) for (jj=0;jj<nc;jj++) MTM[ii][jj] = 0.0;


  /* Allocation matrice M */
  vpColVector M(nc) ; //Matrice des coefficients

  /* construction de la matrice M des coefficients dans le cas general */
  /****
       inconnues du nouveau algorithme
       x1 = a  ; x2 = b  ; x3 = c ;
       x4 = ex ; x5 = ey ; x6 = ez ;
       qui deviennent apres changement :
       v1 = x1*x6 ; v2 = x1*x5 ;
       v3 = x2*x4 ; v4 = x2*x6 ;
       v5 = x3*x5 ; v6 = x3*x4 ;
  ****/
  cont = 0 ;
  for (i=0 ; i<nb_pts-5; i++) {
    for (j = i+1 ; j<nb_pts-4; j++) {
      for (k = j+1 ; k<nb_pts-3; k ++) {
	/* coeff a^2*b  */
	M[0] = pt_cnr[i][2]*pt_cnr[j][2]*pt_cnr[k][1]*pt_dnr[k][0]
	  * (pt_dnr[j][0]*pt_dnr[i][1] - pt_dnr[j][1]*pt_dnr[i][0])
	  + pt_cnr[i][2]*pt_cnr[k][2]*pt_cnr[j][1]*pt_dnr[j][0]
	  *(pt_dnr[i][0]*pt_dnr[k][1] - pt_dnr[i][1]*pt_dnr[k][0])
	  + pt_cnr[j][2]*pt_cnr[k][2]*pt_cnr[i][1]*pt_dnr[i][0]
	  *(pt_dnr[k][0]*pt_dnr[j][1] - pt_dnr[k][1]*pt_dnr[j][0]) ;
	/* coeff a*b^2 */
	M[1] = pt_cnr[i][2]*pt_cnr[j][2]*pt_cnr[k][0]*pt_dnr[k][1]
	  *(pt_dnr[i][0]*pt_dnr[j][1] - pt_dnr[i][1]*pt_dnr[j][0])
	  + pt_cnr[i][2]*pt_cnr[k][2]*pt_cnr[j][0]*pt_dnr[j][1]
	  *(pt_dnr[k][0]*pt_dnr[i][1] - pt_dnr[k][1]*pt_dnr[i][0])
	  + pt_cnr[j][2]*pt_cnr[k][2]*pt_cnr[i][0]*pt_dnr[i][1]
	  *(pt_dnr[j][0]*pt_dnr[k][1] - pt_dnr[j][1]*pt_dnr[k][0]) ;
	/* coeff a^2 */
	M[2] = pt_cnr[i][1]*pt_cnr[j][1]*pt_cnr[k][2]*pt_dnr[k][0]
	  *(pt_dnr[i][2]*pt_dnr[j][0] - pt_dnr[i][0]*pt_dnr[j][2])
	  + pt_cnr[i][1]*pt_cnr[k][1]*pt_cnr[j][2]*pt_dnr[j][0]
	  *(pt_dnr[k][2]*pt_dnr[i][0] - pt_dnr[k][0]*pt_dnr[i][2])
	  + pt_cnr[j][1]*pt_cnr[k][1]*pt_cnr[i][2]*pt_dnr[i][0]
	  *(pt_dnr[j][2]*pt_dnr[k][0] - pt_dnr[j][0]*pt_dnr[k][2]) ;
	/* coeff b^2 */
	M[3] = pt_cnr[i][0]*pt_cnr[j][0]*pt_cnr[k][2]*pt_dnr[k][1]
	  *(pt_dnr[i][2]*pt_dnr[j][1] - pt_dnr[i][1]*pt_dnr[j][2])
	  + pt_cnr[i][0]*pt_cnr[k][0]*pt_cnr[j][2]*pt_dnr[j][1]
	  *(pt_dnr[k][2]*pt_dnr[i][1] - pt_dnr[k][1]*pt_dnr[i][2])
	  + pt_cnr[j][0]*pt_cnr[k][0]*pt_cnr[i][2]*pt_dnr[i][1]
	  *(pt_dnr[j][2]*pt_dnr[k][1] - pt_dnr[j][1]*pt_dnr[k][2]) ;
	/* coeff a*b */
	M[4] = pt_cnr[i][0]*pt_cnr[k][1]*pt_cnr[j][2]
	  *(pt_dnr[k][0]*pt_dnr[j][1]*pt_dnr[i][2] - pt_dnr[j][0]*pt_dnr[i][1]*pt_dnr[k][2])
	  + pt_cnr[k][0]*pt_cnr[i][1]*pt_cnr[j][2]
	  *(pt_dnr[j][0]*pt_dnr[k][1]*pt_dnr[i][2] - pt_dnr[i][0]*pt_dnr[j][1]*pt_dnr[k][2])
	  + pt_cnr[i][0]*pt_cnr[j][1]*pt_cnr[k][2]
	  *(pt_dnr[k][0]*pt_dnr[i][1]*pt_dnr[j][2] - pt_dnr[j][0]*pt_dnr[k][1]*pt_dnr[i][2])
	  + pt_cnr[j][0]*pt_cnr[i][1]*pt_cnr[k][2]
	  *(pt_dnr[i][0]*pt_dnr[k][1]*pt_dnr[j][2] - pt_dnr[k][0]*pt_dnr[j][1]*pt_dnr[i][2])
	  + pt_cnr[k][0]*pt_cnr[j][1]*pt_cnr[i][2]
	  *(pt_dnr[j][0]*pt_dnr[i][1]*pt_dnr[k][2] - pt_dnr[i][0]*pt_dnr[k][1]*pt_dnr[j][2])
	  + pt_cnr[j][0]*pt_cnr[k][1]*pt_cnr[i][2]
	  *(pt_dnr[i][0]*pt_dnr[j][1]*pt_dnr[k][2] - pt_dnr[k][0]*pt_dnr[i][1]*pt_dnr[j][2]) ;
	/* coeff a */
	M[5] = pt_cnr[i][1]*pt_cnr[j][1]*pt_cnr[k][0]*pt_dnr[k][2]
	  *(pt_dnr[i][0]*pt_dnr[j][2] - pt_dnr[i][2]*pt_dnr[j][0])
	  + pt_cnr[i][1]*pt_cnr[k][1]*pt_cnr[j][0]*pt_dnr[j][2]
	  *(pt_dnr[k][0]*pt_dnr[i][2] - pt_dnr[k][2]*pt_dnr[i][0])
	  + pt_cnr[j][1]*pt_cnr[k][1]*pt_cnr[i][0]*pt_dnr[i][2]
	  *(pt_dnr[j][0]*pt_dnr[k][2] - pt_dnr[j][2]*pt_dnr[k][0]) ;
	/* coeff b */
	M[6] = pt_cnr[i][0]*pt_cnr[j][0]*pt_cnr[k][1]*pt_dnr[k][2]
	  *(pt_dnr[i][1]*pt_dnr[j][2] - pt_dnr[i][2]*pt_dnr[j][1])
	  + pt_cnr[i][0]*pt_cnr[k][0]*pt_cnr[j][1]*pt_dnr[j][2]
	  *(pt_dnr[k][1]*pt_dnr[i][2] - pt_dnr[k][2]*pt_dnr[i][1])
	  + pt_cnr[j][0]*pt_cnr[k][0]*pt_cnr[i][1]*pt_dnr[i][2]
	  *(pt_dnr[j][1]*pt_dnr[k][2] - pt_dnr[j][2]*pt_dnr[k][1]) ;
	cont = cont+1 ;
	/* construction de la matrice MTM */
	for (ii=0;ii<nc;ii++) {
	  for (jj=ii;jj<nc;jj++) {
	    MTM[ii][jj] = MTM[ii][jj] + M[ii]*M[jj];
	  }
	}

      }	/* Fin boucle k */
    }	/* Fin boucle j */
  } /* Fin boucle i */

  TRACE(" ") ;
  /* calcul de MTM */
  for (i=0; i<nc ;i++) {
    for (j=i+1; j<nc ;j++) MTM[j][i] = MTM[i][j];
  }
  TRACE(" ") ;

  cout << MTM << endl ;

  nl = cont ;   /* nombre de lignes   */
  nc = 7 ;      /* nombre de colonnes */

  /* Creation de matrice MTM termine */
  /* Allocation matrice V */
  vpMatrix V(nc,nc) ;
  /*****
	Preparation au calcul de la svd (pseudo-inverse)
	pour obtenir une solution il faut au moins 5 equations independantes
	donc il faut au moins la mise en correspondence de 3+5 points
  *****/
  vpColVector sv(nc) ; //Vecteur contenant les valeurs singulières

  MTM.svd(sv,V) ;

  /*****
	Il faut un controle sur le rang de la matrice !!
	La meilleure solution est le vecteur de V associe
	a la valeur singuliere la plus petite en valeur
	absolu
  *****/

  /* Allocation matrice svSorted */
  vpColVector svSorted(nc) ; //Vecteur contenant les valeurs singulières				     en ordre croissant

  /* Rangement des valeurs singulières en ordre croissant */
  for (i=0; i < nc ;i++) svSorted[i] = sv[i] ;
  perm = 1 ;
  double v_temp;
  while (perm != 0) {
    perm = 0;
    for (i=1; i < nc ;i++) {
      if (svSorted[i-1] > svSorted[i]) {
	v_temp = svSorted[i-1] ;
	svSorted[i-1] = svSorted[i] ;
	svSorted[i] = v_temp ;
	perm = perm + 1;
      }
    }
  }

  /*****
	Parcours de la matrice ordonnée des valeurs singulières
	On note "cont_zeros" le nbre de valeurs quasi= à 0.
	On note "vect" le rang de la plus petite valeur singlière en valeur absolu
  *****/
  vect = 0 ; cont_zeros = 0 ; cont = 0 ;
  for (j=0; j < nc; j++) {
    if (fabs(sv[j]) == svSorted[cont]) vect = j ;
    if (fabs(sv[j]/svSorted[nc-1]) < eps) cont_zeros = cont_zeros + 1 ;
  }
  TRACE(" ") ;

  cout << sv.t() ;
  if (cont_zeros > 5) {
    printf("erreur dans le rang de la matrice: %d \r\n ",7-cont_zeros);
    ezio2D(nb_pts,point_des,point_cour,H);
  }
  else
  {
    /*****
	  estimation de a = 1,b,c ; je cherche le min de somme(i=1:n) (0.5*(ei)^2)
	  e1 = V[1][.] * b - V[3][.] = 0 ;
	  e2 = V[2][.] * c - V[3][.] = 0 ;
	  e3 = V[2][.] * b - V[3][.] * c = 0 ;
	  e4 = V[4][.] * b - V[5][.] = 0 ;
	  e5 = V[4][.] * c - V[6][.] = 0 ;
	  e6 = V[6][.] * b - V[5][.] * c = 0 ;
	  e7 = V[7][.] * b - V[8][.] = 0 ;
	  e8 = V[7][.] * c - V[9][.] = 0 ;
    *****/
    d[0] = V[3-1][vect] ; d[1]  = V[5-1][vect] ;
    d[2] = V[2-1][vect] ; d[3] = V[1-1][vect] ;
    d[4] = V[4-1][vect] ; d[5]  = V[5-1][vect] ;
    d[6] = V[1-1][vect] ; d[7] = V[2-1][vect] ;

    c[0][0] = V[6-1][vect] ; c[0][1] = 0.0 ;
    c[1][0] = V[7-1][vect] ; c[1][1] = 0.0 ;
    c[2][0] = V[4-1][vect] ; c[2][1] = 0.0 ;
    c[3][0] = V[5-1][vect] ; c[3][1] = 0.0        ;
    c[4][0] = 0.0        ; c[4][1] = V[7-1][vect] ;
    c[5][0] = 0.0        ; c[5][1] = V[6-1][vect] ;
    c[6][0] = 0.0        ; c[6][1] = V[3-1][vect] ;
    c[7][0] = 0.0        ; c[7][1] = V[5-1][vect] ;

    /* Calcul de la pseudo-inverse de C */
    pinv_c = c.pseudoInverse(1e-16) ;

    vpColVector H_nr(3), temp ;	/*** Homographie diagonale 	***/
    /* Multiplication de la matrice H_nr par le vecteur pinv_c */
    temp =  pinv_c * d;
    H_nr[0] = temp[0] ; H_nr[1] = temp[1] ;
    H_nr[2] = 1.0 ;

    /* multiplication de M_cour * H_nr  */
    H_int = 0.0 ;
    for (i=0; i<3; i++) {
      for (j=0; j<3; j++) H_int[i][j] = M_cour[i][j]*H_nr[j];
    }

    H = H_int * pinv_M_des ;

    epipole[0] = 0; epipole[1] = 0; epipole[2] = 0;

}

/**************************************************************************
* NOM :
* Homographie_CrvMafCalculHomographie
*
* DESCRIPTION :
* Calcul de l'homographie, en fonction de la cible désirée et de la cible
* en cours. C'est une estimation linéaire.
* Cette procédure n'effectue pas elle-même le calcul de l'homographie :
* elle se contente d'appeler la bonne sous-procédure.
* Cette procédure est appellée par "crv_maf_calcul_tomographie".
*
****************************************************************************
* ENTREES :
*  STR_CIBLE_ASSER   *cible_asser  	Pointeur sur structure contenant les
*                                       commandes du robot, les données de la
*					carte...
*					Voir "cvda/edixaa/param/robot.h"
*	STR_VITESSE_ROBOT *data_common   Pointeur sur la structure décrivant la
*					cible d'asservissement.
*					Voir "cvda/edixia/param/param.h"
*	STR_MACH_DIV 		*machine_div   Pointeur sur structure contenant divers
*					paramètres de configuration du robot.
*					Voir "cvda/edixia/param/param.h"
*
* SORTIES :
*
* double **H 			matrice d'homographie
* double epipole[3]  epipole
*
****************************************************************************
* AUTEUR : BOSSARD Nicolas.  INSA Rennes 5ème année.
*
* DATE DE CREATION : 01/12/98
*
* DATES DE MISE A JOUR :
*
****************************************************************************/
void
ezio(int q_cible,
     int nbpt,
     double *xm, double *ym,
     double *xmi, double *ymi,
     vpMatrix &H, vpColVector  &epipole)
{
  TRACE(" ") ;
  int   i;


  /****
       on regarde si il y a au moins un point mais pour l'homographie
       il faut au moins quatre points
  ****/
  vpMatrix point_des(nbpt,3) ;
  vpMatrix point_cour(nbpt,3) ;

  for (i=0;i<nbpt;i++)  {
    /****
	 on assigne les points fournies par la structure robot
	 pour la commande globale
    ****/
    point_des[i][0] = xmi[i];
    point_des[i][1] = ymi[i];
    point_des[i][2] = 1.0 ;
    point_cour[i][0] = xm[i];
    point_cour[i][1] = ym[i];
    point_cour[i][2] = 1.0 ;
  }

  epipole.resize(3) ;
  switch (q_cible) {
  case (1):
  case (2):
    /* La cible est planaire  de type points   */

    ezio2D(nbpt,point_des,point_cour,H);
    epipole[0] = 0.0 ; epipole[1] = 0.0 ; epipole[2] = 0.0 ;

    break;
  case (3) : /* cible non planaire : chateau */
    /* cible non planaire  de type points   */
    epipole[0] = 0 ; epipole[1] = 0 ; epipole[2] = 0 ;
    ezio3D(nbpt,point_des,point_cour,H,epipole);
    break;
  } /* fin switch */

  TRACE(" ") ;


} /* fin procedure calcul_homogaphie */


// ============================================================================
// NOM : MatchesToHomo
// ============================================================================
// A partir de deux liste de points, calcule l'homographie associe
// ============================================================================
// ENTREES : 4
//  LP1         : la liste de points dans la 1ere image
//  LP2         : la liste de correspondants dans la deuxieme image
//  isplan      : vrai si tous les points sont sur le meme plan
//  epi    : l'epipole
// ============================================================================
// SORTIES : l'homographie obtenue
//=============================================================================

/*!
  A partir de deux listes de points, calcule l'homographie associee
  \param LP1 premiere liste de points
  \param LP2 deuxieme liste de points
  \param isplan les points sont-ils tous coplanaires?
  \param epi l'epipole obtenue (le programme d'Ezio ne met pas cette variable a jour)
  \return la collineation associee
 */
void vpHomography::Malis(int n,
			 double *xb, double *yb,
			 double *xa, double *ya ,
			 bool isplan,
			 vpHomography &Homography,
			 vpColVector & epipole)
{
  TRACE(" ") ;
  int i,j;
  int q_cible;
  epipole.resize(3);
  vpMatrix H; // matrice d'homographie en metre

  Homography.setIdentity();


  if (isplan)
    q_cible =1;
  else
    q_cible =3;
  //cout <<"homo: calcul de la collineation " << endl;
  ezio(q_cible,n, xa,ya,xb,yb,H,epipole) ;
  //cout << "homo:ok" << endl;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      Homography[i][j] = H[i][j];
  TRACE(" ") ;

}
