

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>

#include <visp/vpFeatureLuminance.h>


using namespace std ;


/*!
  \file vpFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/



/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
vpFeatureLuminance::init()
{
    nbParameters = 1;
    dim_s = 0 ;
    bord = 10 ;
    flags = NULL;
    pixInfo = NULL;

    if (flags == NULL)
      flags = new bool[nbParameters];
    for (int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value Z (1 meters)
    Z = 1;

    firstTimeIn =0 ;

}


void
vpFeatureLuminance::init(int _nbr, int _nbc, double _Z)
{
  init() ;

  nbr = _nbr ;
  nbc = _nbc ;
  // number of feature = nb column x nb lines in the images
  dim_s = (nbr-2*bord)*(nbc-2*bord) ;

  s.resize(dim_s) ;
  
  if (pixInfo != NULL)
    delete [] pixInfo;

  pixInfo = new vpLuminance[dim_s] ;
  
  Z = _Z ;
}

/*! 
  Default constructor that build a visual feature.
*/
vpFeatureLuminance::vpFeatureLuminance() : vpBasicFeature()
{
    init() ;
}

/*! 
  Default destructor.
*/
vpFeatureLuminance::~vpFeatureLuminance() 
{
  if (pixInfo != NULL) delete [] pixInfo ;
  if (flags != NULL) delete [] flags;
}



/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
vpFeatureLuminance::set_Z(const double Z)
{
    this->Z = Z ;
    flags[0] = true;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
vpFeatureLuminance::get_Z() const
{
    return Z ;
}


void
vpFeatureLuminance::setCameraParameters(vpCameraParameters &_cam) 
{
  cam = _cam ;
}


/*!

  Build a luminance feature directly from the image
*/

void
vpFeatureLuminance::buildFrom(vpImage<unsigned char> &I)
{
  int    l = 0;
  double Ix,Iy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;


  if (firstTimeIn==0)
    { 
      firstTimeIn=1 ;
      l =0 ;
      for (int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
	      vpPixelMeterConversion::convertPoint(cam,
						   i, j,
						   y, x)  ;
	    
	      pixInfo[l].x = x;
	      pixInfo[l].y = y;

	      pixInfo[l].Z   = Z ;

	      l++;
	    }
	}
    }

  l= 0 ;
  for (int i=bord; i < nbr-bord ; i++)
    {
      //   cout << i << endl ;
      for (int j = bord ; j < nbc-bord; j++)
	{
	  // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
	  Ix =  px * vpImageFilter::derivativeFilterX(I,i,j) ;
	  Iy =  py * vpImageFilter::derivativeFilterY(I,i,j) ;
	  
	  // Calcul de Z
	  
	  pixInfo[l].I  =  I[i][j] ;
	  s[l]  =  I[i][j] ;
	  pixInfo[l].Ix  = Ix;
	  pixInfo[l].Iy  = Iy;
	  
	  l++;
	}
    }

}




/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
vpFeatureLuminance::interaction(vpMatrix &L)
{
  double x,y,Ix,Iy,Zinv;

  L.resize(dim_s,6) ;

  for(int m = 0; m< L.getRows(); m++)
    {
      Ix = pixInfo[m].Ix;
      Iy = pixInfo[m].Iy;

      x = pixInfo[m].x ;
      y = pixInfo[m].y ;
      Zinv =  1 / pixInfo[m].Z;

      {
	L[m][0] = Ix * Zinv;
	L[m][1] = Iy * Zinv;
	L[m][2] = -(x*Ix+y*Iy)*Zinv;
	L[m][3] = -Ix*x*y-(1+y*y)*Iy;
	L[m][4] = (1+x*x)*Ix + Iy*x*y;
	L[m][5]  = Iy*x-Ix*y;
      }
    }
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix  vpFeatureLuminance::interaction(const int /* select */)
{
  static vpMatrix L  ;
  interaction(L) ;
  return L ;
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void
vpFeatureLuminance::error(const vpBasicFeature &s_star,
			  vpColVector &e)
{
  e.resize(dim_s) ;

  for (int i =0 ; i < dim_s ; i++)
    {
      e[i] = s[i] - s_star[i] ;
    }
}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.

*/
vpColVector
vpFeatureLuminance::error(const vpBasicFeature &s_star,
			  const int /* select */)
{
  static vpColVector e ;
  
  error(s_star, e) ;
  
  return e ;

}




/*!

  Not implemented.

 */
void
vpFeatureLuminance::print(const int /* select */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
 }



/*!

  Not implemented.

 */
void
vpFeatureLuminance::display(const vpCameraParameters & /* cam */,
			    vpImage<unsigned char> & /* I */,
			    vpColor /* color */,  unsigned int /* thickness */) const
{
 static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void
vpFeatureLuminance::display(const vpCameraParameters & /* cam */,
			    vpImage<vpRGBa> & /* I */,
			    vpColor /* color */, unsigned int /* thickness */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
vpFeatureLuminance *vpFeatureLuminance::duplicate() const
{
  vpFeatureLuminance *feature = new vpFeatureLuminance ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
