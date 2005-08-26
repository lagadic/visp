#include <visp/vpFeatureDisplay.h>

// Meter/pixel conversion
#include <visp/vpMeterPixelConversion.h>

// display
#include <visp/vpDisplay.h>

// Debug trace
#include <visp/vpDebug.h>

// math
#include<visp/vpMath.h>



void vpFeatureDisplay::displayPoint(double x,double y,
				    const vpCameraParameters &cam,
				    vpImage<unsigned char> &I,
				    int color)
{
  try{
    double uf,vf ; // pixel coordinates in float
    vpMeterPixelConversion::convertPoint(cam,x,y,uf,vf) ;

    int u,v ;
    u = vpMath::round(uf) ;
    v = vpMath::round(vf) ;

    vpDisplay::displayCross(I,v,u,5,color) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
void vpFeatureDisplay::displayLine(double rho,double theta,
				   const vpCameraParameters &cam,
				   vpImage<unsigned char> &I,
				   int color )
{


  try{
    //    x cos(theta) + y sin(theta) - rho = 0

    double rhop,thetap ;
    vpMeterPixelConversion::convertLine(cam,rho,theta,rhop,thetap) ;

    //    u cos(thetap) + v sin(thetap) - rhop = 0

    double co = cos(thetap) ;
    double si = sin(thetap) ;
    double c = -rhop ;

    // TRACE("rhop %f %f ",rhop, atan2(si,co)) ;
    // double u1,v1,u2,v2 ;

    double a = si ;
    double b = co ;

  if (fabs(a) < fabs(b))
  for (int i=0 ; i < I.getRows() ; i ++)
  {
    double  j = (-c - a*i)/b  ;
    vpDisplay::displayPoint(I,vpMath::round(i), vpMath::round(j), color);
  }
  else

  for (int j=0 ; j < I.getCols() ; j++)
  {
    double  i = (-c - b*j)/a  ;
    vpDisplay::displayPoint(I,vpMath::round(i), vpMath::round(j), color);
  }
  /*
     //  ERROR_TRACE("********* %f %f %f %f ",si, co, theta, rho) ;
    //      ERROR_TRACE("********* %f %f %f %f ",si, co, thetap, rhop) ;

    // cout << co <<  "  " << si << endl ;
    if (fabs(si) < 1e-5)
    {
      v1 = 0.0 ;
      u1 = c/co ;

      v2 = I.getRows()-1 ;
      u2 = c/co ;
      //       ERROR_TRACE("%f %f %f %f \n",u1,v1,u2,v2) ;
    }
    else
      if (fabs(co) < 1e-5)
      {
	v1 =c/si ;
	u1 = 0 ;
	v2 =c/si ;
	u2 = I.getCols()-1 ;
	//	  ERROR_TRACE("%f %f %f %f \n",u1,v1,u2,v2) ;
      }
      else
      {

	if (fabs(si)<fabs(co))
	{
	  v1 = 0 ;
	  u1 = c/co ;

	  v2 = I.getRows()-1 ;
	  u2 = (c - v2*si)/co ;
	}
	else
	{
	  u1 = 0 ;
	  v1 = c/si ;

	  u2 = v2 = I.getCols()-1 ;
	  v2 = (c-u2*co)/si ;
	}

      }

    //   ERROR_TRACE("-------->%f %f %f %f \n",u1,v1,u2,v2) ;
    vpDisplay::displayLine(I,
			   vpMath::round(v1),vpMath::round(u1),
			   vpMath::round(v2),vpMath::round(u2),
			   color) ;

  */
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
void vpFeatureDisplay::displayCylinder(double rho1,double theta1,
				       double rho2,double theta2,
				       const vpCameraParameters &cam,
				       vpImage<unsigned char> &I,
				       int color)
{
  try
  {
    displayLine(rho1, theta1, cam, I, color) ;
    displayLine(rho2, theta2, cam, I, color) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
void vpFeatureDisplay::displayEllipse(double x,double y,
				      double mu20, double mu11, double mu02,
				      const vpCameraParameters &cam,
				      vpImage<unsigned char> &I,
				      int color)
{


  try{
    {
     int number_of_points = 45 ;
      const double incr = 2 * M_PI/(double)number_of_points ; // angle increment
      int i = 0 ;



      //	 cout << s.t() ;
      double s = sqrt(vpMath::sqr(mu20-mu02)+4*mu11*mu11) ;
      double e ;

      if (fabs(mu11)<1e-6) e =0 ;
      else e =  (mu02-mu20+s)/(2*mu11) ;
      double a =sqrt( (mu02+mu20+s)/2.0) ;
      double b =sqrt( (mu02+mu20-s)/2.0) ;

      //    TRACE("%f %f %f", a,b,e) ;

      double e1  = atan(e) ;


      double k = 0.0 ;

      double ce = cos(e1) ;
      double se = sin(e1) ;

      double x2  = 0;
      double y2 =0;
      for( i = 0; i < number_of_points+2 ; i++)
      {



	double    x1 = a *cos(k) ; // equation of an ellipse
	double    y1 = b *sin(k) ; // equation of an ellipse
	double    x11 = x + ce *x1 - se *y1 ;
	double    y11 = y + se *x1 + ce *y1 ;

	x1=x11*cam.get_px() + cam.get_u0() ;
	y1=y11*cam.get_py() + cam.get_v0() ;

	if (i>1) vpDisplay::displayLine(I,
					(int)y1, (int)x1,
					(int)y2, (int)x2,
					color) ;

	x2 = x1 ;
	y2 = y1 ;
	k += incr ;
      } // end for loop
    }
    //    vpDisplay::getClick(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
