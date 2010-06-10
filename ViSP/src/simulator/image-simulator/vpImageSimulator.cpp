/****************************************************************************
 *
 * $Id: vpPose.h 2453 2010-01-07 10:01:10Z nmelchio $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Description: Class which enables to project an image in the 3D space 
 * and get the view of a virtual camera.
 * 
 *
 * Authors:
 * Amaury Dame
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp/vpImageSimulator.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMatrixException.h>

/*!
  Basic constructor.
  
  You can choose if you want to use a colored or gray scaled image.
  
  \param col : Enable to choose the color space to use for the image which is projected.
  
  By default the class uses colored images.
*/
vpImageSimulator::vpImageSimulator(vpColorPlan col)
{
  for(int i=0;i<4;i++)
    X[i].resize(3);

  for(int i=0;i<4;i++)
    X2[i].resize(3);

  normal_obj.resize(3);
  visible=false;
  normal_Cam.resize(3);

  //Xinter.resize(3);

  vbase_u.resize(3);
  vbase_v.resize(3);

  normal_Cam_optim = new double[3];
  X0_2_optim = new double[3];
  vbase_u_optim = new double[3];
  vbase_v_optim = new double[3];
  Xinter_optim = new double[3];
  
  colorI = col;
  interp = SIMPLE;
}


vpImageSimulator::vpImageSimulator(const vpImageSimulator &text)
{
  for(int i=0;i<4;i++)
    X[i] = text.X[i];

  for(int i=0;i<4;i++)
    X2[i].resize(3);
  
  Ic = text.Ic;
  Ig = text.Ig;

  normal_obj = text.normal_obj;
  euclideanNorm_u = text.euclideanNorm_u;
  euclideanNorm_v = text.euclideanNorm_v;
  
  normal_Cam.resize(3);
  vbase_u.resize(3);
  vbase_v.resize(3);
  
  normal_Cam_optim = new double[3];
  X0_2_optim = new double[3];
  vbase_u_optim = new double[3];
  vbase_v_optim = new double[3];
  Xinter_optim = new double[3];
  
  colorI = text.colorI;
  interp = text.interp;
  
  setCameraPosition(text.cMt);
}

/*!
  Basic destructor.
*/
vpImageSimulator::~vpImageSimulator()
{
  delete[] normal_Cam_optim;
  delete[] X0_2_optim;
  delete[] vbase_u_optim;
  delete[] vbase_v_optim;
  delete[] Xinter_optim;
}

/*!
  Get the view of the virtual camera. Be carefull, the image I is modified. The projected image is not added as an overlay!
  
  \param I : The image used to store the result.
  \param cam : The parameters of the virtual camera.
*/
void
vpImageSimulator::getImage(vpImage<unsigned char> &I, const vpCameraParameters cam)
{
  int nb_point_dessine = 0;
  if(visible)
  {
    for (unsigned int i = 0; i < I.getHeight(); i++)
    {
      for (unsigned int j = 0; j < I.getWidth(); j++)
      {
        double x,y;
        vpPixelMeterConversion::convertPoint(cam,vpImagePoint(i,j), x,y);
	if (colorI == GRAY_SCALED)
	{
	  unsigned char Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    I[i][j]=Ipixelplan;
	    nb_point_dessine++;
	  }
	}
	else if (colorI == COLORED)
	{
	  vpRGBa Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    unsigned char pixelgrey = 0.2126 * Ipixelplan.R + 0.7152 * Ipixelplan.G + 0.0722 * Ipixelplan.B;
	    I[i][j]=pixelgrey;
	    nb_point_dessine++;
	  }
	}
      }
    }
  }
}

/*!
  Get the view of the virtual camera. Be carefull, the image I is modified. The projected image is not added as an overlay!
  
  To take into account the projection of several images, a matrix \f$ zBuffer \f$ is given as argument. This matrix contains the z coordinates of all the pixel of the image \f$ I \f$ in the camera frame. During the projection, the pixels are updated if there is no other plan between the camera and the projected image. The matrix \f$ zBuffer \f$ is updated in this case.
  
  \param I : The image used to store the result.
  \param cam : The parameters of the virtual camera.
  \param zBuffer : A matrix containing the z coordinates of the pixels of the image \f$ I \f$
*/
void
vpImageSimulator::getImage(vpImage<unsigned char> &I, const vpCameraParameters cam, vpMatrix &zBuffer)
{
  if (I.getWidth() != (unsigned int)zBuffer.getCols() || I.getHeight() != (unsigned int)zBuffer.getRows())
    throw (vpMatrixException(vpMatrixException::incorrectMatrixSizeError, " zBuffer must have the same size as the image I ! "));
  
  int nb_point_dessine = 0;
  if(visible)
  {
    for (unsigned int i = 0; i < I.getHeight(); i++)
    {
      for (unsigned int j = 0; j < I.getWidth(); j++)
      {
        double x,y;
        vpPixelMeterConversion::convertPoint(cam,vpImagePoint(i,j), x,y);
	if (colorI == GRAY_SCALED)
	{
	  unsigned char Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    if (Xinter_optim[2] < zBuffer[i][j] || zBuffer[i][j] < 0)
	    {
	      I[i][j]=Ipixelplan;
	      nb_point_dessine++;
	      zBuffer[i][j] = Xinter_optim[2];
	    }
	  }
	}
	else if (colorI == COLORED)
	{
	  vpRGBa Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    if (Xinter_optim[2] < zBuffer[i][j] || zBuffer[i][j] < 0)
	    {
	      unsigned char pixelgrey = 0.2126 * Ipixelplan.R + 0.7152 * Ipixelplan.G + 0.0722 * Ipixelplan.B;
	      I[i][j]=pixelgrey;
	      nb_point_dessine++;
	      zBuffer[i][j] = Xinter_optim[2];
	    }
	  }
	}
      }
    }
  }
}

/*!
  Get the view of the virtual camera. Be carefull, the image I is modified. The projected image is not added as an overlay!
  
  \param I : The image used to store the result.
  \param cam : The parameters of the virtual camera.
*/
void
vpImageSimulator::getImage(vpImage<vpRGBa> &I, const vpCameraParameters cam)
{
  int nb_point_dessine = 0;
  if(visible)
  {
    for (unsigned int i = 0; i < I.getHeight(); i++)
    {
      for (unsigned int j = 0; j < I.getWidth(); j++)
      {
        double x,y;//coordonnees dans plan image
        vpPixelMeterConversion::convertPoint(cam,vpImagePoint(i,j), x,y);
	if (colorI == GRAY_SCALED)
	{
	  unsigned char Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    vpRGBa pixelcolor;
	    pixelcolor.R = Ipixelplan;
	    pixelcolor.G = Ipixelplan;
	    pixelcolor.B = Ipixelplan;
	    I[i][j] = pixelcolor;
	    nb_point_dessine++;
	  }
	}
	else if (colorI == COLORED)
	{
	  vpRGBa Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    I[i][j] = Ipixelplan;
	    nb_point_dessine++;
	  }
	}
      }
    }
  }
}

/*!
  Get the view of the virtual camera. Be carefull, the image I is modified. The projected image is not added as an overlay!
  
  To take into account the projection of several images, a matrix \f$ zBuffer \f$ is given as argument. This matrix contains the z coordinates of all the pixel of the image \f$ I \f$ in the camera frame. During the projection, the pixels are updated if there is no other plan between the camera and the projected image. The matrix \f$ zBuffer \f$ is updated in this case.
  
  \param I : The image used to store the result.
  \param cam : The parameters of the virtual camera.
  \param zBuffer : A matrix containing the z coordinates of the pixels of the image \f$ I \f$
*/
void
vpImageSimulator::getImage(vpImage<vpRGBa> &I, const vpCameraParameters cam, vpMatrix &zBuffer)
{
  if (I.getWidth() != (unsigned int)zBuffer.getCols() || I.getHeight() != (unsigned int)zBuffer.getRows())
    throw (vpMatrixException(vpMatrixException::incorrectMatrixSizeError, " zBuffer must have the same size as the image I ! "));
  
  int nb_point_dessine = 0;
  if(visible)
  {
    for (unsigned int i = 0; i < I.getHeight(); i++)
    {
      for (unsigned int j = 0; j < I.getWidth(); j++)
      {
        double x,y;//coordonnees dans plan image
        vpPixelMeterConversion::convertPoint(cam,vpImagePoint(i,j), x,y);
	if (colorI == GRAY_SCALED)
	{
	  unsigned char Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    if (Xinter_optim[2] < zBuffer[i][j] || zBuffer[i][j] < 0)
	    {
	      vpRGBa pixelcolor;
	      pixelcolor.R = Ipixelplan;
	      pixelcolor.G = Ipixelplan;
	      pixelcolor.B = Ipixelplan;
	      I[i][j] = pixelcolor;
	      nb_point_dessine++;
	      zBuffer[i][j] = Xinter_optim[2];
	    }
	  }
	}
	else if (colorI == COLORED)
	{
	  vpRGBa Ipixelplan;
	  if(getPixel(vpImagePoint(y,x),Ipixelplan))
	  {
	    if (Xinter_optim[2] < zBuffer[i][j] || zBuffer[i][j] < 0)
	    {
	      I[i][j] = Ipixelplan;
	      nb_point_dessine++;
	      zBuffer[i][j] = Xinter_optim[2];
	    }
	  }
	}
      }
    }
  }
}


/*!
  Enable to set the position of the 3D plane relative to the virtual camera.
  
  \param _cMt : The pose of the plane relative to the virtual camera.
*/
void
vpImageSimulator::setCameraPosition(const vpHomogeneousMatrix &_cMt)
{
  cMt = _cMt;
  vpRotationMatrix R;
  cMt.extract(R);

  normal_Cam = R * normal_obj;	
  vpColVector focal(3);
  focal=0;
  focal[2]=1;
  
  visible_result = vpColVector::dotProd(normal_Cam,focal);

  if (visible_result>0)
    visible=true;
  else 
    visible=false;

  if(visible)
  {
    for(int i = 0; i < 4; i++)
      project(X[i],cMt,X2[i]);

    vbase_u = X2[1]-X2[0];
    vbase_v = X2[3]-X2[0];

    distance = vpColVector::dotProd(normal_Cam,X2[1]);
    
    if(distance < 0)
      visible = false;

    for(int i = 0; i < 3; i++)
    {
      normal_Cam_optim[i] = normal_Cam[i];
      X0_2_optim[i] = X2[0][i];
      vbase_u_optim[i] = vbase_u[i];
      vbase_v_optim[i] = vbase_v[i];
    }
    
    vpImagePoint iPa[4];
    for(int i = 0; i < 4; i++)
    {
      iPa[i].set_j(X2[i][0]/X2[i][2]);
      iPa[i].set_i(X2[i][1]/X2[i][2]);
    }
    
    T1.buildFrom(iPa[0],iPa[1],iPa[3]);
    T2.buildFrom(iPa[2],iPa[1],iPa[3]);
  }
}

void
vpImageSimulator::initPlan(vpColVector* _X)
{
  for (int i = 0; i < 4; i++)
    X[i]=_X[i];

  normal_obj=vpColVector::crossProd(X[1]-X[0],X[3]-X[0]);
  normal_obj=normal_obj/normal_obj.euclideanNorm();

  euclideanNorm_u=(X[1]-X[0]).euclideanNorm();
  euclideanNorm_v=(X[3]-X[0]).euclideanNorm();
}

/*!
  Initialise the image thanks to an image \f$ I \f$ and a table of vector containing the 3D coordinates of the image's corners.
  
  The table must have a size of 4!
  
  - \f$ X[0] \f$ :Top left corner.
  - \f$ X[1] \f$ :Top right corner.
  - \f$ X[2] \f$ :Bottom right corner.
  - \f$ X[3] \f$ :Bottom left corner.
  
  \param I : The image which is projected.
  \param _X : table of the 3D coordinates corresponding to the image corners.
*/
void
vpImageSimulator::init(const vpImage<unsigned char> &I,vpColVector* _X)
{
  Ig = I;
  vpImageConvert::convert(I,Ic);
  initPlan(_X);
}

/*!
  Initialise the image thanks to an image \f$ I \f$ and a table of vector containing the 3D coordinates of the image's corners.
  
  The table must have a size of 4!
  
  - \f$ X[0] \f$ :Top left corner.
  - \f$ X[1] \f$ :Top right corner.
  - \f$ X[2] \f$ :Bottom right corner.
  - \f$ X[3] \f$ :Bottom left corner.
  
  \param I : The image which is projected.
  \param _X : table of the 3D coordinates corresponding to the image corners.
*/
void
vpImageSimulator::init(const vpImage<vpRGBa> &I,vpColVector* _X)
{
  Ic = I;
  vpImageConvert::convert(I,Ig);
  initPlan(_X);
}

/*!
  Initialise the image thanks to an image whose adress is given by \f$ file_image \f$ and a table of vector containing the 3D coordinates of the image's corners.
  
  The table must have a size of 4!
  
  - \f$ X[0] \f$ :Top left corner.
  - \f$ X[1] \f$ :Top right corner.
  - \f$ X[2] \f$ :Bottom right corner.
  - \f$ X[3] \f$ :Bottom left corner.
  
  \param file_image : The adress of an image file.
  \param _X : table of the 3D coordinates corresponding to the image corners.
*/
void
vpImageSimulator::init(const char* file_image,vpColVector* _X)
{
  vpImageIo::read(Ig,file_image);
  vpImageIo::read(Ic,file_image);
  initPlan(_X);
}

bool
vpImageSimulator::getPixel(const vpImagePoint iP,unsigned char &Ipixelplan)
{
  //test si pixel dans zone projetee
  if(!T1.inTriangle(iP) && !T2.inTriangle(iP))
    return false;

  //methoed algebrique
  double z;

  //calcul de la profondeur de l'intersection
  z = distance/(normal_Cam_optim[0]*iP.get_u()+normal_Cam_optim[1]*iP.get_v()+normal_Cam_optim[2]);
  //calcul coordonnees 3D intersection
  Xinter_optim[0]=iP.get_u()*z;
  Xinter_optim[1]=iP.get_v()*z;
  Xinter_optim[2]=z;

  //recuperation des coordonnes de l'intersection dans le plan objet
  //repere plan object : 
  //	centre = X0_2_optim[i] (premier point definissant le plan)
  //	base =  u:(X[1]-X[0]) et v:(X[3]-X[0])
  //ici j'ai considere que le plan est un rectangle => coordonnees sont simplement obtenu par un produit scalaire
  double u = 0, v = 0;
  double diff = 0;
  for(int i = 0; i < 3; i++)
  {
    diff = (Xinter_optim[i]-X0_2_optim[i]);
    u += diff*vbase_u_optim[i];
    v += diff*vbase_v_optim[i];
  }
  u = u/(euclideanNorm_u*euclideanNorm_u);
  v = v/(euclideanNorm_v*euclideanNorm_v);

  if( u > 0 && v > 0 && u < 1. && v < 1.)
  {
    double i2,j2;
    i2=v*(Ig.getHeight()-1);
    j2=u*(Ig.getWidth()-1);
    if (interp == BILINEAR_INTERPOLATION)
      Ipixelplan = Ig.getValue(i2,j2);
    else if (interp == SIMPLE)
      Ipixelplan = Ig[(int)i2][(int)j2];
    return true;
  }
  else
    return false;
}


bool
vpImageSimulator::getPixel(const vpImagePoint iP,vpRGBa &Ipixelplan)
{
  //test si pixel dans zone projetee
  if(!T1.inTriangle(iP) && !T2.inTriangle(iP))
    return false;

  //methoed algebrique
  double z;

  //calcul de la profondeur de l'intersection
  z = distance/(normal_Cam_optim[0]*iP.get_u()+normal_Cam_optim[1]*iP.get_v()+normal_Cam_optim[2]);
  //calcul coordonnees 3D intersection
  Xinter_optim[0]=iP.get_u()*z;
  Xinter_optim[1]=iP.get_v()*z;
  Xinter_optim[2]=z;

  //recuperation des coordonnes de l'intersection dans le plan objet
  //repere plan object : 
  //	centre = X0_2_optim[i] (premier point definissant le plan)
  //	base =  u:(X[1]-X[0]) et v:(X[3]-X[0])
  //ici j'ai considere que le plan est un rectangle => coordonnees sont simplement obtenu par un produit scalaire
  double u = 0, v = 0;
  double diff = 0;
  for(int i = 0; i < 3; i++)
  {
    diff = (Xinter_optim[i]-X0_2_optim[i]);
    u += diff*vbase_u_optim[i];
    v += diff*vbase_v_optim[i];
  }
  u = u/(euclideanNorm_u*euclideanNorm_u);
  v = v/(euclideanNorm_v*euclideanNorm_v);

  if( u > 0 && v > 0 && u < 1. && v < 1.)
  {
    double i2,j2;
    i2=v*(Ic.getHeight()-1);
    j2=u*(Ic.getWidth()-1);
    if (interp == BILINEAR_INTERPOLATION)
      Ipixelplan = Ic.getValue(i2,j2);
    else if (interp == SIMPLE)
      Ipixelplan = Ic[(int)i2][(int)j2];
    return true;
  }
  else
    return false;
}

bool 
vpImageSimulator::getPixelDepth(const vpImagePoint iP,double &Zpixelplan)
{
  //test si pixel dans zone projetee
  if(!T1.inTriangle(iP) && !T2.inTriangle(iP))
    return false;

  Zpixelplan = distance/(normal_Cam_optim[0]*iP.get_u()+normal_Cam_optim[1]*iP.get_v()+normal_Cam_optim[2]);
  return true;
}

bool
vpImageSimulator::getPixelVisibility(const vpImagePoint iP,double &Visipixelplan)
{
  //test si pixel dans zone projetee
  if(!T1.inTriangle(iP) && !T2.inTriangle(iP))
    return false;
  
  Visipixelplan = visible_result;
  return true;
}

void
vpImageSimulator::project(const vpColVector &_vin, const vpHomogeneousMatrix &_cMt,vpColVector &_vout)
{
  vpColVector XH(4);
  getHomogCoord(_vin,XH);
  getCoordFromHomog(_cMt*XH,_vout);
}

void
vpImageSimulator::getHomogCoord(const vpColVector &_v,vpColVector &_vH)
{
  for(int i=0;i<3;i++)
    _vH[i]=_v[i];
  _vH[3]=1.;	
}

void
vpImageSimulator::getCoordFromHomog(const vpColVector &_vH,vpColVector &_v)
{
  for(int i=0;i<3;i++)
    _v[i]=_vH[i]/_vH[3];
}