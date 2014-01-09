/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/

#include <limits>   // numeric_limits

#include <visp/vpConfig.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <visp/vpTemplateTrackerZone.h>


vpTemplateTrackerZone::vpTemplateTrackerZone()
{
  nb_sommets_max = 100;
  min_x=-1;
  min_y=-1;
  max_x=-1;
  max_y=-1;
  nb_sommets=0;
  nb_sommets_diff=0;
  nb_tout_sommets=0;

  corresp_sommet=new unsigned int[nb_sommets_max];
  liste_sommets=new vpTemplateTrackerZPoint[nb_sommets_max];
  liste_tout_sommets=new vpTemplateTrackerZPoint[nb_sommets_max];
  liste_tout_sommets_warp=new vpTemplateTrackerZPoint[nb_sommets_max];
}

vpTemplateTrackerZone::vpTemplateTrackerZone(const vpTemplateTrackerZone &tz)
{
  nb_sommets_max = 100;
  min_x=-1;
  min_y=-1;
  max_x=-1;
  max_y=-1;
  nb_sommets=0;
  nb_sommets_diff=0;
  nb_tout_sommets=0;

  corresp_sommet = NULL;
  liste_sommets = NULL;
  liste_tout_sommets = NULL;
  liste_tout_sommets_warp = NULL;

  *this = tz;
}

void vpTemplateTrackerZone::clear()
{
  Zone.clear();
  liste_pt.clear();
  if (corresp_sommet) {
    delete [] corresp_sommet; corresp_sommet = NULL;
  }
  if (liste_sommets) {
    delete [] liste_sommets; liste_sommets = NULL;
  }
  if (liste_tout_sommets) {
    delete [] liste_tout_sommets; liste_tout_sommets = NULL;
  }
  if (liste_tout_sommets_warp) {
    delete [] liste_tout_sommets_warp; liste_tout_sommets_warp = NULL;
  }
  this->nb_sommets = 0;
  this->nb_sommets_diff = 0;
  this->nb_tout_sommets = 0;
}

vpTemplateTrackerZone & vpTemplateTrackerZone::operator=(const vpTemplateTrackerZone &tz)
{
  this->nb_sommets_max = tz.nb_sommets_max;
  this->min_x = tz.min_x;
  this->min_y = tz.min_y;
  this->max_x = tz.max_x;
  this->max_y = tz.max_y;

  clear();

  if (tz.corresp_sommet)
    this->corresp_sommet = new unsigned int[nb_sommets_max];
  if (tz.liste_sommets)
    this->liste_sommets = new vpTemplateTrackerZPoint[nb_sommets_max];
  if (tz.liste_tout_sommets)
    this->liste_tout_sommets = new vpTemplateTrackerZPoint[nb_sommets_max];
  if (tz.liste_tout_sommets_warp) {
    this->liste_tout_sommets_warp = new vpTemplateTrackerZPoint[nb_sommets_max];
  }

  this->copy(tz);
  return (*this);
}
void vpTemplateTrackerZone::initClick(const vpImage<unsigned char> &I, bool delaunay)
{
  Zone.clear();
  liste_pt.clear();

  std::vector<vpImagePoint> vip;

  bool end = false;

  do {
    vpImagePoint p;
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, p, button, false) ) {
      vip.push_back(p);

      vpDisplay::displayCross(I, p, 7, vpColor::red);

      if (vip.size() > 1) {
        if (delaunay) {
          // Draw a line between the 2 last points
          vpDisplay::displayLine(I, p, vip[vip.size()-2], vpColor::blue, 3);
        }
        else {
          if(vip.size() % 3 ==2)
            // draw line between point 2-1
            vpDisplay::displayLine(I, p, vip[vip.size()-2], vpColor::blue, 3);
          else if(vip.size() % 3 ==0) {
            // draw line between point 3-2
            vpDisplay::displayLine(I, p, vip[vip.size()-2], vpColor::blue, 3);
            // draw line between point 3-1
            vpDisplay::displayLine(I, p, vip[vip.size()-3], vpColor::blue, 3);
          }

        }
      }
      vpDisplay::flush(I);

      if (button == vpMouseButton::button3)
        end = true;
    }

    vpTime::wait(20);
  } while(!end);

  initFromPoints(I, vip, delaunay);
}


/*!

  Initialize the zone using a vector of image points.

  \param I : Image to process.
  \param vip : Vector of image points used as initialization.
  \param delaunay :
  - If true, a Delaunay triangulation is perfomed on the vector of image points. This functionality is only available
    if ViSP is build with OpenCV >2.3 third-party.
  - If false, the vector of image points describe triangles. Its size is then a multiple of 3.
 */

//create an area with a pointer of integer
void vpTemplateTrackerZone::initFromPoints(const vpImage<unsigned char>& I, const std::vector< vpImagePoint > &vip, bool delaunay)
{
  if (delaunay) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020300
    // Init Delaunay
    cv::Subdiv2D subdiv(cv::Rect(0, 0, (int)I.getWidth(), (int)I.getHeight()));
    for(size_t i=0; i< vip.size(); i++) {
      cv::Point2f fp((float)vip[i].get_u(), (float)vip[i].get_v());
      std::cout << "Click point: " << vip[i] << std::endl;
      subdiv.insert(fp);
    }

    // Compute Delaunay triangulation
    std::vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);

    // Keep only the Delaunay points that are inside the area
    vpRect rect(vip);

    std::vector<vpImagePoint> vip_delaunay;

    for( size_t i = 0; i < triangleList.size(); i++ ) {
      cv::Vec6f t = triangleList[i];
      std::vector<vpImagePoint> p(3);

      p[0].set_uv(t[0], t[1]);
      p[1].set_uv(t[2], t[3]);
      p[2].set_uv(t[4], t[5]);
      if (p[0].inRectangle(rect) && p[1].inRectangle(rect) && p[2].inRectangle(rect)) {
        vip_delaunay.push_back(p[0]);
        vip_delaunay.push_back(p[1]);
        vip_delaunay.push_back(p[2]);
      }
    }

    initFromPoints(I, vip_delaunay, false);
#else
    throw vpException(vpException::functionNotImplementedError,"Delaunay triangulation is not available!");
#endif
  }
  else {
    vpTemplateTrackerZPoint apt;
    Zone.clear();
    liste_pt.clear();

    for(size_t i=0;i<vip.size();i++)
    {
      apt.x=(int)vip[i].get_u();
      apt.y=(int)vip[i].get_v();
      liste_pt.push_back(apt);
      /*liste_tout_sommets[i].x=pt[2*i];
      liste_tout_sommets[i].y=pt[2*i+1];*/
    }
    initFromListPoints(I);
    //nb_tout_sommets=nb_pt;
  }
}

//create an area with a list of points
void vpTemplateTrackerZone::initFromListPoints(const vpImage<unsigned char>& I)
{
  std::vector<vpTemplateTrackerZPoint>::iterator Iterateur_pt;
  //vpTemplateTrackerZPoint pt;
  int x[3];
  int y[3];

  int cpt=0;
  nb_tout_sommets=(unsigned int)liste_pt.size();
  int pb_nb=0;

  for(Iterateur_pt=liste_pt.begin();Iterateur_pt!=liste_pt.end();Iterateur_pt++)
  {
    liste_tout_sommets[pb_nb].x=Iterateur_pt->x;
    liste_tout_sommets[pb_nb].y=Iterateur_pt->y;
    pb_nb++;

    x[cpt]=Iterateur_pt->x;
    y[cpt]=Iterateur_pt->y;

    bool existe_deja=false;
    unsigned int sommet_trouve=0;
    if(nb_sommets_diff!=0)
      for(unsigned int st=0;st<nb_sommets_diff;st++)
      {
        if((liste_sommets[st].x-Iterateur_pt->x)*(liste_sommets[st].x-Iterateur_pt->x)+(liste_sommets[st].y-Iterateur_pt->y)*(liste_sommets[st].y-Iterateur_pt->y)<500)
        {
          sommet_trouve=st;
          existe_deja=true;
        }
      }
    if(!existe_deja)
    {
      liste_sommets[nb_sommets_diff]=*Iterateur_pt;
      corresp_sommet[nb_sommets]=nb_sommets_diff;
      nb_sommets_diff++;
    }
    else
    {
      corresp_sommet[nb_sommets]=sommet_trouve;
    }
    nb_sommets++;

    cpt++;
    if(cpt==3)
    {
      cpt=0;
      vpTemplateTrackerTriangle Triangle(x[0],y[0],x[1],y[1],x[2],y[2]);
      if((Triangle.getMinx()<min_x)||(min_x==-1))
        min_x=(int)Triangle.getMinx();
      if((Triangle.getMaxx()>max_x)||(max_x==-1))
        max_x=(int)Triangle.getMaxx();
      if((Triangle.getMiny()<min_y)||(min_y==-1))
        min_y=(int)Triangle.getMiny();
      if((Triangle.getMaxy()>max_y)||(max_y==-1))
        max_y=(int)Triangle.getMaxy();

      vpDisplay::displayLine(I,y[0],x[0],y[1],x[1],vpColor::green,1);
      vpDisplay::displayLine(I,y[1],x[1],y[2],x[2],vpColor::green,1);
      vpDisplay::displayLine(I,y[2],x[2],y[0],x[0],vpColor::green,1);
      vpDisplay::flush(I) ;

      add(Triangle);
    }
  }
  // 	std::cout<<"Texte pour garder la meme initialisation"<<std::endl;
  // 	std::cout<<"int Pointeur_pt["<<2*pb_nb<<"]= {";
  // 	int cpt2=0;
  // 	for(Iterateur_pt=liste_pt.begin();Iterateur_pt!=liste_pt.end();Iterateur_pt++)
  // 	{
  // 		cpt2++;
  // 		std::cout<<Iterateur_pt->x<<","<<Iterateur_pt->y;
  // 		if(cpt2!=cpt)std::cout<<",";
  // 	}
  // 	std::cout<<"};"<<std::endl;;
  // 	std::cout<<"int nbSommetZone="<<pb_nb<<";"<<std::endl;;

}

//add a triangle to the area
void vpTemplateTrackerZone::add(const vpTemplateTrackerTriangle &T)
{
  Zone.push_back(T);
}

//add a triangle to the area and update the lists of corners
void vpTemplateTrackerZone::add_new(const vpTemplateTrackerTriangle &T)
{
  Zone.push_back(T);
  for(unsigned int i=0;i<3;i++)
  {
    vpColVector S;
    S=T.getCorner(i);//recupere sommet i
    liste_tout_sommets[nb_tout_sommets].x=vpMath::round(S[0]);
    liste_tout_sommets[nb_tout_sommets].y=vpMath::round(S[1]);

    bool existe_deja=false;
    unsigned int sommet_trouve = 0;
    for(unsigned int is=0;is<nb_sommets_diff;is++)
    {
      if(liste_sommets[is].x==vpMath::round(S[0]) && liste_sommets[is].y==vpMath::round(S[1]))
      {
        existe_deja=true;
        sommet_trouve=is;
      }
    }
    if(!existe_deja)
    {
      liste_sommets[nb_sommets_diff].x=vpMath::round(S[0]);
      liste_sommets[nb_sommets_diff].y=vpMath::round(S[1]);
      corresp_sommet[nb_tout_sommets]=nb_sommets_diff;
      nb_sommets_diff++;
    }
    else
    {
      corresp_sommet[nb_tout_sommets]=sommet_trouve;
    }
    nb_tout_sommets++;


  }

}

bool vpTemplateTrackerZone::inZone(const int &ie, const int &je) const
{

  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for(Iterateurvecteur=Zone.begin();Iterateurvecteur!=Zone.end();Iterateurvecteur++)
  {
    if(Iterateurvecteur->inTriangle(ie,je))
      return true;
  }
  return false;

}
bool vpTemplateTrackerZone::inZone(const double &ie,const double &je) const
{

  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for(Iterateurvecteur=Zone.begin();Iterateurvecteur!=Zone.end();Iterateurvecteur++)
  {
    if(Iterateurvecteur->inTriangle(ie,je))
      return true;
  }
  return false;

}
bool vpTemplateTrackerZone::inZone(const int &ie,const int &je, unsigned int &id_triangle) const
{
  unsigned int id=0;
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for(Iterateurvecteur=Zone.begin();Iterateurvecteur!=Zone.end();Iterateurvecteur++)
  {
    if(Iterateurvecteur->inTriangle(ie,je))
    {
      id_triangle=id;
      return true;
    }
    id++;
  }
  return false;

}
bool vpTemplateTrackerZone::inZone(const double &ie,const double &je, unsigned int &id_triangle) const
{

  unsigned int id=0;
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for(Iterateurvecteur=Zone.begin();Iterateurvecteur!=Zone.end();Iterateurvecteur++)
  {
    if(Iterateurvecteur->inTriangle(ie,je))
    {
      id_triangle=id;
      return true;
    }
    id++;
  }
  return false;

}

//recup�re un triangle dans la liste
void vpTemplateTrackerZone::getTriangle(int i,vpTemplateTrackerTriangle &T) const
{
  int id=0;
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for(Iterateurvecteur=Zone.begin();Iterateurvecteur!=Zone.end();Iterateurvecteur++)
  {
    if(id==i)
    {
      T = (*Iterateurvecteur);
      break;
    }
    id++;
  }

}
/*!
  Return the position of the center of gravity.
  \exception vpException::divideByZeroError: The size of the zone is null.
 */
vpImagePoint vpTemplateTrackerZone::getCenter() const
{
  double xc=0;
  double yc=0;
  int cpt=0;
  for(int i=min_y;i<max_y;i++)
    for(int j=min_x;j<max_x;j++)
      if(inZone(i,j))
      {
        xc+=j;
        yc+=i;
        cpt ++;
      }
  if(! cpt) {
    throw(vpException(vpException::divideByZeroError,
		      "Cannot compute the zone center: size = 0")) ;
  }
  xc=xc/cpt;
  yc=yc/cpt;
  vpImagePoint ip;
  ip.set_uv(xc, yc);
  return ip;
}

int vpTemplateTrackerZone::getMaxx() const
{
  return max_x;
}
int vpTemplateTrackerZone::getMaxy() const
{
  return max_y;
}
int vpTemplateTrackerZone::getMinx() const
{
  return min_x;
}
int vpTemplateTrackerZone::getMiny() const
{
  return min_y;
}
int vpTemplateTrackerZone::getInterpol(double i,double j, unsigned int sommet[3],double coeff[3]) const
{
  //trouve triangle et ses sommets
  unsigned int triangle ;
  double ie=i,je=j;
  if(inZone(ie,je,triangle))
  {
    vpTemplateTrackerZPoint sommets_triangle[3];
    for(unsigned int s=0;s<3;s++)
    {
      sommet[s]=corresp_sommet[3*triangle+s];
      sommets_triangle[s]=liste_sommets[sommet[s]];
    }
    //trouve coordonnees pt dans S2S1, S2S3
    vpMatrix S2S1S2S3(2,2);
    S2S1S2S3[0][0]=sommets_triangle[0].x-sommets_triangle[1].x;
    S2S1S2S3[1][0]=sommets_triangle[0].y-sommets_triangle[1].y;
    S2S1S2S3[0][1]=sommets_triangle[2].x-sommets_triangle[1].x;
    S2S1S2S3[1][1]=sommets_triangle[2].y-sommets_triangle[1].y;
    vpColVector S2p(2);
    S2p[0]=j-sommets_triangle[1].x;
    S2p[1]=i-sommets_triangle[1].y;
    vpColVector a1ap(2);a1ap=S2S1S2S3.inverseByLU()*S2p;
    double alpha1=a1ap[0];//if(alpha1<0)alpha1=0;
    double alpha2=a1ap[1]/(1.-a1ap[0]);//if(alpha2<0)alpha2=0;
    //if(1.-a1ap[0]==0.)
    if (std::fabs(1.-a1ap[0]) <= std::numeric_limits<double>::epsilon())
      alpha2=0.;
    //coeff[0]=(1.-alpha2)*alpha1+alpha2*alpha1;

    //std::cout<<"a1 :"<<alpha1<<"  ; a2=  "<<alpha2<<std::endl;

    coeff[0]=alpha1;
    // if(1.-alpha1!=0.)
    if (std::fabs(1.-alpha1) > std::numeric_limits<double>::epsilon())
    {
      coeff[1]=(1.-alpha2)*(1.-alpha1);
      coeff[2]=alpha2*(1.-alpha1);
    }
    else
    {
      coeff[1]=0;
      coeff[2]=0;
    }
    return 1;
  }
  else
  {
    for(int s=0;s<3;s++)
    {
      sommet[s]=0;
      coeff[s]=0;
    }
    return 0;
  }

}
int vpTemplateTrackerZone::getInterpol(double i,double j,unsigned int sommet[3],float coeff[3]) const
{
  //trouve triangle et ses sommets
  unsigned int triangle ;
  double ie=i,je=j;
  if(inZone(ie,je,triangle))
  {
    vpTemplateTrackerZPoint sommets_triangle[3];
    for(unsigned int s=0;s<3;s++)
    {
      sommet[s]=corresp_sommet[3*triangle+s];
      sommets_triangle[s]=liste_sommets[sommet[s]];
    }
    //trouve coordonnees pt dans S2S1, S2S3
    vpMatrix S2S1S2S3(2,2);
    S2S1S2S3[0][0]=sommets_triangle[0].x-sommets_triangle[1].x;
    S2S1S2S3[1][0]=sommets_triangle[0].y-sommets_triangle[1].y;
    S2S1S2S3[0][1]=sommets_triangle[2].x-sommets_triangle[1].x;
    S2S1S2S3[1][1]=sommets_triangle[2].y-sommets_triangle[1].y;
    vpColVector S2p(2);
    S2p[0]=j-sommets_triangle[1].x;
    S2p[1]=i-sommets_triangle[1].y;
    vpColVector a1ap(2);a1ap=S2S1S2S3.inverseByLU()*S2p;
    float alpha1=(float)a1ap[0];//if(alpha1<0)alpha1=0;
    float alpha2=(float)(a1ap[1]/(1.-a1ap[0]));//if(alpha2<0)alpha2=0;
    //coeff[0]=(1.-alpha2)*alpha1+alpha2*alpha1;

    //std::cout<<"a1 :"<<alpha1<<"  ; a2=  "<<alpha2<<std::endl;

    coeff[0]=alpha1;
    coeff[1]=(float)((1.-alpha2)*(1.-alpha1));
    coeff[2]=(float)(alpha2*(1.-alpha1));
    return 1;
  }
  else
  {
    for(int s=0;s<3;s++)
    {
      sommet[s]=0;
      coeff[s]=0;
    }
    return 0;
  }

}
//EM #ifdef USE_DISPLAY

void vpTemplateTrackerZone::displayReferenceZone(const vpImage<unsigned char> &I, const vpColor &col, const unsigned int thickness)
{
  displayZone(I, liste_tout_sommets, nb_tout_sommets, col, thickness);
}

void vpTemplateTrackerZone::displayReferenceZone(const vpImage<vpRGBa> &I, const vpColor &col, const unsigned int thickness)
{
  displayZone(I, liste_tout_sommets, nb_tout_sommets, col, thickness);
}

void vpTemplateTrackerZone::display(const vpImage<unsigned char> &I, const vpColor &col, const unsigned int thickness)
{
  displayZone(I, liste_tout_sommets_warp, nb_tout_sommets, col, thickness);
}
void vpTemplateTrackerZone::display(const vpImage<vpRGBa> &I, const vpColor &col, const unsigned int thickness)
{
  displayZone(I, liste_tout_sommets_warp, nb_tout_sommets, col, thickness);
}

void vpTemplateTrackerZone::displayZone(const vpImage<unsigned char> &I, vpTemplateTrackerZPoint *list_pt, unsigned int &nb_pts,
                                        const vpColor &col, const unsigned int thickness)
{
  //std::cout<<"nb_pts="<<nb_pts<<std::endl;
  for(unsigned int i=0;i<nb_pts;i+=3)
    //int i=3;
  {
    vpDisplay::displayLine(I,list_pt[i].y,list_pt[i].x,list_pt[i+1].y,list_pt[i+1].x,col,thickness);
    vpDisplay::displayLine(I,list_pt[i].y,list_pt[i].x,list_pt[i+2].y,list_pt[i+2].x,col,thickness);
    vpDisplay::displayLine(I,list_pt[i+2].y,list_pt[i+2].x,list_pt[i+1].y,list_pt[i+1].x,col,thickness);
  }

}
void vpTemplateTrackerZone::displayZone(const vpImage<vpRGBa> &I, vpTemplateTrackerZPoint *list_pt, unsigned int &nb_pts,
                                        const vpColor &col, const unsigned int thickness)
{
  for(unsigned int i=0;i<nb_pts;i+=3)
  {
    vpDisplay::displayLine(I,list_pt[i].y,list_pt[i].x,list_pt[i+1].y,list_pt[i+1].x,col,thickness);
    vpDisplay::displayLine(I,list_pt[i].y,list_pt[i].x,list_pt[i+2].y,list_pt[i+2].x,col,thickness);
    vpDisplay::displayLine(I,list_pt[i+2].y,list_pt[i+2].x,list_pt[i+1].y,list_pt[i+1].x,col,thickness);
  }

}

//void vpTemplateTrackerZone::displayZone_svg(vpDakkarSVGfile *svg,vpColor col,vpTemplateTrackerZPoint *list_pt,int &nb_pts,double width)
//{
//  for(int i=0;i<nb_pts;i+=3)
//  {
//    svg->draw_line(list_pt[i].x,list_pt[i].y,list_pt[i+1].x,list_pt[i+1].y,width,col);
//    svg->draw_line(list_pt[i].x,list_pt[i].y,list_pt[i+2].x,list_pt[i+2].y,width,col);
//    svg->draw_line(list_pt[i+2].x,list_pt[i+2].y,list_pt[i+1].x,list_pt[i+1].y,width,col);
//  }
//
//}

/*void dessineligne(vpImage<vpRGBa> &I, double x1, double y1, double x2, double y2,vpColor col)
{
  double xt,yt,xt2,yt2;
  int nb_segm=20;
  double di=1./nb_segm;
  double i;
  for(int it=0;it<nb_segm;it++)
  {
    i=it*di;
    xt=x1+i*(x2-x1);
    yt=y1+i*(y2-y1);
    xt2=x1+(i+di)*(x2-x1);
    yt2=y1+(i+di)*(y2-y1);
    if(((xt>=0)&&(yt>=0)&&(yt<I.getHeight())&&(xt<I.getWidth()))
           ||((xt2>=0)&&(yt2>=0)&&(yt2<I.getHeight())&&(xt2<I.getWidth())))
      vpDisplay::displayLine(I,vpMath::round(yt),vpMath::round(xt),vpMath::round(yt2),vpMath::round(xt2),col,2);
  }
}
void dessineligne(vpImage<unsigned char> &I, double x1, double y1, double x2, double y2,vpColor col)
{
  double xt,yt,xt2,yt2;
  int nb_segm=20;
  double di=1./nb_segm;
  double i;
  for(int it=0;it<nb_segm;it++)
  {
    i=it*di;
    xt=x1+i*(x2-x1);
    yt=y1+i*(y2-y1);
    xt2=x1+(i+di)*(x2-x1);
    yt2=y1+(i+di)*(y2-y1);
    if(((xt>=0)&&(yt>=0)&&(yt<I.getHeight())&&(xt<I.getWidth()))
           ||((xt2>=0)&&(yt2>=0)&&(yt2<I.getHeight())&&(xt2<I.getWidth())))
      vpDisplay::displayLine(I,vpMath::round(yt),vpMath::round(xt),vpMath::round(yt2),vpMath::round(xt2),col,2);
  }
}
*/
//#endif
vpTemplateTrackerZone::~vpTemplateTrackerZone()
{
  clear();
}

/*!
  Get the corners of a specific triangle describing the zone.
   \param t: index of the triangle.
   \param corners: Corners of the triangle.
 */
void vpTemplateTrackerZone::getCornersTriangle(unsigned int t, unsigned int corners[3]) const
{
  for(unsigned int s=0;s<3;s++)
  {
    corners[s]=corresp_sommet[3*t+s];
  }
}

/*!
   Modify all the pixels inside a triangle with a given gray level.
   \param I: Output image.
   \param t: Triangle id.
   \param gray_level: Color used to fill the triangle with.
 */
void vpTemplateTrackerZone::fillTriangle(vpImage<unsigned char>& I,int t,unsigned int gray_level)
{
  assert(gray_level < 256);
  vpTemplateTrackerTriangle tTriangle;
  getTriangle(t,tTriangle);
  for (int i=0 ; i < (int) I.getHeight() ; i++)
  {
    for (int j=0 ; j < (int) I.getWidth() ; j++)
    {
      if(tTriangle.inTriangle(i,j))
      {
        I[i][j]=gray_level;
      }
    }
  }
}
vpTemplateTrackerZone vpTemplateTrackerZone::getPyramidDown() const
{
  vpTemplateTrackerZone tempZone;
  vpTemplateTrackerTriangle Ttemp;
  vpTemplateTrackerTriangle TtempDown;
  for(int i=0;i<getNbTriangle();i++)
  {
    getTriangle(i,Ttemp);
    TtempDown=Ttemp.getPyramidDown();
    tempZone.add_new(TtempDown);
  }
  return tempZone;

}
void vpTemplateTrackerZone::copy(const vpTemplateTrackerZone& Z)
{
  vpTemplateTrackerTriangle Ttemp;
  for(int i=0;i<Z.getNbTriangle();i++)
  {
    Z.getTriangle(i,Ttemp);
    add_new(Ttemp);
  }
}

/*!
  Return the position of the center of gravity in a given area.
  \param borne_x : Right coordinate of the area to consider.
  \param borne_y : Bottom coordinate of the area to consider.
  \exception vpException::divideByZeroError: The size of the zone is null.
 */

vpImagePoint vpTemplateTrackerZone::getCenter(int borne_x, int borne_y) const
{
  int cpt_pt=0;
  double x_center=0,y_center=0;
  for(int j=0;j<borne_x;j++)
    for(int i=0;i<borne_y;i++)
      if(inZone(i,j))
      {
        x_center+=j;
        y_center+=i;
        cpt_pt++;
      }

  if(! cpt_pt) {
    throw(vpException(vpException::divideByZeroError,
          "Cannot compute the zone center: size = 0")) ;
  }

  x_center=x_center/cpt_pt;
  y_center=y_center/cpt_pt;
  vpImagePoint center;
  center.set_uv(x_center, y_center);
  return center;
}

