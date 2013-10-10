/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
#ifndef vpTemplateTrackerZone_hh
#define vpTemplateTrackerZone_hh

#include <visp/vpException.h>
#include <visp/vpTemplateTrackerTriangle.h>
#include <visp/vpTemplateTrackerHeader.h>
#include <visp/vpImage.h>

class VISP_EXPORT vpTemplateTrackerZone
{
  protected:
    std::vector<vpTemplateTrackerZPoint> liste_pt;
    std::vector<vpTemplateTrackerTriangle> Zone;
    int min_x,min_y,max_x,max_y;
    unsigned int nb_sommets;
    unsigned int nb_sommets_diff;
    unsigned int *corresp_sommet;
    vpTemplateTrackerZPoint *liste_sommets;

    //pour affichage
    unsigned int nb_tout_sommets;
    vpTemplateTrackerZPoint *liste_tout_sommets;
    vpTemplateTrackerZPoint *liste_tout_sommets_warp;

    unsigned int nb_sommets_max;

  protected:
    void clear();
    //create an area with a list of points
    void initFromListPoints(const vpImage<unsigned char>& I);

  private:
    void displayZone(const vpImage<unsigned char> &I,vpTemplateTrackerZPoint *list_pt,unsigned int &nb_pts, const vpColor &col = vpColor::green, const unsigned int thickness=3);
    void displayZone(const vpImage<vpRGBa> &I,vpTemplateTrackerZPoint *list_pt,unsigned int &nb_pts, const vpColor &col = vpColor::green, const unsigned int thickness=3);

  public:
    vpTemplateTrackerZone();
    vpTemplateTrackerZone(const vpTemplateTrackerZone &tz);
    ~vpTemplateTrackerZone();

    void copy(const vpTemplateTrackerZone& Z);
    //create an area by clicking on an image
    void initClick(const vpImage<unsigned char>& I, bool delaunay = false);
    //create an area with a pointer of integer that describes a series of triangles:
    // *pt= t0.S1.x,t0.S1.y,t0.S2.x,t0.S2.y,t0.S3.x,t0.S3.y, t1.S1.x ...
    void initFromPoints(const vpImage<unsigned char>& I, const std::vector< vpImagePoint > &ip, bool delaunay = false);

    //add a triangle to the area
    void add(const vpTemplateTrackerTriangle &T);
    //add a triangle to the area and update the lists of corners
    void add_new(const vpTemplateTrackerTriangle &T);
    //check if a point is in the area
    bool inZone(const int &ie,const int &je) const;
    bool inZone(const double &ie,const double &je) const;
    //check if a point is in the area and return the corresponding triangle id_triangle where the point is.
    bool inZone(const int &ie,const int &je, int &id_triangle) const;
    bool inZone(const double &ie,const double &je, int &id_triangle) const;

    vpImagePoint getCenter() const;
    vpImagePoint getCenter(int borne_x, int borne_y) const;
    //renvoie interpolation lineaire d'un point en fonction des 3 sommets du triangle defini par leurs index dans liste_sommets
    int getInterpol(double i,double j,int sommet[3],double coeff[3]) const;
    int getInterpol(double i,double j,int sommet[3],float coeff[3]) const;
    //get bounds of the area
    //renvoie le pointeur de la liste de sommet
    vpTemplateTrackerZPoint *getListPt() const {return liste_tout_sommets;}
    vpTemplateTrackerZPoint *getListPtWarpes() const {return liste_tout_sommets_warp;}
    int getMaxx() const;
    int getMaxy() const;
    int getMinx() const;
    int getMiny() const;
    unsigned int getNbSommetDiff() const {return nb_sommets_diff;}
    unsigned int getNbToutSommets() const {return nb_tout_sommets;}
    int getNbTriangle() const {return Zone.size();}
    vpTemplateTrackerZone getPyramidDown() const;
    vpTemplateTrackerZPoint getCorner(int i) const {return liste_sommets[i];}
    void getCornerDiff(int n,int &x,int &y) const {x=liste_sommets[n].x;y=liste_sommets[n].y;}
    //renvoie les indices des sommets du tieme triangle
    void getCornersTriangle(int t,int corners[3]) const;
    //renvoie le ieme triangle de la zone
    void getTriangle(int i,vpTemplateTrackerTriangle &T) const;

    //display the area on an image
    void displayReferenceZone(const vpImage<unsigned char> &I, const vpColor &col = vpColor::green, const unsigned int thickness=3);
    void displayReferenceZone(const vpImage<vpRGBa> &I, const vpColor &col = vpColor::green, const unsigned int thickness=3);
    
    void display(const vpImage<unsigned char> &I, const vpColor &col = vpColor::green, const unsigned int thickness=3);
    void display(const vpImage<vpRGBa> &I, const vpColor &col = vpColor::green, const unsigned int thickness=3);

    //colorie le tieme triangle
    void fillTriangle(vpImage<unsigned char>& I,int t,unsigned int gray_level);

    vpTemplateTrackerZone & operator=(const vpTemplateTrackerZone &tz);
};
#endif

