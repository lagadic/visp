/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Image display.
 */

/*!
  \file vpDisplayGTK.cpp
  \brief Define the GTK console to display images.
*/

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_GTK))

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>

// Display stuff
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayGTK.h>

// debug / exception
#include <visp3/core/vpDisplayException.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpMath.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <gdk/gdk.h>
#include <gdk/gdkrgb.h>
#include <gtk/gtk.h>

BEGIN_VISP_NAMESPACE

class vpDisplayGTK::Impl
{
public:
  Impl()
    : m_widget(nullptr), m_background(nullptr), m_gc(nullptr), m_blue(), m_red(), m_yellow(), m_green(), m_cyan(), m_orange(),
    m_white(), m_black(), m_gdkcolor(), m_lightBlue(), m_darkBlue(), m_lightRed(), m_darkRed(), m_lightGreen(),
    m_darkGreen(), m_purple(), m_lightGray(), m_gray(), m_darkGray(), m_colormap(nullptr), m_font(nullptr), m_vectgtk(nullptr),
    m_col(nullptr)
  { }

  ~Impl() { }

  void init(unsigned int win_width, unsigned int win_height, int win_x, int win_y, const std::string &title)
  {
    gint width = static_cast<gint>(win_width);
    gint height = static_cast<gint>(win_height);

    /* Initialisation of the gdk et gdk_rgb library */
    int *argc = nullptr;
    char **argv;

    gtk_init(argc, &argv);

    /* Create the window*/
    m_widget = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    gtk_widget_add_events(m_widget, GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK);

    gtk_window_set_default_size(GTK_WINDOW(m_widget), width, height);

    gtk_window_move(GTK_WINDOW(m_widget), win_x, win_y);

    gtk_widget_show(m_widget);

    gdk_rgb_init();

    /* Create background pixmap */
    m_background = gdk_pixmap_new(m_widget->window, width, height, -1);

    /* Create graphic context */
    m_gc = gdk_gc_new(m_widget->window);

    /* get the colormap  */
    m_colormap = gdk_window_get_colormap(m_widget->window);

    m_col = new GdkColor *[vpColor::id_unknown]; // id_unknown = number of predefined colors

    /* Create color */
    gdk_color_parse("light blue", &m_lightBlue);
    gdk_colormap_alloc_color(m_colormap, &m_lightBlue, FALSE, TRUE);
    m_col[vpColor::id_lightBlue] = &m_lightBlue;

    gdk_color_parse("blue", &m_blue);
    gdk_colormap_alloc_color(m_colormap, &m_blue, FALSE, TRUE);
    m_col[vpColor::id_blue] = &m_blue;

    gdk_color_parse("dark blue", &m_darkBlue);
    gdk_colormap_alloc_color(m_colormap, &m_darkBlue, FALSE, TRUE);
    m_col[vpColor::id_darkBlue] = &m_darkBlue;

    gdk_color_parse("#FF8C8C", &m_lightRed);
    gdk_colormap_alloc_color(m_colormap, &m_lightRed, FALSE, TRUE);
    m_col[vpColor::id_lightRed] = &m_lightRed;

    gdk_color_parse("red", &m_red);
    gdk_colormap_alloc_color(m_colormap, &m_red, FALSE, TRUE);
    m_col[vpColor::id_red] = &m_red;

    gdk_color_parse("dark red", &m_darkRed);
    gdk_colormap_alloc_color(m_colormap, &m_darkRed, FALSE, TRUE);
    m_col[vpColor::id_darkRed] = &m_darkRed;

    gdk_color_parse("light green", &m_lightGreen);
    gdk_colormap_alloc_color(m_colormap, &m_lightGreen, FALSE, TRUE);
    m_col[vpColor::id_lightGreen] = &m_lightGreen;

    gdk_color_parse("green", &m_green);
    gdk_colormap_alloc_color(m_colormap, &m_green, FALSE, TRUE);
    m_col[vpColor::id_green] = &m_green;

    gdk_color_parse("dark green", &m_darkGreen);
    gdk_colormap_alloc_color(m_colormap, &m_darkGreen, FALSE, TRUE);
    m_col[vpColor::id_darkGreen] = &m_darkGreen;

    gdk_color_parse("yellow", &m_yellow);
    gdk_colormap_alloc_color(m_colormap, &m_yellow, FALSE, TRUE);
    m_col[vpColor::id_yellow] = &m_yellow;

    gdk_color_parse("cyan", &m_cyan);
    gdk_colormap_alloc_color(m_colormap, &m_cyan, FALSE, TRUE);
    m_col[vpColor::id_cyan] = &m_cyan;

    gdk_color_parse("orange", &m_orange);
    gdk_colormap_alloc_color(m_colormap, &m_orange, FALSE, TRUE);
    m_col[vpColor::id_orange] = &m_orange;

    gdk_color_parse("purple", &m_purple);
    gdk_colormap_alloc_color(m_colormap, &m_purple, FALSE, TRUE);
    m_col[vpColor::id_purple] = &m_purple;

    gdk_color_parse("white", &m_white);
    gdk_colormap_alloc_color(m_colormap, &m_white, FALSE, TRUE);
    m_col[vpColor::id_white] = &m_white;

    gdk_color_parse("black", &m_black);
    gdk_colormap_alloc_color(m_colormap, &m_black, FALSE, TRUE);
    m_col[vpColor::id_black] = &m_black;

    gdk_color_parse("#C0C0C0", &m_lightGray);
    gdk_colormap_alloc_color(m_colormap, &m_lightGray, FALSE, TRUE);
    m_col[vpColor::id_lightGray] = &m_lightGray;

    gdk_color_parse("#808080", &m_gray);
    gdk_colormap_alloc_color(m_colormap, &m_gray, FALSE, TRUE);
    m_col[vpColor::id_gray] = &m_gray;

    gdk_color_parse("#404040", &m_darkGray);
    gdk_colormap_alloc_color(m_colormap, &m_darkGray, FALSE, TRUE);
    m_col[vpColor::id_darkGray] = &m_darkGray;

    // Try to load a default font
    m_font = gdk_font_load("-*-times-medium-r-normal-*-16-*-*-*-*-*-*-*");
    if (m_font == nullptr)
      m_font = gdk_font_load("-*-courier-bold-r-normal-*-*-140-*-*-*-*-*-*");
    if (m_font == nullptr)
      m_font = gdk_font_load("-*-courier 10 pitch-medium-r-normal-*-16-*-*-*-*-*-*-*");

    if (!title.empty())
      gdk_window_set_title(m_widget->window, title.c_str());
  }

  void setFont(const std::string &fontname) { m_font = gdk_font_load((const gchar *)fontname.c_str()); }

  void setTitle(const std::string &title) { gdk_window_set_title(m_widget->window, title.c_str()); }

  void setWindowPosition(int win_x, int win_y) { gtk_window_move(GTK_WINDOW(m_widget), win_x, win_y); }

  void displayImage(const vpImage<unsigned char> &I, unsigned int scale, gint width, gint height)
  {
    if (scale == 1) {
      /* Copie de l'image dans le pixmap fond */
      gdk_draw_gray_image(m_background, m_gc, 0, 0, width, height, GDK_RGB_DITHER_NONE, I.bitmap, width);
    }
    else {
      vpImage<unsigned char> sampled;
      I.subsample(scale, scale, sampled);
      gdk_draw_gray_image(m_background, m_gc, 0, 0, width, height, GDK_RGB_DITHER_NONE, sampled.bitmap, width);
    }

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(m_widget->window, m_background, FALSE);
  }

  void displayImage(const vpImage<vpRGBa> &I, unsigned int scale, gint width, gint height)
  {
    if (scale == 1) {
      /* Copie de l'image dans le pixmap fond */
      gdk_draw_rgb_32_image(m_background, m_gc, 0, 0, width, height, GDK_RGB_DITHER_NONE, (unsigned char *)I.bitmap,
                            4 * width);
    }
    else {
      vpImage<vpRGBa> sampled;
      I.subsample(scale, scale, sampled);
      gdk_draw_rgb_32_image(m_background, m_gc, 0, 0, width, height, GDK_RGB_DITHER_NONE,
                            (unsigned char *)sampled.bitmap, 4 * width);
    }

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(m_widget->window, m_background, FALSE);
  }

  void displayImageROI(const vpImage<unsigned char> &I, gint j_min, gint i_min, gint width, gint height)
  {
    gdk_draw_gray_image(m_background, m_gc, j_min, i_min, width, height, GDK_RGB_DITHER_NONE, I.bitmap, width);
    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(m_widget->window, m_background, FALSE);
  }

  void displayImageROI(const vpImage<vpRGBa> &I, gint j_min, gint i_min, gint width, gint height)
  {
    gdk_draw_rgb_32_image(m_background, m_gc, j_min, i_min, width, height, GDK_RGB_DITHER_NONE,
                          (unsigned char *)I.bitmap, width * 4);

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(m_widget->window, m_background, FALSE);
  }

  void closeDisplay()
  {
    if (m_col != nullptr) {
      delete[] m_col;
      m_col = nullptr;
    }

    if (m_widget != nullptr) {
      gdk_window_hide(m_widget->window);
      gdk_window_destroy(m_widget->window);
      gtk_widget_destroy(m_widget);
      m_widget = nullptr;
    }
  }

  void flushDisplay()
  {
    gdk_window_clear(m_widget->window);
    gdk_flush();
  }

  void displayText(const vpImagePoint &ip, const std::string &text, const vpColor &color, unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }
    if (m_font != nullptr)
      gdk_draw_string(m_background, m_font, m_gc, vpMath::round(ip.get_u() / scale), vpMath::round(ip.get_v() / scale),
                      (const gchar *)text.c_str());
    else
      std::cout << "Cannot draw string: no font is selected" << std::endl;
  }

  void displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                     unsigned int thickness, unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }

    gdk_gc_set_line_attributes(m_gc, static_cast<gint>(thickness), GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_BEVEL);

    if (fill == false)
      gdk_draw_arc(m_background, m_gc, FALSE, vpMath::round((center.get_u() - radius) / scale),
                   vpMath::round((center.get_v() - radius) / scale), static_cast<gint>(2. * radius / scale),
                   static_cast<gint>(2. * radius / scale), 23040, 23040); /* 23040 = 360*64 */
    else
      gdk_draw_arc(m_background, m_gc, TRUE, vpMath::round((center.get_u() - radius) / scale),
                   vpMath::round((center.get_v() - radius) / scale), static_cast<gint>(2. * radius / scale),
                   static_cast<gint>(2. * radius / scale), 23040, 23040); /* 23040 = 360*64 */
  }

  void displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness,
                      unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }

    gdk_gc_set_line_attributes(m_gc, static_cast<gint>(thickness), GDK_LINE_ON_OFF_DASH, GDK_CAP_BUTT, GDK_JOIN_BEVEL);
    gdk_draw_line(m_background, m_gc, vpMath::round(ip1.get_u() / scale), vpMath::round(ip1.get_v() / scale),
                  vpMath::round(ip2.get_u() / scale), vpMath::round(ip2.get_v() / scale));
    gdk_gc_set_line_attributes(m_gc, 0, GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_BEVEL);
  }

  void displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness,
                   unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }

    gdk_gc_set_line_attributes(m_gc, static_cast<gint>(thickness), GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_BEVEL);
    gdk_draw_line(m_background, m_gc, vpMath::round(ip1.get_u() / scale), vpMath::round(ip1.get_v() / scale),
                  vpMath::round(ip2.get_u() / scale), vpMath::round(ip2.get_v() / scale));
  }

  void displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness, unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }

    if (thickness == 1) {
      gdk_draw_point(m_background, m_gc, vpMath::round(ip.get_u() / scale), vpMath::round(ip.get_v() / scale));
    }
    else {
      gdk_draw_rectangle(m_background, m_gc, TRUE, vpMath::round(ip.get_u() / scale), vpMath::round(ip.get_v() / scale),
                         static_cast<gint>(thickness), static_cast<gint>(thickness));
    }
  }

  void displayRectangle(const vpImagePoint &topLeft, unsigned int w, unsigned int h, const vpColor &color, bool fill,
                        unsigned int thickness, unsigned int scale)
  {
    if (color.id < vpColor::id_unknown)
      gdk_gc_set_foreground(m_gc, m_col[color.id]);
    else {
      m_gdkcolor.red = 256 * color.R;
      m_gdkcolor.green = 256 * color.G;
      m_gdkcolor.blue = 256 * color.B;
      gdk_colormap_alloc_color(m_colormap, &m_gdkcolor, FALSE, TRUE);
      gdk_gc_set_foreground(m_gc, &m_gdkcolor);
    }
    gdk_gc_set_line_attributes(m_gc, static_cast<gint>(thickness), GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_BEVEL);

    if (fill == false)
      gdk_draw_rectangle(m_background, m_gc, FALSE, vpMath::round(topLeft.get_u() / scale),
                         vpMath::round(topLeft.get_v() / scale), static_cast<gint>(w / scale),
                         static_cast<gint>(h / scale));
    else
      gdk_draw_rectangle(m_background, m_gc, TRUE, vpMath::round(topLeft.get_u() / scale),
                         vpMath::round(topLeft.get_v() / scale), static_cast<gint>(w / scale),
                         static_cast<gint>(h / scale));

    if (thickness > 1)
      gdk_gc_set_line_attributes(m_gc, 0, GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_BEVEL);
  }

  bool getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking, unsigned int scale,
                const GdkEventType &event_type)
  {
    bool ret = false;
    do {
      GdkEvent *ev = nullptr;
      while ((ev = gdk_event_get())) {
        if (ev->any.window == m_widget->window && ev->type == event_type) {
          double u = ((GdkEventButton *)ev)->x;
          double v = ((GdkEventButton *)ev)->y;
          ip.set_u(u * scale);
          ip.set_v(v * scale);

          switch (static_cast<int>(((GdkEventButton *)ev)->button)) {
          case 1:
            button = vpMouseButton::button1;
            break;
          case 2:
            button = vpMouseButton::button2;
            break;
          case 3:
            button = vpMouseButton::button3;
            break;
          }
          ret = true;
        }
        gdk_event_free(ev);
      }
      if (blocking) {
        flushDisplay();
        vpTime::wait(100);
      }

    } while (ret == false && blocking == true);
    return ret;
  }

  void getImage(vpImage<vpRGBa> &I, gint width, gint height)
  {
    GdkImage *ImageGtk;
    ImageGtk = gdk_image_get(m_background, 0, 0, width, height);

    I.resize(height, width);
    guint32 pixel;

    guchar OctetRouge, OctetVert, OctetBleu, mask;
    mask = 0x000000FF;

    for (gint y = 0; y < height; ++y) {
      for (gint x = 0; x < width; ++x) {
        pixel = gdk_image_get_pixel(ImageGtk, x, y);
        OctetBleu = static_cast<guchar>(pixel) & mask;
        OctetVert = static_cast<guchar>(pixel >> 8) & mask;
        OctetRouge = static_cast<guchar>(pixel >> 16) & mask;
        I[y][x].R = OctetRouge;
        I[y][x].G = OctetVert;
        I[y][x].B = OctetBleu;
        I[y][x].A = vpRGBa::alpha_default; // default opacity
      }
    }
  }

  unsigned int getScreenDepth() { return static_cast<unsigned int>(gdk_window_get_visual(m_widget->window)->depth); }

  bool getKeyboardEvent(std::string &key, bool blocking)
  {
    bool ret = false;
    int cpt = 0;
    do {
      GdkEvent *ev = nullptr;
      while ((ev = gdk_event_get()) != nullptr) {
        cpt++;

        if (ev->any.window == m_widget->window && ev->type == GDK_KEY_PRESS) {
          key = gdk_keyval_name(ev->key.keyval);
          ret = true;
        }
        gdk_event_free(ev);
      }
      if (blocking) {
        flushDisplay();
        vpTime::wait(100);
      }
    } while (ret == false && blocking == true);
    return ret;
  }

  bool getPointerMotionEvent(vpImagePoint &ip, unsigned int scale)
  {
    bool ret = false;
    GdkEvent *ev = nullptr;
    if ((ev = gdk_event_get())) {
      if (ev->any.window == m_widget->window && ev->type == GDK_MOTION_NOTIFY) {
        double u = ((GdkEventMotion *)ev)->x;
        double v = ((GdkEventMotion *)ev)->y;
        ip.set_u(u * scale);
        ip.set_v(v * scale);

        ret = true;
      }
      gdk_event_free(ev);
    }
    return ret;
  }

  void getPointerPosition(vpImagePoint &ip, unsigned int scale)
  {
    gint u, v;
    gdk_window_get_pointer(m_widget->window, &u, &v, nullptr);
    ip.set_u(static_cast<double>(u) * scale);
    ip.set_v(static_cast<double>(v) * scale);
  }

  void getScreenSize(bool is_init, unsigned int &w, unsigned int &h)
  {
    if (!is_init) {
      int *argc = nullptr;
      char **argv;

      gtk_init(argc, &argv);

      GtkWidget *widget_ = gtk_window_new(GTK_WINDOW_TOPLEVEL);
      gtk_window_set_default_size(GTK_WINDOW(widget_), 100, 100);
      gtk_widget_show(widget_);

      GdkScreen *screen_ = gdk_window_get_screen(widget_->window);
      w = static_cast<unsigned int>(gdk_screen_get_width(screen_));
      h = static_cast<unsigned int>(gdk_screen_get_height(screen_));
      gtk_widget_destroy(widget_);
    }
    else {
      GdkScreen *screen_ = gdk_window_get_screen(m_widget->window);
      w = static_cast<unsigned int>(gdk_screen_get_width(screen_));
      h = static_cast<unsigned int>(gdk_screen_get_height(screen_));
    }
  }

private:
  GtkWidget *m_widget;
  GdkPixmap *m_background;
  GdkGC *m_gc;
  GdkColor m_blue, m_red, m_yellow, m_green, m_cyan, m_orange, m_white, m_black, m_gdkcolor;
  GdkColor m_lightBlue, m_darkBlue, m_lightRed, m_darkRed, m_lightGreen, m_darkGreen, m_purple;
  GdkColor m_lightGray, m_gray, m_darkGray;
  GdkColormap *m_colormap;

  GdkFont *m_font;
  guchar *m_vectgtk;
  GdkColor **m_col;
};

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!

  Constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized).
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is down scaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is down scaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is down scaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is down scaled by 5 along the lines
  and the columns.

*/
vpDisplayGTK::vpDisplayGTK(vpImage<unsigned char> &I, vpScaleType scaleType) : vpDisplay(), m_impl(new Impl())
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I);
}

/*!

  Constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized).
  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is down scaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is down scaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is down scaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is down scaled by 5 along the lines
  and the columns.

*/
vpDisplayGTK::vpDisplayGTK(vpImage<unsigned char> &I, int win_x, int win_y, const std::string &win_title,
                           vpScaleType scaleType)
  : vpDisplay(), m_impl(new Impl())
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I, win_x, win_y, win_title);
}

/*!
  Constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is down scaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is down scaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is down scaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is down scaled by 5 along the lines
  and the columns.
*/
vpDisplayGTK::vpDisplayGTK(vpImage<vpRGBa> &I, vpScaleType scaleType) : vpDisplay(), m_impl(new Impl())
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I);
}

/*!
  Constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is down scaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is down scaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is down scaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is down scaled by 5 along the lines
  and the columns.
*/
vpDisplayGTK::vpDisplayGTK(vpImage<vpRGBa> &I, int win_x, int win_y, const std::string &win_title,
                           vpScaleType scaleType)
  : vpDisplay(), m_impl(new Impl())
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I, win_x, win_y, win_title);
}

/*!

  Constructor that just initialize the display position in the screen
  and the display title.

  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.

  To initialize the display size, you need to call init().

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/gui/vpDisplayGTK.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpDisplayGTK d(100, 200, "My display");
    vpImage<unsigned char> I(240,384);
    d.init(I);
  }
  \endcode
*/
vpDisplayGTK::vpDisplayGTK(int win_x, int win_y, const std::string &win_title) : vpDisplay(), m_impl(new Impl())
{
  m_windowXPosition = win_x;
  m_windowYPosition = win_y;
  m_title = win_title;
}

/*!
  Basic constructor.

  To initialize the window position, title and size you may call
  init(vpImage<unsigned char> &, int, int, const std::string &) or
  init(vpImage<vpRGBa> &, int, int, const std::string &).

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/gui/vpDisplayGTK.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpDisplayGTK d;
    vpImage<unsigned char> I(240,384);
    d.init(I, 100, 200, "My display");
  }
  \endcode
*/
vpDisplayGTK::vpDisplayGTK() : vpDisplay(), m_impl(new Impl()) { }

/*!
  Destructor.
*/
vpDisplayGTK::~vpDisplayGTK()
{
  closeDisplay();
  delete m_impl;
}

/*!
  Initialize the display (size, position and title) of a gray level image.

  \param I : Image to be displayed (not that image has to be initialized)
  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.

*/
void vpDisplayGTK::init(vpImage<unsigned char> &I, int win_x, int win_y, const std::string &win_title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }

  if (win_x != -1)
    m_windowXPosition = win_x;
  if (win_y != -1)
    m_windowYPosition = win_y;

  if (!win_title.empty())
    m_title = win_title;

  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), m_windowXPosition, m_windowYPosition, m_title);

  I.display = this;
  m_displayHasBeenInitialized = true;
}

/*!
  Initialize the display (size, position and title) of a color
  image in RGBa format.

  \param I : Image to be displayed (not that image has to be initialized)
  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.

*/
void vpDisplayGTK::init(vpImage<vpRGBa> &I, int win_x, int win_y, const std::string &win_title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }

  if (win_x != -1)
    m_windowXPosition = win_x;
  if (win_y != -1)
    m_windowYPosition = win_y;

  if (!win_title.empty())
    m_title = win_title;

  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), m_windowXPosition, m_windowYPosition, m_title);

  I.display = this;
  m_displayHasBeenInitialized = true;
}

/*!
  Initialize the display size, position and title.

  \param win_width, win_height : Width and height of the window.
  \param win_x, win_y : The window is set at position (win_x,win_y) with column index and row index respectively.
  \param win_title : Window title.

*/
void vpDisplayGTK::init(unsigned int win_width, unsigned int win_height, int win_x, int win_y,
                        const std::string &win_title)
{
  setScale(m_scaleType, win_width, win_height);

  m_width = win_width / m_scale;
  m_height = win_height / m_scale;

  if (win_x != -1)
    m_windowXPosition = win_x;
  if (win_y != -1)
    m_windowYPosition = win_y;

  m_title = win_title;

  m_impl->init(m_width, m_height, m_windowXPosition, m_windowYPosition, m_title);

  m_displayHasBeenInitialized = true;
}

/*!

  Set the font used to display a text in overlay. The display is
  performed using displayText().

  \param fontname : The expected font name.

  \note Under UNIX, to know all the available fonts, use the
  "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

  \sa displayText()
*/
void vpDisplayGTK::setFont(const std::string &fontname) { m_impl->setFont(fontname); }

/*!
  Set the window title.
  \param title : Window title.
*/
void vpDisplayGTK::setTitle(const std::string &title)
{
  if (m_displayHasBeenInitialized) {
    m_title = title;
    if (!title.empty()) {
      m_impl->setTitle(title);
    }
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Set the window position in the screen.

  \param win_x, win_y : Position of the upper-left window's border in the
  screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplayGTK::setWindowPosition(int win_x, int win_y)
{
  if (m_displayHasBeenInitialized) {
    m_impl->setWindowPosition(win_x, win_y);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(const vpImage<unsigned char> &I)
{
  if (m_displayHasBeenInitialized) {
    m_impl->displayImage(I, m_scale, static_cast<gint>(m_width), static_cast<gint>(m_height));
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a selection of the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.

  \param iP : Top left corner of the region of interest

  \param w : Width of the region of interest

  \param h : Height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int w,
                                   unsigned int h)
{
  if (m_displayHasBeenInitialized) {
    vpImage<unsigned char> Itemp;
    vpImageTools::crop(I, iP.get_i(), iP.get_j(), h, w, Itemp, m_scale, m_scale);

    /* Copie de l'image dans le pixmap fond */
    int i_min = std::max<int>(static_cast<int>(ceil(iP.get_i() / m_scale)), 0);
    int j_min = std::max<int>(static_cast<int>(ceil(iP.get_j() / m_scale)), 0);

    m_impl->displayImageROI(Itemp, static_cast<gint>(j_min), static_cast<gint>(i_min),
                            static_cast<gint>(Itemp.getWidth()), static_cast<gint>(Itemp.getHeight()));
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(const vpImage<vpRGBa> &I)
{
  if (m_displayHasBeenInitialized) {
    m_impl->displayImage(I, m_scale, static_cast<gint>(m_width), static_cast<gint>(m_height));
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a selection of the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.

  \param iP : Top left corner of the region of interest

  \param w : Width of the region of interest

  \param h : Height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, unsigned int w, unsigned int h)
{
  if (m_displayHasBeenInitialized) {
    vpImage<vpRGBa> Itemp;
    vpImageTools::crop(I, iP.get_i(), iP.get_j(), h, w, Itemp, m_scale, m_scale);

    /* Copie de l'image dans le pixmap fond */
    int i_min = std::max<int>(static_cast<int>(ceil(iP.get_i() / m_scale)), 0);
    int j_min = std::max<int>(static_cast<int>(ceil(iP.get_j() / m_scale)), 0);

    m_impl->displayImageROI(Itemp, static_cast<gint>(j_min), static_cast<gint>(i_min),
                            static_cast<gint>(Itemp.getWidth()), static_cast<gint>(Itemp.getHeight()));
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  \warning Not implemented yet.

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(const unsigned char * /* I */)
{
  // not implemented
}

/*!
  Close the window.

  \sa init()
*/
void vpDisplayGTK::closeDisplay()
{
  if (m_displayHasBeenInitialized) {
    m_impl->closeDisplay();

    m_displayHasBeenInitialized = false;
  }
}

/*!
  Flushes the display buffer.
  It's necessary to use this function to see the results of any drawing.
*/
void vpDisplayGTK::flushDisplay()
{
  if (m_displayHasBeenInitialized) {
    m_impl->flushDisplay();
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Flushes the display buffer.
  It's necessary to use this function to see the results of any drawing.
*/
void vpDisplayGTK::flushDisplayROI(const vpImagePoint & /*iP*/, const unsigned int /*width*/,
                                   const unsigned int /*height*/)
{
  if (m_displayHasBeenInitialized) {
    m_impl->flushDisplay();
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  \warning Not implemented yet.
*/
void vpDisplayGTK::clearDisplay(const vpColor & /* color */)
{
  // Not implemented
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayGTK::displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w,
                                unsigned int h, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double a = ip2.get_i() - ip1.get_i();
    double b = ip2.get_j() - ip1.get_j();
    double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

    if ((std::fabs(a) > std::numeric_limits<double>::epsilon()) &&
        (std::fabs(b) > std::numeric_limits<double>::epsilon())) {
      a /= lg;
      b /= lg;

      vpImagePoint ip3;
      ip3.set_i(ip2.get_i() - w * a);
      ip3.set_j(ip2.get_j() - w * b);

      vpImagePoint ip4;
      ip4.set_i(ip3.get_i() - b * h);
      ip4.set_j(ip3.get_j() + a * h);

      if (lg > 2 * vpImagePoint::distance(ip2, ip4))
        displayLine(ip2, ip4, color, thickness);

      ip4.set_i(ip3.get_i() + b * h);
      ip4.set_j(ip3.get_j() - a * h);

      if (lg > 2 * vpImagePoint::distance(ip2, ip4))
        displayLine(ip2, ip4, color, thickness);

      displayLine(ip1, ip2, color, thickness);
    }
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param ip : Upper left image point location of the string in the display.
  \param text : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplayGTK::displayText(const vpImagePoint &ip, const std::string &text, const vpColor &color)
{
  if (m_displayHasBeenInitialized) {
    m_impl->displayText(ip, text, color, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a circle.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void vpDisplayGTK::displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                                 unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    m_impl->displayCircle(center, radius, color, fill, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}
/*!
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayGTK::displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double i = ip.get_i();
    double j = ip.get_j();
    vpImagePoint ip1, ip2;

    ip1.set_i(i - size / 2);
    ip1.set_j(j);
    ip2.set_i(i + size / 2);
    ip2.set_j(j);
    displayLine(ip1, ip2, color, thickness);

    ip1.set_i(i);
    ip1.set_j(j - size / 2);
    ip2.set_i(i);
    ip2.set_j(j + size / 2);

    displayLine(ip1, ip2, color, thickness);
  }

  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}
/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayGTK::displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                  unsigned int thickness)
{

  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    m_impl->displayDotLine(ip1, ip2, color, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayGTK::displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                               unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    m_impl->displayLine(ip1, ip2, color, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Point thickness.
*/
void vpDisplayGTK::displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    m_impl->displayPoint(ip, color, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param topLeft : Top-left corner of the rectangle.
  \param w,h : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplayGTK::displayRectangle(const vpImagePoint &topLeft, unsigned int w, unsigned int h, const vpColor &color,
                                    bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    m_impl->displayRectangle(topLeft, w, h, color, fill, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a rectangle.

  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplayGTK::displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight, const vpColor &color,
                                    bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    unsigned int w = static_cast<unsigned int>(vpMath::round(bottomRight.get_u() - topLeft.get_u()));
    unsigned int h = static_cast<unsigned int>(vpMath::round(bottomRight.get_v() - topLeft.get_v()));

    m_impl->displayRectangle(topLeft, w, h, color, fill, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Display a rectangle.

  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void vpDisplayGTK::displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    vpImagePoint topLeft = rectangle.getTopLeft();
    unsigned int w = static_cast<unsigned int>(vpMath::round(rectangle.getWidth()));
    unsigned int h = static_cast<unsigned int>(vpMath::round(rectangle.getRight()));
    m_impl->displayRectangle(topLeft, w, h, color, fill, thickness, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  Wait for a click from one of the mouse button.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayGTK::getClick(bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button;
    ret = m_impl->getClick(ip, button, blocking, m_scale, GDK_BUTTON_PRESS);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!
  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

*/
bool vpDisplayGTK::getClick(vpImagePoint &ip, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    vpMouseButton::vpMouseButtonType button;
    ret = m_impl->getClick(ip, button, blocking, m_scale, GDK_BUTTON_PRESS);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!

  Wait for a mouse button click and get the position of the clicked
  pixel. The button used to click is also set.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplayGTK::getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    ret = m_impl->getClick(ip, button, blocking, m_scale, GDK_BUTTON_PRESS);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!

  Wait for a mouse button click release and get the position of the
  image point were the click release occurs.  The button used to click is
  also set. Same method as getClick(unsigned int&, unsigned int&,
  vpMouseButton::vpMouseButtonType &, bool).

  \param ip [out] : Position of the clicked image point.

  \param button [in] : Button used to click.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

  \sa getClick(vpImagePoint &, vpMouseButton::vpMouseButtonType &, bool)

*/
bool vpDisplayGTK::getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    ret = m_impl->getClick(ip, button, blocking, m_scale, GDK_BUTTON_RELEASE);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*
  \brief gets the displayed image (including the overlay plane)
  and returns an RGBa image
*/
void vpDisplayGTK::getImage(vpImage<vpRGBa> &I)
{
  // should certainly be optimized.
  if (m_displayHasBeenInitialized) {
    m_impl->getImage(I, static_cast<gint>(m_width), static_cast<gint>(m_height));
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
}

/*!
  \brief get the window depth (8,16,24,32)

  usualy it 24 bits now...
*/
unsigned int vpDisplayGTK::getScreenDepth()
{
  unsigned int depth = m_impl->getScreenDepth();

  return (depth);
}

/*!
  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayGTK::getKeyboardEvent(bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    std::string key;
    ret = m_impl->getKeyboardEvent(key, blocking);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!

  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \param key [out]: If possible, an ISO Latin-1 character
  corresponding to the keyboard key.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayGTK::getKeyboardEvent(std::string &key, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    ret = m_impl->getKeyboardEvent(key, blocking);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!

  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool vpDisplayGTK::getPointerMotionEvent(vpImagePoint &ip)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    ret = m_impl->getPointerMotionEvent(ip, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }
  return ret;
}

/*!
  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.
*/
bool vpDisplayGTK::getPointerPosition(vpImagePoint &ip)
{
  if (m_displayHasBeenInitialized) {
    m_impl->getPointerPosition(ip, m_scale);
  }
  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "GTK not initialized"));
  }

  return true;
}

/*!
  Gets screen resolution.
  \param w, h : Horizontal and vertical screen resolution.
 */
void vpDisplayGTK::getScreenSize(unsigned int &w, unsigned int &h)
{
  w = h = 0;

  m_impl->getScreenSize(m_displayHasBeenInitialized, w, h);
}

/*!
  Gets the screen horizontal resolution in pixel.
 */
unsigned int vpDisplayGTK::getScreenWidth()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return width;
}

/*!
  Gets the screen vertical resolution.
 */
unsigned int vpDisplayGTK::getScreenHeight()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return height;
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpDisplayGTK.cpp.o) has no symbols
void dummy_vpDisplayGTK() { };
#endif
