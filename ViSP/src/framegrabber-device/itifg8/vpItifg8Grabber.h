/****************************************************************************
 *
 * $Id$
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
 * Description:
 * Framegrabber based on itifg-8.x (Coreco Imaging Technology) driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file vpItifg8Grabber.h
  \brief Class for the itifg-8.x (Coreco Imaging Technology) video device.

  This class is an interface for itifg framegrabber driver; see
  http://itifg.sourceforge.net/

  This class is interfaced with itifg-8.2.2-0 and itifg-8.3.1-12 driver. You
  can download these drivers from http://sourceforge.net/projects/itifg/

*/

#ifndef vpItifg8Grabber_hh
#define vpItifg8Grabber_hh

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_ITIFG8

#include <setjmp.h>
#include <signal.h>

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

/* itifg headers */
#include <itifgExt.h>
#include <libitifg.h>


/*
  Warning 1:

  Because amcmpReg.h file is not installed with the itifg-8.x driver, in
  vpItifg8Grabber.cpp you will find specific code comming from
  itifg-8.2.2-0/include/amcmpReg.h or itifg-8.3.1-12/include/amcmpReg.hfile

  u_int8_t    field         Field Status R/W

  #define CMP_BT829A_FIELD_MASK           0x20
  #define CMP_BT829A_FIELD_SHIFT          5

  introduced in the vpItifg8Grabber as static const variables:

  vpCMP_BT829A_FIELD_MASK
  vpCMP_BT829A_FIELD_SHIFT

*/



/*!
  \class vpItifg8Grabber

  \ingroup Framegrabber CameraDriver

  \brief Class providing an interface for the itifg-8.x (Coreco
  Imaging Technology) video device.

  \author  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

  Was tested with driver itifg-8.2.2.0 and itifg-8.3.1-12 coming from
  sourceforge (http://sourceforge.net/projects/itifg/) with an IC-COMP Coreco
  Imaging frame grabber board.

  This driver should support three generations of Coreco Imaging Technology
  frame grabber boards:

  First generation:
  - IC-FA (analog b/w, 1 Converter/8 Input 50MHz, 2/4MB VRAM))
  - IC-CLR (analog b/w-color, 3 Converter XXMHz, 2/4MB VRAM)
  - IC-VS (analog b/w-color, 1 Converter/4 Input 20MHz, 2/4MB VRAM,
    enhanced sync)
  - IC-DIG (digital b/w-color, 1-3 taps 8-12bit, 20/40MHz TTL/RS422,
    2/4MB VRAM, enhanched trig and signal)
  - IC-MTD (digital b/w-no area, 1-8taps 8bit, 2/4MB VRAM)
  - IC-COMP (PC-Comp) (analog, color composite, 1 Converter PAL/S-VHS,
    1MB VRAM)
  - IC-RGB (analog b/w-color, 3 Converter XXMHz, 2/4MB VRAM, color space
    converter)
  - PC-Vision (analog, b/w, 1 Converter/4 Input 20MHz, enhanced sync and trig)

  Second generation:
  - PC-Dig (digital, b/w-color, 1-4taps 8-14bit, 20/40MHz RS422/RS644,
    4MB SDRAM, enhanced trig and signal)
  - PC-RGB (analog, b/w-color, 3Converter XXMHz, 4MB SDRAM)
  - PC-VisionPlus (analog, b/w, 1Converter/4 Input 40MHz, 4MB SDRAM)
  - PCLineScan (digital, b/w-no area, 1-4taps 8-14bit, 10/40MHz, RS422/RS644,
    32MB SGRAM, enhanced trig and signal, preprocessing)
  - PC-CamLink (digital, b/w-color, 1-4taps, 8-14bit, 66MHz, BaseMediumFull,
    32MB SGRAM enhanced trig and signal)

  Third generation:
  - X64-Full
  - X64-Dual
  - X64-Pro
  - X64-LVDS
  - X64-Analog

  For further information see http://itifg.sourceforge.net/

  \warning In case of an AM-STD COMP board, the usage of getField() and
  setFramerate(framerateEnum rate) needs a modification of the itifg-8.2.2.0 or
  itifg-8.3.1-12 driver coming from sourceforge. See the documentation of these
  functions to have a description of the driver's modifications.

  The example below shows how to use this framegrabber.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpItifg8Grabber.h>

int main()
{
#if defined(VISP_HAVE_ITIFG8)
  vpImage<unsigned char> I; // Create a gray level image container
  vpItifg8Grabber g;        // Create a grabber based on itifg-8 third party lib
  g.setScale(2);            // Acquisition of subsampled images (384x288)
  g.setFramerate(vpItifg8Grabber::framerate_50fps); //  50 fps
  g.open(I);                // Open the grabber

  g.acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

*/



class VISP_EXPORT vpItifg8Grabber : public vpFrameGrabber
{
private:
  static const int vpCMP_BT829A_FIELD_MASK;
  static const int vpCMP_BT829A_FIELD_SHIFT;

public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;

  /*! Allowed framerates for AM-STD COMP board. */
  typedef enum 
    {
      framerate_50fps, //!< 50 frames per second
      framerate_25fps  //!< 25 frames per second
    } vpItifg8FramerateType;

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
public:
  typedef enum
    {
      READ_MODE,
      MMAP_MODE
    } vpItifg8OpmodeType;

private:
  struct vpItifg8OpmodeName_t
  {
    vpItifg8OpmodeType number;
    char name[8];
  };
#endif

public:
  typedef enum
    {
      BLOCK_MODE,
      SIGNAL_MODE,
      SELECT_MODE,
      MANUAL_MODE,
      POLL_MODE
    } vpItifg8SyncmdType;

private:
  struct vpItifg8SyncmdName_t
  {
    vpItifg8SyncmdType number;
    char name[8];
  };

public:
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  typedef enum
    {
      NORMAL_MODE = 0x0,
      LOCK_MODE = 0x1,
      SYNC_MODE = 0x2,
      APPEND_MODE = 0x4,
      DELAY_MODE = 0x8
    } vpItifg8SacqmdType;
#else
  typedef enum
    {
      NORMAL_MODE = 0x0,
      LOCK_MODE = 0x1,
      NODMA_MODE = 0x2,
      NOSYNC_MODE = 0x4,
      NOAPPEND_MODE = 0x8
    } vpItifg8SacqmdType;

#endif
private:
  struct vpItifg8SacqmdName_t
  {
    vpItifg8SacqmdType number;
    char name[16];
  };

  struct vpItifg8Image_t
  {
    char *srcptr;
    /*char *dstptr;*/
    off_t srcoff /*, dstoff*/;
    size_t raw_size, paged_size;
    short width, height;
    int srcbpp, srcbpl /*, dstbpp, dstbpl*/;
  };

  struct vpItifg8Args_t
  {
    int board_i;
    char conffile[ITI_BOARDS_MAX][PATH_MAX];

    int module[ITI_BOARDS_MAX];
    int camera[ITI_BOARDS_MAX];
    int depth[ITI_BOARDS_MAX];
    int scales[ITI_BOARDS_MAX];
    int buffer[ITI_BOARDS_MAX];
    int frames[ITI_BOARDS_MAX];
    int hdec[ITI_BOARDS_MAX];
    int vdec[ITI_BOARDS_MAX];
    float rate[ITI_BOARDS_MAX];
    vpItifg8FramerateType amcmp_rate[ITI_BOARDS_MAX];
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
    vpItifg8OpmodeType opmode[ITI_BOARDS_MAX];
#else
    bool other[ITI_BOARDS_MAX];
    int window[ITI_BOARDS_MAX];
    int contig[ITI_BOARDS_MAX];
#endif
    vpItifg8SyncmdType syncmd[ITI_BOARDS_MAX];
    vpItifg8SacqmdType sacqmd[ITI_BOARDS_MAX];
  };



private:
  unsigned int input ; //!< video entry
  unsigned int scale ;
  vpItifg8FramerateType framerate;
  bool field; // The type of the acquired frame (0 if odd, 1 if even).

public:
  vpItifg8Grabber();
  vpItifg8Grabber(unsigned int input,
		  unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE);
  vpItifg8Grabber(vpImage<unsigned char> &I,
		  unsigned int input,
		  unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE);
  vpItifg8Grabber(vpImage<vpRGBa> &I,
		  unsigned int input,
		  unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE);
  virtual ~vpItifg8Grabber();

  void open();
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  unsigned char * acquire();
  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);
  bool getField();
  void close();

  void setVerboseMode(bool activate=true);
  void setBoard(unsigned int board = 0);
  unsigned int getBoard();
  unsigned int getNumBoards();
  unsigned int getModule();
  void setConfFile(const char *filename);
  void setConfFile(std::string filename);
  void setInput(unsigned int input = vpItifg8Grabber::DEFAULT_INPUT);
  void setScale(unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE) ;
  void setHDecimation(unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE) ;
  void setVDecimation(unsigned int scale = vpItifg8Grabber::DEFAULT_SCALE) ;
  unsigned int getHDecimation();
  unsigned int getVDecimation();
  void setDepth(unsigned int depth);
  void setBuffer(unsigned int buffer);
  void setFramerate(float rate);
  void setFramerate(vpItifg8FramerateType rate);
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  void setOpmode  (vpItifg8OpmodeType opmode);
#else
  void setContigmode(bool contig);
#endif
  void setSyncmode(vpItifg8SyncmdType synmode);
  void setAcqmode (vpItifg8SacqmdType sacqmode);

private:
  void setupBufs();
  void initialise();

private:
  bool stop_it;
  int devdesc;
  int  rawdesc;
  int zerodesc;
  char *dataptr;
  size_t mapsize;
  int boards; // number of boards
  vpItifg8Args_t args;
#if VISP_HAVE_ITIFG8_VERSION >= 83 // 8.3.1-12
  char *ringptr;
  size_t winsize;
#else
  vpItifg8OpmodeName_t opmode_name[2];
#endif
  vpItifg8SyncmdName_t syncmd_name[5];
  vpItifg8SacqmdName_t sacqmd_name[5];

  iti_setup_t setup;

  struct sigaction copyact;
  struct sigaction ignore;
  sigset_t local_set;
  vpItifg8Image_t image[ITI_BOARDS_MAX]; // we suppose one camera per board

  bool ref_field;
  bool first_acq;
  bool verbose;
};



#endif
#endif

