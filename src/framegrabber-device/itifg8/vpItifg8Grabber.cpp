/****************************************************************************
 *
 * $Id: vpItifg8Grabber.cpp,v 1.10 2007-04-20 13:26:59 fspindle Exp $
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
 * Framegrabber based on itifg-8.x (Coreco Imaging Technology) driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file vpItifg8Grabber.cpp

  \brief Member functions for the itifg-8.x (Coreco Imaging) video device
  class.

*/

#include <iostream>
#include <stdio.h>

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_ITIFG8

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

//#include <vpItifg8Grabber.h>
#include <visp/vpItifg8Grabber.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpIoTools.h>

#undef vpITIFG8_USE_POLL

/*!
  Warning:

  Because amcmpReg.h file is not installed with the itifg-8 driver, in
  vpItifg8Grabber.cpp you will find specific code comming from
  itifg-8.2.2-0/include/amcmpReg.h or itifg-8.3.1-12/include/amcmpReg.h file

  \code
  u_int8_t    field         Field Status R/W

  #define CMP_BT829A_FIELD_MASK           0x20
  #define CMP_BT829A_FIELD_SHIFT          5
  \endcode

  introduced in the vpItifg8Grabber as static const variables:

  - vpItifg8Grabber::vpCMP_BT829A_FIELD_MASK
  - vpItifg8Grabber::vpCMP_BT829A_FIELD_SHIFT

*/

const int vpItifg8Grabber::DEFAULT_INPUT = 2;
const int vpItifg8Grabber::DEFAULT_SCALE = 2;
const int vpItifg8Grabber::vpCMP_BT829A_FIELD_MASK  = 0x20;
const int vpItifg8Grabber::vpCMP_BT829A_FIELD_SHIFT = 5;

static volatile int vpItifg8Timeout;

void
vpItifg8SigioCatcher (int num, siginfo_t *info, void */*ptr*/);

void
vpItifg8SigioCatcher (int num, siginfo_t *info, void */*ptr*/)
{
  if (info->si_band & POLLIN)
    vpItifg8Timeout = false;
  else if (info->si_band & POLLPRI)
    vpItifg8Timeout = true;
}


/*!
  Constructor.

  Uses the current framegrabber settings for input, scale and framerate.

  \exception vpFrameGrabberException::initializationError : If initialisation
  failed.

*/
vpItifg8Grabber::vpItifg8Grabber()
{
  init = false ;
  initialise();
}
/*!
  Constructor.

  \param input : Video input port for the first board.
  \param scale : Decimation factor for the first board.

*/
vpItifg8Grabber::vpItifg8Grabber( unsigned int input, unsigned int scale)
{
  init = false ;
  initialise();

  setBoard(0);
  setInput(input);
  setScale(scale);
}

/*!
  Constructor. Initialise and open the device.

  \param I : Image data structure (8 bits image).
  \param input : Video input port for the first board.
  \param scale : Decimation factor for the first board.


*/
vpItifg8Grabber::vpItifg8Grabber(vpImage<unsigned char> &I,
				 unsigned int input, unsigned int scale )
{
  init = false ;
  initialise();

  setBoard(0);
  setInput(input);
  setScale(scale);
  open(I);
}

/*!
  Constructor. Initialise and open the device.

  \param I : Image data structure (32 bits image).
  \param input : Video input port for the first board.
  \param scale : Decimation factor for the first board.

*/
vpItifg8Grabber::vpItifg8Grabber(vpImage<vpRGBa> &I,
				 unsigned int input, unsigned int scale)
{
  init = false ;
  initialise();

  setBoard(0);
  setInput(input);
  setScale(scale);
  open(I);
}

/*!
  Destructor.
  \sa close()
*/
vpItifg8Grabber::~vpItifg8Grabber()
{

  close();
}

/*!
  Internal initialisation.

  Uses the default framegrabber settings for input, scale and framerate.
  Contiguous grab mode is set by default.

  \exception vpFrameGrabberException::initializationError : If initialisation
  failed.

*/
void vpItifg8Grabber::initialise()
{
  if (init == false) {
    first_acq = false;
    setVerboseMode(false);

    // private members initialisation
    stop_it = false;
    devdesc = -1;
    rawdesc = -1;
    zerodesc = -1;
    dataptr = NULL;
    mapsize = (size_t)0;
#if VISP_HAVE_ITIFG8_VERSION >= 83 // 8.3.1-12
    ringptr = NULL;
    winsize = (size_t)0;
#endif
    boards = 1;
    vpItifg8Timeout = false;

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
    // Initialisation of opmode_name structure only for itifg-8.2.x
    opmode_name[0].number = READ_MODE;
    sprintf(opmode_name[0].name, "read");
    opmode_name[1].number = MMAP_MODE;
    sprintf(opmode_name[1].name, "mmap");
#endif
    // Initialisation of syncmd structure
    syncmd_name[0].number = BLOCK_MODE;
    sprintf(syncmd_name[0].name, "block");
    syncmd_name[1].number = SIGNAL_MODE;
    sprintf(syncmd_name[1].name, "signal");
    syncmd_name[2].number = SELECT_MODE;
    sprintf(syncmd_name[2].name, "select");
    syncmd_name[3].number = MANUAL_MODE;
    sprintf(syncmd_name[3].name, "manual");
    syncmd_name[4].number = POLL_MODE;
    sprintf(syncmd_name[4].name, "poll");

    // Initialisation of sacqmd structure
    sacqmd_name[0].number = NORMAL_MODE;
    sprintf(sacqmd_name[0].name, "normal");
    sacqmd_name[1].number = LOCK_MODE;
    sprintf(sacqmd_name[1].name, "lock");
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
    sacqmd_name[2].number = SYNC_MODE;
    sprintf(sacqmd_name[2].name, "sync");
    sacqmd_name[3].number = APPEND_MODE;
    sprintf(sacqmd_name[3].name, "append");
    sacqmd_name[4].number = DELAY_MODE;
    sprintf(sacqmd_name[4].name, "delay");
#else
    sacqmd_name[2].number = NODMA_MODE;
    sprintf(sacqmd_name[2].name, "noappend");
    sacqmd_name[3].number = NOSYNC_MODE;
    sprintf(sacqmd_name[3].name, "nosync");
    sacqmd_name[4].number = NOAPPEND_MODE;
    sprintf(sacqmd_name[4].name, "noappend");

#endif

    // Initialization of args structure
    for (int i=0; i < ITI_BOARDS_MAX; i ++) {
      setBoard(i);       // Default board
      setConfFile("/usr/share/itifg/conffiles/robot.cam");
      setInput(vpItifg8Grabber::DEFAULT_INPUT);       // Default input
      setScale(vpItifg8Grabber::DEFAULT_SCALE);       // Default decimation factor
      setDepth(8);      // Default image depth
      setBuffer(1);     // Default number of buffers
      setFramerate(1.); // Default framerate
      setFramerate(vpItifg8Grabber::framerate_50fps);
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
      setOpmode(vpItifg8Grabber::MMAP_MODE);     // Default operation mode
#else
      args.other[i] = false;
      args.window[i] = 4;
      setContigmode(true);                      // Default contiguous grab mode
#endif
      setSyncmode(vpItifg8Grabber::SIGNAL_MODE); // Default synchronisation mode
      setAcqmode(vpItifg8Grabber::NORMAL_MODE);  // Default special acq mode
      // Initialisation of image structure
      image[i].srcptr = NULL;
      image[i].srcoff = 0;
      image[i].raw_size = image[i].paged_size = 0;
      image[i].width = image[i].height = 0;
      image[i].srcbpp = image[i].srcbpl = 0;
    }

    setBoard(0);       // Default board

    // comes from Parse_Args ()

    char file_name[PATH_MAX];
    int error;
    strcpy (file_name, ITI_PFS_PREFIX);
    strcat (file_name, ITI_PFS_STRING);
    if (verbose)
      fprintf (stderr, "\n");
    if ((error = iti_parse_info (file_name, &setup)))
    {
      switch (error)
      {
      case -ITI_EARG:
	vpERROR_TRACE("Wrong call argument\n"
		      "(file_name:%s, setup%p).\n", file_name, &setup);
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "Wrong call argument") );
	break;
      case -ITI_EFMT:
	vpERROR_TRACE("Wrong data format (%s%s).\n",
		      ITI_PFS_PREFIX, ITI_PFS_STRING);
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "Wrong data format") );
	break;
      case -ITI_EENT:
	vpERROR_TRACE("File not found (%s%s).\n",
		      ITI_PFS_PREFIX, ITI_PFS_STRING);
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "File not found") );
	break;
      case -ITI_ESYS:
	vpERROR_TRACE("Error while system call (%s).\n", strerror (errno));
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "Error while system call") );
	break;
      default:
	vpERROR_TRACE("Error not specified!\n");
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "Error not specified!") );
      }
    }

    if (verbose) {
      fprintf (stdout, "Driver version: %d.%d.%d-%d.\n",
	       (setup.version & 0xFF000000) >> 24,
	       (setup.version & 0x00FF0000) >> 16,
	       (setup.version & 0x0000FF00) >> 8,
	       setup.version & 0x000000FF);
      fprintf (stdout, "Compilation Date: %s.\n", setup.date);
      fprintf (stdout, "Boards detected: %d.\n", setup.boards);
      for (int board = 0; board < setup.boards; board++)
      {
	switch (setup.modules[board])
	{
	case ICP_AMVS:
	  fprintf (stdout, "Module type board %d: AM-VS.\n", board);
	  break;
	case ICP_AMDG:
	  fprintf (stdout, "Module type board %d: AM-DIG.\n", board);
	  break;
	case ICP_AMPV:
	  fprintf (stdout, "Module type board %d: PCVision.\n", board);
	  break;
	case ICP_AMCMP:
	  fprintf (stdout, "Module type board %d: AM-STD-COMP.\n", board);
	  break;
	case ITI_PCDIG:
	  fprintf (stdout, "Module type board %d: PCDig.\n", board);
	  break;
	case ITI_PCLNK:
	  fprintf (stdout, "Module type board %d: PCCamLink.\n", board);
	  break;
	}
      }
    }

    memcpy (&args.module, &setup.modules, sizeof(setup.modules));
    init = true;
  }
}

/*!
  Initialize the device for grey level image acquisition.

  \param I : Image data structure (8 bits image)

  \sa setScale(), setInput(), setFramerate(), acquire(vpImage<unsigned char>)
*/
void vpItifg8Grabber::open(vpImage<unsigned char> &I)
{

  open();

  I.resize(image[args.board_i].height, image[args.board_i].width);
}

/*!
  Initialize the device for color image acquisition.

  \param I : Image data structure (32 bits image)

  \sa setScale(), setInput(), setFramerate(), acquire(vpImage<vpRGBa>)
*/
void
vpItifg8Grabber::open(vpImage<vpRGBa> &I)
{
  open();

  I.resize(image[args.board_i].height, image[args.board_i].width);
}

/*!

  Close the driver by stopping the acquisition and freeing memory.

  \exception vpFrameGrabberException::otherError : If the driver cannot be
  closed.

*/
void vpItifg8Grabber::close ()
{

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  struct iti_acc_t acctinfo;

  if (devdesc != -1)
  {
    if (stop_it) {
      if (ioctl (devdesc, GIOC_SET_STOP, NULL) < 0)
      {
	perror ("GIOC_SET_STOP");
	vpERROR_TRACE("GIOC_SET_STOP\n");
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "GIOC_SET_STOP") );
      }
    }

    if (verbose) {
      if (ioctl (devdesc, GIOC_GET_STATS, &acctinfo) < 0)
      {
	perror ("GIOC_GET_STATS");
	vpERROR_TRACE("GIOC_GET_STATS\n");
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "GIOC_GET_STATS") );
      }
      fprintf (stdout, "\nrun statistics:\n");
      fprintf (stdout, "\tframes captured %ld,\n",
	       acctinfo.captured);
      fprintf (stdout, "\tframes transfered %ld,\n",
	       acctinfo.transfered);
      fprintf (stdout, "\tframes timedout %ld,\n",
	       acctinfo.timedout);
      fprintf (stdout, "\tframes get_by_blocked %ld.\n",
	       acctinfo.by_block);
      fprintf (stdout, "\tframes get_by_signal %ld.\n",
	       acctinfo.by_signal);
      fprintf (stdout, "\tframes get_by_select %ld.\n",
	       acctinfo.by_select);
    }
  }

  if (dataptr != NULL)
  {
    if (mapsize == 0)
      free (dataptr);
    else
      munmap (dataptr, mapsize);
  }

  if (zerodesc != -1)
  {
    if (dataptr != NULL && mapsize != 0)
      munmap (dataptr, mapsize);
    ::close (zerodesc);
  }
#else
  if (ringptr != NULL && winsize != (size_t)0)
    munmap (ringptr, winsize);

  if (devdesc != -1)
    {
      if (stop_it)
	lseek (devdesc, -(off_t)LONG_MAX, SEEK_END);
    }

  if (zerodesc != -1)
    {
      if (dataptr != NULL && mapsize != (size_t)0)
	munmap (dataptr, mapsize);
      ::close (zerodesc);
    }
  else
    {
      if (dataptr != NULL && dataptr != ringptr)
	free (dataptr);
    }
#endif

  if (rawdesc != -1)
    ::close (rawdesc);

  if (devdesc != -1)
    ::close (devdesc);

  init = false;
  stop_it = false;
  devdesc = -1;

  first_acq == false;
}

/*!

  Set the verbose mode.

  \param activate : Verbose mode; true to activate debug information, false to
  turn off the debug mode.

*/
void vpItifg8Grabber::setVerboseMode(bool activate)
{
  verbose = activate;
}

/*!

  In case of multiple Coreco Imaging boards connected on same computer,
  select the current board for image acquisition.

  \param board : Coreco Imaging board number. The first board number is 0. This
  number must be less than ITI_BOARDS_MAX.

  \exception vpFrameGrabberException::settingError : If board number is
  invalid.

  \sa getBoard()

*/
void vpItifg8Grabber::setBoard(unsigned int board)
{
  if (board >= ITI_BOARDS_MAX)	{
    vpERROR_TRACE("Wrong board selected (%ud).\n", board);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong board selected") );

  }
  args.board_i = board;
}

/*!

  In case of multiple Coreco Imaging boards connected on a same computer,
  get the current board index for image acquisition.

  \return The Coreco Imaging board number.

  \sa setBoard()

*/
unsigned int vpItifg8Grabber::getBoard()
{
  return args.board_i;
}

/*!

  Get the number of Coreco Imaging detected boards.

  \return The number of detected boards. Useful in case of multiple Coreco
  Imaging boards connected on a same computer.

  \sa setBoard()

*/
unsigned int vpItifg8Grabber::getNumBoards()
{
  return setup.boards;
}

/*!

  Get the name of the Coreco Imaging module board.

  \return The name of the module: ICP_AMVS, ICP_AMDG, ICP_AMPV, ICP_AMCMP,
  ITI_PCDIG, ITI_PCLNK, COR_X64CL

*/
unsigned int vpItifg8Grabber::getModule()
{
  return args.module[args.board_i];
}


/*!

  Set the camera configuration filename for the current board which number is
  given by a getBoard().

  \param filename : Configuration filename with extension .cam.

  \exception vpFrameGrabberException::settingError : If camera configuration file is
  inexistant.

  \sa setBoard(), getBoard()
*/
void vpItifg8Grabber::setConfFile(const char *filename)
{
  sprintf(args.conffile[args.board_i], "%s", filename);

  if (vpIoTools::checkFilename(filename) == false) {
    vpERROR_TRACE("Inexistant camera configuration file: %s", filename) ;
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Inexistant camera configuration file") );

  }
}

/*!

  Set the camera configuration filename for the current board which number is
  given by a getBoard().

  \param filename : Configuration filename with extension .cam.

  \sa setBoard(), getBoard()
*/
void vpItifg8Grabber::setConfFile(string filename)
{
  setConfFile(filename.c_str());
}

/*!
  Set the input video port.

  \exception vpFrameGrabberException::settingError : If camera number is
  invalid.
*/
void vpItifg8Grabber::setInput(unsigned int input)
{
  if (input > ITI_CAMERAS_MAX)
  {
    vpERROR_TRACE("Wrong camera selected (%ud)", input) ;
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong camera selected") );
  }

  args.camera[args.board_i] = input;
}

/*!
  Set the horizontal and vertical decimation factor.

  \param scale : Decimation factor [1|2|4|8|16].

  \exception vpFrameGrabberException::settingError : Wrong decimation factor.

  \sa setHDecimation(), setVDecimation()
*/

void vpItifg8Grabber::setScale(unsigned int scale)
{
  setHDecimation(scale);
  setVDecimation(scale);
}

/*!
  Set the horizontal decimation factor.

  \param scale : Decimation factor [1|2|4|8|16].

  \exception vpFrameGrabberException::settingError : Wrong decimation factor.

  \sa setVDecimation()
*/

void vpItifg8Grabber::setHDecimation(unsigned int scale)
{
  switch (scale) {
  case 1:
  case 2:
  case 4:
  case 8:
  case 16:
    break;
  default:
    vpERROR_TRACE("Wrong horiz. decimation %d, shoud be [1|2|4|8|16]",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong scale") );
  }

  args.hdec[args.board_i] = scale;
}

/*!
  Get the horizontal decimation factor.

  \return Horizontal decimation factor [1|2|4|8|16].

  \sa setHDecimation()
*/

unsigned int vpItifg8Grabber::getHDecimation()
{
  return args.hdec[args.board_i];
}

/*!
  Get the vertical decimation factor.

  \return Vertical decimation factor [1|2|4|8|16].

  \sa setVDecimation()
*/

unsigned int vpItifg8Grabber::getVDecimation()
{
  return args.vdec[args.board_i];
}

/*!
  Set the vertical decimation factor.

  \param scale : Decimation factor [1|2|4|8|16].

  \exception vpFrameGrabberException::settingError : Wrong decimation factor.

  \sa getVDecimation()
*/

void vpItifg8Grabber::setVDecimation(unsigned int scale)
{
  switch (scale) {
  case 1:
  case 2:
  case 4:
  case 8:
  case 16:
    break;
  default:
    vpERROR_TRACE("Wrong vert. decimation %d, shoud be [1|2|4|8|16]",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong scale") );
  }

  args.vdec[args.board_i] = scale;
}

/*!
  Set the image depth.

  \param depth : Image depth [8|10|12|14|16|24|32].

  \exception vpFrameGrabberException::settingError : If image depth is
  invalid.
*/
void vpItifg8Grabber::setDepth(unsigned int depth)
{
  switch(depth) {
  case 0:
  case 8:
  case 10:
  case 12:
  case 14:
  case 16:
  case 24:
  case 32:
    break;
  default:
    vpERROR_TRACE("Wrong image depth selected (%ud).\n", depth);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong image depth selected") );
  }

  args.depth[args.board_i] = depth;
}

/*!
  Set the number of buffers used for acquisition.

  \param buffer : Number of buffers [1-8].

  \exception vpFrameGrabberException::settingError : If number of buffers is
  invalid.
*/
void vpItifg8Grabber::setBuffer(unsigned int buffer)
{
  if (buffer < 1 || buffer > ITI_SNAPS_MAX)
  {
    fprintf (stderr, "Wrong number of buffers selected (%ud).\n", buffer);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong number of buffers selected") );
  }
  args.buffer[args.board_i] = buffer;
}

/*!

  Set the acquisition framerate. Did not work for AM-STD COMP framegrabber
  board. For such a board, see setFramerate(framerateEnum rate).

  \param rate : Framerate [0.01-100.0].

  \exception vpFrameGrabberException::settingError : If framerate is
  invalid.

  \sa setFramerate(framerateEnum rate)
*/
void vpItifg8Grabber::setFramerate(float rate)
{
  if (rate < ITI_FPS_MIN || rate > ITI_FPS_MAX)
  {
    fprintf (stderr, "Wrong frames per second selected (%f).\n", rate);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong frames per second selected") );
  }
  args.rate[args.board_i] = rate;
}

/*!

  Set the framerate of the acquisition.

  \warning Dedicated AM-STD-COMP function member.

  To be able to acquire frames at 50 fps, the itifg-8.x driver was modified
  by replacing the original code in itifg-8.2.2-0/src/itifgIoctl.c

  \code
  int iti_frm_ioctl(...)
  {
    case GIOC_SET_VDEC:
      ...
      if ((error = iti_enable_irq (frm->osp, frm->brdif, ITI_FRM,
                                   frm->fmt.ilace,
                                   !!(iti_file_flags (file) & O_SYNC))))
      ...
  }
  \endcode

  by the modified code:

  \code
  int iti_frm_ioctl(...)
  {
    case GIOC_SET_VDEC:
      ...
      if ((error = iti_enable_irq (frm->osp, frm->brdif, ITI_FRM,
                                   frm->fmt.ilace && inparam->set_vdec == 1,
                                   !!(iti_file_flags (file) & O_SYNC))))
      ...
  }
  \endcode


  \param rate The framerate for the acquisition.

  \sa getFramerate()

*/
void
vpItifg8Grabber::setFramerate(vpItifg8Grabber::framerateEnum rate)
{
  args.amcmp_rate[args.board_i]  = rate;
}

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
/*!
  Set the acquisition operation mode.

  \param opmode : Operation mode [read|mmap].

  \exception vpFrameGrabberException::settingError : If operation mode is
  invalid.
*/
void vpItifg8Grabber::setOpmode(vpItifg8Opmode_t opmode)
{
  switch(opmode) {
  case READ_MODE:
  case MMAP_MODE:
    break;
  default:
    fprintf (stderr, "Wrong operation mode selected (%d).\n", opmode);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong operation mode selected") );
  }
  args.opmode[args.board_i] = opmode;
}
#endif

#if VISP_HAVE_ITIFG8_VERSION >= 83 // 8.3.1-12
/*!
  Set contiguous (grab) mode.

  \param contig : Contiguous (grab) mode mode.

*/
void vpItifg8Grabber::setContigmode(bool contig)
{
  args.contig[args.board_i] = contig;
}
#endif

/*!
  Set the acquisition synchronisation mode.

  \param syncmode : Syncronisation mode [block|signal|select|manual|poll]. In
  manual mode, acquisition is preset by a character hit.

  \exception vpFrameGrabberException::settingError : If synchronisation mode is
  invalid.
*/
void vpItifg8Grabber::setSyncmode(vpItifg8Syncmd_t syncmode)
{
  switch(syncmode) {
  case BLOCK_MODE:
  case SIGNAL_MODE:
  case SELECT_MODE:
  case MANUAL_MODE:
  case POLL_MODE:
    break;
  default:
    fprintf (stderr, "Wrong synchronisation mode selected (%d).\n", syncmode);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong synchronisation mode selected") );
  }
  args.syncmd[args.board_i] = syncmode;
}

/*!
  Set the special acquisition mode.

  \param sacqmode : Special acquisition mode [none,lock,sync,append,delay].

  \exception vpFrameGrabberException::settingError : If special acquisition
  mode is invalid.
*/
void vpItifg8Grabber::setAcqmode(vpItifg8Sacqmd_t sacqmode)
{
  switch(sacqmode) {
  case NORMAL_MODE:
  case LOCK_MODE:
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  case SYNC_MODE:
  case APPEND_MODE:
  case DELAY_MODE:
#else
  case NODMA_MODE:
  case NOSYNC_MODE:
  case NOAPPEND_MODE:
#endif
    break;
  default:
    fprintf (stderr, "Wrong special acqisition mode selected (%d).\n", sacqmode);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong special acqisition mode selected") );
  }
  args.sacqmd[args.board_i] = sacqmode;
}

/*!
  Device initialization.


  \exception vpFrameGrabberException::initializationError : If device
  initialization fails.
*/
void vpItifg8Grabber::open()
{

  if (args.board_i >= setup.boards)
  {
    fprintf (stderr, "Wrong board selected (%d). Only (%d) boards detected\n", args.board_i, setup.boards);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Wrong board selected") );
  }
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  if (args.opmode[args.board_i] == MMAP_MODE &&
      args.syncmd[args.board_i] == BLOCK_MODE)
  {
    fprintf (stderr, "If using 'mmap' mode, 'blocking' mode is not");
    fprintf (stderr, " useful. Switching to 'signal' mode.\n");
    args.syncmd[args.board_i] = SIGNAL_MODE;
  }
  if (args.sacqmd[args.board_i] == DELAY_MODE &&
      args.syncmd[args.board_i] != MANUAL_MODE)
  {
    fprintf (stderr, "If using 'delay' mode, only 'manual' mode");
    fprintf (stderr, " useful. Switching to 'manual' mode.\n");
    args.syncmd[args.board_i] = MANUAL_MODE;
  }

  if (args.opmode[args.board_i] == MMAP_MODE &&
      args.sacqmd[args.board_i] & LOCK_MODE)
    fprintf (stderr, "If using 'mmap' mode, locking is useless.");

  if (args.sacqmd[args.board_i] & LOCK_MODE && geteuid())
  {
    fprintf (stderr, "One have to be root to lock the read buffer.");
    fprintf (stderr, " Disable locking.");
    args.sacqmd[args.board_i] &= ~LOCK_MODE;
   }
#else
  if (args.buffer[args.board_i] > 1 && args.contig[args.board_i])
    {
      fprintf (stderr, "If using 'contig' mode, more than one buffer");
      fprintf (stderr, "doesn't make sense. Ignoring buffer number.\n");
      args.buffer[args.board_i] = 1;
    }

  if (args.syncmd[args.board_i] == BLOCK_MODE)
    {
      if (args.contig[args.board_i])
	{
	  fprintf (stderr, "If using 'contig' mode, 'blocking' mode is not");
	  fprintf (stderr, " useful. Switching to 'signal' mode.\n");
	  args.syncmd[args.board_i] = SIGNAL_MODE;
	}
      if (args.other[args.board_i])
	{
	  fprintf (stderr, "If using 'mmap' mode, 'blocking' mode is not");
	  fprintf (stderr, " useful. Switching to 'signal' mode.\n");
	  args.syncmd[args.board_i] = SIGNAL_MODE;
	}
    }
  if (args.sacqmd[args.board_i] & LOCK_MODE)
    {
      if (args.other[args.board_i])
	fprintf (stderr, "If using 'mmap' mode, locking is useless.");

      if (geteuid())
	{
	  fprintf (stderr, "One have to be root to lock the read buffer.");
	  fprintf (stderr, " Disable locking.");
	  //	  args.sacqmd[args.board_i] &= (vpItifg8Sacqmd_t) (~LOCK_MODE);
	  int tmpval = args.sacqmd[args.board_i];
	  tmpval  &= (~LOCK_MODE);
	  args.sacqmd[args.board_i] = (vpItifg8Sacqmd_t) tmpval;
	}
    }
  if (args.sacqmd[args.board_i] & NODMA_MODE)
    {
      if (args.window[args.board_i])
      fprintf (stderr, "If using 'nodma' mode, window adjustment is useless.");

      if (args.other[args.board_i])
	{
	  fprintf (stderr, "If using 'nodma' mode, 'mmap' mode isn't possible.");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
					 "If using 'nodma' mode, 'mmap' mode isn't possible.") );
	}
    }
 #endif


  if (verbose) {
    fprintf (stdout, "\n");
    fprintf (stdout, "board:  %d\n", args.board_i);
    fprintf (stdout, "%d - conffile: %s\n", args.board_i,
	     args.conffile[args.board_i]);

    fprintf (stdout, "%d - module: %d\n", args.board_i, args.module[args.board_i]);
    fprintf (stdout, "%d - camera: %d\n", args.board_i, args.camera[args.board_i]);
    fprintf (stdout, "%d - depth:  %d\n", args.board_i, args.depth[args.board_i]);
    fprintf (stdout, "%d - buffer: %d\n", args.board_i, args.buffer[args.board_i]);
    fprintf (stdout, "%d - hdec:   %d\n", args.board_i, args.hdec[args.board_i]);
    fprintf (stdout, "%d - vdec:   %d\n", args.board_i, args.vdec[args.board_i]);
    fprintf (stdout, "%d - rate:   %.2f\n", args.board_i, args.rate[args.board_i]);
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
    fprintf (stdout, "%d - opmode: %s\n", args.board_i,
	     opmode_name[args.opmode[args.board_i]].name);
#else
    fprintf (stdout, "%d - contig: %d\n", args.board_i,
	     args.contig[args.board_i]);
#endif
    fprintf (stdout, "%d - syncmd: %s\n", args.board_i,
	     syncmd_name[args.syncmd[args.board_i]].name);
    fprintf (stdout, "%d - sacqmd: %s\n", args.board_i,
	     sacqmd_name[args.sacqmd[args.board_i]].name);
  }

  int flags, error;
  char device_name[PATH_MAX];

  /* FIXME: perhaps use snprintf */
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  sprintf (device_name, "%s%d%s", ITI_NAME_PREFIX,
	   args.board_i, ITI_NAME_FRM_POSTFIX);
  flags = O_RDWR;

  if (args.sacqmd[args.board_i] & SYNC_MODE)
    flags |= O_SYNC;
  if (args.sacqmd[args.board_i] & APPEND_MODE)
    flags |= O_APPEND;
#else
  sprintf (device_name, "%s%d%s", ITI_NAME_PREFIX, args.board_i,
	   args.sacqmd[args.board_i] & NODMA_MODE ?
	   ITI_NAME_ACQ_POSTFIX :ITI_NAME_DMA_POSTFIX);

  flags = O_RDWR | O_SYNC | O_APPEND;

  if (args.sacqmd[args.board_i] & NOSYNC_MODE)
    flags &= ~O_SYNC;
  if (args.sacqmd[args.board_i] & NOAPPEND_MODE)
    flags &= ~O_APPEND;
#endif


#ifdef __NetBSD__ /* fcntl does not work here!? */
  if (args.syncmd[args.board_i] != BLOCK_MODE)
    flags |= O_NONBLOCK;
  if (args.syncmd[args.board_i] == SIGNAL_MODE)
    flags |= O_ASYNC;
#endif

  if ((devdesc = ::open (device_name, flags)) < 0)
  {
    perror ("Open module device");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Cannot open module device") );
  }

  struct timeval timeout;
  time_t timeout_ms;
  if (args.rate[args.board_i])
  {
    timeout_ms = (time_t)( 1000 / args.rate[args.board_i] );
    MS_TO_TV (timeout_ms, timeout);
  }
  else
  { timeout.tv_sec = 0L; timeout.tv_usec = 0L; }

  if (ioctl (devdesc, GIOC_SET_TIMEOUT, &timeout) < 0)
  {
    perror ("GIOC_SET_TIMEOUT");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_SET_TIMEOUT") );
  }

  int cameras;
  switch (getModule())
  {
  case ICP_AMVS:
    cameras = VS_CAMERAS_MAX;
    break;
  case ICP_AMDG:
    cameras = DG_CAMERAS_MAX;
    break;
  case ICP_AMPV:
    cameras = PV_CAMERAS_MAX;
    break;
  case ICP_AMCMP:
    cameras = CMP_CAMERAS_MAX;
    break;
  case ITI_PCDIG:
    cameras = PCD_CAMERAS_MAX;
    break;
  case ITI_PCLNK:
    cameras = LNK_CAMERAS_MAX;
    break;
  case COR_X64CL:
    cameras = X64_CAMERAS_MAX;
    break;
  default:
    cameras = 0;
  }

  if (args.camera[args.board_i] > cameras)
  {
    vpERROR_TRACE ("Camera %d not valid on this module type.\n",
	     args.camera[args.board_i]);
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Camera not valid on this module type") );
  }

  if (ioctl (devdesc, GIOC_SET_CAMERA, &args.camera[args.board_i]) < 0)
  {
    perror ("GIOC_SET_CAMERA");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_SET_CAMERA") );
  }

  union iti_cam_t camconf;
  if (ioctl (devdesc, GIOC_GET_CAMCNF, &camconf) < 0)
  {
    perror ("GIOC_GET_CAMCNF");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_GET_CAMCNF") );
  }

  fprintf (stderr, "\n");

  int board = 0;
  char camera_name[MAX_ENTRY_LENGTH], exo_filename[MAX_ENTRY_LENGTH];
  /* starting with version 8 we use two different config mechanisms */
  if (args.module[args.board_i] == COR_X64CL)
    error = cor_read_config (args.conffile[args.board_i], &camconf,
			     board, args.module[args.board_i],
			     args.camera[args.board_i], camera_name, exo_filename);
  else
    error = iti_read_config (args.conffile[args.board_i], &camconf,
			     board, args.module[args.board_i],
			     args.camera[args.board_i], camera_name, exo_filename);

  if (error < 0)
  {
    switch (error)
    {
    case -ITI_EARG:
      vpERROR_TRACE ("read_config: Wrong call Argument\n"
	       "(file:%s, conf:%p, board:%d, module:%d, camera:%d).\n",
	       args.conffile[args.board_i], &camconf, board,
	       args.module[args.board_i], args.camera[args.board_i]);
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Wrong call Argument") );
      break;
    case -ITI_EFMT:
      vpERROR_TRACE ("read_config: Wrong data format (%s).\n",
	       args.conffile[args.board_i]);
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Wrong data format") );
      break;
    case -ITI_EENT:
      vpERROR_TRACE ("read_config: File not found (%s).\n",
	       args.conffile[args.board_i]);
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "File not found") );
      break;
    case -ITI_ESYS:
      vpERROR_TRACE ("read_config: Error while system call (%s).\n",
	       strerror (errno));
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Error while system call") );
      break;
    default:
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Unknown error") );
    }
  }

  if (verbose) {
    fprintf (stdout, "CameraName : %s.\n", camera_name);
    fprintf (stdout, "ExoFilename: %s.\n", exo_filename);
  }

  if (ioctl (devdesc, GIOC_SET_CAMCNF, &camconf) < 0)
  {
    perror ("GIOC_SET_CAMCNF");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_SET_CAMCNF") );
  }

  switch (args.depth[args.board_i])
  {
  case 0:
    break;
  case 8:
    if (ioctl (devdesc, GIOC_SET_NORM_TO8, NULL) < 0)
    {
      perror ("GIOC_SET_NORM_TO8");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "GIOC_SET_NORM_TO8") );
    }
    break;
  case 10:
  case 12:
  case 14:
    break;
  case 16:
    switch (args.module[args.board_i])
    {
    case ICP_AMCMP:
    case ICP_AMDG:
    case ITI_PCDIG:
    case ITI_PCLNK:
    case COR_X64CL:
      if (ioctl (devdesc, GIOC_SET_NORM_TO16, NULL) < 0)
      {
	perror ("GIOC_SET_NORM_TO16");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "GIOC_SET_NORM_TO16") );
      }
      break;
    case ICP_AMVS:
    case ICP_AMPV:
    default:
      vpERROR_TRACE ("16bpp not supported at AM-VS and PCVision.\n");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "16bpp not supported at AM-VS and PCVision") );
      break;
    }
    break;
  case 24:
  case 32:
    if (args.module[args.board_i] == ICP_AMCMP)
    {
      vpERROR_TRACE ("24 or 32 bpp not supported at AM-STD/COMP.\n");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "24 or 32 bpp not supported at AM-STD/COMP") );
      break;
    }
  }

  if (ioctl (devdesc, GIOC_SET_HDEC, &args.hdec[args.board_i]) < 0)
  {
    perror ("GIOC_SET_HDEC");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_SET_HDEC") );
  }
  if (ioctl (devdesc, GIOC_SET_VDEC, &args.vdec[args.board_i]) < 0)
  {
    perror ("GIOC_SET_VDEC");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_SET_VDEC") );
  }

  // setup image buffers
  setupBufs();


  if (args.syncmd[args.board_i] == SIGNAL_MODE)
  {
    ignore.sa_handler = SIG_IGN;
    sigemptyset (&ignore.sa_mask);
    ignore.sa_flags = 0;
    if (sigaction (SIGIO, &ignore, NULL) < 0)
    {
      perror ("Can't install IGNORE handler");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "Can't install IGNORE handler") );
    }
    copyact.sa_sigaction = vpItifg8SigioCatcher;
    sigemptyset (&copyact.sa_mask);
#ifndef __NetBSD__
    copyact.sa_flags = SA_ONESHOT | SA_SIGINFO;
#endif
    if (fcntl (devdesc, F_SETOWN, getpid()) < 0)
    {
      perror ("F_SETOWN");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "F_SETOWN") );
    }
#ifndef __NetBSD__ /* FIXME: fcntl does not work here!? */
    if (fcntl (devdesc, F_SETFL,
	       fcntl (devdesc, F_GETFL, NULL) | O_ASYNC) < 0)
    {
      perror ("F_SETFL FASYNC");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "F_SETFL FASYNC") );
    }
#endif
  }

#ifndef __NetBSD__ /* FIXME: fcntl does not work here!? */
  if (args.syncmd[args.board_i] != BLOCK_MODE)
  {
    if (fcntl (devdesc, F_SETFL,
	       fcntl (devdesc, F_GETFL, NULL) | O_NONBLOCK) < 0)
    {
      perror ("F_SETFL O_NONBLOCK");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "F_SETFL O_NONBLOCK") );
    }
  }
#endif

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  if (args.opmode[args.board_i] == MMAP_MODE)
  {
    if (ioctl (devdesc, GIOC_SET_STRT, NULL) < 0)
    {
      perror ("GIOC_SET_STRT");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "GIOC_SET_STRT") );
    }
    stop_it = true;
  }
#else // VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  if (args.contig[args.board_i])
    {
      if (lseek (devdesc, +(off_t)LONG_MAX, SEEK_END) < 0)
	{
	  perror ("lseek strt acq");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
					 "lseek strt acq") );
	}
      stop_it = true;
    }
#endif // VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12

  init = true;

}
/*!
  Setup buffer's characteristics.


  \exception vpFrameGrabberException::settingError : If image buffer cannot be
  setup.

*/
void vpItifg8Grabber::setupBufs()
{
  if (ioctl (devdesc, GIOC_GET_WIDTH, &image[args.board_i].width) < 0)
  {
    perror ("GIOC_GET_WIDTH");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "GIOC_GET_WIDTH") );
  }
  if (ioctl (devdesc, GIOC_GET_HEIGHT, &image[args.board_i].height) < 0)
  {
    perror ("GIOC_GET_HEIGHT");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "GIOC_GET_HEIGHT") );
  }
  if (ioctl (devdesc, GIOC_GET_DEPTH, &image[args.board_i].srcbpp) < 0)
  {
    perror ("GIOC_GET_DEPTH");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "GIOC_GET_DEPTH") );
  }

  if (ioctl (devdesc, GIOC_GET_RAWSIZE, &image[args.board_i].raw_size) < 0)
  {
    perror ("GIOC_GET_RAWSIZE");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "GIOC_GET_RAWSIZE") );
  }
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  if (ioctl (devdesc, GIOC_GET_PAGEDSIZE, &image[args.board_i].paged_size) < 0)
  {
    perror ("GIOC_GET_PAGEDSIZE");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "GIOC_GET_PAGESIZE") );
  }
#else
  if (args.sacqmd[args.board_i] & NODMA_MODE)
    image[args.board_i].paged_size = image[args.board_i].raw_size;
  else
    if (ioctl (devdesc, GIOC_GET_PAGEDSIZE, &image[args.board_i].paged_size) < 0)
      {
	perror ("GIOC_GET_PAGEDSIZE");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				       "GIOC_GET_PAGESIZE") );
      }

#endif

  if (verbose) {
    fprintf (stdout, "\n");
    fprintf (stdout, "Image width: %d.\n", image[args.board_i].width);
    fprintf (stdout, "Image height: %d.\n", image[args.board_i].height);
    fprintf (stdout, "Image depth: %d.\n", image[args.board_i].srcbpp);
  }

#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  mapsize = args.buffer[args.board_i] * image[args.board_i].paged_size;
  switch (args.opmode[args.board_i])
  {
  case READ_MODE:
#if (1)
    if ((zerodesc = ::open ("/dev/zero", O_RDONLY)) < 0)
    {
      perror ("Open anonymous mapping device");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Open anonymous mapping device") );
    }
    if ((dataptr = (char *)mmap (0, mapsize, PROT_READ | PROT_WRITE,
				MAP_PRIVATE, zerodesc, 0)) == (void*)-1)
    {
      perror ("Can't map anonymous device");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Can't map anonymous device") );
    }
#else // 1
    if (!(dataptr = malloc(mapsize)))
    {
      perror ("Can't allocate save buffer");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Can't allocate save buffer") );
    }
#endif // 1
    memset (dataptr, 0x00, mapsize);
    /* if you want some realtime beahivior, locking is useful */
    if (args.sacqmd[args.board_i] & LOCK_MODE)
      if (mlock (dataptr, mapsize) < 0)
      {
	perror ("Can't lock anonymous device");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				       "Can't lock anonymous device") );
      }
    break;
  case MMAP_MODE:
    if ((dataptr = (char *)mmap (0, mapsize, PROT_READ | PROT_WRITE,
				MAP_SHARED, devdesc, 0)) == (void*)-1)
    {
      perror ("Can't map camera device");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Can't map camera device") );
    }
    break;
  }
#else // VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
  if (!(args.sacqmd[args.board_i] & NODMA_MODE))
    {
      winsize = args.window[args.board_i] << 20; /* Megabytes */

      if ((ringptr = (char *)mmap (0, winsize, PROT_READ | PROT_WRITE,
				  MAP_SHARED, devdesc, 0)) == (void*)-1)
	{
	  perror ("Can't map camera device");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					 "Can't map camera device") );
	}
    }

  if (args.other[args.board_i])
    dataptr = ringptr;
  else
    {
      mapsize = args.buffer[args.board_i] * image[args.board_i].paged_size;

#if (1)
      if ((zerodesc = ::open ("/dev/zero", O_RDONLY)) < 0)
	{
	  perror ("Open anonymous mapping device");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					 "Open anonymous mapping device") );

	}

      if ((dataptr = (char*)mmap (0, mapsize, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE, zerodesc, 0)) == (void*)-1)
	{
	  perror ("Can't map anonymous device");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					 "Can't map anonymous device") );

	}
#else // 1
      if (!(dataptr = malloc(mapsize)))
	{
	  perror ("Can't allocate save buffer");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					 "Can't allocate save buffer") );
	}
#endif // 1
      memset (dataptr, 0x00, mapsize);
      /* if you want some realtime beahivior, locking is useful */
      if (args.sacqmd[args.board_i] & LOCK_MODE)
	if (mlock (dataptr, mapsize) < 0)
	  {
	    perror ("Can't lock anonymous device");
	    close();
	    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					   "Can't lock anonymous device") );
	  }
    }
#endif // VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12

  image[args.board_i].srcbpl = image[args.board_i].width * ((image[args.board_i].srcbpp + 7) / 8);

  if (verbose) {
    fprintf (stdout, "Source bits_per_pixel: %d.\n", image[args.board_i].srcbpp);
    fprintf (stdout, "Source bytes_per_line: %d.\n", image[args.board_i].srcbpl);
  }

  image[args.board_i].srcptr = dataptr;
  image[args.board_i].srcoff = 0;
  if (verbose) {
    fprintf (stdout, "Source pointer: %p.\n", image[args.board_i].srcptr);
    fprintf (stdout, "Source offset: %ld.\n", (long)image[args.board_i].srcoff);
  }
}
/*!
  Image acquisition.

  \return Address of the acquired image.

  \exception vpFrameGrabberException::otherError : If image buffer cannot be
  acquired.

*/
#if VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
unsigned char * vpItifg8Grabber::acquire()
{
  int error;

  /* image acquisition */
  vpItifg8Timeout = false;

  if  (args.opmode[args.board_i] == READ_MODE)
  {
    if (read (devdesc,
	      (args.sacqmd[args.board_i] & DELAY_MODE) ?
	      NULL : image[args.board_i].srcptr,
	      args.buffer[args.board_i] * image[args.board_i].paged_size) < 0)
    {
      if (errno == ETIME)
	vpItifg8Timeout = true;
      else
      {
// 	if (errno == EINTR)
// 	  goto done;
// 	else
// 	{
	  perror ("Can't read image");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "Can't read image") );
// 	}
      }
    }
    else
      if (verbose)
	fprintf(stdout, "-");
  }

  switch (args.syncmd[args.board_i])
  {
  case BLOCK_MODE:
    break;
  case SIGNAL_MODE:
    if (sigaction (SIGIO, &copyact, NULL) < 0)
    {
      perror ("Can't install SIGIO handler");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Can't install SIGIO handler") );
    }
    sigemptyset (&local_set);
    if (sigsuspend (&local_set) < 0)
    {
      if (errno != EINTR)
      {
	perror ("Can't call sigsuspend");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Can't call sigsuspend") );
      }
    }
    if (sigaction (SIGIO, &ignore, NULL) < 0)
    {
      perror ("Can't install IGNORE handler");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Can't install IGNORE handler") );
    }
    if (!vpItifg8Timeout) if (verbose) fprintf(stdout, "\\");
    break;
  case SELECT_MODE:
    {
#ifdef vpITIFG8_USE_POLL
      struct pollfd wait_fd;

      wait_fd.fd = devdesc;
      wait_fd.events = POLLIN | POLLPRI;
      error = poll (&wait_fd, 1, -1);
      switch (error)
      {
      case -1:
      case 0:
	perror ("Some error during poll occured");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Some error during poll occured") );
	break;
      case 1:
	if (wait_fd.revents & POLLIN)
	  if (verbose)
	    fprintf(stdout, "/");
	if (wait_fd.revents & POLLPRI)
	  vpItifg8Timeout = true;
	break;
      default:
	perror ("unexpected poll return value");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "unexpected poll return value") );
      }
#else /* USE_SELECT */
      fd_set in_fdset, ex_fdset;

      FD_ZERO (&in_fdset);
      FD_ZERO (&ex_fdset);
      FD_SET (devdesc, &in_fdset);
      FD_SET (devdesc, &ex_fdset);
      error = select (getdtablesize (),
		      &in_fdset, NULL, &ex_fdset, NULL);
      switch (error)
      {
      case -1:
      case 0:
	perror ("Some error during select occured");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Some error during select occured") );
	break;
      case 1:
      case 2:
	if (FD_ISSET (devdesc, &in_fdset))
	  if (verbose)
	    fprintf(stdout, "/");
	if (FD_ISSET (devdesc, &ex_fdset))
	  vpItifg8Timeout = true;
	break;
      default:
	perror ("unexpected select return value");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "unexpected select return value") );
      }
#endif
    }
    break;
  case MANUAL_MODE:
    if (verbose)
      fprintf(stdout, "?");
    getchar ();
    break;
  case POLL_MODE:
    while (true)
    {
      off_t offset;

      if ((offset = lseek (devdesc, 0L, SEEK_CUR)) < 0)
      {
	perror ("lseek 0/SEEK_CUR");
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "lseek 0/SEEK_CUR") );
      }
      if (args.opmode[args.board_i] == MMAP_MODE)
      {
	static off_t last_offset = (off_t)0;

	/* if the offset have changed */
	if (offset != last_offset)
	{
	  /* and the last_offset belongs to another frame */
	  if ((offset / (off_t)image[args.board_i].paged_size) !=
	      (last_offset / (off_t)image[args.board_i].paged_size))
	  {
	    last_offset = offset;
	    break;
	  }
	  last_offset = offset;
	}
      }
      else
	if (offset >=
	    args.buffer[args.board_i] * (off_t)image[args.board_i].paged_size)
	  break;
      usleep (10000);
    }
    break;
  }

  if (args.opmode[args.board_i] == READ_MODE &&
      args.syncmd[args.board_i] != BLOCK_MODE)
  {
    if (read (devdesc, image[args.board_i].srcptr,
	      args.buffer[args.board_i] * image[args.board_i].paged_size) < 0)
    {
      if (errno == ETIME)
	vpItifg8Timeout = true;
      else
      {
// 	if (errno == EINTR)
// 	  goto done;
// 	else
// 	{
	  perror ("Can't read image");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Can't read image") );
// 	}
      }
    }
    else
      if (verbose)
	fprintf(stdout, "-");
  }

  /* image processing */
  int count = 0;
  switch (args.opmode[args.board_i])
  {
  case READ_MODE:
    image[args.board_i].srcoff = count * image[args.board_i].paged_size;
    break;
  case MMAP_MODE:
    do
      if ((image[args.board_i].srcoff = lseek (devdesc, 0L, SEEK_CUR)) < 0)
      {
	if (errno != EINTR)
	{
	  perror ("lseek 0/SEEK_CUR");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "lseek 0/SEEK_CUR") );
	}
      }
      else
	break;
    while (true);

    if (image[args.board_i].srcoff >= (off_t)image[args.board_i].paged_size)
      /* the last image is the interesting one */
      image[args.board_i].srcoff -= (off_t)image[args.board_i].paged_size;

    /* align offset to a whole image */
    image[args.board_i].srcoff = ((image[args.board_i].srcoff / (off_t)image[args.board_i].paged_size) *
		    (off_t)image[args.board_i].paged_size);
    /* avoid buffer overflow, so use wraparound */
    image[args.board_i].srcoff %= ((args.buffer[args.board_i]) *
		     image[args.board_i].paged_size);
    break;
  }
  if (verbose)
    fprintf(stdout, "%ld", (long)image[args.board_i].srcoff / image[args.board_i].paged_size);

  return((u_char *)(image[args.board_i].srcptr + image[args.board_i].srcoff));

}
#else // VISP_HAVE_ITIFG8_VERSION < 83 // 8.3.1-12
unsigned char * vpItifg8Grabber::acquire()
{
  int error;
  int srcidx;
  loff_t srcoff[FILENAME_MAX];
  loff_t todo, thisone, nextone, done;

  todo = args.buffer[args.board_i] * image[args.board_i].paged_size;
  done = (size_t)0;

  /* 1. initiate acquisition */
  vpItifg8Timeout = false;

  if (!args.contig[args.board_i])
    {
      if (lseek (devdesc, todo, SEEK_END) < 0)
	{
	  perror ("lseek strt acq");
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "lseek strt acq") );
	}
      if (verbose) fprintf(stdout,"+");
    }

  /* 2. wait for acquisition */
  if (!args.contig[args.board_i]) stop_it = true;
  srcidx = 0;
  do
    {

      switch (args.syncmd[args.board_i])
	{
	case BLOCK_MODE:
	  break;
	case SIGNAL_MODE:
	  if (verbose) fprintf(stdout,"i");
	  if (sigaction (SIGIO, &copyact, NULL) < 0)
	    {
	      perror ("Can't install SIGIO handler");
	      close();
	      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					     "Can't install SIGIO handler") );
	    }
	  sigemptyset (&local_set);
	  if (sigsuspend (&local_set) < 0)
	    {
	      if (errno != EINTR)
		{
		  perror ("Can't call sigsuspend");
		  close();
		  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						 "Can't call sigsuspend") );
		}
	    }
	  if (sigaction (SIGIO, &ignore, NULL) < 0)
	    {
	      perror ("Can't install IGNORE handler");
	      close();
	      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					     "Can't install IGNORE handler") );
	    }
	  break;
	case SELECT_MODE:
	  {
#ifdef vpITIFG8_USE_POLL
	    struct pollfd wait_fd;

	  retry:
	    if (verbose) fprintf(stdout,"p");
	    wait_fd.fd = devdesc;
	    wait_fd.events = POLLIN | POLLPRI;
	    error = poll (&wait_fd, 1, -1);
	    switch (error)
	      {
	      case -1:
	      case 0:
		if (errno == EINTR) goto retry;
		perror ("Some error during poll occured");
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					       "Some error during poll occured") );
		break;
	      case 1:
		if (wait_fd.revents & POLLIN)
		  ;
		if (wait_fd.revents & POLLPRI)
		  vpItifg8Timeout = true;
		break;
	      default:
		perror ("unexpected poll return value");
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					       "unexpected poll return value") );
	      }
#else /* USE_SELECT */
	    fd_set in_fdset, ex_fdset;

	  retry:
	    if (verbose) fprintf(stdout,"s");
	    FD_ZERO (&in_fdset);
	    FD_ZERO (&ex_fdset);
	    FD_SET (devdesc, &in_fdset);
	    FD_SET (devdesc, &ex_fdset);
	    error = select (getdtablesize (),
			    &in_fdset, NULL, &ex_fdset, NULL);
	    switch (error)
	      {
	      case -1:
	      case 0:
		if (errno == EINTR) goto retry;
		perror ("Some error during select occured");
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					       "Some error during select occured") );
		break;
	      case 1:
	      case 2:
		if (FD_ISSET (devdesc, &in_fdset))
		  ;
		if (FD_ISSET (devdesc, &ex_fdset))
		  vpItifg8Timeout = true;
		break;
	      default:
		perror ("unexpected select return value");
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					       "unexpected select return value") );
	      }
#endif
	  }
	  break;
	case MANUAL_MODE:
	  if (verbose) fprintf(stdout,"m");
	  getchar ();
	  break;
	case POLL_MODE:
	  {
	    off_t start, current;

	    if (verbose) fprintf(stdout,"m");
	    if ((start = lseek (devdesc, 0L, SEEK_END)) < 0)
	      {
		perror ("lseek 0/SEEK_END");
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					       "lseek 0/SEEK_END") );
	      }

	    current = start;
	    while (current == start)
	      {
		if ((current = lseek (devdesc, 0L, SEEK_END)) < 0)
		  {
		    perror ("lseek 0/SEEK_END");
		    close();
		    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						   "lseek 0/SEEK_END") );
		  }
		usleep (10000L);
	      }
	  }
	  break;
	}

      /* 3. confirm acquisition */
      if (args.other[args.board_i])
	{
	  do
	    if ((nextone = lseek (devdesc, (loff_t)0, SEEK_END)) < 0)
	      {
		if (errno != EINTR)
		  {
		    perror ("lseek query SEEK_END");
		    close();
		    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						   "lseek query SEEK_END") );
		  }
	      }
	    else
	      break;
	  while (true);
	  do
	    if ((thisone = lseek (devdesc, (loff_t)0, SEEK_CUR)) < 0)
	      {
		if (errno != EINTR)
		  {
		    perror ("lseek query SEEK_CUR");
		    close();
		    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						   "lseek query SEEK_CUR") );
		  }
	      }
	    else
	      break;
	  while (true);
	  nextone -= thisone;

	  if (vpItifg8Timeout) break;

	  for (int count = 0; count < nextone / image[args.board_i].paged_size; count++)
	    {
	      if (args.contig[args.board_i])
		srcoff[srcidx] = thisone;
	      else
		srcoff[srcidx] = done;
	      /* avoid window overflow, so use wraparound */
	      srcoff[srcidx] %= ((winsize / (loff_t)image[args.board_i].paged_size) *
				 (loff_t)image[args.board_i].paged_size);
	      if (verbose)
		fprintf(stdout, "%ld",srcoff[srcidx] / image[args.board_i].paged_size);
	      srcidx++;
	    }

	  do
	    if ((thisone = lseek (devdesc, nextone, SEEK_CUR)) < 0)
	      {
		if (errno != EINTR)
		  {
		    perror ("lseek accept SEEK_CUR");
		    close();
		    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						   "lseek accept SEEK_CUR") );
		  }
	      }
	    else
	      break;
	  while (TRUE);

	  done += nextone;
	}
      else
	{
	  if ((nextone = read (devdesc, image[args.board_i].srcptr + done, todo - done)) < 0)
	    {
	      if (errno == ETIME)
		vpItifg8Timeout = TRUE;
	      else
		{
// 		  if (errno == EINTR)
// 		    goto done;
// 		  else
//		    {
		      perror ("Can't read image");
		      close();
		      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
						     "Can't read image") );
// 		    }
		}
	    }

	  if (vpItifg8Timeout) break;

	  for (int count = 0; count < nextone / image[args.board_i].paged_size; count++)
	    {
	      if (verbose) fprintf(stdout,"-");
	      srcoff[srcidx] = done;
	      srcidx++;
	    }

	  done += nextone;
	}
    }
  while (done < todo);
  if (!args.contig[args.board_i]) stop_it = false;

  return((u_char *)(image[args.board_i].srcptr + image[args.board_i].srcoff));
}
#endif


/*!
  Acquire a grey level image.

  \param I : Image data structure (8 bits image)

  \exception vpFrameGrabberException::initializationError : Driver not
  initialized. You have to call open() before acquire().

  \exception vpFrameGrabberException::otherError : Can't acquire an image.

  \sa open(vpImage<unsigned char> &), getField()
*/
void
vpItifg8Grabber::acquire(vpImage<unsigned char> &I)
{

  if (init == false) {
    vpERROR_TRACE("Itifg not initialized ") ;

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Itifg not initialized ") );
  }

  unsigned char *bitmap = NULL ;

  try {
    unsigned int width = image[args.board_i].width;
    unsigned int height = image[args.board_i].height;
    unsigned int size = height * width;

    bitmap = acquire();

    // Specific code for AM-STD COMP board to be able to acquire at 25 or 50
    // fps
    if (getModule() == ICP_AMCMP) {
      bool field = getField(); // Field of the acquired frame (odd or even)
      switch (args.amcmp_rate[args.board_i]) {
      case framerate_25fps:
	// If its the first acquisition, set the reference frame type to the
	// last acquire frame type (odd, even)
	if (first_acq == false) {
	  ref_field = field;
	  first_acq = true;
	}
	if (getVDecimation() != 1) {
	  // If vertical subsampling, we get only the even frame
	  while (field != ref_field) {
	    // Last acquired field is odd, restart a new acquisition
	    bitmap = acquire();
	    field = getField(); // Field of the acquired frame (odd or even)
	  }
	}
	break;
      case framerate_50fps:
      default:
	break;
      }
    }

    if ((I.getWidth() != width) || (I.getHeight() != height))
      I.resize(height, width) ;

    switch (image[args.board_i].srcbpp) {
    case 8:
      memcpy(I.bitmap, bitmap, size);
      break;
    case 24:
      vpImageConvert::RGBToGrey(bitmap, I.bitmap, size);
      break;
    case 32:
      vpImageConvert::RGBaToGrey(bitmap, I.bitmap, size);
      break;
    default:
      vpERROR_TRACE("Image conversion from itifg %d bpp to vpImage<uchar> not implemented", image[args.board_i].srcbpp);
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't acquire an image") );
    }
  }
  catch(...) {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't acquire an image") );
  }
}

/*!
  Acquire a color image

  \param I : Image data structure (32 bits image)

  \exception vpFrameGrabberException::initializationError : Driver not
  initialized. You have to call open() before acquire().

  \exception vpFrameGrabberException::otherError : Can't acquire an image.

  \sa open(vpImage<vpRGBa> &), getField()
*/
void
vpItifg8Grabber::acquire(vpImage<vpRGBa> &I)
{
  if (init == false) {
    vpERROR_TRACE("Itifg not initialized ") ;

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Itifg not initialized ") );
  }

  unsigned char *bitmap = NULL ;

  try {
    unsigned int width = image[args.board_i].width;
    unsigned int height = image[args.board_i].height;
    unsigned int size = height * width;

    bitmap = acquire();

    // Specific code for AM-STD COMP board to be able to acquire at 25 or 50
    // fps
    if (getModule() == ICP_AMCMP) {
      bool field = getField(); // Field of the acquired frame (odd or even)
      switch (args.amcmp_rate[args.board_i]) {
      case framerate_25fps:
	// If its the first acquisition, set the reference frame type to the
	// last acquire frame type (odd, even)
	if (first_acq == false) {
	  ref_field = field;
	  first_acq = true;
	}
	if (getVDecimation() != 1) {
	  // If vertical subsampling, we get only the even frame
	  while (field != ref_field) {
	    // Last acquired field is odd, restart a new acquisition
	    bitmap = acquire();
	    field = getField(); // Field of the acquired frame (odd or even)
	  }
	}
	break;
      case framerate_50fps:
      default:
	break;
      }
    }

    if ((I.getWidth() != width) || (I.getHeight() != height))
      I.resize(height, width) ;

    switch (image[args.board_i].srcbpp) {
    case 8:
      vpImageConvert::GreyToRGBa(bitmap, &I.bitmap[0].R, size);
      break;
    case 16:
      vpImageConvert::YCbCrToRGBa(bitmap, &I.bitmap[0].R, size);
      break;
    case 24:
      vpImageConvert::RGBToRGBa(bitmap, &I.bitmap[0].R, size);
      break;
    case 32:
      memcpy(I.bitmap, bitmap, size * 4);
      break;
    default:
      vpERROR_TRACE("Image conversion from itifg %d bpp to vpImage<uchar> not implemented", image[args.board_i].srcbpp);
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't acquire an image") );
    }
  }
  catch(...) {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't acquire an image") );
  }
}

/*!

  In case of using an AM-STD-COMP board, get the field (odd or even) of the
  last acquired frame. For other boards, return always 0.

  \warning Dedicated AM-STD-COMP function member.

  To be able to get the type of the frames (odd or even), the itifg-8.x driver
  was modified in itifg-8.2.2-0/src/module/amcmpIface.c or
  itifg-8.3.1-12/include/amcmpReg.h like:

  \code
  static int
  amcmp_copy_config (icp_mod_t *mod,
		    union iti_cam_t *dstcnf, union iti_cam_t *srccnf)
  {
    int error;

    if ((error = cmp_get_status (mod->c, &srccnf->cmp_cam.status)))
      return error;
    iti_memcpy (dstcnf, srccnf, sizeof(struct cmp_cam_t));

    return OK;
  }

  \endcode


  \return 0: Even frame field.
  \return 1: Odd frame field.

*/
bool
vpItifg8Grabber::getField()
{

  if (getModule() != ICP_AMCMP)
    return 0;

  union iti_cam_t camconf;
  if (ioctl (devdesc, GIOC_GET_CAMCNF, &camconf) < 0)
  {
    perror ("GIOC_GET_CAMCNF");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "GIOC_GET_CAMCNF") );
  }

  u_char status;
  bool field_status;

  status = camconf.cmp_cam.status;

  field_status = (status &  vpCMP_BT829A_FIELD_MASK) >> vpCMP_BT829A_FIELD_SHIFT;

  if (verbose)
    fprintf(stdout, "status: 0x%x field: 0x%x\n", status, field_status);
  return field_status;
}



#endif
