
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageIoPnm.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand, Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageIoPnm.cpp,v 1.4 2006-06-23 14:45:05 brenier Exp $
 *
 * Description
 * ============
 *
 * Read/write pnm images
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpImageIoPnm.cpp
  \brief Read/write pnm images
*/

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

//image  conversion
#include <visp/vpImageConvert.h>
#define MAX_LEN 100



//--------------------------------------------------------------------------
// PGM
//--------------------------------------------------------------------------

/*!
  \brief
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.
*/

void
vpImageIo::writePGM(const vpImage<unsigned char> &I,
		    const char *filename)
{

  FILE* fd;

  // Test the filename
  if (filename == '\0')   {
     vpERROR_TRACE("no filename\n");
    throw (vpImageException(vpImageException::ioError,
		       "no filename")) ;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
     vpERROR_TRACE("couldn't write to file %s\n",  filename);
    throw (vpImageException(vpImageException::ioError,
		       "cannot write file")) ;
  }

  // Write the head
  fprintf(fd, "P5\n");					// Magic number
  fprintf(fd, "%d %d\n", I.getCols(), I.getRows());	// Image size
  fprintf(fd, "255\n");					// Max level

  // Write the bitmap
  int ierr;
  unsigned nbyte = I.getCols()*I.getRows();

  ierr = fwrite(I.bitmap, sizeof(unsigned char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fclose(fd);
    vpERROR_TRACE("couldn't write %d bytes to file %s\n",
	    nbyte, filename) ;
    throw (vpImageException(vpImageException::ioError,
		       "cannot write file")) ;
  }

  fflush(fd);
  fclose(fd);

}
/*!
  \brief
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.
*/

void
vpImageIo::writePGM(const vpImage<short> &I,
		    const char *filename)
{
  vpImage<unsigned char> Iuc ;
  int nrows = I.getRows();
  int ncols = I.getCols();

  Iuc.resize(nrows, ncols);

  for (int i=0 ; i < nrows * ncols ; i++)
    Iuc.bitmap[i] =  (unsigned char)I.bitmap[i] ;

  vpImageIo::writePGM(Iuc, filename) ;


}
/*!
  \brief
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.
  Color image is converted into a grayscale image.
*/

void
vpImageIo::writePGM(const vpImage<vpRGBa> &I,
		    const char *filename)
{

  FILE* fd;

  // Test the filename
  if (filename == '\0')   {
     vpERROR_TRACE("no filename\n");
    throw (vpImageException(vpImageException::ioError,
		       "no filename")) ;
  }

  fd = fopen(filename, "wb");

  if (fd == NULL) {
     vpERROR_TRACE("couldn't write to file %s\n",  filename);
    throw (vpImageException(vpImageException::ioError,
		       "cannot write file")) ;
  }

  // Write the head
  fprintf(fd, "P5\n");					// Magic number
  fprintf(fd, "%d %d\n", I.getCols(), I.getRows());	// Image size
  fprintf(fd, "255\n");					// Max level

  // Write the bitmap
  int ierr;
  unsigned nbyte = I.getCols()*I.getRows();


  vpImage<unsigned char> Itmp ;
  vpImageConvert::convert(I,Itmp) ;

  ierr = fwrite(Itmp.bitmap, sizeof(unsigned char), nbyte, fd) ;
  if (ierr == ! nbyte) {
    fclose(fd);
    vpERROR_TRACE("couldn't write %d bytes to file %s\n",
	    nbyte, filename) ;
    throw (vpImageException(vpImageException::ioError,
		       "cannot write file")) ;
  }

  fflush(fd);
  fclose(fd);

}


/*!
  \brief read a PGM file and initialize  a scalar image


  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap whith the content of
  the file.


  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

*/

void
vpImageIo::readPGM(vpImage<unsigned char> &I,
		   const char *filename)
{
  FILE* fd = NULL; // File descriptor
  int   ierr;
  int   line;
  int   is255;
  char* err ;
  char  str[MAX_LEN];
  int   w, h;

  // Test the filename
  if (filename == '\0')
  {
    vpERROR_TRACE("no filename") ;
    throw (vpImageException(vpImageException::ioError,
			    " no filename")) ;

  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL)
  {
    vpERROR_TRACE("couldn't read file %s", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  // Read the first line with magic number P5
  line = 0;

  err = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (err == NULL)
  {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",  line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if (strlen(str) != 3)
  {
    fclose (fd);
    vpERROR_TRACE("%s is not a PGM file\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "this is not a pgm file")) ;
  }

  str[2] = '\0';
  if (strcmp(str, "P5") != 0)
  {
    fclose (fd);
    vpERROR_TRACE("%s is not a PGM file\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "this is not a pgm file")) ;
  }

  // Jump the possible comment, or empty line and read the following line
  do {
    err = fgets(str, MAX_LEN - 1, fd);
    line++;
    if (err == NULL) {
      fprintf(stderr, "couldn't read line %d of file %s\n", line, filename);
      fclose (fd);
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract image size
  ierr = sscanf(str, "%d %d", &w, &h);
  if (ierr == EOF)
  {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if ((h != I.getRows())||( w != I.getCols()))
  {

    try
    {
      I.resize(h,w) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }

  // Read 255
  err = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (err == NULL) {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  ierr = sscanf(str, "%d", &is255);
  if (ierr == EOF) {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n", line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if (is255 != 255)
  {
    fclose (fd);
    vpERROR_TRACE("MAX_VAL is not 255 in file %s\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "error reading pgm file")) ;
  }

  unsigned int nbyte = I.getRows()*I.getCols();
  if (fread (I.bitmap, sizeof(unsigned char), nbyte, fd ) != nbyte)
  {
    fclose (fd);
    vpERROR_TRACE("couldn't read %d bytes in file %s\n", nbyte, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "error reading pgm file")) ;
  }

  fclose (fd);


}


/*!
  \brief read a PGM file and initialize  a scalar image


  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap whith the content of
  the file.


  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

*/

void
vpImageIo::readPGM(vpImage<vpRGBa> &I,
		   const char *filename)
{

  try
  {
    vpImage<unsigned char> Itmp ;

    vpImageIo::readPGM(Itmp, filename) ;


    vpImageConvert::convert(Itmp, I) ;


    printf("%d \n",I[10][10].R);
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}


//--------------------------------------------------------------------------
// PPM
//--------------------------------------------------------------------------

/*!
  \brief
  Read the contents of the portable pixmap (PPM P6) filename, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap whith the gray level data. That means that the image \e I is a
  "black and white" rendering of the original image in \e filename, as in a
  black and white photograph. The quantization formula used is \f$0,299 r +
  0,587 g + 0,114 b\f$.


  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.


*/
void
vpImageIo::readPPM(vpImage<unsigned char> &I, const char *filename)
{

  try
  {
    vpImage<vpRGBa> Itmp ;

    vpImageIo::readPPM(Itmp, filename) ;

    vpImageConvert::convert(Itmp, I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}


/*!  \brief Read the contents of the portable pixmap (PPM P6) filename,
  allocate memory for the corresponding vpRGBa image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.
*/
void
vpImageIo::readPPM(vpImage<vpRGBa> &I, const char *filename)
{

  FILE* fd = NULL; // File descriptor
  int   ierr;
  int   line;
  int   is255;
  char* err ;
  char  str[MAX_LEN];
  int   w, h;

  // Test the filename
  if (filename == '\0')
  {
    vpERROR_TRACE("no filename") ;
    throw (vpImageException(vpImageException::ioError,
			    " no filename")) ;

  }

  // Open the filename
  fd = fopen(filename, "rb");
  if (fd == NULL)
  {
    vpERROR_TRACE("couldn't read file %s", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  // Read the first line with magic number P5
  line = 0;

  err = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (err == NULL)
  {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",  line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if (strlen(str) != 3)
  {
    fclose (fd);
    vpERROR_TRACE("%s is not a PPM file\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "this is not a ppm file")) ;
  }

  str[2] = '\0';
  if (strcmp(str, "P6") != 0)
  {
    fclose (fd);
    vpERROR_TRACE("%s is not a PPM file\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "this is not a ppm file")) ;
  }

  // Jump the possible comment, or empty line and read the following line
  do {
    err = fgets(str, MAX_LEN - 1, fd);
    line++;
    if (err == NULL) {
      fprintf(stderr, "couldn't read line %d of file %s\n", line, filename);
      fclose (fd);
    }
  } while ((str[0] == '#') || (str[0] == '\n'));

  // Extract image size
  ierr = sscanf(str, "%d %d", &w, &h);
  if (ierr == EOF)
  {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if ((h != I.getRows())||( w != I.getCols()))
  {

    try
    {
      I.resize(h,w) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }

  // Read 255
  err = fgets(str, MAX_LEN - 1, fd);
  line++;
  if (err == NULL) {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n",line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  ierr = sscanf(str, "%d", &is255);
  if (ierr == EOF) {
    fclose (fd);
    vpERROR_TRACE("couldn't read line %d of file %s\n", line, filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "couldn't read file")) ;
  }

  if (is255 != 255)
  {
    fclose (fd);
    vpERROR_TRACE("MAX_VAL is not 255 in file %s\n", filename) ;
    throw (vpImageException(vpImageException::ioError,
			    "error reading ppm file")) ;
  }


  int i,j ;
  for(i=0;i<I.getRows();i++)
  {
    for(j=0;j<I.getCols();j++)
    {
      vpRGBa v ;
      int res = fread(&v.R,sizeof(v.R),1,fd) ;
      res |= fread(&v.G,sizeof(v.G),1,fd) ;
      res |= fread(&v.B,sizeof(v.B),1,fd) ;
      if (res==0)
      {
	 fclose (fd);
	 vpERROR_TRACE("couldn't read  bytes in file %s\n", filename) ;
	 throw (vpImageException(vpImageException::ioError,
				 "error reading ppm file")) ;
      }
      I[i][j] = v ;
    }
  }
  fclose(fd) ;

}

/*!
  \brief
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PPM P6) file.
  grayscale image is converted into a color image vpRGBa.
*/

void
vpImageIo::writePPM(const vpImage<unsigned char> &I, const char *filename)
{

  try
  {
    vpImage<vpRGBa> Itmp ;

    vpImageConvert::convert(I, Itmp) ;

    vpImageIo::writePPM(Itmp, filename) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  \brief
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PPM P6) file.
*/
void
vpImageIo::writePPM(const vpImage<vpRGBa> &I, const char *filename)
{

  FILE* f;


  // Test the filename
  if (filename == '\0')   {
     vpERROR_TRACE("no filename\n");
    throw (vpImageException(vpImageException::ioError,
		       "no filename")) ;
  }

  f = fopen(filename, "wb");

  if (f == NULL) {
     vpERROR_TRACE("couldn't write to file %s\n",  filename);
     throw (vpImageException(vpImageException::ioError,
			     "cannot write file")) ;
  }



  fprintf(f,"P6\n");			         // Magic number
  fprintf(f,"%d %d\n", I.getCols(), I.getRows());	// Image size
  fprintf(f,"%d\n",255);	        	// Max level

  int i,j ;
  for(i=0;i<I.getRows();i++)
  {
    for(j=0;j<I.getCols();j++)
    {
      vpRGBa P ;
      int res ;
      P = I[i][j] ;
      unsigned char   tmp ;
      tmp = P.R ;
      res = fwrite(&tmp,sizeof(tmp),1,f) ;
      if (res==0)
      {
	fclose(f);
	vpERROR_TRACE("couldn't write file") ;
	throw (vpImageException(vpImageException::ioError,
				"cannot write file")) ;
      }
      tmp = P.G;
      res = fwrite(&tmp,sizeof(tmp),1,f) ;
      if (res==0)
      {
	fclose(f);
	vpERROR_TRACE("couldn't write file") ;
	throw (vpImageException(vpImageException::ioError,
				"cannot write file")) ;
      }
      tmp = P.B ;
      res = fwrite(&tmp,sizeof(tmp),1,f) ;
      if (res==0)
      {
	fclose(f);
	vpERROR_TRACE("couldn't write file") ;
	throw (vpImageException(vpImageException::ioError,
				"cannot write file")) ;
      }
    }
  }

  fflush(f);
  fclose(f);
}

#undef MAX_LEN
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
