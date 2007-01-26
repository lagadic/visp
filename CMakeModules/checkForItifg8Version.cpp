#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <itifgExt.h>
#include <libitifg.h>


int main()
{
  struct iti_setup_t setup;
  char file_name[FILENAME_MAX];
  int error;
  int version = 0;

  strcpy (file_name, ITI_PFS_PREFIX);
  strcat (file_name, ITI_PFS_STRING);

  if ((error = iti_parse_info (file_name, &setup)))
    {
      switch (error)
	{
	case -ITI_EARG:
	  fprintf (stderr, "read_procfs: Wrong call argument\n"
		   "(file_name:%s, setup%p).\n", file_name, &setup);
	  exit (-1);
	  break;
	case -ITI_EFMT:
	  fprintf (stderr, "read_procfs: Wrong data format (%s%s).\n",
		   ITI_PFS_PREFIX, ITI_PFS_STRING);
	  exit (-1);
	  break;
	case -ITI_EENT:
	  fprintf (stderr, "read_procfs: File not found (%s%s).\n",
		   ITI_PFS_PREFIX, ITI_PFS_STRING);
	  exit (-1);
	  break;
	case -ITI_ESYS:
	  fprintf (stderr, "read_procfs: Error while system call (%s).\n",
		   strerror (errno));
	  exit (-1);
	  break;
	default:
	  fprintf (stderr, "read_procfs: Error not specified!\n");
	  exit (-1);
	}
    }
//   fprintf (stderr, "Driver version %0x = %d: %d.%d.%d-%d.\n",
// 	   setup.version, setup.version,
// 	   (setup.version & 0xFF000000) >> 24,
// 	   (setup.version & 0x00FF0000) >> 16,
// 	   (setup.version & 0x0000FF00) >> 8,
// 	   setup.version & 0x000000FF);

  int major = (setup.version & 0xFF000000) >> 24;
  int minor = (setup.version & 0x00FF0000) >> 16;

  return (major*10 + minor);

}
