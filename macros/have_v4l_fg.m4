dnl Calls HAVE_V4L_FG (contained in this file) as a subroutine.
AC_DEFUN([HAVE_V4L_FG],
[
#-----------------------------------------------------------------------
# check if video 4 Linux is reachable
AC_MSG_CHECKING(whether Video For Linux api works)
AC_PREPROC_IFELSE([AC_LANG_SOURCE([[#include <linux/videodev.h>]])],[have_include_videodev=yes],[have_include_videodev=no])

if test "x$have_include_videodev" = "xyes"; then
  AC_DEFINE(HAVE_INCLUDE_VIDEODEV, 1, 
    [Define to 1 if the include linux/videodev.h for Video4Linux acquisition is available])
  AC_MSG_RESULT(yes)
else
  AC_MSG_RESULT(no)
fi
])
