# Set SRC_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update SRC_ALL variable if you add/remove a SRC_subdir 
# variable
#
# If you add/remove a directory, modify here
SET (SRC_EXAMPLE
  example/exExample.cpp
  )

SET (SRC_ALL
  ${SRC_EXAMPLE}
  )
