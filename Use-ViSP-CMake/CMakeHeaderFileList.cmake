# Set HEADER_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update HEADER_ALL variable if you add/remove a 
# HEADER_subdir variable
#
# If you add/remove a directory, modify here

SET (HEADER_EXAMPLE
  example/exExample.h
  )


SET (HEADER_ALL 
  ${HEADER_EXAMPLE}
  )
