#ifndef __record_helper_
#define __record_helper_

#include <string>
#include <visp3/core/vpImage.h>

bool record_helper(const std::string &opt_seqname, int opt_record_mode, const vpImage<unsigned char> &I);

#endif

