#############################################################################
#
# $Id: DartConfig.cmake,v 1.9 2008-02-08 15:10:13 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Dart configuration.
#
# Authors:
# Fabien Spindler
#
#############################################################################

SET (DROP_METHOD "xmlrpc")
SET (DROP_SITE "http://dart.irisa.fr")
SET (DROP_LOCATION "ViSP")
SET (COMPRESS_SUBMISSION ON)
SET (NIGHTLY_START_TIME "9:00PM")

# Project Home Page
SET (PROJECT_URL "http://www.irisa.fr/lagadic/visp/visp.html")

# Problem build email delivery variables
SET (DELIVER_BROKEN_BUILD_EMAIL "Nightly")
SET (SMTP_MAILHOST "smtp.national.inria.fr")
SET (EMAIL_FROM "Fabien.Spindler@irisa.fr")
SET (DARTBOARD_BASE_URL "http://dart.irisa.fr/ViSP/Dashboard/")
SET (EMAIL_PROJECT_NAME "ViSP")

SET (CVS_IDENT_TO_EMAIL "{fspindle Fabien.Spindler@irisa.fr}")
SET (DELIVER_BROKEN_BUILD_EMAIL_WITH_CONFIGURE_FAILURES 1)
SET (DELIVER_BROKEN_BUILD_EMAIL_WITH_BUILD_ERRORS 1)
SET (DELIVER_BROKEN_BUILD_EMAIL_WITH_BUILD_WARNINGS 1)
SET (DELIVER_BROKEN_BUILD_EMAIL_WITH_TEST_NOT_RUNS 1)
SET (DELIVER_BROKEN_BUILD_EMAIL_WITH_TEST_FAILURES 1)
