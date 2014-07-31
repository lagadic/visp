/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Test functions in vpIoTools.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!

  \example testIoTools.cpp

  \brief Test functions in IoTools.

*/

#include <iostream>
#include <visp/vpIoTools.h>


int
main(int argc, const char ** argv)
{
	const char c = vpIoTools::separator;
	if(c == '\\')
	{
		std::cout << "The directory separator character is '" << c << "' (Windows platform)." << std::endl;
	}
	else {
		std::cout << "The directory separator character is '" << c << "' (Unix like platform)." << std::endl;
	}


	std::string pathname = "";
#if defined(_WIN32)
	pathname = "C:\\Program Files (x86)\\Java\\jre7";
#else
	pathname = "/usr/bin/java";
#endif

	std::cout << "Parent of " << pathname << " is " << vpIoTools::getParent(pathname) << std::endl;
	std::cout << "Name of " << pathname << " is " << vpIoTools::getName(pathname) << std::endl;


	if(argc == 3 && std::string(argv[1]) == std::string("-i"))
	{
		std::cout << "Parent of " << argv[2] << " is " << vpIoTools::getParent(argv[2]) << std::endl;
		std::cout << "Name of " << argv[2] << " is " << vpIoTools::getName(argv[2]) << std::endl;
	}

	std::string windowsPathnameStyle = "\\usr\\bin\\java";
	std::cout << "Parent of " << windowsPathnameStyle << " is " << vpIoTools::getParent(windowsPathnameStyle) << std::endl;
	std::cout << "Name of " << windowsPathnameStyle << " is " << vpIoTools::getName(windowsPathnameStyle) << std::endl;

	std::string parent = "/usr/toto/", child = "\\blabla\\java";
	std::cout << "parent=" << vpIoTools::path(parent) << " ; child=" << vpIoTools::path(child) << std::endl;
	std::cout << "Create file path from parent=" << parent << " and child=" << child << " is "
			<< vpIoTools::createFilePath(parent, child) << std::endl;

	std::string expandPath = "~/Documents/fictional directory/fictional file";
	std::cout << "Path for " << expandPath << " is " << vpIoTools::path(expandPath) << std::endl;

	std::cout << "Test get name with an empty pathname=" << vpIoTools::getName("") << std::endl;
	std::cout << "Get parent with an empty pathname=" << vpIoTools::getParent("") << std::endl;
	std::cout << "Get parent with a filename=" << vpIoTools::getParent("my_file.txt") << std::endl;
	expandPath = "~/Documents/fictional dir/fictional file.txt";
	std::cout << "Get name with a unix expand pathname " << expandPath << "=" << vpIoTools::getName(expandPath) << std::endl;
	std::cout << "Get parent with a unix expand pathname " << expandPath << "=" << vpIoTools::getParent(expandPath) << std::endl;


	pathname = "c:/dir";
	std::cout << "pathname=" << vpIoTools::splitDrive(pathname).first << " ; " << vpIoTools::splitDrive(pathname).second << std::endl;

	std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

	pathname = "c:/dir/fictional directory/fictional file.txt";
	std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

	pathname = "/home/user/Documents/fictional directory/fictional file.txt";
	std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

	pathname = "~/Documents/fictional directory/fictional file.txt";
	std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

	pathname = "fictional directory/fictional file.txt";
	std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;


	std::cout << std::endl << "End" << std::endl;

	return 0;
}
