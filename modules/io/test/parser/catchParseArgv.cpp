/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Test for vpParseArgv class.
 */
/*!
  \example catchParseArgv.cpp

  \brief Test parsing command line argument with vpParseArgv functionalities.

*/

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <functional>

#if defined(_WIN32)
#  include <io.h>
#  define fd_dup _dup
#  define fd_dup2 _dup2
#  define fd_fileno _fileno
#else
#  include <unistd.h>
#  define fd_dup dup
#  define fd_dup2 dup2
#  define fd_fileno fileno
#endif

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#if defined(VISP_BUILD_CATCH2)
#include <catch_amalgamated.hpp>
#else // Since v3.1.1
#include <catch2/catch_all.hpp>
#endif


#include <visp3/io/vpParseArgv.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
/*
 * Small helper that owns the argument strings and exposes a mutable
 * `const char **argv` array (with a dummy argv[0] program name) that
 * vpParseArgv::parse() is allowed to rewrite in place.
 */
class ArgvHolder
{
public:
  explicit ArgvHolder(const std::vector<std::string> &tokens)
  {
    m_storage.push_back("program_name");
    for (const auto &token : tokens) {
      m_storage.push_back(token);
    }
    for (auto &s : m_storage) {
      m_ptrs.push_back(s.c_str());
    }
  }

  int argc() const { return static_cast<int>(m_ptrs.size()); }
  const char **argv() { return m_ptrs.data(); }

private:
  std::vector<std::string> m_storage;
  std::vector<const char *> m_ptrs;
};
} // namespace

TEST_CASE("Parse all supported option types", "[vpParseArgv]")
{
  bool b_val = false;
  int i_val = 0;
  long l_val = 0;
  float f_val = 0.f;
  double d_val = 0.;
  const char *s_val = nullptr;

  vpParseArgv::vpArgvInfo argTable[] = {
    {"-bool", vpParseArgv::ARGV_CONSTANT_BOOL, (char *)nullptr, (char *)&b_val, "Boolean flag."},
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {"-long", vpParseArgv::ARGV_LONG, (char *)nullptr, (char *)&l_val, "A long value."},
    {"-float", vpParseArgv::ARGV_FLOAT, (char *)nullptr, (char *)&f_val, "A float value."},
    {"-double", vpParseArgv::ARGV_DOUBLE, (char *)nullptr, (char *)&d_val, "A double value."},
    {"-string", vpParseArgv::ARGV_STRING, (char *)nullptr, (char *)&s_val, "A string value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-bool", "-int", "42", "-long", "987654", "-float", "2.5", "-double", "6.28", "-string", "hello" });
  int argc = holder.argc();

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable, vpParseArgv::ARGV_NO_DEFAULTS);

  CHECK_FALSE(error);
  CHECK(b_val == true);
  CHECK(i_val == 42);
  CHECK(l_val == 987654);
  CHECK(f_val == Catch::Approx(2.5f));
  CHECK(d_val == Catch::Approx(6.28));
  REQUIRE(s_val != nullptr);
  CHECK(std::string(s_val) == "hello");
}

TEST_CASE("Unique abbreviation resolves to the matching option", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-integer", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-int", "7" });
  int argc = holder.argc();

  // Abbreviation allowed since ARGV_NO_ABBREV is not set and "-int" is an
  // unambiguous prefix of "-integer".
  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable, vpParseArgv::ARGV_NO_DEFAULTS);

  CHECK_FALSE(error);
  CHECK(i_val == 7);
}

TEST_CASE("Ambiguous abbreviation is rejected", "[vpParseArgv]")
{
  int i1_val = 0, i2_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-integer1", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i1_val, "First integer value."},
    {"-integer2", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i2_val, "Second integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-integer", "7" });
  int argc = holder.argc();

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_DEFAULTS | vpParseArgv::ARGV_NO_PRINT);

  CHECK(error);
}

TEST_CASE("ARGV_NO_ABBREV forces an exact key match", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-integer", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-int", "7" });
  int argc = holder.argc();

  // "-int" no longer matches "-integer": it is not recognized and becomes
  // a leftover argument instead of raising an "ambiguous option" error.
  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_ABBREV | vpParseArgv::ARGV_NO_DEFAULTS |
                                   vpParseArgv::ARGV_NO_PRINT);

  CHECK_FALSE(error);
  CHECK(i_val == 0);
  REQUIRE(argc == 3); // program name + "-int" + "7" left over
  CHECK(std::string(holder.argv()[1]) == "-int");
}

TEST_CASE("Unrecognized argument is reported as a leftover", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-int", "5", "leftover_arg" });
  int argc = holder.argc();

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_DEFAULTS | vpParseArgv::ARGV_NO_PRINT);

  CHECK_FALSE(error);
  CHECK(i_val == 5);
  REQUIRE(argc == 2); // program name + "leftover_arg"
  CHECK(std::string(holder.argv()[1]) == "leftover_arg");
}

TEST_CASE("Missing value after an option that expects one is an error", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-int" }); // no value provided
  int argc = holder.argc();

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_DEFAULTS | vpParseArgv::ARGV_NO_PRINT);

  CHECK(error);
}

TEST_CASE("Non numeric value for an integer option is an error", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-int", "not_a_number" });
  int argc = holder.argc();

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_DEFAULTS | vpParseArgv::ARGV_NO_PRINT);

  CHECK(error);
}

TEST_CASE("Default -help option triggers usage and returns an error", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-help" });
  int argc = holder.argc();

  // Default table (-help) is enabled here since ARGV_NO_DEFAULTS is not set.
  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable, vpParseArgv::ARGV_NO_PRINT);

  CHECK(error);
}

TEST_CASE("Help option prints usage output", "[vpParseArgv]")
{
  int i_val = 0;
  vpParseArgv::vpArgvInfo argTable[] = {
    {"-help", vpParseArgv::ARGV_HELP, (char *)nullptr, (char *)nullptr, "Show this help message."},
    {"-int", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&i_val, "An integer value."},
    {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr}
  };

  ArgvHolder holder({ "-help" });
  int argc = holder.argc();

  int savedStderr = fd_dup(fd_fileno(stderr));
  REQUIRE(savedStderr != -1);

  FILE *tmp = tmpfile();
  REQUIRE(tmp != nullptr);

  fflush(stderr);
  REQUIRE(fd_dup2(fd_fileno(tmp), fd_fileno(stderr)) != -1);

  bool error = vpParseArgv::parse(&argc, holder.argv(), argTable,
                                   vpParseArgv::ARGV_NO_DEFAULTS);

  fflush(stderr);
  REQUIRE(fd_dup2(savedStderr, fd_fileno(stderr)) != -1);
  close(savedStderr);

  std::string output;
  rewind(tmp);
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), tmp)) {
    output += buffer;
  }
  fclose(tmp);

  CHECK(error);
  CHECK(output.find("Command-specific options:") != std::string::npos);
  CHECK(output.find("-help:") != std::string::npos);
  CHECK(output.find("Show this help message.") != std::string::npos);
  CHECK(output.find("-int:") != std::string::npos);
  CHECK(output.find("An integer value.") != std::string::npos);
}

TEST_CASE("Getopt-style single character parsing", "[vpParseArgv]")
{
  // vpParseArgv::parse(argc, argv, validOpts, param) keeps its position in
  // a function-local *static* variable, so it must be driven from a single,
  // uninterrupted sequence of calls (no Catch2 SECTIONs here) to keep the
  // expected option order under control.
  const char *validOpts = "bi:l:h";

  ArgvHolder holder({ "-b", "-i", "42", "-l", "1000", "standalone", "-h" });
  int argc = holder.argc();
  const char **argv = holder.argv();
  const char *param = nullptr;

  int c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 'b');
  CHECK(param == nullptr);

  c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 'i');
  REQUIRE(param != nullptr);
  CHECK(std::string(param) == "42");

  c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 'l');
  REQUIRE(param != nullptr);
  CHECK(std::string(param) == "1000");

  c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 1); // standalone argument, no option specifier
  REQUIRE(param != nullptr);
  CHECK(std::string(param) == "standalone");

  c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 'h');
  CHECK(param == nullptr);

  c = vpParseArgv::parse(argc, argv, validOpts, &param);
  CHECK(c == 0); // end of argument list
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  std::cout << (numFailed ? "Test failed" : "Test succeed") << std::endl;
  return numFailed;
}

#else

int main()
{
  std::cout << "Test skipped because Catch2 is not available" << std::endl;
  return EXIT_SUCCESS;
}

#endif
