/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test vpJsonArgumentParser
 */

 /*!
   \file testJsonArgumentParser.cpp

   Test parsing command line argument and json files with vpJsonArgumentParser
 */

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpJsonArgumentParser.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)
#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

std::pair<int, std::vector<char *>> convertToArgcAndArgv(const std::vector<std::string> &args)
{
  std::vector<char *> argvs;
  argvs.reserve(args.size());
  int argc = static_cast<int>(args.size());
  for (unsigned i = 0; i < args.size(); ++i) {
    argvs.push_back(const_cast<char *>(args[i].c_str()));
  }
  return std::make_pair(argc, argvs);
}

json loadJson(const std::string &path)
{
  std::ifstream json_file(path);
  if (!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file");
  }
  json j = json::parse(json_file);
  json_file.close();
  return j;
}

void saveJson(const json &j, const std::string &path)
{
  std::ofstream json_file(path);
  if (!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file to write modifications");
  }
  json_file << j.dump();
  json_file.close();
}

SCENARIO("Parsing arguments from JSON file", "[json]")
{
  // setup test dir
  // Get the user login name

  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath() + vpIoTools::path("/") + "visp_test_json_argument_parsing");
  const std::string jsonPath = tmp_dir + "/" + "arguments.json";

  const auto modifyJson = [&jsonPath](std::function<void(json &)> modify) -> void {
    json j = loadJson(jsonPath);
    modify(j);
    saveJson(j, jsonPath);
    };

  GIVEN("Some specific arguments")
  {
    const std::string s = "hello";
    WHEN("Converting a string to a json rep")
    {
      const json js = convertCommandLineArgument<std::string>(s);
      const json truejs = s;
      THEN("Conversion is correct")
      {
        REQUIRE(js == truejs);
      }
    }
  }

  GIVEN("Some JSON parameters saved in a file, and some C++ variables")
  {
    json j = json {
      {"a", 2},
      {"b", 2.0},
      {"c", "a string"},
      {"d", true},
      {"e", {
        {"a", 5}
      }}
    };
    saveJson(j, jsonPath);

    int a = 1;
    double b = 1.0;
    std::string c = "";
    bool d = false;
    int ea = 4;
    WHEN("Declaring a parser with all parameters required")
    {
      vpJsonArgumentParser parser("A program", "--config", "/");
      parser.addArgument("a", a, true)
        .addArgument("b", b, true)
        .addArgument("c", c, true)
        .addArgument("d", d, true)
        .addArgument("e/a", ea, true);

      THEN("Calling the parser without any argument fails")
      {
        const int argc = 1;
        const char *argv[] = {
          "program"
        };

        REQUIRE_THROWS(parser.parse(argc, argv));
      }

      THEN("Calling the parser with only the JSON file works")
      {
        const int argc = 3;
        const char *argv[] = {
          "program",
          "--config",
          jsonPath.c_str()
        };
        REQUIRE_NOTHROW(parser.parse(argc, argv));
        REQUIRE(a == j["a"]);
        REQUIRE(b == j["b"]);
        REQUIRE(c == j["c"]);
        REQUIRE(d == j["d"]);
        REQUIRE(ea == j["e"]["a"]);

      }
      THEN("Calling the parser by specifying the json argument but leaving the file path empty throws an error")
      {
        const int argc = 2;
        const char *argv[] = {
          "program",
          "--config",
        };
        REQUIRE_THROWS(parser.parse(argc, argv));
      }
      THEN("Calling the parser with only the json file but deleting a random field throws an error")
      {
        const int argc = 3;
        for (const auto &jsonElem : j.items()) {
          modifyJson([&jsonElem](json &j) { j.erase(jsonElem.key()); });
          const char *argv[] = {
            "program",
            "--config",
            jsonPath.c_str()
          };
          REQUIRE_THROWS(parser.parse(argc, argv));
        }
      }
      THEN("Calling the parser with only the json file but setting a random field to null throws an error")
      {
        const int argc = 3;
        for (const auto &jsonElem : j.items()) {
          modifyJson([&jsonElem](json &j) { j[jsonElem.key()] = nullptr; });
          const char *argv[] = {
            "program",
            "--config",
            jsonPath.c_str()
          };
          REQUIRE_THROWS(parser.parse(argc, argv));
        }
      }
      THEN("Calling the parser with an invalid json file path throws an error")
      {
        const int argc = 3;
        const char *argv[] = {
          "program",
          "--config",
          "some_invalid_json/file/path.json"
        };
        REQUIRE_THROWS(parser.parse(argc, argv));
      }
      THEN("Calling the parser with only the command line arguments works")
      {
        const int newa = a + 1, newea = ea + 6;
        const double newb = b + 2.0;
        const std::string newc = c + "hello";
        const bool newd = !d;

        const std::string newdstr(newd ? "true" : "false");
        std::vector<std::string> args = {
          "program",
          "a", std::to_string(newa),
          "b", std::to_string(newb),
          "c", newc,
          "d", newdstr,
          "e/a", std::to_string(newea)
        };
        int argc;
        std::vector<char *> argv;
        std::tie(argc, argv) = convertToArgcAndArgv(args);
        REQUIRE_NOTHROW(parser.parse(argc, (const char **)(&argv[0])));
        REQUIRE(a == newa);
        REQUIRE(b == newb);
        REQUIRE(c == newc);
        REQUIRE(d == newd);
        REQUIRE(ea == newea);

      }
      THEN("Calling the parser with JSON and command line argument works")
      {
        const int newa = a + 1;
        const double newb = b + 2.0;
        std::vector<std::string> args = {
          "program",
          "--config", jsonPath,
          "a", std::to_string(newa),
          "b", std::to_string(newb)
        };
        int argc;
        std::vector<char *> argv;
        std::tie(argc, argv) = convertToArgcAndArgv(args);
        REQUIRE_NOTHROW(parser.parse(argc, (const char **)(&argv[0])));
        REQUIRE(a == newa);
        REQUIRE(b == newb);
        REQUIRE(c == j["c"]);
        REQUIRE(d == j["d"]);
        REQUIRE(ea == j["e"]["a"]);

      }
      THEN("Calling the parser with a missing argument value throws an error")
      {

        std::vector<std::string> args = {
          "program",
          "--config", jsonPath,
          "a"
        };
        int argc;
        std::vector<char *> argv;
        std::tie(argc, argv) = convertToArgcAndArgv(args);
        REQUIRE_THROWS(parser.parse(argc, (const char **)(&argv[0])));
      }
    }
  }
  THEN("Declaring a parser with an undefined nesting delimiter fails")
  {
    REQUIRE_THROWS(vpJsonArgumentParser("A program", "--config", ""));
  }
  THEN("Declaring a parser with an invalid JSON file argument fails")
  {
    REQUIRE_THROWS(vpJsonArgumentParser("A program", "", "/"));
  }

  WHEN("Instanciating a parser with some optional fields")
  {
    vpJsonArgumentParser parser("A program", "--config", "/");
    float b = 0.0;
    parser.addArgument("b", b, false);

    THEN("Calling the parser without any argument works and does not modify the default value")
    {
      float bcopy = b;
      const int argc = 1;
      const char *argv[] = {
        "program"
      };

      REQUIRE_NOTHROW(parser.parse(argc, argv));
      REQUIRE(b == bcopy);

    }
  }
  WHEN("Instanciating a parser with nested parameters")
  {
    vpJsonArgumentParser parser("A program", "--config", "/");
    float b = 0.0;
    parser.addArgument("b", b, false);

    THEN("Calling the parser without any argument works and does not modify the default value")
    {
      float bcopy = b;
      const int argc = 1;
      const char *argv[] = {
        "program"
      };

      REQUIRE_NOTHROW(parser.parse(argc, argv));
      REQUIRE(b == bcopy);

    }
  }


  WHEN("Instanciating a parser with some documentation")
  {
    const std::string programString = "ProgramString";
    const std::string firstArg = "FirstArgName", firstArgDescription = "FirstArgDescription";
    const std::string secondArg = "secondArgName", secondArgDescription = "secondArgDescription";
    vpJsonArgumentParser parser(programString, "--config", "/");
    std::string a = "DefaultFirstArg", b = "DefaultSecondArg";
    parser.addArgument(firstArg, a, true, firstArgDescription);
    parser.addArgument(secondArg, b, false, secondArgDescription);
    WHEN("Getting the help string")
    {
      const std::string help = parser.help();
      THEN("Output should contain the basic program description")
      {
        REQUIRE(help.find(programString) < help.size());
      }
      THEN("Output should contain the json argument")
      {
        REQUIRE(help.find("--config") < help.size());
      }
      THEN("Output should contain the arguments, their description and their default value")
      {
        const std::vector<std::string> requireds = { firstArg, secondArg, firstArgDescription, secondArgDescription, a, b };
        for (const auto &required : requireds) {
          REQUIRE(help.find(required) < help.size());
        }
      }
    }


  }
}
int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main()
{
  return EXIT_SUCCESS;
}

#endif
