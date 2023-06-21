#include <visp3/io/vpJsonArgumentParser.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpConfig.h>
#include <fstream>


#if defined(VISP_HAVE_NLOHMANN_JSON)

using json = nlohmann::json;


vpJsonArgumentParser::vpJsonArgumentParser(const std::string &description, const std::string &jsonFileArgumentName,
                                           const std::string &nestSeparator) :
  description(description),
  jsonFileArgumentName(jsonFileArgumentName),
  nestSeparator(nestSeparator)
{
  if (jsonFileArgumentName.empty()) {
    throw vpException(vpException::badValue, "The JSON file argument must not be empty");
  }

  if (nestSeparator.empty()) {
    throw vpException(vpException::badValue, "You must provide a JSON nesting delimiter to be able to parse JSON");
  }

  helpers[jsonFileArgumentName] = []() -> std::string {
    return "Path to the JSON configuration file. Values in this files are loaded, and can be overriden by command line arguments.\nOptional";
  };
}

std::string vpJsonArgumentParser::help() const
{
  std::stringstream ss;

  ss << "Program description: " << description << std::endl;
  ss << "Arguments: " << std::endl;
  unsigned spacesBetweenArgAndDescription = 0;
  for (const auto &helper : helpers) {
    if (helper.first.size() > spacesBetweenArgAndDescription) {
      spacesBetweenArgAndDescription = static_cast<unsigned int>(helper.first.size());
    }
  }
  spacesBetweenArgAndDescription += 4;

  for (const auto &helper : helpers) {
    std::stringstream argss(helper.second());
    std::string line;
    bool first = true;
    while (getline(argss, line, '\n')) {
      const unsigned lineSpace = first ? spacesBetweenArgAndDescription - static_cast<unsigned>(helper.first.size()) : spacesBetweenArgAndDescription;
      const std::string spaceBetweenArgAndDescription(lineSpace, ' ');
      if (first) {
        ss << "\t" << helper.first << spaceBetweenArgAndDescription << line << std::endl;
      }
      else {
        ss << "\t" << spaceBetweenArgAndDescription << line << std::endl;
      }
      first = false;

    }
    ss << std::endl;
  }
  ss << "Example JSON configuration file: " << std::endl << std::endl;
  ss << exampleJson.dump(2) << std::endl;
  return ss.str();
}


void vpJsonArgumentParser::parse(int argc, const char *argv [])
{
  json j;
  const std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<unsigned> ignoredArguments;
  const auto jsonFileArgumentPos = std::find(arguments.begin(), arguments.end(), jsonFileArgumentName);
  // Load JSON file if present
  if (jsonFileArgumentPos != arguments.end()) {
    ignoredArguments.push_back(jsonFileArgumentPos - arguments.begin() + 1);
    ignoredArguments.push_back(jsonFileArgumentPos - arguments.begin() + 2);

    if (jsonFileArgumentPos == arguments.end() - 1) {
      throw vpException(vpException::ioError, "No JSON file was provided");
    }
    const std::string jsonFileName = *(jsonFileArgumentPos + 1);
    std::ifstream jsonFile(jsonFileName);
    if (!jsonFile.good()) {
      std::stringstream ss;
      ss << "Could not open JSON file " << jsonFileName << "! Make sure it exists and is readable" << std::endl;
      throw vpException(vpException::ioError, ss.str());
    }
    j = json::parse(jsonFile);
    jsonFile.close();
  }
  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (std::find(ignoredArguments.begin(), ignoredArguments.end(), i) != ignoredArguments.end()) {
      continue;
    }
    if (arg == "-h" || arg == "--help") {
      std::cout << help() << std::endl;
      exit(1);
    }

    if (parsers.find(arg) != parsers.end()) {
      if (i < argc - 1) {
        updaters[arg](j, std::string(argv[i + 1]));
        ++i;
      }
      else {
        std::stringstream ss;
        ss << "Argument " << arg << " was passed but no value was provided" << std::endl;
        throw vpException(vpException::ioError, ss.str());
      }
    }
    else {
      std::cerr << "Unknown parameter when parsing: " << arg << std::endl;
    }
  }

  // Get the values from json document and store them in the arguments passed by ref in addArgument
  for (const auto &parser : parsers) {
    parser.second(j);
  }
}

#endif