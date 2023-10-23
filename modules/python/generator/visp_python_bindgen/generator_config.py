
from typing import Dict, Final, List, Optional
import re
from pathlib import Path
from dataclasses import dataclass
import json

@dataclass
class PreprocessorConfig(object):
  '''
  Preprocessor config. Contains the arguments that are passed to pcpp to preprocess a header.
  This does not include the header search path (-I) or the input and output paths
  '''

  defines: Dict[str, str] # Mapping from a #define to its value (#define A 1 is equal to a pair "A": "1")
  never_defined: List[str] # List of macros that should never be defined, even if they are defined in other included headers
  include_directories: List[str]
  passthrough_includes_regex: str # Regex to see which header should be included (expanded and put in the resulting pcpp output) or ignored
  line_directive: Optional[str] # prefix for Warning/logging emitted by pcpp. If none, no warning
  other_args: List[str]

  def to_pcpp_args_list(self) -> List[str]:
    args = []
    for k,v in self.defines.items():
      args += ['-D', f'{k}={v}'] if v is not None else ['-D', k]
    for v in self.never_defined:
      args += ['-N', v]
    for v in self.include_directories:
      args += ['-I', v]
    args += self.other_args
    args.extend(['--passthru-includes', self.passthrough_includes_regex])
    if self.line_directive is not None:
      args.extend(['--line-directive', self.line_directive])
    else:
      args.extend(['--line-directive', ''])
    return args

'''
Regular expressions that should match with types that are considered as immutable on the Python side
This only encompasses raw types
'''
IMMUTABLE_TYPES_REGEXS = [
  '^(float|double|u?int\d+_t|unsigned|char|long|long\wlong)$',
  '^std::string$'
]

'''
Regular expressions that should match with types that are considered as immutable on the Python side
This only encompasses raw types
'''
IMMUTABLE_CONTAINERS_REGEXS = [
  '^std::vector', '^std::list'
]


'''
Specific argument regexs for which having default arguments is specifically forbidden
'''
FORBIDDEN_DEFAULT_ARGUMENT_TYPES_REGEXS = [
  '^std::ostream',
  '^std::initializer_list',
  '^rs2::',
  '^cv::'
]

'''
Regexes for names of functions that should be ignored
'''
FORBIDDEN_FUNCTION_NAMES_REGEXS = [
  '^(from|to)_.*json',
  '^operator.*',
  '^ignored$'
]

class GeneratorConfig(object):
  pcpp_config: Final[PreprocessorConfig] = PreprocessorConfig(
    defines={
      'VISP_EXPORT': '', # remove symbol as it messes up the cxxheaderparsing
      'vp_deprecated': '', # remove symbol as it messes up the cxxheaderparsing
      'DOXYGEN_SHOULD_SKIP_THIS': None, # Do not generate methods that do not appear in public api doc
      'NLOHMANN_JSON_SERIALIZE_ENUM(a,...)': 'void ignored() {}', # Remove json enum serialization as it cnanot correctly be parsed
      '__cplusplus' : '201103L' # To silence OpenCV warnings
    },
    never_defined=[
      'VISP_BUILD_DEPRECATED_FUNCTIONS', # Do not bind deprecated functions
      'VISP_RUBIK_REGULAR_FONT_RESOURCES'
    ],
    include_directories=[], # Populate through the main configuration file
    passthrough_includes_regex="^.*$", # Never output the result of other includes.
    line_directive=None,
    other_args=["--passthru-unfound-includes", "--passthru-comments"]
  )

  @staticmethod
  def _matches_regex_in_list(s: str, regexes: List[str]) -> bool:
    return any(map(lambda regex: re.match(regex, s) is not None, regexes))

  @staticmethod
  def is_immutable_type(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, IMMUTABLE_TYPES_REGEXS)


  @staticmethod
  def is_immutable_container(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, IMMUTABLE_CONTAINERS_REGEXS)


  @staticmethod
  def is_forbidden_default_argument_type(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, FORBIDDEN_DEFAULT_ARGUMENT_TYPES_REGEXS)

  @staticmethod
  def is_forbidden_function_name(name: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(name, FORBIDDEN_FUNCTION_NAMES_REGEXS)

  @staticmethod
  def update_from_main_config_file(path: Path) -> None:
    assert path.exists()
    with open(path, 'r') as main_config_file:
      main_config = json.load(main_config_file)
      GeneratorConfig.pcpp_config.include_directories = main_config['include_dirs']