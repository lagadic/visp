from typing import Dict, List
import itertools
TEMPLATE_EXPANSION_MAP: Dict[str, List[str]] = {
  'TypeReal': ['float', 'double'],
  'TypePythonScalar': ['int', 'double'], # Python itself doesn't make the distinction between int, uint, int16_t etc.
  'TypeFilterable': ['unsigned char', 'float', 'double'],
  'TypeErodableDilatable': ['unsigned char', 'float', 'double'],

  'TypeBaseImagePixel': ['unsigned char', 'vpRGBa']
}

def expand_templates(specializations: List[List[str]]) -> List[List[str]]:
  result = []
  for spec in specializations:
    expanded_params = []
    for param in spec:
      if param in TEMPLATE_EXPANSION_MAP:
        expanded_params.append(TEMPLATE_EXPANSION_MAP[param])
      else:
        expanded_params.append([param])
    # Cartesian product: compute all possible combinations when expansions are taken into account
    result.extend(list(itertools.product(*expanded_params)))
  return result
