#!/bin/sh

script_path=`dirname $0`
visp_root=`(cd $script_path/..; pwd)`

echo "Apply coding-style to visp: " $visp_root
# Change this if your clang-format executable is somewhere else
clang_format="clang-format"

read -r -p "Are you sure? [Y/n] " answer
answer="$(echo ${answer} | tr 'A-Z' 'a-z')"

if echo "$answer" | grep -iq "^y" ;then
  echo "We are applying coding-style rules..."
else
  echo "Exit the script"
  exit 0
fi

find "$visp_root" \( -name '*.h' -or -name '*.hpp' -or -name '*.c' -or -name '*.cpp' \) -not -path "${visp_root}/3rdparty/*" -prune -print0 | xargs -0 "$clang_format" -i

