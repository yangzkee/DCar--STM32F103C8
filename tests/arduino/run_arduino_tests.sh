#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/dfcom-arduino.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

sanitizer_flags=
if [ "${DFCOM_SANITIZE:-0}" = "1" ]; then
    sanitizer_flags="-fsanitize=address,undefined -fno-omit-frame-pointer"
fi

c++ -std=c++11 -Wall -Wextra -Werror \
    $sanitizer_flags \
    -I"$repo_dir/tests/arduino/stubs" \
    -I"$repo_dir/DFCom_Arduino/DFCom/src" \
    "$repo_dir/tests/arduino/test_dfcom_arduino.cpp" \
    "$repo_dir/DFCom_Arduino/DFCom/src/DFCom.cpp" \
    -o "$build_dir/test_dfcom_arduino"

"$build_dir/test_dfcom_arduino"

c++ -x c++ -std=c++11 -Wall -Wextra -Werror \
    $sanitizer_flags \
    -I"$repo_dir/tests/arduino/stubs" \
    "$repo_dir/tests/arduino/test_single_file.cpp" \
    -o "$build_dir/test_single_file"

"$build_dir/test_single_file"
