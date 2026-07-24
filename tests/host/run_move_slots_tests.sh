#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/dfcom-move-slots.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

sanitizer_flags=
if [ "${DFCOM_SANITIZE:-0}" = "1" ]; then
    sanitizer_flags="-fsanitize=address,undefined -fno-omit-frame-pointer"
fi

for variant in DFCom_Example DFCom_PatchOnly; do
    cc -std=c99 -Wall -Wextra -Werror \
        $sanitizer_flags \
        -I"$repo_dir/tests/host/stubs" \
        -I"$repo_dir/$variant/HARDWARE" \
        "$repo_dir/tests/host/test_move_slots.c" \
        "$repo_dir/$variant/HARDWARE/DFCom_Rx.c" \
        "$repo_dir/$variant/HARDWARE/DFCom_Tx.c" \
        -o "$build_dir/test_move_slots"

    echo "[$variant]"
    "$build_dir/test_move_slots"
done
