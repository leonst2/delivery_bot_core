#!/usr/bin/env bash
# Runs every formatter check and static analyser of the repo and fails if any of them reports something:
#   Python  ruff check                      (ruff.toml)
#   C++     clang-format --dry-run --Werror (.clang-format), clang-tidy (.clang-tidy)
#   Rust    cargo fmt --check, cargo clippy -D warnings
#
# Run it in the dev image (docker/Dockerfile), from a sourced workspace that was built with compile commands:
#   colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
#   src/delivery_bot_core/scripts/lint.sh [build dir]
# The build dir defaults to <ws>/build for a checkout in <ws>/src/delivery_bot_core. clang-tidy needs its
# compile_commands.json files, cargo needs the sourced workspace to find the interface crates.
set -uo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
build=${1:-$repo/../../build}
cd "$repo"

failed=()
check() {
  local name=$1
  shift
  echo "=== $name"
  if ! "$@"; then
    failed+=("$name")
  fi
}

# Source files of this repo, without build output (cargo target/ dirs, a colcon workspace inside the checkout).
sources() {
  find . \( -name target -o -name build -o -name install -o -name log -o -name .git \) -prune -o \
    -type f \( "$@" \) -print | sort
}

mapfile -t cpp_files < <(sources -name '*.cpp' -o -name '*.hpp')
mapfile -t cpp_units < <(sources -name '*.cpp')
mapfile -t cargo_tomls < <(sources -name Cargo.toml)

check 'ruff' ruff check .

check 'clang-format' clang-format --dry-run --Werror "${cpp_files[@]}"

clang_tidy() {
  local status=0 file package
  for file in "${cpp_units[@]}"; do
    package=$(cut -d/ -f2 <<< "$file")
    if [ ! -f "$build/$package/compile_commands.json" ]; then
      echo "$build/$package/compile_commands.json missing: build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON" >&2
      status=1
      continue
    fi
    # Headers are checked through the translation units that include them (HeaderFilterRegex).
    clang-tidy --quiet -p "$build/$package" "$file" || status=1
  done
  return $status
}
check 'clang-tidy' clang_tidy

cargo_lint() {
  local status=0 toml
  for toml in "${cargo_tomls[@]}"; do
    (cd "$(dirname "$toml")" && cargo fmt --check && cargo clippy --quiet --all-targets -- -D warnings) || status=1
  done
  return $status
}
check 'cargo fmt + clippy' cargo_lint

if [ ${#failed[@]} -gt 0 ]; then
  echo "Lint failed: ${failed[*]}" >&2
  exit 1
fi
echo 'All lint checks passed.'
