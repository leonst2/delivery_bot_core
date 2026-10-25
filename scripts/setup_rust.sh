#!/usr/bin/env bash
# Rust toolchain for ROS 2 Jazzy on a native Ubuntu 24.04 machine (delivery_bot_core#2).
# Mirrors docker/Dockerfile; versions must stay in sync with it. Usage: scripts/setup_rust.sh [overlay_dir]
set -eo pipefail

RUST_VERSION=1.98.0
CARGO_AMENT_BUILD_VERSION=0.1.11
COLCON_CARGO_REF=518a4f90d7de33d685cfba32a45fb4cb3aa08950
COLCON_ROS_CARGO_REF=dc26ba5f4e7d016a5f8b78abdaa1c22de9d35442
OVERLAY=${1:-$HOME/ros2_rust_ws}
REPOS="$(cd "$(dirname "$0")/.." && pwd)/docker/ros2_rust_jazzy.repos"

sudo apt-get update
sudo apt-get install -y --no-install-recommends curl git libclang-dev python3-pip python3-vcstool \
  python3-colcon-common-extensions

if ! command -v rustup >/dev/null; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain none
  source "$HOME/.cargo/env"
fi
rustup toolchain install "$RUST_VERSION" --profile minimal --component rustfmt --component clippy
rustup default "$RUST_VERSION"
cargo install --locked cargo-ament-build --version "$CARGO_AMENT_BUILD_VERSION"
pip install --user --break-system-packages \
  "git+https://github.com/colcon/colcon-cargo.git@${COLCON_CARGO_REF}" \
  "git+https://github.com/colcon/colcon-ros-cargo.git@${COLCON_ROS_CARGO_REF}"

# Overlay with rosidl_generator_rs and the standard interfaces rebuilt with Rust bindings.
mkdir -p "$OVERLAY/src"
vcs import --shallow "$OVERLAY/src" < "$REPOS"
source /opt/ros/jazzy/setup.bash
(cd "$OVERLAY" && colcon build --cmake-args -DBUILD_TESTING=OFF)

echo "Done. Source /opt/ros/jazzy/setup.bash and $OVERLAY/install/setup.bash before building this workspace."
