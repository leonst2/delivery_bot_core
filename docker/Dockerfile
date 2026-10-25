# Development image for delivery_bot_core: ROS 2 Jazzy (ros-core) with C++, Python and Rust toolchains.
#
# Based on ros-core on purpose: the ros-base image ships robot_state_publisher and kdl_parser, which the
# self-programmed stack must not use. Dependencies are installed here instead of by rosdep at runtime, so a
# package.xml can't pull a forbidden package back in. check_forbidden_packages.sh fails the build otherwise.
#
#   docker build -t delivery_bot_core_dev:jazzy docker/
FROM ros:jazzy-ros-core

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang-format \
        clang-tidy \
        cmake \
        curl \
        git \
        libclang-dev \
        libeigen3-dev \
        libtinyxml2-dev \
        netcat-openbsd \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-vcstool \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ros2cli-common-extensions \
        ros-jazzy-rosbag2 \
        ros-jazzy-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Rust: pinned toolchain (rclrs 0.8 needs >= 1.85; current crates such as uuid need >= 1.89), cargo plugin for ament, colcon plugins for cargo packages.
ARG RUST_VERSION=1.98.0
ARG CARGO_AMENT_BUILD_VERSION=0.1.11
ARG COLCON_CARGO_REF=518a4f90d7de33d685cfba32a45fb4cb3aa08950
ARG COLCON_ROS_CARGO_REF=dc26ba5f4e7d016a5f8b78abdaa1c22de9d35442
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --no-modify-path --profile minimal --default-toolchain "${RUST_VERSION}" \
            --component rustfmt --component clippy
RUN cargo install --locked cargo-ament-build --version "${CARGO_AMENT_BUILD_VERSION}" \
    && rm -rf "${CARGO_HOME}/registry" "${CARGO_HOME}/git" \
    && chmod -R a+rwX "${CARGO_HOME}" "${RUSTUP_HOME}"
RUN pip install --no-cache-dir --break-system-packages \
        "git+https://github.com/colcon/colcon-cargo.git@${COLCON_CARGO_REF}" \
        "git+https://github.com/colcon/colcon-ros-cargo.git@${COLCON_ROS_CARGO_REF}"

# ros2_rust overlay: rosidl_generator_rs + standard interfaces rebuilt with Rust bindings (pinned commits).
COPY ros2_rust_jazzy.repos /opt/ros2_rust/ros2_rust_jazzy.repos
RUN mkdir -p /opt/ros2_rust/src \
    && cd /opt/ros2_rust \
    && vcs import --shallow src < ros2_rust_jazzy.repos \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --event-handlers console_cohesion+ --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY check_forbidden_packages.sh entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/check_forbidden_packages.sh /usr/local/bin/entrypoint.sh \
    && check_forbidden_packages.sh

WORKDIR /ws
ENTRYPOINT ["entrypoint.sh"]
CMD ["bash"]
