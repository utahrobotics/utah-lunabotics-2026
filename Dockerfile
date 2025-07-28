# Use Ubuntu 22.04 as base image for better compatibility
FROM ubuntu:22.04 as builder

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    git \
    curl \
    wget \
    pkg-config \
    # SSL development libraries
    libssl-dev \
    # LAPACK/OpenBLAS libraries
    liblapack-dev \
    libblas-dev \
    libopenblas-dev \
    # GStreamer development libraries
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libacl1-dev \
    libgstrtspserver-1.0-dev \
    libges-1.0-dev \
    # Video4Linux development
    libv4l-dev \
    # udev development
    libudev-dev \
    # Additional dependencies that might be needed
    python3 \
    python3-pip \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install Rust with specific toolchain version
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain nightly
ENV PATH="/root/.cargo/bin:${PATH}"

# Ensure we have the exact toolchain version needed for Bazel rules_rust
RUN rustup toolchain install nightly && rustup default nightly

# Install Bazelisk (which manages Bazel versions)
RUN curl -fsSL "https://github.com/bazelbuild/bazelisk/releases/download/v1.26.0/bazelisk-amd64.deb" -o /tmp/bazelisk.deb \
    && dpkg -i /tmp/bazelisk.deb \
    && rm /tmp/bazelisk.deb \
    && bazel version

# Install Intel RealSense SDK via vcpkg
WORKDIR /tmp
RUN git clone https://github.com/Microsoft/vcpkg.git \
    && cd vcpkg \
    && ./bootstrap-vcpkg.sh \
    && ./vcpkg integrate install \
    && ./vcpkg install realsense2 \
    && cp -r /tmp/vcpkg/installed/x64-linux/lib/* /usr/local/lib/ \
    && cp -r /tmp/vcpkg/installed/x64-linux/include/* /usr/local/include/ \
    && cp -r /tmp/vcpkg/installed/x64-linux/share/pkgconfig/* /usr/local/lib/pkgconfig/ || true \
    && ldconfig \
    && cd /tmp \
    && rm -rf vcpkg

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
    && echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-15 main" >> /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y clang-15 libclang-15-dev

# Install AprilTag library from source
WORKDIR /tmp
RUN git clone https://github.com/AprilRobotics/apriltag.git \
    && cd apriltag \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel 4 \
    && cmake --build build --target install \
    && cd .. \
    && rm -rf apriltag

# Set up working directory
WORKDIR /workspace

# Note: .bazelversion should be provided by your project, not created here

# Set environment variables for development
ENV RUST_BACKTRACE=1
ENV PKG_CONFIG_PATH="/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
# ENV RULES_RUST_TOOLCHAIN_VERSION=1.81.0

# Pre-compile some common Rust dependencies to speed up builds
# RUN cargo install cargo-watch

# Install make for running your Makefile
RUN apt-get update && apt-get install -y make && rm -rf /var/lib/apt/lists/*

# Final stage - keep the development environment
FROM builder as final

# Copy any additional configuration files you might need
# COPY .bazelrc /workspace/.bazelrc

# Default command
CMD ["/bin/bash"]

# Health check to ensure key components are available
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD bazel version && \
    pkg-config --exists gstreamer-1.0 && \
    pkg-config --exists realsense2 && \
    pkg-config --exists lapack && \
    rustc --version
