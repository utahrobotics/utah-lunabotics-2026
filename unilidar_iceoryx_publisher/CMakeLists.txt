cmake_minimum_required(VERSION 3.16)
project(UnilidarIceoryxPublisher)

# Set C++ standard (iceoryx2 requires C++17 or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find iceoryx packages
find_package(iceoryx_hoofs REQUIRED)
#find_package(unitree_lidar_sdk REQUIRED)

# iceoryx2 C++ FFI paths (built by `cargo build -p iceoryx2-ffi --features cxx-ffi`)
set(ICEORYX2_SRC_ROOT "$ENV{HOME}/iceoryx2")

# Header and library directories
set(ICEORYX2_INCLUDE_DIR "${ICEORYX2_SRC_ROOT}/iceoryx2-ffi/cxx/include")
set(ICEORYX2_LIB_DIR     "${ICEORYX2_SRC_ROOT}/target/ffi/build/iceoryx2-ffi/cxx")

# Include iceoryx2 headers manually
set(ICEORYX2_C_INCLUDE_DIR "${ICEORYX2_SRC_ROOT}/target/ffi/build/rust/native/release/iceoryx2-ffi-cbindgen/include")

include_directories(
    ${ICEORYX2_INCLUDE_DIR}
    ${ICEORYX2_C_INCLUDE_DIR}
)

# Library directories
link_directories(${ICEORYX2_LIB_DIR})

# Unilidar SDK paths
set(UNILIDAR_SDK_ROOT "${CMAKE_SOURCE_DIR}/../unilidar_sdk2/unitree_lidar_sdk")
include_directories(${UNILIDAR_SDK_ROOT}/include)
link_directories(${UNILIDAR_SDK_ROOT}/lib/x86_64)

# Add your executable
add_executable(unilidar_publisher
    src/main.cpp
)

# Link against iceoryx and Unilidar SDK libraries
target_link_libraries(unilidar_publisher
    iceoryx_hoofs::iceoryx_hoofs
    ${UNILIDAR_SDK_ROOT}/lib/x86_64/libunilidar_sdk2.a
)

# Include directories (usually handled automatically by find_package)
target_include_directories(unilidar_publisher PRIVATE
    include/
)

# Optional: Set compiler warnings
target_compile_options(unilidar_publisher PRIVATE
    -Wall -Wextra -Wpedantic
)

# Optional: Add compile definitions
target_compile_definitions(unilidar_publisher PRIVATE
    ICEORYX2_PROJECT=1
)