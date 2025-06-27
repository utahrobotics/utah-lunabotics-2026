cmake_minimum_required(VERSION 3.16)
project(UnilidarIceoryxPublisher)

# Set C++ standard (iceoryx2 requires C++17 or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# iceoryx2 C++ FFI paths (built by `cargo build -p iceoryx2-ffi --features cxx-ffi`)
set(ICEORYX2_SRC_ROOT "$ENV{HOME}/iceoryx2")

# Header and library directories
set(ICEORYX2_INCLUDE_DIR   "${ICEORYX2_SRC_ROOT}/iceoryx2-ffi/cxx/include")
set(ICEORYX2_LIB_DIR       "${ICEORYX2_SRC_ROOT}/target/ffi/build/iceoryx2-ffi/cxx")

# Include iceoryx2 headers manually
set(ICEORYX2_C_INCLUDE_DIR "${ICEORYX2_SRC_ROOT}/target/ffi/build/rust/native/release/iceoryx2-ffi-cbindgen/include")

# Add comprehensive iceoryx dependencies
set(ICEORYX_BASE_DIR "${ICEORYX2_SRC_ROOT}/target/ffi/build/_deps/iceoryx-src")
set(ICEORYX_HOOFS_BASE_DIR "${ICEORYX_BASE_DIR}/iceoryx_hoofs")

include_directories(
    ${ICEORYX2_INCLUDE_DIR}
    ${ICEORYX2_C_INCLUDE_DIR}
    # All iceoryx_hoofs include directories
    ${ICEORYX_HOOFS_BASE_DIR}/buffer/include
    ${ICEORYX_HOOFS_BASE_DIR}/cli/include
    ${ICEORYX_HOOFS_BASE_DIR}/concurrent/buffer/include
    ${ICEORYX_HOOFS_BASE_DIR}/concurrent/sync_extended/include
    ${ICEORYX_HOOFS_BASE_DIR}/concurrent/sync/include
    ${ICEORYX_HOOFS_BASE_DIR}/container/include
    ${ICEORYX_HOOFS_BASE_DIR}/design/include
    ${ICEORYX_HOOFS_BASE_DIR}/filesystem/include
    ${ICEORYX_HOOFS_BASE_DIR}/functional/include
    ${ICEORYX_HOOFS_BASE_DIR}/legacy/include
    ${ICEORYX_HOOFS_BASE_DIR}/memory/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/auth/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/design/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/filesystem/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/ipc/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/sync/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/time/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/utility/include
    ${ICEORYX_HOOFS_BASE_DIR}/posix/vocabulary/include
    ${ICEORYX_HOOFS_BASE_DIR}/primitives/include
    ${ICEORYX_HOOFS_BASE_DIR}/reporting/include
    ${ICEORYX_HOOFS_BASE_DIR}/testing/include
    ${ICEORYX_HOOFS_BASE_DIR}/time/include
    ${ICEORYX_HOOFS_BASE_DIR}/utility/include
    ${ICEORYX_HOOFS_BASE_DIR}/vocabulary/include
    # Platform includes
    ${ICEORYX_BASE_DIR}/iceoryx_platform/linux/include
    ${ICEORYX_BASE_DIR}/iceoryx_platform/generic/include
    # Generated includes
    ${ICEORYX2_SRC_ROOT}/target/ffi/build/generated/iceoryx_platform/include
    ${ICEORYX2_SRC_ROOT}/target/ffi/build/generated/iceoryx_hoofs/include
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

# Link against iceoryx2 and Unilidar SDK libraries
target_link_libraries(unilidar_publisher
    ${ICEORYX2_LIB_DIR}/libiceoryx2_cxx.a
    ${ICEORYX2_SRC_ROOT}/target/ffi/build/rust/native/release/libiceoryx2_ffi.a
    ${ICEORYX2_SRC_ROOT}/target/iceoryx/build/hoofs/libiceoryx_hoofs.so
    ${ICEORYX2_SRC_ROOT}/target/iceoryx/build/platform/libiceoryx_platform.so
    ${UNILIDAR_SDK_ROOT}/lib/x86_64/libunilidar_sdk2.a
)

# Ensure runtime can locate iceoryx shared libraries without needing LD_LIBRARY_PATH
target_link_options(unilidar_publisher PRIVATE "-Wl,-rpath,${ICEORYX2_SRC_ROOT}/target/iceoryx/install/lib")

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