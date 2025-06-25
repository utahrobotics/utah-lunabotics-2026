cmake_minimum_required(VERSION 3.16)
project(UnilidarIceoryxPublisher)

# Set C++ standard (iceoryx2 requires C++17 or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find iceoryx packages
find_package(iceoryx_hoofs REQUIRED)
#find_package(unitree_lidar_sdk REQUIRED)

# For iceoryx2, we might need to find it manually since it might not have cmake config
# Let's check what's actually available first
set(ICEORYX2_FFI_ROOT "$ENV{HOME}/iceoryx2/target/ffi/install")

# Include iceoryx2 headers manually
include_directories(${ICEORYX2_FFI_ROOT}/include)

# Library directories  
link_directories(${ICEORYX2_FFI_ROOT}/lib)

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