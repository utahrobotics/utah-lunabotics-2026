// SPDX-License-Identifier: Apache-2.0 OR MIT
#pragma once
#include <cstdint>

struct ImuMsg {
    static constexpr const char* IOX2_TYPE_NAME = "ImuMsg";
    float quaternion[4];
    float angular_velocity[3];
    float linear_acceleration[3];
};

struct PointXYZIR {
    static constexpr const char* IOX2_TYPE_NAME = "PointXYZIR";
    float x;
    float y;
    float z;
    float intensity;
    float time;
    std::uint16_t ring;
};

struct IceoryxPointCloud {
    static constexpr const char* IOX2_TYPE_NAME = "IceoryxPointCloud";
    uint64_t publish_count;
    PointXYZIR points[60000];  // Increased to support accumulated frames
};