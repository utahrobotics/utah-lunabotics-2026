/**********************************************************************
 Copyright (c) 2024 Contributors
 SPDX-License-Identifier: Apache-2.0 OR MIT
***********************************************************************/

#include "example.h"              // Unitree SDK helper utilities
#include "imu_point_types.hpp"    // ImuMsg & PointXYZIR structures

#include "iox/duration.hpp"
#include "iox2/log.hpp"
#include "iox2/node.hpp"
#include "iox2/sample_mut.hpp"
#include "iox2/service_name.hpp"
#include "iox2/service_type.hpp"

#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <cstring>

using namespace iox2;

constexpr iox::units::Duration CYCLE_TIME = iox::units::Duration::fromMilliseconds(10);

// Helper constants
constexpr std::size_t MAX_POINTS_PER_CLOUD = 6000;

// Convert Unitree point cloud to the fixed-size Iceoryx structure
static IceoryxPointCloud toIceoryxPointCloud(const PointCloudUnitree &src) {
    IceoryxPointCloud dst{}; // zero-initialise all fields

    // Cap publish_count if the incoming cloud is larger than our fixed array
    dst.publish_count = std::min<std::size_t>(src.points.size(), MAX_POINTS_PER_CLOUD);

    for (std::size_t i = 0; i < dst.publish_count; ++i) {
        const auto &p_src = src.points[i];
        auto &p_dst       = dst.points[i];
        p_dst.x           = p_src.x;
        p_dst.y           = p_src.y;
        p_dst.z           = p_src.z;
        p_dst.intensity   = p_src.intensity;
        p_dst.time        = p_src.time;
        p_dst.ring        = static_cast<std::uint16_t>(p_src.ring);
    }

    // The remaining entries of dst.points are already zero due to the initialisation above.
    return dst;
}

int main() {
    // -------------------- LiDAR initialisation --------------------
    UnitreeLidarReader* lreader = createUnitreeLidarReader();
    if (lreader == nullptr) {
        std::cerr << "Failed to create UnitreeLidarReader" << std::endl;
        return -1;
    }

    const std::string port     = "/dev/ttyACM0";
    const uint32_t    baudrate = 4'000'000;

    if (lreader->initializeSerial(port, baudrate)) {
        std::cerr << "Unilidar initialization failed!" << std::endl;
        return -1;
    }

    lreader->startLidarRotation();
    sleep(1);
    lreader->setLidarWorkMode(8);
    sleep(1);

    // ---------------------- iceoryx2 setup -----------------------
    set_log_level_from_env_or(LogLevel::Info);
    auto node = NodeBuilder().create<ServiceType::Ipc>().expect("node creation");

    // IMU service
    auto imu_service = node.service_builder(ServiceName::create("unilidar/imu").expect("name"))
                           .publish_subscribe<ImuMsg>()
                           .open_or_create()
                           .expect("imu service");

    // Point service
    auto point_service = node.service_builder(ServiceName::create("unilidar/cloud_full").expect("name"))
                             .publish_subscribe<IceoryxPointCloud>()
                             .open_or_create()
                             .expect("cloud");

    auto imu_publisher   = imu_service.publisher_builder().create().expect("imu publisher");
    auto cloud_publisher = point_service.publisher_builder().create().expect("cloud publisher");

    // ---------------------- Processing loop ----------------------
    LidarImuData      imu_raw;
    PointCloudUnitree cloud_raw;

    while (node.wait(CYCLE_TIME).has_value()) {
        int parse_result = lreader->runParse();

        switch (parse_result) {
        case LIDAR_IMU_DATA_PACKET_TYPE: {
            if (lreader->getImuData(imu_raw)) {
                ImuMsg msg{};
                std::copy(std::begin(imu_raw.quaternion), std::end(imu_raw.quaternion), std::begin(msg.quaternion));
                std::copy(std::begin(imu_raw.angular_velocity), std::end(imu_raw.angular_velocity), std::begin(msg.angular_velocity));
                std::copy(std::begin(imu_raw.linear_acceleration), std::end(imu_raw.linear_acceleration), std::begin(msg.linear_acceleration));

                auto sample      = imu_publisher.loan_uninit().expect("imu loan");
                auto initialized = sample.write_payload(msg);
                send(std::move(initialized)).expect("imu send");
            }
            break;
        }
        case LIDAR_POINT_DATA_PACKET_TYPE: {
            if (lreader->getPointCloud(cloud_raw)) {
                IceoryxPointCloud cloud = toIceoryxPointCloud(cloud_raw);
                std::cout << "publish_count: " << cloud.publish_count << std::endl;
                auto sample      = cloud_publisher.loan_uninit().expect("cloud loan");
                auto initialized = sample.write_payload(cloud);
                send(std::move(initialized)).expect("cloud send");
            }
            break;
        }
        default:
            break;
        }
    }

    return 0;
}