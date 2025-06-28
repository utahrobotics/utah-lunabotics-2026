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
#include <vector>

using namespace iox2;

constexpr iox::units::Duration CYCLE_TIME = iox::units::Duration::fromMilliseconds(10);

// Helper constants  
constexpr std::size_t MAX_POINTS_PER_CLOUD = 60000;  // Increased to accommodate accumulated frames

// Frame accumulation parameters (similar to Point LIO's con_frame mechanism)
constexpr bool ACCUMULATE_FRAMES = false;   // Set to true to enable frame accumulation
constexpr int ACCUMULATION_COUNT = 3;      // Number of frames to accumulate (adjust as needed)

// Frame accumulation state
struct FrameAccumulator {
    std::vector<PointXYZIR> accumulated_points;
    int frame_count = 0;
    double first_frame_time = 0.0;
    
    void reset() {
        accumulated_points.clear();
        frame_count = 0;
        first_frame_time = 0.0;
    }
    
    bool addFrame(const PointCloudUnitree &cloud) {
        if (frame_count == 0) {
            first_frame_time = cloud.stamp;
        }
        
        // Add points with adjusted timestamps for temporal coherence
        double time_offset = cloud.stamp - first_frame_time;
        for (const auto &point : cloud.points) {
            PointXYZIR p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.intensity = point.intensity;
            p.time = point.time + time_offset * 1000.0;  // Convert to milliseconds
            p.ring = static_cast<std::uint16_t>(point.ring);
            accumulated_points.push_back(p);
        }
        
        frame_count++;
        return frame_count >= ACCUMULATION_COUNT;
    }
};

// Convert accumulated points to the fixed-size Iceoryx structure
static IceoryxPointCloud toIceoryxPointCloud(const std::vector<PointXYZIR> &points) {
    IceoryxPointCloud dst{}; // zero-initialise all fields

    // Cap publish_count if the accumulated cloud is larger than our fixed array
    dst.publish_count = std::min<std::size_t>(points.size(), MAX_POINTS_PER_CLOUD);

    for (std::size_t i = 0; i < dst.publish_count; ++i) {
        dst.points[i] = points[i];
    }

    // The remaining entries of dst.points are already zero due to the initialisation above.
    return dst;
}

// Original conversion function for single frames
static IceoryxPointCloud toIceoryxPointCloudSingle(const PointCloudUnitree &src) {
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
    const uint16_t    cloud_scan_num = 36;  // Increase from default 18 to 36
    const bool        use_system_timestamp = true;
    const float       range_min = 0.1f;     // Slightly higher minimum to filter noise
    const float       range_max = 150.0f;   // Increase maximum range

    if (lreader->initializeSerial(port, baudrate, cloud_scan_num, use_system_timestamp, range_min, range_max)) {
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
    FrameAccumulator accumulator;

    std::cout << "Frame accumulation: " << (ACCUMULATE_FRAMES ? "ENABLED" : "DISABLED") << std::endl;
    if (ACCUMULATE_FRAMES) {
        std::cout << "Accumulating " << ACCUMULATION_COUNT << " frames before sending" << std::endl;
    }

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
                // std::cout << "imu sent" << std::endl;
            }
            break;
        }
        case LIDAR_POINT_DATA_PACKET_TYPE: {
            if (lreader->getPointCloud(cloud_raw)) {
                if (ACCUMULATE_FRAMES) {
                    // Accumulate frames
                    if (accumulator.addFrame(cloud_raw)) {
                        // Ready to send accumulated cloud
                        IceoryxPointCloud cloud = toIceoryxPointCloud(accumulator.accumulated_points);
                        std::cout << "Accumulated " << accumulator.frame_count << " frames, total points: " 
                                  << cloud.publish_count << " (raw accumulated: " << accumulator.accumulated_points.size() << ")" << std::endl;
                        
                        auto sample      = cloud_publisher.loan_uninit().expect("cloud loan");
                        auto initialized = sample.write_payload(cloud);
                        send(std::move(initialized)).expect("cloud send");
                        std::cout << "Accumulated cloud sent" << std::endl;
                        
                        // Reset accumulator for next batch
                        accumulator.reset();
                    } else {
                        std::cout << "Frame " << accumulator.frame_count << "/" << ACCUMULATION_COUNT 
                                  << " accumulated (" << cloud_raw.points.size() << " points)" << std::endl;
                    }
                } else {
                    // Send individual frames (original behavior)
                    IceoryxPointCloud cloud = toIceoryxPointCloudSingle(cloud_raw);
                    std::cout << "publish_count: " << cloud.publish_count << std::endl;
                    auto sample      = cloud_publisher.loan_uninit().expect("cloud loan");
                    auto initialized = sample.write_payload(cloud);
                    send(std::move(initialized)).expect("cloud send");
                    std::cout << "cloud sent" << std::endl;
                }
            }
            break;
        }
        default:
            break;
        }
    }

    return 0;
}