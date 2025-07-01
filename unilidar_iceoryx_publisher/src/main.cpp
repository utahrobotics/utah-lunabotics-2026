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
#include <limits>

using namespace iox2;

constexpr iox::units::Duration CYCLE_TIME = iox::units::Duration::fromMilliseconds(10);

// Helper constants  
constexpr std::size_t MAX_POINTS_PER_CLOUD = 130000;  // Increased to accommodate accumulated frames

// Frame accumulation parameters (similar to Point LIO's con_frame mechanism)
constexpr bool ACCUMULATE_FRAMES = false;    // Enable frame accumulation for higher point density
constexpr int ACCUMULATION_COUNT = 5;       // Accumulate 5 frames for ~25k points per cloud

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
        
        // Find the time range within this cloud for normalization to [0,1] range
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();
        for (const auto &point : cloud.points) {
            min_time = std::min(min_time, static_cast<double>(point.time));
            max_time = std::max(max_time, static_cast<double>(point.time));
        }
        
        double time_range = max_time - min_time;
        if (time_range <= 0.0) time_range = 1.0; // Avoid division by zero
        
        for (const auto &point : cloud.points) {
            PointXYZIR p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            p.intensity = point.intensity;
            
            // Normalize timestamp to [0,1] range for KISS-ICP deskewing
            // This ensures proper motion compensation within each accumulated frame
            p.time = (point.time - min_time) / time_range;
            
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
    IceoryxPointCloud dst{};

    dst.publish_count = std::min<std::size_t>(src.points.size(), MAX_POINTS_PER_CLOUD);

    // Find time range for normalization to [0,1] range for KISS-ICP deskewing
    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::lowest();
    for (std::size_t i = 0; i < dst.publish_count; ++i) {
        min_time = std::min(min_time, static_cast<double>(src.points[i].time));
        max_time = std::max(max_time, static_cast<double>(src.points[i].time));
    }
    
    double time_range = max_time - min_time;
    if (time_range <= 0.0) time_range = 1.0; 

    for (std::size_t i = 0; i < dst.publish_count; ++i) {
        const auto &p_src = src.points[i];
        auto &p_dst       = dst.points[i];
        p_dst.x           = p_src.x;
        p_dst.y           = p_src.y;
        p_dst.z           = p_src.z;
        p_dst.intensity   = p_src.intensity;
        p_dst.time        = (p_src.time - min_time) / time_range;
        p_dst.ring        = static_cast<std::uint16_t>(p_src.ring);
    }

    return dst;
}

int main() {
    // -------------------- LiDAR initialisation --------------------
    UnitreeLidarReader* lreader = createUnitreeLidarReader();
    if (lreader == nullptr) {
        std::cerr << "Failed to create UnitreeLidarReader" << std::endl;
        return -1;
    }

    // -------------------- UDP (Ethernet) initialisation --------------------
    const unsigned short lidar_port  = 6101;             // L2 default transmit port
    const std::string    lidar_ip    = "192.168.1.62";   // L2 default IP
    const unsigned short local_port  = 6201;             // Target PC receive port
    const std::string    local_ip    = "192.168.1.2";    // PC NIC IP (must match NIC config)

    const uint16_t cloud_scan_num      = 18;     // default: one full 360° sweep (18×300 = 5400 pts)
    const bool     use_system_timestamp = true;
    const float    range_min            = 0.0f;
    const float    range_max            = 100.0f;

    if (lreader->initializeUDP(lidar_port, lidar_ip,
                               local_port, local_ip,
                               cloud_scan_num, use_system_timestamp,
                               range_min, range_max)) {
        std::cerr << "Unilidar UDP initialisation failed!" << std::endl;
        return -1;
    }

    lreader->startLidarRotation();
    sleep(1);
    // Work-mode 0 = Ethernet, 3-D, IMU enabled, self-start, normal FOV
    lreader->setLidarWorkMode(0);
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