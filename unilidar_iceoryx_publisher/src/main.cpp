/**********************************************************************
 Copyright (c) 2024 Contributors
 SPDX-License-Identifier: Apache-2.0 OR MIT
***********************************************************************/

#include "example.h"           // Unitree SDK helper utilities
#include "imu_point_types.hpp" // ImuMsg & PointXYZIR structures

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
#include <ctime>

using namespace iox2;

constexpr iox::units::Duration CYCLE_TIME = iox::units::Duration::fromMilliseconds(10);


constexpr int ACCUMULATION_COUNT = 2;
constexpr time_t WARMUP_DURATION_SECONDS = 3;
constexpr bool ACCUMULATE_FRAMES = true;

struct FrameAccumulator
{
    std::vector<PointXYZIR> accumulated_points;
    int frame_count = 0;
    time_t first_frame_time = 0;
    bool warmup_complete = false;
    bool warmup_started = false;

    void reset()
    {
        accumulated_points.clear();
        frame_count = 0;
        // Don't reset first_frame_time and warmup flags during normal reset
    }

    bool isWarmupComplete() const
    {
        if (!warmup_started) return false;
        double current_time = static_cast<double>(std::time(nullptr)); // fallback if no frame time available
        return (current_time - first_frame_time) >= WARMUP_DURATION_SECONDS;
    }

    bool addFrame(const PointCloudUnitree &cloud, int accumulation_count)
    {
        // Start warmup on first frame
        if (frame_count == 0)
        {
            if (!warmup_started)
            {
                std::time(&first_frame_time);
                warmup_started = true;
            }
        }

        // Update warmup status
        if (!warmup_complete && warmup_started)
        {
            time_t now;
            std::time(&now);
            warmup_complete = (now - first_frame_time) >= WARMUP_DURATION_SECONDS;
        }

        for (const auto &point : cloud.points)
        {
            PointXYZIR p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            p.intensity = point.intensity;

            p.time = point.time;

            p.ring = static_cast<std::uint16_t>(point.ring);
            accumulated_points.push_back(p);
        }

        frame_count++;
        
        int target_count = warmup_complete ? 1 : accumulation_count;
        return frame_count >= target_count;
    }
};

// Convert accumulated points to the fixed-size Iceoryx structure
static IceoryxPointCloud toIceoryxPointCloud(const std::vector<PointXYZIR> &points)
{
    IceoryxPointCloud dst{}; // zero-initialise all fields
    dst.is_last = true;
    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::lowest();
    for (const auto &point : points)
    {
        min_time = std::min(min_time, static_cast<double>(point.time));
        max_time = std::max(max_time, static_cast<double>(point.time));
    }
    double time_range = max_time - min_time;
    if (time_range <= 0.0)
        time_range = 1.0;

    if (points.size() > MAX_POINTS_PER_CLOUD) {
        std::cout << "WARNING: max points per cloud set too low." << std::endl;
    }
    dst.publish_count = std::min<std::size_t>(points.size(), MAX_POINTS_PER_CLOUD);

    for (std::size_t i = 0; i < dst.publish_count; ++i)
    {
        dst.points[i] = points[i];
        dst.points[i].time = (points[i].time - min_time) / time_range;
    }

    return dst;
}


int main()
{
    // -------------------- LiDAR initialisation --------------------
    UnitreeLidarReader *lreader = createUnitreeLidarReader();
    if (lreader == nullptr)
    {
        std::cerr << "Failed to create UnitreeLidarReader" << std::endl;
        return -1;
    }

    // -------------------- UDP (Ethernet) initialisation --------------------
    const unsigned short lidar_port = 6101;      // L2 default transmit port
    const std::string lidar_ip = "192.168.1.62"; // L2 default IP
    const unsigned short local_port = 6201;      // Target PC receive port
    const std::string local_ip = "192.168.1.2";  // PC NIC IP (must match NIC config)

    const uint16_t cloud_scan_num = 8;
    const bool use_system_timestamp = true;
    const float range_min = 0.0f;
    const float range_max = 100.0f;

    if (lreader->initializeUDP(lidar_port, lidar_ip,
                               local_port, local_ip,
                               cloud_scan_num, use_system_timestamp,
                               range_min, range_max))
    {
        std::cerr << "Unilidar UDP initialisation failed!" << std::endl;
        return -1;
    }

    lreader->startLidarRotation();
    sleep(1);
    // Work-mode 0 = Ethernet, 3-D, IMU enabled, self-start, normal FOV
    lreader->setLidarWorkMode(0);
    sleep(1);

    // ---------------------- iceoryx2 setup -----------------------
    set_log_level_from_env_or(LogLevel::Fatal);
    auto node = NodeBuilder().create<ServiceType::Ipc>().expect("node creation");

    // IMU service
    auto imu_service = node.service_builder(ServiceName::create("unilidar/imu").expect("name"))
                           .publish_subscribe<ImuMsg>()
                           .enable_safe_overflow(true)
                           .open_or_create()
                           .expect("imu service");

    // Point service
    auto point_service = node.service_builder(ServiceName::create("unilidar/cloud_full").expect("name"))
                             .publish_subscribe<IceoryxPointCloud>()
                             .enable_safe_overflow(true)
                             .open_or_create()
                             .expect("cloud");

    auto imu_publisher = imu_service.publisher_builder().create().expect("imu publisher");
    auto cloud_publisher = point_service.publisher_builder().create().expect("cloud publisher");

    // ---------------------- Processing loop ----------------------
    LidarImuData imu_raw;
    PointCloudUnitree cloud_raw;
    FrameAccumulator accumulator;

    while (true)
    {
        int parse_result = lreader->runParse();

        switch (parse_result)
        {
        case LIDAR_IMU_DATA_PACKET_TYPE:
        {
            if (lreader->getImuData(imu_raw))
            {
                ImuMsg msg{};
                std::copy(std::begin(imu_raw.quaternion), std::end(imu_raw.quaternion), std::begin(msg.quaternion));
                std::copy(std::begin(imu_raw.angular_velocity), std::end(imu_raw.angular_velocity), std::begin(msg.angular_velocity));
                std::copy(std::begin(imu_raw.linear_acceleration), std::end(imu_raw.linear_acceleration), std::begin(msg.linear_acceleration));

                auto sample = imu_publisher.loan_uninit().expect("imu loan");
                auto initialized = sample.write_payload(msg);
                send(std::move(initialized)).expect("imu send");
            }
            break;
        }
        case LIDAR_POINT_DATA_PACKET_TYPE:
        {
            if (lreader->getPointCloud(cloud_raw))
            {
                if (ACCUMULATE_FRAMES)
                {
                    if (accumulator.addFrame(cloud_raw, ACCUMULATION_COUNT))
                    {
                        IceoryxPointCloud cloud = toIceoryxPointCloud(accumulator.accumulated_points);
                        std::cout << "Accumulated " << accumulator.frame_count << " frames, total points: "
                                  << cloud.publish_count << " (raw accumulated: " << accumulator.accumulated_points.size() << ")" << std::endl;

                        auto sample = cloud_publisher.loan_uninit().expect("cloud loan");
                        auto initialized = sample.write_payload(cloud);
                        send(std::move(initialized)).expect("cloud send");
                        std::cout << "Accumulated cloud sent" << std::endl;

                        accumulator.reset();
                    }
                    else
                    {
                        std::cout << "Frame " << accumulator.frame_count << "/" << ACCUMULATION_COUNT
                                  << " accumulated (" << cloud_raw.points.size() << " points)" << std::endl;
                    }
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
