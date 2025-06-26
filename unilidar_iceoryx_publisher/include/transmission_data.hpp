// Copyright (c) 2024
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef UNILIDAR_ICEORYX_PUBLISHER_TRANSMISSION_DATA_HPP
#define UNILIDAR_ICEORYX_PUBLISHER_TRANSMISSION_DATA_HPP

#include <cstdint>
#include <iostream>

struct TransmissionData {
    std::int32_t x;
    std::int32_t y;
    double funky;
};

inline std::ostream& operator<<(std::ostream& os, const TransmissionData& value) {
    os << "TransmissionData { x: " << value.x << ", y: " << value.y << ", funky: " << value.funky << " }";
    return os;
}

#endif // UNILIDAR_ICEORYX_PUBLISHER_TRANSMISSION_DATA_HPP 