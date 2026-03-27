#pragma once

#include <cstdint>

enum class JointName : uint8_t {
    FrontLeft = 0,
    FrontRight = 1,
    BackLeft = 2,
    BackRight = 3,
};

enum class DriveMode : uint8_t
{
    NoUpdate = 0,
    Disabled = 1,
    Crabermann = 2,
    Ackermann = 3,
    Npt = 4,
    Linear = 5,
};

enum class DriveLimits : uint8_t
{
    NoUpdate = 0,
    Enabled = 1,
    Disabled = 2,
};