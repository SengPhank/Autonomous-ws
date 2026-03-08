#include <stdint.h>
enum class JointIndex : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3,
};

enum class DriveMode : uint8_t {
    NO_UPDATE = 0,
    DISABLED = 1,
    CRABERMANN = 2,
    ACKERMANN = 3,
    NPT = 4,
    LINEAR = 5,
};

enum class DriveLimits : uint8_t {
    NO_UPDATE = 0,
    ENABLED = 1,
    DISABLED = 2,
};