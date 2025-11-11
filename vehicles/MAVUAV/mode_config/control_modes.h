#pragma once

#include <cstdint>

//==================================================================================================
// Control Mode Definitions
//==================================================================================================

/**
 * @brief Control mode enumeration
 */
enum class ControlMode : uint8_t {
    EMERGENCY_STOP = 0,    ///< Emergency stop mode (channel 6 down position)
    REMOTE_CONTROL = 1,    ///< Remote control mode (channel 6 middle position)
    COMPUTER_CONTROL = 2   ///< Computer control mode (channel 6 up position)
};

/**
 * @brief Transform mode enumeration
 */
enum class TransformMode : uint8_t {
    VEHICLE = 0,           ///< Vehicle mode (channel 5 down position)
    FLIGHT = 1             ///< Flight mode (channel 5 up position)
};