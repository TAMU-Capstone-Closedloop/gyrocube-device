#pragma once
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 50'000.0f,  // 600
    .ki = 0.0f,
    .kd = 1'000.0f,  // 500
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// -------------------------------------------------------------------------------------------------------------------------

static Vector3f IMU_MOUNT_POSITION{0.0992f, 0.0f, 0.0534f};