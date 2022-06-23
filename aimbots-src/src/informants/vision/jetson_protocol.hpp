#pragma once

namespace src::Informants::vision {

enum CVState : uint8_t {
    LOOK_AT_COORDS = 0,
    FIRE = 1,
    LOOK_AROUND = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a';

struct JetsonMessage {
    uint8_t magic;           // 97
    float targetYawOffset;   // units=radians if target is to the left of camera-center, this will be negative
    float targetPitchOffset; // units=radians if target is below camera-center, this will be positive for some reason
    CVState cvState;         // 0 is track, 1 is shoot, 2 is patrol
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 10, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::vision