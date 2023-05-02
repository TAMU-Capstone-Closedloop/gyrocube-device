#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include "subsystems/chassis/chassis_helper.hpp"

#include "chassis_follow_gimbal_command.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {}

// Debug variables
float gimbalYawFieldRelativeDisplay = 0.0f;
float yawAngleFromChassisCenterDisplay2 = 0.0f;
float chassisYawDisplay = 0.0f;
float rotationControllerOutputDisplay = 0.0f;
float rotationLimitedMaxTranslationalSpeedDisplay = 0.0f;

void ChassisFollowGimbalCommand::execute() {
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    // gets desired user input from operator interface
    Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    if (gimbal->isOnline()) {  // if the gimbal is online, follow the gimbal's yaw
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        // Find rotation correction power
        rotationController.runController(
            yawAngleFromChassisCenter,
            RADPS_TO_RPM * drivers->kinematicInformant.getIMUAngularVelocity(
                               src::Informants::AngularAxis::YAW_AXIS,
                               AngleUnit::Radians));
        rotationControllerOutputDisplay = rotationController.getOutput();

        // overwrite desired rotation with rotation controller output, range [-1, 1]
        desiredRotation = rotationController.getOutput();

        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

        tap::algorithms::rotateVector(&desiredX, &desiredY, -yawAngleFromChassisCenter);

    } else {  // if the gimbal is offline, run the normal manual drive command
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisFollowGimbalCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisFollowGimbalCommand::isReady() { return true; }

bool ChassisFollowGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif