#include "utils/ballistics_solver.hpp"
#include "utils/robot_specific_inc.hpp"

#ifdef GIMBAL_UNTETHERED

#include "subsystems/chassis/chassis_helper.hpp"
#include "subsystems/chassis/chassis_shakira_command.hpp"

namespace src::Chassis {

ChassisShakiraCommand::ChassisShakiraCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    SmoothPIDConfig* rotationControllerConfig,
    BallisticsSolver* ballisticsSolver,
    const SnapSymmetryConfig& snapSymmetryConfig,
    float angularMagnitude,
    uint32_t timePeriod)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      rotationController(*rotationControllerConfig),
      ballisticsSolver(ballisticsSolver),
      snapSymmetryConfig(snapSymmetryConfig),
      angularMagnitude(angularMagnitude),
      timePeriod(timePeriod)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisShakiraCommand::initialize() { startingTimestamp = tap::arch::clock::getTimeMilliseconds(); }

void ChassisShakiraCommand::execute() {
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    float angularMagnitudeMultiplier = 1.0f;

    Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        std::optional<BallisticsSolver::BallisticsSolution> ballisticsSolution = ballisticsSolver->solve();

        if (ballisticsSolution != std::nullopt) {
            float targetYawAngle = ballisticsSolution->yawAngle;  // chassis relative gimbal target

            if (fabsf(desiredX) > TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED ||
                fabsf(desiredY) > TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED) {
                angularMagnitudeMultiplier = TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
            }

            float offsetFromTarget =
                angularMagnitude * angularMagnitudeMultiplier *
                sinf(M_TWOPI * (tap::arch::clock::getTimeMilliseconds() - startingTimestamp) / timePeriod);

            float chassisErrorAngle =
                Helper::findNearestChassisErrorTo(targetYawAngle + offsetFromTarget, snapSymmetryConfig);

            rotationController.runController(
                chassisErrorAngle,
                -RADPS_TO_RPM(drivers->kinematicInformant.getIMUAngularVelocity(
                    src::Informants::AngularAxis::YAW_AXIS,
                    AngleUnit::Radians)));

            desiredRotation = rotationController.getOutput();

            Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(
                drivers,
                chassis,
                &desiredX,
                &desiredY,
                &desiredRotation);

            tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
        } else {
            rotationController.runController(
                yawAngleFromChassisCenter,
                -RADPS_TO_RPM(drivers->kinematicInformant.getIMUAngularVelocity(
                    src::Informants::AngularAxis::YAW_AXIS,
                    AngleUnit::Radians)));

            desiredRotation = rotationController.getOutput();

            Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(
                drivers,
                chassis,
                &desiredX,
                &desiredY,
                &desiredRotation);

            tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
        }
    } else {
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }
    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

bool ChassisShakiraCommand::isReady() { return true; }

bool ChassisShakiraCommand::isFinished() const { return false; }

void ChassisShakiraCommand::end(bool interrupted) { UNUSED(interrupted); }

}  // namespace src::Chassis

#endif