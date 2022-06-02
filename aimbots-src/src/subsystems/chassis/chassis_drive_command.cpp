#include "chassis_drive_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisDriveCommand::ChassisDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis),
      currMode(MANUAL)
#ifdef TARGET_SENTRY
      ,
      profileConstraints(
          {1.0f,     // max velocity
           2.0f,     // max acceleration
           10.0f}),  // max jerk
      leftRailBound(leftRailBoundArray),
      rightRailBound(rightRailBoundArray)
#endif
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}  // namespace src::Chassis

void ChassisDriveCommand::initialize() {
}

void ChassisDriveCommand::execute() {
    switch (currMode) {
        case MANUAL:
            Movement::Relative::onExecute(drivers, chassis);
            break;
        case PATROL:

            break;
        case EVADE_SLOW:
            break;
        case EVADE_FAST:
            break;
    }
}

void ChassisDriveCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisDriveCommand::isReady() {
    return true;
}

bool ChassisDriveCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis