#include "gimbal_toggle_aiming_command.hpp"

namespace src::Gimbal {

GimbalToggleAimCommand::GimbalToggleAimCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbal,
    GimbalControllerInterface* controller,
    src::Utils::RefereeHelper* refHelper,
    BarrelID& barrelID,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      gimbalCVCommand(drivers, gimbal, controller, refHelper, barrelID, ballisticsSolver),
      gimbalFreeAimCommand(drivers, gimbal, controller) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalToggleAimCommand::initialize() {
    if (!comprisedCommandScheduler.isCommandScheduled(&gimbalCVCommand))
        comprisedCommandScheduler.removeCommand(&gimbalCVCommand, true);
    if (!comprisedCommandScheduler.isCommandScheduled(&gimbalFreeAimCommand))
        comprisedCommandScheduler.addCommand(&gimbalFreeAimCommand);

    ignoreQuickTurn.restart(0);
}

void GimbalToggleAimCommand::execute() {
    // This needs to match the button in Chassis Toggle Drive!
    if (drivers->remote.keyPressed(Remote::Key::F)) {
        gimbalCVCommand.setIgnoreQuickTurn(true);
        gimbalFreeAimCommand.setIgnoreQuickTurn(true);
        ignoreQuickTurn.restart(500);
    } else if (ignoreQuickTurn.execute()) {
        gimbalCVCommand.setIgnoreQuickTurn(false);
        gimbalFreeAimCommand.setIgnoreQuickTurn(false);
    }

    if (drivers->remote.getMouseR()) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &gimbalCVCommand);
    } else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &gimbalFreeAimCommand);
    }
    comprisedCommandScheduler.run();
}

void GimbalToggleAimCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &gimbalCVCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &gimbalFreeAimCommand, interrupted);
}

bool GimbalToggleAimCommand::isReady() { return true; }

bool GimbalToggleAimCommand::isFinished() const { return false; }

}  // namespace src::Gimbal