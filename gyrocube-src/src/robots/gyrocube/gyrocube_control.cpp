#ifdef TARGET_GYROCUBE

#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

//
#include "informants/transformers/robot_frames.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;

namespace GyrocubeControl {

// Define subsystems here ------------------------------------------------
// ChassisSubsystem chassis(drivers());

// Define commands here ---------------------------------------------------
// ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),  // gimbalFieldRelativeControlCommand
    {/*&chassisToggleDriveCommand, &gimbalToggleAimCommand, &gimbalChaseCommand*/},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),  // gimbalFieldRelativeControlCommand2
    {/*&chassisTokyoCommand, &chassisAutoNavTokyoCommand, &gimbalChaseCommand2*/},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {/*&openHopperCommand*/},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(
    drivers(),
    {/*&runShooterCommand, &toggleHopperCommand*/},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {/*&runDoubleBarrelFeederCommand, &runShooterWithFeederCommand, &closeHopperCommand2*/},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldCommandMapping leftClickMouse(
    drivers(),
    {/*&runDoubleBarrelFeederCommandFromMouse*/},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) { /*drivers->commandScheduler.registerSubsystem(&chassis);*/
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() { /*chassis.initialize();*/ }

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
    // drivers->refSerial.attachRobotToRobotMessageHandler(SENTRY_RESPONSE_MESSAGE_ID, &responseHandler);
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftClickMouse);
    // drivers->commandMapper.addMap(&bCtrlPressed);
}

}  // namespace GyrocubeControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    GyrocubeControl::initializeSubsystems();
    GyrocubeControl::registerSubsystems(drivers);
    GyrocubeControl::setDefaultCommands(drivers);
    GyrocubeControl::startupCommands(drivers);
    GyrocubeControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_GYROCUBE
