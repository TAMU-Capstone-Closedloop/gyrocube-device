#ifdef TARGET_SENTRY

#include "drivers.hpp"
#include "drivers_singleton.hpp"
#include "utils/common_types.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_bounce_command.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_evade_command.hpp"
#include "subsystems/chassis/sentry_commands/sentry_match_chassis_control_command.hpp"
//
#include "subsystems/feeder/burst_feeder_command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/sentry_commands/sentry_match_firing_control_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
#include "subsystems/gimbal/sentry_commands/gimbal_patrol_command.hpp"
#include "subsystems/gimbal/sentry_commands/sentry_match_gimbal_control_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;

using namespace src::Control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace SentryControl {

src::Chassis::ChassisMatchStates chassisMatchState = src::Chassis::ChassisMatchStates::NONE;
// src::Feeder::FeederMatchStates feederMatchState = src::Feeder::FeederMatchStates::ANNOYED;

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers());

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalController(&gimbal);

// Match Controllers ------------------------------------------------
SentryMatchFiringControlCommand matchFiringControlCommand(drivers(), &feeder, &shooter, chassisMatchState);
SentryMatchChassisControlCommand matchChassisControlCommand(drivers(), &chassis, chassisMatchState);
SentryMatchGimbalControlCommand matchGimbalControlCommand(drivers(), &gimbal, &gimbalController, 500.0f);

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
ChassisRailBounceCommand chassisRailBounceCommand(drivers(), &chassis);
ChassisRailEvadeCommand chassisRailEvadeCommand(drivers(), &chassis, 25.0f);  // velocity ramp value is 25.0f

GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalController, USER_JOYSTICK_YAW_SCALAR, USER_JOYSTICK_PITCH_SCALAR);
GimbalPatrolCommand gimbalPatrolCommand(drivers(), &gimbal, &gimbalController);
// pass chassisRelative controller to gimbalChaseCommand on sentry, pass fieldRelative for other robots
GimbalChaseCommand gimbalChaseCommand(drivers(), &gimbal, &gimbalController);

FullAutoFeederCommand fullAutoFeederCommand(drivers(), &feeder);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&matchChassisControlCommand, &matchFiringControlCommand, &matchGimbalControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Enables both chassis and gimbal manual control
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&chassisManualDriveCommand, &gimbalChaseCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Runs shooter only
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&fullAutoFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
    feeder.setDefaultCommand(&stopFeederCommand);
    // gimbal.setDefaultCommand(&gimbalControlCommand);
    // chassis.setDefaultCommand(&chassisRailBounceCommand);
    // gimbal.setDefaultCommand(&gimbalChaseCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
}

}  // namespace SentryControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    SentryControl::initializeSubsystems();
    SentryControl::registerSubsystems(drivers);
    SentryControl::setDefaultCommands(drivers);
    SentryControl::startupCommands(drivers);
    SentryControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_SENTRY