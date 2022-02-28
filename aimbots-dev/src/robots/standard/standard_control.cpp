#ifdef TARGET_STANDARD

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
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/constants/gimbal_controller_constants.hpp"

using namespace src::Gimbal;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace StandardControl {

// Define subsystems here ------------------------------------------------
GimbalSubsystem gimbal(drivers());

GimbalChassisRelativeController gimbalController(&gimbal, Constants::ChassisRelative::YAW_PID_CONFIG, Constants::ChassisRelative::PITCH_PID_CONFIG);

// Define commands here ---------------------------------------------------
GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalController, 1.0f);

HoldCommandMapping leftSwitchUp(
    drivers(),
    {&gimbalControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&gimbal);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    gimbal.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers* drivers) { }

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the standard's parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
}

}  // namespace StandardControl

namespace src::Control {
    // Initialize subsystems ---------------------------------------------------
    void initializeSubsystemCommands(src::Drivers * drivers) {
        StandardControl::initializeSubsystems();
        StandardControl::registerSubsystems(drivers);
        StandardControl::setDefaultCommands(drivers);
        StandardControl::startupCommands(drivers);
        StandardControl::registerIOMappings(drivers);
    }
}  // namespace src::Control

#endif