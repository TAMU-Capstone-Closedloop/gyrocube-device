/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of gyrocube-src.
 *
 * gyrocube-src is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * gyrocube-src is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gyrocube-src.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"
//
#include "robots/robot_control.hpp"
#include "utils/music/player.hpp"
#include "utils/nxp_imu/magnetometer/ist8310_data.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

// bmi088 is at 1000Hz.. coincidence? I think not!!11!
static constexpr float SAMPLE_FREQUENCY = 1000.0f;

uint32_t loopTimeDisplay = 0;

int main() {
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();

    tap::arch::PeriodicMilliTimer mainLoopTimeout(1000.0f / SAMPLE_FREQUENCY);

    initializeIo(drivers);
    src::Control::initializeSubsystemCommands(drivers);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1) {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (mainLoopTimeout.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }
        // every 2ms...
        if (sendMotorTimeout.execute()) {
            uint32_t loopStartTime = tap::arch::clock::getTimeMicroseconds();

            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            // PROFILE(drivers->profiler, drivers->terminalSerial.update, ()); // don't turn this on, it slows down UART
            // comms
            drivers->kinematicInformant.updateRobotFrames();
            utils::Music::playPacMan(drivers);

            loopTimeDisplay = tap::arch::clock::getTimeMicroseconds() - loopStartTime;
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers) {
    modm::platform::RandomNumberGenerator::enable();

    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->kinematicInformant.initialize(SAMPLE_FREQUENCY, 0.1f, 0.0f);

    drivers->remote.initialize();
    drivers->refSerial.initialize();
    // drivers->magnetometer.init();
    drivers->kinematicInformant.recalibrateIMU();

    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
}

float yawDisplay, pitchDisplay, rollDisplay;
float gXDisplay, gYDisplay, gZDisplay;
float aXDisplay, aYDisplay, aZDisplay;
tap::communication::sensors::imu::ImuInterface::ImuState imuStatus;

static void updateIo(src::Drivers *drivers) {
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();  // should probably also be updating for turret imu??
    drivers->refSerial.updateSerial();
    drivers->remote.read();

    // utils::Music::continuePlayingXPStartupTune(drivers);

    // imuStatus = drivers->kinematicInformant.getIMUState();

    // yawDisplay = drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::YAW_AXIS,
    // AngleUnit::Degrees); pitchDisplay =
    //     drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::PITCH_AXIS, AngleUnit::Degrees);
    // rollDisplay =
    //     drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::ROLL_AXIS, AngleUnit::Degrees);

    // gZDisplay =
    //     drivers->kinematicInformant.getChassisIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS,
    //     AngleUnit::Radians);
    // gYDisplay =
    //     drivers->kinematicInformant.getChassisIMUAngularVelocity(src::Informants::AngularAxis::PITCH_AXIS,
    //     AngleUnit::Radians);
    // gXDisplay =
    //     drivers->kinematicInformant.getChassisIMUAngularVelocity(src::Informants::AngularAxis::ROLL_AXIS,
    //     AngleUnit::Radians);

    // yawDisplay = modm::toDegree(yaw);
    // pitchDisplay = modm::toDegree(pitch);
    // rollDisplay = modm::toDegree(roll);
}
