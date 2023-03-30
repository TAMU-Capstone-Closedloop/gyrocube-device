#pragma once

#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "informants/communication/communication_message.hpp"
// #include "informants/communication/communication_response_handler.hpp"
// #include "utils/common_types.hpp"
#include "tap/control/subsystem.hpp"

#include "informants/communication/communication_response_handler.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
using namespace src::Communication;

namespace src::RobotStates {
class RobotStates : public tap::control::Subsystem {
private:
    Matrix<Robot, 2, 9> robotStates;
    tap::Drivers& drivers;
    CommunicationResponseHandler messageHandler;

public:
    RobotStates(tap::Drivers& drivers);
    ~RobotStates();

    void refresh() override;

    void setIndivualRobotState(Robot robot);
    Robot getIndivualRobotState(int robotNumber, Team teamColor);

    void updateRobotState(int robotNumber, Team teamColor, short x, short y, short z, int health);
    void updateRobotStateHealth(int robotNumber, Team teamColor, int health);
    void updateRobotStatePosition(int robotNumber, Team teamColor, short x, short y, short z);

#ifdef TARGET_SENTRY
    robot_state_message_team createMessage();
#else
    robot_state_message createMesssage();
    robot_state_message_enemy createMessageEnemy();
#endif
    void respond();

    // void updateRobotStateHero();
    // void updateRobotStateStandard();
    // void updateRobotStateSentry();
};

}  // namespace src::RobotStates