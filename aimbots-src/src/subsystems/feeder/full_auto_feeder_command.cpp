#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-250.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(1000);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

bool unjaming = false;
void FullAutoFeederCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(500);
        unjaming = true;
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        unjaming = false;
        // if (startupThreshold.isExpired()) {
            startupThreshold.restart(500);
        // }
    }
}

void FullAutoFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool FullAutoFeederCommand::isReady() { return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold); }

bool FullAutoFeederCommand::isFinished() const { return !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold); }

}  // namespace src::Feeder