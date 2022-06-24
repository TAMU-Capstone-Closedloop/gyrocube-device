#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(speed / 2.0f) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(1000);
}

void FullAutoFeederCommand::execute() {
    // if (!startupThreshold.isExpired()) {
    //     if (fabs(feeder->getCurrentRPM()) < 100.0f && unjamTimer.isExpired()) {
    //         unjamTimer.restart(300);
    //     }
    // }
    // if (unjamTimer.isExpired() || unjamTimer.isStopped()) {
    feeder->setTargetRPM(speed);
    // } else {
    //     feeder->setTargetRPM(unjamSpeed);
    // }
}

void FullAutoFeederCommand::end(bool) {}

bool FullAutoFeederCommand::isReady() {
    return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool FullAutoFeederCommand::isFinished() const {
    return !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Feeder