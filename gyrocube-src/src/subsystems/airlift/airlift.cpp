#include "airlift.hpp"

#include "tap/board/board.hpp"

#include "drivers.hpp"

using namespace Board;

namespace src::Airlift {

Airlift::Airlift(src::Drivers* drivers) : drivers(drivers) {}

void Airlift::initialize() {
    AirliftSpiMaster::connect<AirliftMiso::Miso, AirliftMosi::Mosi, AirliftSck::Sck>();
    AirliftSpiMaster::initialize<SystemClock, 10_MHz>();
}

}  // namespace src::Airlift