#pragma once

#include "modm/processing/protothread.hpp"

namespace src {
class Drivers;
}

namespace src::Airlift {

class Airlift {
public:
    Airlift(src::Drivers* drivers);
    ~Airlift() = default;

    void initialize();

private:
    src::Drivers* drivers;
};

}  // namespace src::Airlift