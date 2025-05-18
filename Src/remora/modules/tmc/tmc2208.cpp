#include "tmc.h"
#include <cstdint>

#define TOFF_VALUE  4 // [1... 15]

std::shared_ptr<Module> TMC2208::create(const JsonObject& config, Remora* instance) {
    printf("Creating TMC2208 module\n\r");

    const char* comment = config["Comment"];
    printf("Comment: %s\n\r", comment);

    std::string RxPin = config["RX pin"];
    float RSense = config["RSense"];
    uint16_t current = config["Current"];
    uint16_t microsteps = config["Microsteps"];
    bool stealthchop = (strcmp(config["Stealth chop"], "on") == 0);

    return std::make_shared<TMC2208>(std::move(RxPin), RSense, current, microsteps, stealthchop, instance);
}

TMC2208::TMC2208(std::string _rxtxPin, float _Rsense, uint16_t _mA, uint16_t _microsteps, bool _stealth, Remora* _instance)
    : TMC{_instance, _Rsense},  // Call base class constructor
      rxtxPin(std::move(_rxtxPin)),
      mA(_mA),
      microsteps(_microsteps),
      stealth(_stealth),
      driver(std::make_unique<TMC2208Stepper>(rxtxPin, rxtxPin, Rsense)) {}


void TMC2208::configure()
{
    printf("Starting the Serial thread\n\r");
    instance->getSerialThread()->startThread();

    auto self = shared_from_this();
    instance->getSerialThread()->registerModule(self);

    driver->begin();

    printf("Testing connection to TMC driver... ");
    uint16_t result = driver->test_connection();
    
    if (result) {
        printf("Failed!\nLikely cause: ");
        switch(result) {
            case 1: printf("Loose connection\n\r"); break;
            case 2: printf("No power\n\r"); break;
            default: printf("Unknown issue\n\r"); break;
        }
        printf("Fix the problem and reset the board.\n\r");
    } else {
        printf("OK\n\r");
    }

    // Configure driver settings
    driver->toff(TOFF_VALUE);
    driver->blank_time(24);
    driver->rms_current(mA);
    driver->microsteps(microsteps);
    driver->en_spreadCycle(!stealth);
    driver->pwm_autoscale(true);
    driver->iholddelay(10);
    driver->TPOWERDOWN(128);  // ~2s until driver lowers to hold current

    printf("Stopping the Serial thread\n\r");
    instance->getSerialThread()->stopThread();
    instance->getSerialThread()->unregisterModule(self);
}

void TMC2208::update()
{
    driver->SWSerial->tickerHandler();
}
