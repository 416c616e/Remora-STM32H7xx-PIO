#include "tmc.h"
#include <cstdint>

#define TOFF_VALUE  4 // [1... 15]

std::shared_ptr<Module> TMC5160::create(const JsonObject& config, Remora* instance) {
    printf("Creating TMC5160 module\n\r");

    const char* comment = config["Comment"];
    printf("Comment: %s\n\r", comment);

    std::string pinCS = config["CS pin"];
    std::string pinMOSI = config["MOSI pin"];
    std::string pinMISO = config["MISO pin"];
    std::string pinSCK = config["SCK pin"];
    float RSense = config["RSense"];
    uint8_t address = config["Address"];
    uint16_t current = config["Current"];
    uint16_t microsteps = config["Microsteps"];
    uint16_t stall = config["Stall sensitivity"];
    bool stealthchop = (strcmp(config["Stealth chop"], "on") == 0);

    return std::make_shared<TMC5160>(std::move(pinCS), std::move(pinMOSI), std::move(pinMISO), std::move(pinSCK), RSense, address, current, microsteps, stealthchop, stall, instance);
}

TMC5160::TMC5160(std::string _pinCS, std::string _pinMOSI, std::string _pinMISO, std::string _pinSCK, float _Rsense, uint8_t _addr, uint16_t _mA, uint16_t _microsteps, bool _stealth, uint16_t _stall, Remora* _instance)
    : TMC{_instance, _Rsense},  // Call base class constructor
      pinCS(std::move(_pinCS)),
	  pinMOSI(std::move(_pinMOSI)),
	  pinMISO(std::move(_pinMISO)),
	  pinSCK(std::move(_pinSCK)),
      addr(_addr),
      mA(_mA),
      microsteps(_microsteps),
      stealth(_stealth),
      stall(_stall),
      driver(std::make_unique<TMC5160Stepper>(pinCS, _Rsense, pinMOSI, pinMISO, pinSCK)) {}


void TMC5160::configure()
{
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
    driver->TCOOLTHRS(0xFFFFF);  // 20-bit max threshold for smart energy CoolStep
    driver->semin(5);             // CoolStep lower threshold
    driver->semax(2);             // CoolStep upper threshold
    driver->sedn(0b01);           // CoolStep decrement rate
    //driver->en_spreadCycle(!stealth);
    driver->pwm_autoscale(true);

    if (stealth && stall) {
        // StallGuard sensitivity threshold (higher = more sensitive)
        //driver->SGTHRS(stall);
    }

    driver->iholddelay(10);
    driver->TPOWERDOWN(128);  // ~2s until driver lowers to hold current
}

void TMC5160::update(){}
