#include "tmc.h"
#include <cstdint>

#define TOFF_VALUE  4 // [1... 15]

std::shared_ptr<Module> TMC2209::create(const JsonObject& config, Remora* instance) {
    printf("Creating TMC2209 module\n\r");

    const char* comment = config["Comment"];
    printf("Comment: %s\n\r", comment);

    std::string RxPin = config["RX pin"];
    float RSense = config["RSense"];
    uint8_t address = config["Address"];
    uint16_t current = config["Current"];
    uint16_t microsteps = config["Microsteps"];
    uint16_t stall = config["Stall sensitivity"];
    bool stealthchop = (strcmp(config["Stealth chop"], "on") == 0);

    return std::make_shared<TMC2209>(std::move(RxPin), RSense, address, current, microsteps, stealthchop, stall, instance);
}

TMC2209::TMC2209(std::string _rxtxPin, float _Rsense, uint8_t _addr, uint16_t _mA, uint16_t _microsteps, bool _stealth, uint16_t _stall, Remora* _instance)
    : TMC{_instance, _Rsense},  // Call base class constructor
      rxtxPin(std::move(_rxtxPin)),
      addr(_addr),
      mA(_mA),
      microsteps(_microsteps),
      stealth(_stealth),
      stall(_stall),
      driver(std::make_unique<TMC2209Stepper>(rxtxPin, rxtxPin, Rsense, addr)) {}


void TMC2209::configure()
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
    driver->TCOOLTHRS(0xFFFFF);  // 20-bit max threshold for smart energy CoolStep
    driver->semin(5);             // CoolStep lower threshold
    driver->semax(2);             // CoolStep upper threshold
    driver->sedn(0b01);           // CoolStep decrement rate
    driver->en_spreadCycle(!stealth);
    driver->pwm_autoscale(true);

    if (stealth && stall) {
        // StallGuard sensitivity threshold (higher = more sensitive)
        driver->SGTHRS(stall);
    }

    driver->iholddelay(10);
    driver->TPOWERDOWN(128);  // ~2s until driver lowers to hold current

    printf("Stopping the Serial thread\n\r");
    instance->getSerialThread()->stopThread();
    instance->getSerialThread()->unregisterModule(self);
}

void TMC2209::update()
{
    driver->SWSerial->tickerHandler();
}

// Multi-layer protection implementations
void TMC2209::setStallThreshold(uint8_t threshold) {
    driver->SGTHRS(threshold);
    stallThreshold = threshold;
}

uint8_t TMC2209::getStallThreshold() {
    return driver->SGTHRS();
}

int16_t TMC2209::getStallGuardResult() {
    return static_cast<int16_t>(driver->SG_RESULT());
}

int16_t TMC2209::getCurrentA() {
    uint32_t mcur = driver->MSCURACT();
    TMC2208_n::MSCURACT_t r{0};
    r.sr = mcur;
    return r.cur_a;
}

int16_t TMC2209::getCurrentB() {
    uint32_t mcur = driver->MSCURACT();
    TMC2208_n::MSCURACT_t r{0};
    r.sr = mcur;
    return r.cur_b;
}

bool TMC2209::checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB) {
    return (abs(currentA) > thresholdA || abs(currentB) > thresholdB);
}

bool TMC2209::checkStallGuard() {
    int16_t sgResult = getStallGuardResult();
    // StallGuard result is 10-bit (0-1023), higher = less load
    // When stall detected, value drops significantly
    // Threshold comparison: if sgResult < (256 - stallThreshold), stall detected
    return (sgResult < (256 - stallThreshold));
}

void TMC2209::enableLayer1(bool enabled) {
    layer1Enabled = enabled;
    if (enabled) {
        // Enable StallGuard2 monitoring
        driver->en_spreadCycle(true);  // SpreadCycle required for StallGuard2
    }
}

void TMC2209::enableLayer2(bool enabled) {
    layer2Enabled = enabled;
}

void TMC2209::enableLayer3(bool enabled) {
    layer3Enabled = enabled;
}

bool TMC2209::isLayer1Enabled() { return layer1Enabled; }
bool TMC2209::isLayer2Enabled() { return layer2Enabled; }
bool TMC2209::isLayer3Enabled() { return layer3Enabled; }

bool TMC2209::validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) {
    bool layer1Triggered = false;
    bool layer2Triggered = false;
    bool layer3Triggered = false;
    
    // Layer 1: StallGuard2
    if (layer1Enabled) {
        layer1Triggered = checkStallGuard();
    }
    
    // Layer 2: Current spike
    if (layer2Enabled) {
        layer2Triggered = checkCurrentSpike(currentA, currentB, layer2ThresholdA, layer2ThresholdB);
    }
    
    // Layer 3: Position deviation
    if (layer3Enabled) {
        layer3Triggered = checkPositionDeviation(currentPosition, layer3PositionDeviation);
    }
    
    // Multi-layer validation
    if (requireMultipleLayers) {
        int triggeredCount = (layer1Triggered ? 1 : 0) +
                            (layer2Triggered ? 1 : 0) +
                            (layer3Triggered ? 1 : 0);
        return triggeredCount >= 2;
    }
    
    return layer1Triggered || layer2Triggered || layer3Triggered;
}
