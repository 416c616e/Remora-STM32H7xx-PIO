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

// Multi-layer protection implementations
// Note: TMC2208 does NOT support StallGuard2

void TMC2208::setStallThreshold(uint8_t threshold) {
    // Not supported on TMC2208
    stallThreshold = 0;
}

uint8_t TMC2208::getStallThreshold() {
    return 0;  // Not supported
}

int16_t TMC2208::getStallGuardResult() {
    return 0;  // Not supported
}

int16_t TMC2208::getCurrentA() {
    uint32_t mcur = driver->MSCURACT();
    union {
        uint32_t sr;
        struct {
            int16_t cur_a : 9;
            int16_t : 7;
            int16_t cur_b : 9;
        };
    } r;
    r.sr = mcur;
    return r.cur_a;
}

int16_t TMC2208::getCurrentB() {
    uint32_t mcur = driver->MSCURACT();
    union {
        uint32_t sr;
        struct {
            int16_t cur_a : 9;
            int16_t : 7;
            int16_t cur_b : 9;
        };
    } r;
    r.sr = mcur;
    return r.cur_b;
}

bool TMC2208::checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB) {
    return (abs(currentA) > thresholdA || abs(currentB) > thresholdB);
}

bool TMC2208::checkStallGuard() {
    // Not supported on TMC2208
    return false;
}

void TMC2208::enableLayer1(bool enabled) {
    // Layer 1 (StallGuard2) not supported on TMC2208
    layer1Enabled = false;
}

void TMC2208::enableLayer2(bool enabled) {
    layer2Enabled = enabled;
}

void TMC2208::enableLayer3(bool enabled) {
    layer3Enabled = enabled;
}

bool TMC2208::isLayer1Enabled() { return false; }  // Not supported
bool TMC2208::isLayer2Enabled() { return layer2Enabled; }
bool TMC2208::isLayer3Enabled() { return layer3Enabled; }

bool TMC2208::validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) {
    bool layer1Triggered = false;
    bool layer2Triggered = false;
    bool layer3Triggered = false;
    
    // Layer 1: StallGuard2 - NOT SUPPORTED
    // Always false for TMC2208
    
    // Layer 2: Current spike
    if (layer2Enabled) {
        layer2Triggered = checkCurrentSpike(currentA, currentB, static_cast<int16_t>(currentSpikeThresholdA), static_cast<int16_t>(currentSpikeThresholdB));
    }
    
    // Layer 3: Position deviation
    if (layer3Enabled) {
        layer3Triggered = checkPositionDeviation(currentPosition, positionTolerance);
    }
    
    // Multi-layer validation
    if (requireMultipleLayers) {
        int triggeredCount = (layer1Triggered ? 1 : 0) +
                            (layer2Triggered ? 1 : 0) +
                            (layer3Triggered ? 1 : 0);
        return triggeredCount >= 2;
    }
    
    return layer2Triggered || layer3Triggered;
}
