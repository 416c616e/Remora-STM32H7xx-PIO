#include "tmc.h"
#include <cstdint>

// CHOPCONF
#define TMC5160_INTPOL              1   // Step interpolation: 0 = off, 1 = on
#define TMC5160_TOFF                5   // Off time: 1 - 15, 0 = MOSFET disable (8)
#define TMC5160_TBL                 1   // Blanking time: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC5160_CHM                 0   // Chopper mode: 0 = spreadCycle, 1 = constant off time
// TMC5160_CHM 0 defaults
#define TMC5160_HSTRT               3   // Hysteresis start: 1 - 8
#define TMC5160_HEND                5   // Hysteresis end: -3 - 12
#define TMC5160_HMAX               16   // HSTRT + HEND
// TMC5160_CHM 1 defaults
#define TMC5160_TFD                 13  // fd3 & hstrt: 0 - 15

// IHOLD_IRUN
#define TMC5160_IHOLDDELAY          6

// TPOWERDOWN
#define TMC5160_TPOWERDOWN          128 // 0 - ((2^8)-1) * 2^18 tCLK

// TPWMTHRS
#define TMC5160_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// PWMCONF - StealthChop defaults
#define TMC5160_PWM_FREQ            1   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC5160_PWM_AUTOGRAD        1   // boolean (0 or 1)
#define TMC5160_PWM_GRAD            14  // 0 - 255
#define TMC5160_PWM_LIM             12  // 0 - 15
#define TMC5160_PWM_REG             8   // 1 - 15
#define TMC5160_PWM_OFS             36  // 0 - 255

// TCOOLTHRS
#define TMC5160_COOLSTEP_THRS       0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// COOLCONF - CoolStep defaults
#define TMC5160_SEMIN               5   // 0 = coolStep off, 1 - 15 = coolStep on
#define TMC5160_SEUP                0   // 0 - 3 (1 - 8)
#define TMC5160_SEMAX               2   // 0 - 15
#define TMC5160_SEDN                1   // 0 - 3
#define TMC5160_SEIMIN              0   // boolean (0 or 1)

std::shared_ptr<Module> TMC5160::create(const JsonObject& config, Remora* instance) {
    printf("Creating TMC5160 module\n\r");

    const char* comment = config["Comment"];
    printf("Comment: %s\n\r", comment);

    std::string pinCS = config["CS pin"];
    std::string pinMOSI = config["MOSI pin"];
    std::string pinMISO = config["MISO pin"];
    std::string pinSCK = config["SCK pin"];
    uint8_t address = config["Address"];

    float RSense = config["RSense"];
    uint16_t current = config["Current"];
    float holdCurrent = config["Hold current"];
    uint16_t microsteps = config["Microsteps"];
    uint8_t mode = config["Driver mode"];
    uint16_t stall = config["Stall sensitivity"];

    // Read multi-layer protection configuration
    uint8_t stallThreshold = 64;  // Default
    bool sgEnabled = true;
    float currentSpikeThreshold = 2.0f;
    bool sensorlessHomingEnabled = false;
    bool crashDetectionEnabled = false;
    bool layer1Enabled = true;
    bool layer2Enabled = true;
    bool layer3Enabled = false;

    // Check for sensorless homing configuration
    if (config.containsKey("Sensorless homing")) {
        const JsonObject& homingConfig = config["Sensorless homing"];
        sensorlessHomingEnabled = homingConfig["enabled"] | false;
        printf("Sensorless homing: %s\n\r", sensorlessHomingEnabled ? "enabled" : "disabled");
    }

    // Check for crash detection configuration
    if (config.containsKey("Crash detection")) {
        const JsonObject& crashConfig = config["Crash detection"];
        crashDetectionEnabled = crashConfig["enabled"] | false;
        layer1Enabled = crashConfig["layer1"]["enabled"] | true;
        layer2Enabled = crashConfig["layer2"]["enabled"] | true;
        layer3Enabled = crashConfig["layer3"]["enabled"] | false;
        currentSpikeThreshold = static_cast<float>(crashConfig["currentSpikeThreshold"] | 2.0f);
        printf("Crash detection: %s\n\r", crashDetectionEnabled ? "enabled" : "disabled");
        printf("  Layer1 (StallGuard2): %s\n\r", layer1Enabled ? "enabled" : "disabled");
        printf("  Layer2 (Current): %s\n\r", layer2Enabled ? "enabled" : "disabled");
        printf("  Layer3 (Position): %s\n\r", layer3Enabled ? "enabled" : "disabled");
    }

    // Read stall sensitivity if provided in crash detection config
    if (config.containsKey("Crash detection")) {
        const JsonObject& crashConfig = config["Crash detection"];
        if (crashConfig.containsKey("stallThreshold")) {
            stallThreshold = static_cast<uint8_t>(crashConfig["stallThreshold"] | 64);
        }
    }

    return std::make_shared<TMC5160>(std::move(pinCS), std::move(pinMOSI), std::move(pinMISO), std::move(pinSCK), RSense, address, current, microsteps, mode, stall, holdCurrent, instance,
                                     stallThreshold, sgEnabled, currentSpikeThreshold,
                                     sensorlessHomingEnabled, crashDetectionEnabled,
                                     layer1Enabled, layer2Enabled, layer3Enabled);
}

TMC5160::TMC5160(std::string _pinCS, std::string _pinMOSI, std::string _pinMISO, std::string _pinSCK, float _Rsense, uint8_t _addr, uint16_t _mA, uint16_t _microsteps, uint8_t _mode, uint16_t _stall, float _holdCurrent, Remora* _instance,
                   uint8_t stallThreshold, bool sgEnabled, float currentSpikeThreshold,
                   bool sensorlessHomingEnabled, bool crashDetectionEnabled,
                   bool layer1Enabled, bool layer2Enabled, bool layer3Enabled)
    : TMC{_instance, _Rsense},  // Call base class constructor
      pinCS(std::move(_pinCS)),
	  pinMOSI(std::move(_pinMOSI)),
	  pinMISO(std::move(_pinMISO)),
	  pinSCK(std::move(_pinSCK)),
      addr(_addr),
      mA(_mA),
      microsteps(_microsteps),
      mode(_mode),
      stall(_stall),
      holdCurrent(_holdCurrent),
      driver(std::make_unique<TMC5160Stepper>(pinCS, _Rsense, pinMOSI, pinMISO, pinSCK)),
      // Multi-layer protection configuration
      stallThreshold(stallThreshold),
      sgEnabled(sgEnabled),
      nominalCurrentA(static_cast<float>(_mA) / 1000.0f),
      nominalCurrentB(static_cast<float>(_mA) / 1000.0f),
      currentSpikeThreshold(currentSpikeThreshold),
      lastKnownPosition(0),
      positionTolerance(100),
      sensorlessHomingEnabled(sensorlessHomingEnabled),
      crashDetectionEnabled(crashDetectionEnabled),
      layer1Enabled(layer1Enabled),
      layer2Enabled(layer2Enabled),
      layer3Enabled(layer3Enabled)
      {}


void TMC5160::configure()
{
    driver->begin();

    printf("Testing connection to TMC driver... ");
    uint16_t result = driver->test_connection();
    
    if (result != 0) {
        printf("Failed!\nLikely cause: ");
        switch(result) {
            case 1: printf("Loose connection\n\r"); break;
            case 2: printf("No power\n\r"); break;
            default: printf("Unknown issue\n\r"); break;
        }
        printf("Fix the problem and reset the board.\n\r");
    } else {
        printf("OK - Version: %i, DRV_STATUS: 0x%08lX\n\r", driver->version(), driver->DRV_STATUS());

        if (driver->ola())  printf("\tOLA (Open Load A)\n\r");
        if (driver->olb())  printf("\tOLB (Open Load B)\n\r");
        if (driver->s2ga()) printf("\tS2GA (Short to Gnd A)\n\r");
        if (driver->s2gb()) printf("\tS2GB (Short to Gnd B)\n\r");
        if (driver->otpw()) printf("\tOTPW (Overtemp Prewarning)\n\r");
        if (driver->ot())   printf("\tOT (Overtemperature)\n\r");
        if (driver->stst()) printf("\tSTST (Standstill)\n\r");
    }

    driver->GSTAT(0b111);
    driver->defaults();
    driver->microsteps(this->microsteps);
    driver->rms_current(mA, holdCurrent);

    // GCONF
    switch(this->mode)
    {
        case TMC_MODE::STALLGUARD:
            driver->en_pwm_mode(false);
            break;
        case TMC_MODE::STEALTHCHOP:
            driver->en_pwm_mode(true);
            break;
        case TMC_MODE::COOLSTEP:
        default:
            driver->en_pwm_mode(false);
            break;
    }

    // CHOPCONF
    driver->intpol(TMC5160_INTPOL);
    driver->toff(TMC5160_TOFF);
    driver->tbl(TMC5160_TBL);
    driver->chm(TMC5160_CHM);
    driver->hend(TMC5160_HEND + 3);

    // CHM
    #if TMC5160_CHM == 0
        driver->hstrt(TMC5160_HSTRT - 1);
    #else
        driver->fd3((TMC5160_TFD & 0x08) >> 3);
        driver->hstrt(TMC5160_TFD & 0x07);
    #endif

    // COOLCONF
    driver->semin(TMC5160_SEMIN);
    driver->seup(TMC5160_SEUP);
    driver->semax(TMC5160_SEMAX);
    driver->sedn(TMC5160_SEDN);
    driver->seimin(TMC5160_SEIMIN);
    driver->TCOOLTHRS(TMC5160_COOLSTEP_THRS);
    
    // PWMCONF
    switch(this->mode)
    {
        case TMC_MODE::STALLGUARD:
            driver->pwm_autoscale(false);
            break;
        case TMC_MODE::STEALTHCHOP:
            driver->pwm_autoscale(true);
            break;
        case TMC_MODE::COOLSTEP:
        default:
            driver->pwm_autoscale(false);
            break;
    }
    driver->pwm_lim(TMC5160_PWM_LIM);
    driver->pwm_reg(TMC5160_PWM_REG);
    driver->pwm_autograd(TMC5160_PWM_AUTOGRAD);
    driver->pwm_freq(TMC5160_PWM_FREQ);
    driver->pwm_grad(TMC5160_PWM_GRAD);
    driver->pwm_ofs(TMC5160_PWM_OFS);

    // OTHERS
    driver->iholddelay(TMC5160_IHOLDDELAY);
    driver->TPOWERDOWN(TMC5160_TPOWERDOWN);
    driver->TPWMTHRS(TMC5160_TPWM_THRS);

    // Apply multi-layer protection configuration
    setStallThreshold(stallThreshold);
    enableStallGuard(sgEnabled);
    setNominalCurrent(nominalCurrentA, nominalCurrentB);
    setCurrentSpikeThreshold(currentSpikeThreshold);
    enableLayer1(layer1Enabled);
    enableLayer2(layer2Enabled);
    enableLayer3(layer3Enabled);

    printf("TMC5160 configured - StallGuard2: %s, Layer1: %s, Layer2: %s, Layer3: %s\n\r",
           sgEnabled ? "enabled" : "disabled",
           layer1Enabled ? "enabled" : "disabled",
           layer2Enabled ? "enabled" : "disabled",
           layer3Enabled ? "enabled" : "disabled");
}

void TMC5160::update(){}

// ============================================================================
// StallGuard2 Configuration Methods
// ============================================================================

void TMC5160::setStallThreshold(uint8_t threshold) {
    stallThreshold = threshold;
    // Apply threshold to COOLCONF register (sgt field)
    driver->sgt(static_cast<int8_t>(threshold));
}

uint8_t TMC5160::getStallThreshold() {
    return stallThreshold;
}

int16_t TMC5160::getStallGuardResult() {
    // Read DRV_STATUS register and extract sg_result (bits 15:0)
    uint32_t drvStatus = driver->DRV_STATUS();
    int16_t sgResult = static_cast<int16_t>(drvStatus & 0xFFFF);
    return sgResult;
}

void TMC5160::enableStallGuard(bool enable) {
    sgEnabled = enable;
    // Note: StallGuard2 is enabled via COOLCONF.semin > 0
    if (enable && driver->semin() == 0) {
        driver->semin(1);  // Minimum value to enable
    } else if (!enable) {
        driver->semin(0);  // Disable
    }
}

bool TMC5160::isStallGuardEnabled() {
    return sgEnabled;
}

// ============================================================================
// Current Monitoring Methods
// ============================================================================

void TMC5160::setNominalCurrent(float currentA, float currentB) {
    nominalCurrentA = currentA;
    nominalCurrentB = currentB;
}

void TMC5160::setCurrentSpikeThreshold(float threshold) {
    currentSpikeThreshold = threshold;
}

bool TMC5160::checkCurrentSpike() {
    int16_t currentA = getCurrentA();
    int16_t currentB = getCurrentB();
    
    // Convert current readings to amps (MSCURACT is in 10-bit format)
    // The actual current depends on the driver's current scaling
    // For TMC5160, we use the nominal current as reference
    
    float actualCurrentA = static_cast<float>(currentA) / 1024.0f * static_cast<float>(mA) / 1000.0f;
    float actualCurrentB = static_cast<float>(currentB) / 1024.0f * static_cast<float>(mA) / 1000.0f;
    
    // Check if either current exceeds the spike threshold
    bool spikeA = (nominalCurrentA > 0) && (actualCurrentA > nominalCurrentA * currentSpikeThreshold);
    bool spikeB = (nominalCurrentB > 0) && (actualCurrentB > nominalCurrentB * currentSpikeThreshold);
    
    return spikeA || spikeB;
}

int16_t TMC5160::getCurrentA() {
    // Read MSCURACT register and extract cur_a (bits 15:0)
    uint32_t mscuract = driver->MSCURACT();
    int16_t curA = static_cast<int16_t>(mscuract & 0xFFFF);
    return curA;
}

int16_t TMC5160::getCurrentB() {
    // Read MSCURACT register and extract cur_b (bits 31:16)
    uint32_t mscuract = driver->MSCURACT();
    int16_t curB = static_cast<int16_t>((mscuract >> 16) & 0xFFFF);
    return curB;
}

// ============================================================================
// Position Tracking Methods
// ============================================================================

void TMC5160::setLastKnownPosition(int32_t position) {
    lastKnownPosition = position;
}

int32_t TMC5160::getLastKnownPosition() {
    return lastKnownPosition;
}

void TMC5160::setPositionTolerance(int32_t tolerance) {
    positionTolerance = tolerance;
}

bool TMC5160::checkPositionDeviation() {
    // Read actual position from XACTUAL register
    int32_t currentPosition = driver->XACTUAL();
    
    // Calculate deviation from last known position
    int32_t deviation = currentPosition - lastKnownPosition;
    
    // Return true if deviation exceeds tolerance (absolute value)
    return (deviation > positionTolerance) || (deviation < -positionTolerance);
}

// ============================================================================
// Layer Control Methods
// ============================================================================

void TMC5160::enableLayer1(bool enable) {
    layer1Enabled = enable;
    enableStallGuard(enable);
}

void TMC5160::enableLayer2(bool enable) {
    layer2Enabled = enable;
}

void TMC5160::enableLayer3(bool enable) {
    layer3Enabled = enable;
}

bool TMC5160::isLayer1Enabled() {
    return layer1Enabled;
}

bool TMC5160::isLayer2Enabled() {
    return layer2Enabled;
}

bool TMC5160::isLayer3Enabled() {
    return layer3Enabled;
}

// ============================================================================
// Multi-Layer Validation Method
// ============================================================================

bool TMC5160::validateCrashCondition() {
    // Only validate if at least one layer is enabled
    if (!layer1Enabled && !layer2Enabled && !layer3Enabled) {
        return false;
    }
    
    bool layer1Trigger = false;
    bool layer2Trigger = false;
    bool layer3Trigger = false;
    
    // Layer 1: StallGuard2 check
    if (layer1Enabled && sgEnabled) {
        int16_t sgResult = getStallGuardResult();
        // Lower SG_RESULT indicates higher load (more sensitive)
        // When sg_result <= threshold, a stall/crash is detected
        if (sgResult <= static_cast<int16_t>(stallThreshold)) {
            layer1Trigger = true;
        }
    }
    
    // Layer 2: Current spike check
    if (layer2Enabled) {
        if (checkCurrentSpike()) {
            layer2Trigger = true;
        }
    }
    
    // Layer 3: Position deviation check
    if (layer3Enabled) {
        if (checkPositionDeviation()) {
            layer3Trigger = true;
        }
    }
    
    // Crash is confirmed if:
    // - Any single layer is enabled and triggered, OR
    // - Multiple layers are enabled and at least one triggers
    // This allows flexible configuration based on application needs
    
    bool crashDetected = layer1Trigger || layer2Trigger || layer3Trigger;
    
    return crashDetected;
}
