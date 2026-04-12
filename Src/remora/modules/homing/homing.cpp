#include "homing.h"
#include <STM32H7_SPIComms.h>
#include <cmath>

// Default configuration values
constexpr int32_t DEFAULT_HOMING_SPEED = 100;       // mm/s
constexpr int32_t DEFAULT_FINE_SPEED = 10;          // mm/s
constexpr int32_t DEFAULT_RETREAT_DISTANCE = 5;     // mm
constexpr int32_t DEFAULT_RETREAT_SPEED = 50;       // mm/s
constexpr uint32_t DEFAULT_MAX_TRAVEL = 100;        // mm
constexpr uint32_t DEFAULT_DEBOUNCE_TIME = 10;      // ms
constexpr uint8_t DEFAULT_STALL_THRESHOLD = 64;     // 0-255

std::shared_ptr<Module> Homing::create(const JsonObject& config, Remora* instance) {
    printf("Creating Homing module\n\r");
    
    Homing* homing = new Homing(instance);
    
    // Read configuration
    const char* axisStr = config["Axis"];
    if (axisStr) {
        if (strcmp(axisStr, "X") == 0) homing->setAxis(Homing::Axis::X);
        else if (strcmp(axisStr, "Y") == 0) homing->setAxis(Homing::Axis::Y);
        else if (strcmp(axisStr, "Z") == 0) homing->setAxis(Homing::Axis::Z);
        else if (strcmp(axisStr, "A") == 0) homing->setAxis(Homing::Axis::A);
        else if (strcmp(axisStr, "B") == 0) homing->setAxis(Homing::Axis::B);
        else if (strcmp(axisStr, "C") == 0) homing->setAxis(Homing::Axis::C);
        else if (strcmp(axisStr, "D") == 0) homing->setAxis(Homing::Axis::D);
        else if (strcmp(axisStr, "E") == 0) homing->setAxis(Homing::Axis::E);
    }
    
    const char* modeStr = config["Mode"];
    if (modeStr) {
        if (strcmp(modeStr, "SENSORLESS") == 0) homing->setMode(Homing::HomingMode::SENSORLESS);
        else if (strcmp(modeStr, "LIMIT_SWITCH") == 0) homing->setMode(Homing::HomingMode::LIMIT_SWITCH);
        else if (strcmp(modeStr, "HYBRID") == 0) homing->setMode(Homing::HomingMode::HYBRID);
    }
    
    homing->setHomingSpeed(config["Homing Speed"] | DEFAULT_HOMING_SPEED);
    homing->setFineSpeed(config["Fine Speed"] | DEFAULT_FINE_SPEED);
    homing->setStallThreshold(config["Stall Threshold"] | DEFAULT_STALL_THRESHOLD);
    homing->setRetreatDistance(config["Retreat Distance"] | DEFAULT_RETREAT_DISTANCE);
    homing->setRetreatSpeed(config["Retreat Speed"] | DEFAULT_RETREAT_SPEED);
    homing->setMaxTravel(config["Max Travel"] | DEFAULT_MAX_TRAVEL);
    homing->setDebounceTime(config["Debounce Time"] | DEFAULT_DEBOUNCE_TIME);
    homing->setInvertDirection(strcmp(config["Invert Direction"] | "off", "on") == 0);
    homing->setLimitSwitchInvert(strcmp(config["Limit Switch Invert"] | "off", "on") == 0);
    
    printf("Homing configured - Axis: %s, Mode: %s, Speed: %d mm/s, Fine: %d mm/s\n\r",
           axisStr ? axisStr : "X",
           modeStr ? modeStr : "SENSORLESS",
           homing->getHomingSpeed(),
           homing->getFineSpeed());
    
    return std::shared_ptr<Module>(homing);
}

Homing::Homing(Remora* _instance) : instance(_instance) {
    // Initialize config with defaults
    config.axis = Axis::X;
    config.mode = HomingMode::SENSORLESS;
    config.homingSpeed = DEFAULT_HOMING_SPEED;
    config.fineSpeed = DEFAULT_FINE_SPEED;
    config.stallThreshold = DEFAULT_STALL_THRESHOLD;
    config.retreatDistance = DEFAULT_RETREAT_DISTANCE;
    config.retreatSpeed = DEFAULT_RETREAT_SPEED;
    config.invertDirection = false;
    config.maxTravel = DEFAULT_MAX_TRAVEL;
    config.debounceTime = DEFAULT_DEBOUNCE_TIME;
    config.limitSwitchInvert = false;
    
    // Initialize result
    result.success = false;
    result.homePosition = 0;
    result.state = HomingState::IDLE;
    result.errorCode = 0;
    result.elapsedTime = 0;
    
    // Initialize state
    state = HomingState::IDLE;
    active = false;
    triggerDetected = false;
    triggerDebounceStart = 0;
    currentCount = 0;
    targetCount = 0;
    tmcModule = nullptr;
}

void Homing::setConfig(const HomingConfig& cfg) {
    config = cfg;
}

HomingConfig Homing::getConfig() const {
    return config;
}

void Homing::setAxis(Axis axis) {
    config.axis = axis;
}

Homing::Axis Homing::getAxis() const {
    return config.axis;
}

void Homing::setMode(HomingMode mode) {
    config.mode = mode;
}

Homing::HomingMode Homing::getMode() const {
    return config.mode;
}

void Homing::setHomingSpeed(int32_t speed) {
    config.homingSpeed = speed;
}

int32_t Homing::getHomingSpeed() const {
    return config.homingSpeed;
}

void Homing::setFineSpeed(int32_t speed) {
    config.fineSpeed = speed;
}

int32_t Homing::getFineSpeed() const {
    return config.fineSpeed;
}

void Homing::setStallThreshold(uint8_t threshold) {
    config.stallThreshold = threshold;
    if (tmcModule) {
        tmcModule->setStallThreshold(threshold);
    }
}

uint8_t Homing::getStallThreshold() const {
    return config.stallThreshold;
}

void Homing::setRetreatDistance(int32_t distance) {
    config.retreatDistance = distance;
}

int32_t Homing::getRetreatDistance() const {
    return config.retreatDistance;
}

void Homing::setRetreatSpeed(int32_t speed) {
    config.retreatSpeed = speed;
}

int32_t Homing::getRetreatSpeed() const {
    return config.retreatSpeed;
}

void Homing::setMaxTravel(uint32_t distance) {
    config.maxTravel = distance;
}

uint32_t Homing::getMaxTravel() const {
    return config.maxTravel;
}

void Homing::setDebounceTime(uint32_t time) {
    config.debounceTime = time;
}

uint32_t Homing::getDebounceTime() const {
    return config.debounceTime;
}

void Homing::setInvertDirection(bool invert) {
    config.invertDirection = invert;
}

bool Homing::getInvertDirection() const {
    return config.invertDirection;
}

void Homing::setLimitSwitchInvert(bool invert) {
    config.limitSwitchInvert = invert;
}

bool Homing::getLimitSwitchInvert() const {
    return config.limitSwitchInvert;
}

void Homing::setTMCModule(std::shared_ptr<TMC> tmc) {
    tmcModule = tmc;
    if (tmc) {
        tmc->setStallThreshold(config.stallThreshold);
    }
}

std::shared_ptr<TMC> Homing::getTMCModule() {
    return tmcModule;
}

void Homing::startHoming() {
    if (active) {
        return;  // Already homing
    }
    
    active = true;
    state = HomingState::HOMING;
    triggerDetected = false;
    triggerDebounceStart = 0;
    result.success = false;
    result.errorCode = 0;
    result.elapsedTime = 0;
    result.homePosition = 0;
    startTime = millis();
    currentCount = 0;
    targetCount = 0;
    
    printf("Homing sequence started for axis %d, mode %d\n\r", 
           static_cast<int>(config.axis), 
           static_cast<int>(config.mode));
    
    homingSequence();
}

void Homing::cancelHoming() {
    if (!active) {
        return;
    }
    
    stopMotion();
    active = false;
    state = HomingState::IDLE;
    result.success = false;
    result.errorCode = 1;  // Cancelled
    result.elapsedTime = millis() - startTime;
    
    printf("Homing sequence cancelled\n\r");
}

void Homing::reset() {
    active = false;
    state = HomingState::IDLE;
    triggerDetected = false;
    result.success = false;
    result.errorCode = 0;
    result.elapsedTime = 0;
    result.homePosition = 0;
    currentCount = 0;
    targetCount = 0;
    
    printf("Homing module reset\n\r");
}

HomingResult Homing::getResult() {
    result.elapsedTime = millis() - startTime;
    result.state = state;
    return result;
}

Homing::HomingState Homing::getState() {
    return state;
}

bool Homing::isHomingComplete() {
    return (state == HomingState::COMPLETE) || (state == HomingState::ERROR);
}

bool Homing::isHomingActive() {
    return active;
}

int32_t Homing::getHomePosition() {
    return result.homePosition;
}

uint32_t Homing::getLastError() {
    return result.errorCode;
}

const char* Homing::getErrorString() {
    switch (result.errorCode) {
        case 0: return "No error";
        case 1: return "Cancelled";
        case 2: return "Max travel exceeded";
        case 3: return "Trigger timeout";
        case 4: return "Invalid configuration";
        case 5: return "TMC communication error";
        default: return "Unknown error";
    }
}

void Homing::homingSequence() {
    if (!active || state != HomingState::HOMING) {
        return;
    }
    
    // Calculate target position based on max travel
    targetCount = mmToSteps(static_cast<float>(config.maxTravel));
    
    // Set direction (negative for homing towards limit)
    setDirection(!config.invertDirection);
    
    // Set frequency based on homing speed
    int32_t frequency = config.homingSpeed * 10;
    setFrequency(frequency);
    
    // Monitor for trigger condition
    if (checkTriggerCondition()) {
        handleTrigger();
        return;
    }
    
    // Check for max travel exceeded
    if (currentCount >= targetCount) {
        handleError(2);  // Max travel exceeded
        return;
    }
    
    // Check for timeout (no trigger after max travel)
    uint32_t elapsed = millis() - startTime;
    if (elapsed > 30000) {  // 30 second timeout
        handleError(3);  // Trigger timeout
        return;
    }
    
    // Continue homing
    updatePosition();
}

void Homing::retreatSequence() {
    if (state != HomingState::RETREAT) {
        return;
    }
    
    // Calculate retreat distance in steps
    int32_t retreatSteps = mmToSteps(static_cast<float>(config.retreatDistance));
    
    // Set direction for retreat (opposite of homing direction)
    setDirection(config.invertDirection);
    
    // Set frequency for retreat speed
    int32_t frequency = config.retreatSpeed * 10;
    setFrequency(frequency);
    
    // Move until retreat distance is reached
    if (currentCount < retreatSteps) {
        updatePosition();
        return;
    }
    
    // Retreat complete
    stopMotion();
    active = false;
    state = HomingState::COMPLETE;
    result.success = true;
    result.homePosition = currentCount;
    
    printf("Homing complete - Position: %d steps\n\r", currentCount);
}

bool Homing::checkTriggerCondition() {
    if (!active || state != HomingState::HOMING) {
        return false;
    }
    
    // For sensorless homing, check StallGuard2
    if (config.mode == HomingMode::SENSORLESS || config.mode == HomingMode::HYBRID) {
        if (tmcModule && tmcModule->isLayer1Enabled()) {
            if (tmcModule->validateCrashCondition()) {
                // Start debounce timer
                if (triggerDebounceStart == 0) {
                    triggerDebounceStart = millis();
                }
                
                // Check if debounce time has passed
                if (millis() - triggerDebounceStart >= config.debounceTime) {
                    return true;
                }
            } else {
                // Reset debounce if condition cleared
                triggerDebounceStart = 0;
            }
        }
    }
    
    // For limit switch homing, check limit switch input
    if (config.mode == HomingMode::LIMIT_SWITCH || config.mode == HomingMode::HYBRID) {
        // TODO: Implement limit switch reading
    }
    
    return false;
}

void Homing::handleTrigger() {
    printf("Homing trigger detected\n\r");
    
    // Stop motion immediately
    stopMotion();
    
    // Set home position
    result.homePosition = currentCount;
    
    // Start retreat sequence
    state = HomingState::RETREAT;
    triggerDebounceStart = 0;
}

void Homing::handleError(uint32_t errorCode) {
    printf("Homing error: %s\n\r", getErrorString());
    
    stopMotion();
    active = false;
    state = HomingState::ERROR;
    result.success = false;
    result.errorCode = errorCode;
    result.elapsedTime = millis() - startTime;
}

void Homing::updatePosition() {
    // Update position based on current count
    currentCount += 1;
}

int32_t Homing::mmToSteps(float mm) {
    // Simplified conversion - assume 100 steps/mm
    return static_cast<int32_t>(mm * 100.0f);
}

float Homing::stepsToMm(int32_t steps) {
    // Inverse of mmToSteps
    return static_cast<float>(steps) / 100.0f;
}

void Homing::setDirection(bool forward) {
    // Set direction pin based on forward parameter
    (void)forward;
}

void Homing::setFrequency(int32_t frequency) {
    // Set step frequency
    (void)frequency;
}

void Homing::stopMotion() {
    // Stop all motion
}

void Homing::update(void) {
    // Main update loop for homing
    if (active) {
        if (state == HomingState::HOMING) {
            homingSequence();
        } else if (state == HomingState::RETREAT) {
            retreatSequence();
        }
    }
}

void Homing::slowUpdate(void) {
    // Slow update - not used for homing
}

void Homing::updatePost(void) {
    // Post update - not used for homing
}
