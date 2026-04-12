#include "crashDetection.h"
#include <STM32H7_SPIComms.h>
#include <Arduino.h>

// Default configuration values
constexpr uint32_t DEFAULT_LAYER1_DEBOUNCE = 10;      // ms
constexpr uint32_t DEFAULT_LAYER1_CONFIRMATION = 3;   // counts
constexpr uint32_t DEFAULT_LAYER2_DEBOUNCE = 5;       // ms
constexpr uint32_t DEFAULT_LAYER2_CONFIRMATION = 2;   // counts
constexpr uint32_t DEFAULT_LAYER3_DEBOUNCE = 20;      // ms
constexpr uint32_t DEFAULT_LAYER3_CONFIRMATION = 5;   // counts

std::shared_ptr<Module> CrashDetection::create(const JsonObject& config, Remora* instance) {
    printf("Creating CrashDetection module\n\r");
    
    CrashDetection* crashDetection = new CrashDetection(instance);
    
    // Read configuration
    bool enabled = config["enabled"] | false;
    crashDetection->enable(enabled);
    
    // Layer 1 (StallGuard2) configuration
    if (config.containsKey("layer1")) {
        const JsonObject& layer1Config = config["layer1"];
        crashDetection->setLayerEnabled(Layer::STALLGUARD2, layer1Config["enabled"] | true);
        crashDetection->setLayerDebounceTime(Layer::STALLGUARD2, layer1Config["debounceTime"] | DEFAULT_LAYER1_DEBOUNCE);
        crashDetection->setLayerConfirmationCount(Layer::STALLGUARD2, layer1Config["confirmationCount"] | DEFAULT_LAYER1_CONFIRMATION);
    }
    
    // Layer 2 (Current spike) configuration
    if (config.containsKey("layer2")) {
        const JsonObject& layer2Config = config["layer2"];
        crashDetection->setLayerEnabled(Layer::CURRENT_SPIKE, layer2Config["enabled"] | true);
        crashDetection->setLayerDebounceTime(Layer::CURRENT_SPIKE, layer2Config["debounceTime"] | DEFAULT_LAYER2_DEBOUNCE);
        crashDetection->setLayerConfirmationCount(Layer::CURRENT_SPIKE, layer2Config["confirmationCount"] | DEFAULT_LAYER2_CONFIRMATION);
    }
    
    // Layer 3 (Position deviation) configuration
    if (config.containsKey("layer3")) {
        const JsonObject& layer3Config = config["layer3"];
        crashDetection->setLayerEnabled(Layer::POSITION_DEVIATION, layer3Config["enabled"] | false);
        crashDetection->setLayerDebounceTime(Layer::POSITION_DEVIATION, layer3Config["debounceTime"] | DEFAULT_LAYER3_DEBOUNCE);
        crashDetection->setLayerConfirmationCount(Layer::POSITION_DEVIATION, layer3Config["confirmationCount"] | DEFAULT_LAYER3_CONFIRMATION);
    }
    
    // Overall configuration
    crashDetection->setRequireMultipleLayers(config["requireMultipleLayers"] | false);
    crashDetection->setEnableImmediateStop(config["enableImmediateStop"] | true);
    crashDetection->setEnableLogging(config["enableLogging"] | true);
    crashDetection->setEnableNotifications(config["enableNotifications"] | true);
    
    printf("CrashDetection configured - Enabled: %s\n\r", enabled ? "yes" : "no");
    printf("  Layer1 (StallGuard2): %s\n\r", crashDetection->isLayerEnabled(Layer::STALLGUARD2) ? "enabled" : "disabled");
    printf("  Layer2 (Current): %s\n\r", crashDetection->isLayerEnabled(Layer::CURRENT_SPIKE) ? "enabled" : "disabled");
    printf("  Layer3 (Position): %s\n\r", crashDetection->isLayerEnabled(Layer::POSITION_DEVIATION) ? "enabled" : "disabled");
    
    return std::shared_ptr<Module>(crashDetection);
}

CrashDetection::CrashDetection(Remora* _instance) : instance(_instance), enabled(false) {
    // Initialize config with defaults
    config.layer1.enabled = true;
    config.layer1.debounceTime = DEFAULT_LAYER1_DEBOUNCE;
    config.layer1.confirmationCount = DEFAULT_LAYER1_CONFIRMATION;
    
    config.layer2.enabled = true;
    config.layer2.debounceTime = DEFAULT_LAYER2_DEBOUNCE;
    config.layer2.confirmationCount = DEFAULT_LAYER2_CONFIRMATION;
    
    config.layer3.enabled = false;
    config.layer3.debounceTime = DEFAULT_LAYER3_DEBOUNCE;
    config.layer3.confirmationCount = DEFAULT_LAYER3_CONFIRMATION;
    
    config.requireMultipleLayers = false;
    config.enableImmediateStop = true;
    config.enableLogging = true;
    config.enableNotifications = true;
    
    // Initialize state
    state = CrashState::IDLE;
    eventCount = 0;
    layer1Triggered = false;
    layer2Triggered = false;
    layer3Triggered = false;
    layer1Count = 0;
    layer2Count = 0;
    layer3Count = 0;
    layer1DebounceStart = 0;
    layer2DebounceStart = 0;
    layer3DebounceStart = 0;
    
    // Initialize last event
    lastEvent.timestamp = 0;
    lastEvent.layersTriggered = 0;
    lastEvent.stallGuardResult = 0;
    lastEvent.currentA = 0;
    lastEvent.currentB = 0;
    lastEvent.positionDeviation = 0;
    
    tmcModule = nullptr;
}

void CrashDetection::setConfig(const CrashConfig& cfg) {
    config = cfg;
}

CrashConfig CrashDetection::getConfig() const {
    return config;
}

void CrashDetection::setLayerEnabled(Layer layer, bool enabled) {
    switch (layer) {
        case Layer::STALLGUARD2:
            config.layer1.enabled = enabled;
            break;
        case Layer::CURRENT_SPIKE:
            config.layer2.enabled = enabled;
            break;
        case Layer::POSITION_DEVIATION:
            config.layer3.enabled = enabled;
            break;
    }
}

bool CrashDetection::isLayerEnabled(Layer layer) {
    switch (layer) {
        case Layer::STALLGUARD2:
            return config.layer1.enabled;
        case Layer::CURRENT_SPIKE:
            return config.layer2.enabled;
        case Layer::POSITION_DEVIATION:
            return config.layer3.enabled;
    }
    return false;
}

void CrashDetection::setLayerDebounceTime(Layer layer, uint32_t ms) {
    switch (layer) {
        case Layer::STALLGUARD2:
            config.layer1.debounceTime = ms;
            break;
        case Layer::CURRENT_SPIKE:
            config.layer2.debounceTime = ms;
            break;
        case Layer::POSITION_DEVIATION:
            config.layer3.debounceTime = ms;
            break;
    }
}

void CrashDetection::setLayerConfirmationCount(Layer layer, uint32_t count) {
    switch (layer) {
        case Layer::STALLGUARD2:
            config.layer1.confirmationCount = count;
            break;
        case Layer::CURRENT_SPIKE:
            config.layer2.confirmationCount = count;
            break;
        case Layer::POSITION_DEVIATION:
            config.layer3.confirmationCount = count;
            break;
    }
}

void CrashDetection::setRequireMultipleLayers(bool require) {
    config.requireMultipleLayers = require;
}

void CrashDetection::setEnableImmediateStop(bool enable) {
    config.enableImmediateStop = enable;
}

void CrashDetection::setEnableLogging(bool enable) {
    config.enableLogging = enable;
}

void CrashDetection::setEnableNotifications(bool enable) {
    config.enableNotifications = enable;
}

void CrashDetection::enable(bool enable) {
    enabled = enable;
    if (enabled) {
        state = CrashState::IDLE;
    }
}

bool CrashDetection::isEnabled() {
    return enabled;
}

void CrashDetection::reset() {
    enabled = false;
    state = CrashState::RESET;
    layer1Triggered = false;
    layer2Triggered = false;
    layer3Triggered = false;
    layer1Count = 0;
    layer2Count = 0;
    layer3Count = 0;
    layer1DebounceStart = 0;
    layer2DebounceStart = 0;
    layer3DebounceStart = 0;
    eventCount = 0;
    
    printf("CrashDetection module reset\n\r");
}

CrashDetection::CrashState CrashDetection::getState() {
    return state;
}

bool CrashDetection::isCrashDetected() {
    return (state == CrashState::DETECTING) || (state == CrashState::CONFIRMED) || (state == CrashState::TRIGGERED);
}

bool CrashDetection::isCrashConfirmed() {
    return (state == CrashState::CONFIRMED) || (state == CrashState::TRIGGERED);
}

bool CrashDetection::isCrashTriggered() {
    return state == CrashState::TRIGGERED;
}

CrashEvent CrashDetection::getLastEvent() {
    return lastEvent;
}

uint32_t CrashDetection::getEventCount() {
    return eventCount;
}

void CrashDetection::clearEventCount() {
    eventCount = 0;
}

bool CrashDetection::isLayer1Triggered() {
    return layer1Triggered;
}

bool CrashDetection::isLayer2Triggered() {
    return layer2Triggered;
}

bool CrashDetection::isLayer3Triggered() {
    return layer3Triggered;
}

uint32_t CrashDetection::getLayerTriggerCount(Layer layer) {
    switch (layer) {
        case Layer::STALLGUARD2:
            return layer1Count;
        case Layer::CURRENT_SPIKE:
            return layer2Count;
        case Layer::POSITION_DEVIATION:
            return layer3Count;
    }
    return 0;
}

void CrashDetection::setTMCModule(std::shared_ptr<TMC> tmc) {
    tmcModule = tmc;
}

std::shared_ptr<TMC> CrashDetection::getTMCModule() {
    return tmcModule;
}

void CrashDetection::update() {
    if (!enabled) {
        return;
    }
    
    // Check each layer
    checkLayer1();
    checkLayer2();
    checkLayer3();
    
    // Validate crash condition
    validateCrashCondition();
}

void CrashDetection::checkLayer1() {
    if (!config.layer1.enabled || !tmcModule) {
        return;
    }
    
    // Use TMC module's validateCrashCondition for layer 1
    if (tmcModule->validateCrashCondition()) {
        if (layer1DebounceStart == 0) {
            layer1DebounceStart = millis();
        }
        
        if (millis() - layer1DebounceStart >= config.layer1.debounceTime) {
            layer1Triggered = true;
            layer1Count++;
            
            if (layer1Count >= config.layer1.confirmationCount) {
                layer1Triggered = true;
            }
        }
    } else {
        layer1DebounceStart = 0;
        if (layer1Count > 0) {
            layer1Count--;
        }
    }
}

void CrashDetection::checkLayer2() {
    if (!config.layer2.enabled || !tmcModule) {
        return;
    }
    
    if (tmcModule->checkCurrentSpike()) {
        if (layer2DebounceStart == 0) {
            layer2DebounceStart = millis();
        }
        
        if (millis() - layer2DebounceStart >= config.layer2.debounceTime) {
            layer2Triggered = true;
            layer2Count++;
            
            if (layer2Count >= config.layer2.confirmationCount) {
                layer2Triggered = true;
            }
        }
    } else {
        layer2DebounceStart = 0;
        if (layer2Count > 0) {
            layer2Count--;
        }
    }
}

void CrashDetection::checkLayer3() {
    if (!config.layer3.enabled || !tmcModule) {
        return;
    }
    
    if (tmcModule->checkPositionDeviation()) {
        if (layer3DebounceStart == 0) {
            layer3DebounceStart = millis();
        }
        
        if (millis() - layer3DebounceStart >= config.layer3.debounceTime) {
            layer3Triggered = true;
            layer3Count++;
            
            if (layer3Count >= config.layer3.confirmationCount) {
                layer3Triggered = true;
            }
        }
    } else {
        layer3DebounceStart = 0;
        if (layer3Count > 0) {
            layer3Count--;
        }
    }
}

void CrashDetection::validateCrashCondition() {
    if (!enabled) {
        return;
    }
    
    // Count triggered layers
    uint32_t triggeredLayers = 0;
    if (layer1Triggered) triggeredLayers++;
    if (layer2Triggered) triggeredLayers++;
    if (layer3Triggered) triggeredLayers++;
    
    // Check if crash condition is met
    bool crashCondition = false;
    
    if (config.requireMultipleLayers) {
        // Require at least 2 layers to trigger
        crashCondition = (triggeredLayers >= 2);
    } else {
        // Any single layer trigger is sufficient
        crashCondition = (triggeredLayers >= 1);
    }
    
    if (crashCondition && state != CrashState::TRIGGERED) {
        state = CrashState::CONFIRMED;
        
        // Record event details
        lastEvent.timestamp = millis();
        lastEvent.layersTriggered = triggeredLayers;
        
        if (tmcModule) {
            lastEvent.stallGuardResult = tmcModule->getStallGuardResult();
            lastEvent.currentA = tmcModule->getCurrentA();
            lastEvent.currentB = tmcModule->getCurrentB();
            lastEvent.positionDeviation = tmcModule->getLastKnownPosition();
        }
        
        triggerCrash();
    } else if (!crashCondition && state == CrashState::CONFIRMED) {
        // Reset if condition cleared before trigger
        state = CrashState::IDLE;
        layer1Triggered = false;
        layer2Triggered = false;
        layer3Triggered = false;
        layer1Count = 0;
        layer2Count = 0;
        layer3Count = 0;
    }
}

void CrashDetection::triggerCrash() {
    printf("CRASH DETECTED! Layers: %d, SG_RESULT: %d\n\r", 
           lastEvent.layersTriggered, lastEvent.stallGuardResult);
    
    state = CrashState::TRIGGERED;
    eventCount++;
    
    // Log event
    if (config.enableLogging) {
        logEvent();
    }
    
    // Notify host
    if (config.enableNotifications) {
        notifyHost();
    }
    
    // Immediate stop if enabled
    if (config.enableImmediateStop && tmcModule) {
        // Disable all layers
        tmcModule->enableLayer1(false);
        tmcModule->enableLayer2(false);
        tmcModule->enableLayer3(false);
        
        // TODO: Trigger emergency stop in step generator
        printf("Emergency stop triggered\n\r");
    }
}

void CrashDetection::logEvent() {
    // Log crash event details
    printf("Crash Event #%d at %lu ms\n\r", eventCount, lastEvent.timestamp);
    printf("  Layers Triggered: %d\n\r", lastEvent.layersTriggered);
    printf("  StallGuard Result: %d\n\r", lastEvent.stallGuardResult);
    printf("  Current A: %d, Current B: %d\n\r", lastEvent.currentA, lastEvent.currentB);
    printf("  Position Deviation: %d\n\r", lastEvent.positionDeviation);
}

void CrashDetection::notifyHost() {
    // TODO: Send crash notification to host system
    // This would typically use the communication interface
    (void)lastEvent;  // Suppress unused warning
}

void CrashDetection::slowUpdate(void) {
    // Slow update - not used for crash detection
}

void CrashDetection::updatePost(void) {
    // Post update - not used for crash detection
}
