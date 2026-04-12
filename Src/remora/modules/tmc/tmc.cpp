#include "tmc.h"

// Base TMC class method implementations

void TMC::setStallThreshold(uint8_t threshold) {
    stallThreshold = threshold;
}

uint8_t TMC::getStallThreshold() {
    return stallThreshold;
}

int16_t TMC::getStallGuardResult() {
    return 0;
}

int16_t TMC::getCurrentA() {
    return 0;
}

int16_t TMC::getCurrentB() {
    return 0;
}

void TMC::setLastKnownPosition(int32_t position) {
    lastKnownPosition = position;
}

int32_t TMC::getLastKnownPosition() {
    return lastKnownPosition;
}

void TMC::setPositionTolerance(int32_t tolerance) {
    positionTolerance = tolerance;
}

bool TMC::checkPositionDeviation(int32_t currentPosition, int32_t tolerance) {
    return (abs(currentPosition - lastKnownPosition) > tolerance);
}

void TMC::enableLayer1(bool enable) {
    layer1Enabled = enable;
}

void TMC::enableLayer2(bool enable) {
    layer2Enabled = enable;
}

void TMC::enableLayer3(bool enable) {
    layer3Enabled = enable;
}

bool TMC::isLayer1Enabled() {
    return layer1Enabled;
}

bool TMC::isLayer2Enabled() {
    return layer2Enabled;
}

bool TMC::isLayer3Enabled() {
    return layer3Enabled;
}

bool TMC::checkStallGuard() {
    return false;
}

bool TMC::checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB) {
    return (abs(currentA) > thresholdA || abs(currentB) > thresholdB);
}

bool TMC::validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) {
    bool layer1Triggered = false;
    bool layer2Triggered = false;
    bool layer3Triggered = false;
    
    // Layer 1: StallGuard2
    if (layer1Enabled) {
        layer1Triggered = checkStallGuard();
    }
    
    // Layer 2: Current spike
    if (layer2Enabled) {
        layer2Triggered = checkCurrentSpike(currentA, currentB, 
                                             static_cast<int16_t>(currentSpikeThresholdA), 
                                             static_cast<int16_t>(currentSpikeThresholdB));
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
    
    return layer1Triggered || layer2Triggered || layer3Triggered;
}
