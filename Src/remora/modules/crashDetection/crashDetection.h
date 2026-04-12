#ifndef CRASHDETECTION_H
#define CRASHDETECTION_H

#include <cstdint>
#include <vector>

#include "../../remora.h"
#include "../../modules/module.h"
#include "../../modules/tmc/tmc.h"
#include <memory>

/**
 * @class CrashDetection
 * @brief Multi-layer crash detection module for TMC5160-based stepper motors.
 * 
 * The CrashDetection class provides comprehensive crash detection using multiple
 * sensor layers:
 * - Layer 1: StallGuard2 monitoring
 * - Layer 2: Current spike detection
 * - Layer 3: Position deviation tracking
 */
class CrashDetection : public Module {
public:
    /**
     * @enum Layer
     * @brief Available detection layers
     */
    enum class Layer { STALLGUARD2 = 0, CURRENT_SPIKE = 1, POSITION_DEVIATION = 2 };
    
    /**
     * @enum CrashState
     * @brief Current state of crash detection
     */
    enum class CrashState { IDLE = 0, DETECTING, CONFIRMED, TRIGGERED, RESET };
    
    /**
     * @struct LayerConfig
     * @brief Configuration for a single detection layer
     */
    struct LayerConfig {
        bool enabled;
        uint32_t debounceTime;      // ms
        uint32_t confirmationCount; // Number of consecutive triggers
    };
    
    /**
     * @struct CrashConfig
     * @brief Overall crash detection configuration
     */
    struct CrashConfig {
        LayerConfig layer1;         // StallGuard2
        LayerConfig layer2;         // Current spike
        LayerConfig layer3;         // Position deviation
        bool requireMultipleLayers; // Require 2+ layers to trigger
        bool enableImmediateStop;   // Disable drivers immediately
        bool enableLogging;         // Log all events
        bool enableNotifications;   // Notify host system
    };
    
    /**
     * @struct CrashEvent
     * @brief Details of a detected crash event
     */
    struct CrashEvent {
        uint32_t timestamp;
        uint32_t layersTriggered;   // Bitmask of layers
        int16_t stallGuardResult;
        int16_t currentA;
        int16_t currentB;
        int32_t positionDeviation;
    };
    
    /**
     * @brief Create a new CrashDetection module instance
     * @param config JSON configuration object
     * @param instance Pointer to Remora instance
     * @return Shared pointer to the CrashDetection module
     */
    static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
    
    // Configuration methods
    void setConfig(const CrashConfig& config);
    CrashConfig getConfig() const;
    void setLayerEnabled(Layer layer, bool enabled);
    bool isLayerEnabled(Layer layer);
    void setLayerDebounceTime(Layer layer, uint32_t ms);
    void setLayerConfirmationCount(Layer layer, uint32_t count);
    void setRequireMultipleLayers(bool require);
    void setEnableImmediateStop(bool enable);
    void setEnableLogging(bool enable);
    void setEnableNotifications(bool enable);
    
    // Execution methods
    void enable(bool enable);
    bool isEnabled();
    void reset();
    
    // Status methods
    CrashState getState();
    bool isCrashDetected();
    bool isCrashConfirmed();
    bool isCrashTriggered();
    
    // Event handling
    CrashEvent getLastEvent();
    uint32_t getEventCount();
    void clearEventCount();
    
    // Layer status
    bool isLayer1Triggered();
    bool isLayer2Triggered();
    bool isLayer3Triggered();
    uint32_t getLayerTriggerCount(Layer layer);
    
    // TMC module access
    void setTMCModule(std::shared_ptr<TMC> tmc);
    std::shared_ptr<TMC> getTMCModule();
    
private:
    Remora* instance;
    CrashConfig config;
    CrashState state;
    CrashEvent lastEvent;
    uint32_t eventCount;
    bool enabled;
    
    // Layer state
    bool layer1Triggered;
    bool layer2Triggered;
    bool layer3Triggered;
    uint32_t layer1Count;
    uint32_t layer2Count;
    uint32_t layer3Count;
    uint32_t layer1DebounceStart;
    uint32_t layer2DebounceStart;
    uint32_t layer3DebounceStart;
    
    std::shared_ptr<TMC> tmcModule;
    
    // Constructor
    CrashDetection(Remora* instance);
    
    // Update methods
    void update();
    void checkLayer1();
    void checkLayer2();
    void checkLayer3();
    void validateCrashCondition();
    void triggerCrash();
    void logEvent();
    void notifyHost();
};

#endif // CRASHDETECTION_H
