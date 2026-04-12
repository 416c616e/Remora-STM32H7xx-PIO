#ifndef HOMING_H
#define HOMING_H

#include <cstdint>
#include <string>

#include "../../remora.h"
#include "../../modules/module.h"
#include "../../modules/tmc/tmc.h"
#include <memory>

/**
 * @class Homing
 * @brief Stepper motor homing module supporting sensorless, limit switch, and hybrid modes.
 * 
 * The Homing class provides a comprehensive homing solution for TMC5160-based stepper motors.
 * It supports three homing modes:
 * - SENSORLESS: Uses StallGuard2 to detect mechanical limits
 * - LIMIT_SWITCH: Uses physical limit switches
 * - HYBRID: Combines both for reliable and precise homing
 */
class Homing : public Module {
public:
    /**
     * @enum Axis
     * @brief Available axes for homing
     */
    enum class Axis { X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5, D = 6, E = 7 };
    
    /**
     * @enum HomingMode
     * @brief Available homing modes
     */
    enum class HomingMode { SENSORLESS = 0, LIMIT_SWITCH = 1, HYBRID = 2 };
    
    /**
     * @enum HomingState
     * @brief Current state of the homing sequence
     */
    enum class HomingState { IDLE = 0, HOMING, RETREAT, COMPLETE, ERROR };
    
    /**
     * @struct HomingConfig
     * @brief Configuration structure for homing
     */
    struct HomingConfig {
        Axis axis;
        HomingMode mode;
        int32_t homingSpeed;        // mm/s for approach
        int32_t fineSpeed;          // mm/s for final approach
        uint8_t stallThreshold;     // 0-255 for TMC (lower = more sensitive)
        int32_t retreatDistance;    // mm to retreat after trigger
        int32_t retreatSpeed;       // mm/s for retreat
        bool invertDirection;       // Invert homing direction
        uint32_t maxTravel;         // Maximum travel distance (mm)
        uint32_t debounceTime;      // ms debounce for trigger
        bool limitSwitchInvert;     // Invert limit switch logic (for LIMIT_SWITCH and HYBRID modes)
    };
    
    /**
     * @struct HomingResult
     * @brief Result of a homing sequence
     */
    struct HomingResult {
        bool success;
        int32_t homePosition;
        HomingState state;
        uint32_t errorCode;
        uint32_t elapsedTime;       // ms
    };
    
    /**
     * @brief Create a new Homing module instance
     * @param config JSON configuration object
     * @param instance Pointer to Remora instance
     * @return Shared pointer to the Homing module
     */
    static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
    
    // Configuration methods
    void setConfig(const HomingConfig& config);
    HomingConfig getConfig() const;
    void setAxis(Axis axis);
    Axis getAxis() const;
    void setMode(HomingMode mode);
    HomingMode getMode() const;
    void setHomingSpeed(int32_t speed);
    int32_t getHomingSpeed() const;
    void setFineSpeed(int32_t speed);
    int32_t getFineSpeed() const;
    void setStallThreshold(uint8_t threshold);
    uint8_t getStallThreshold() const;
    void setRetreatDistance(int32_t distance);
    int32_t getRetreatDistance() const;
    void setRetreatSpeed(int32_t speed);
    int32_t getRetreatSpeed() const;
    void setMaxTravel(uint32_t distance);
    uint32_t getMaxTravel() const;
    void setDebounceTime(uint32_t time);
    uint32_t getDebounceTime() const;
    void setInvertDirection(bool invert);
    bool getInvertDirection() const;
    void setLimitSwitchInvert(bool invert);
    bool getLimitSwitchInvert() const;
    
    // Execution methods
    void startHoming();
    void cancelHoming();
    HomingResult getResult();
    HomingState getState();
    bool isHomingComplete();
    bool isHomingActive();
    void reset();
    
    // Status methods
    int32_t getHomePosition();
    uint32_t getLastError();
    const char* getErrorString();
    
    // Layer access (for multi-layer validation)
    void setTMCModule(std::shared_ptr<TMC> tmc);
    std::shared_ptr<TMC> getTMCModule();
    
    // Constructor
    Homing(Remora* instance);
    
private:
    Remora* instance;
    HomingConfig config;
    HomingState state;
    HomingResult result;
    uint32_t startTime;
    int32_t currentCount;
    int32_t targetCount;
    bool active;
    bool triggerDetected;
    uint32_t triggerDebounceStart;
    std::shared_ptr<TMC> tmcModule;
    
    // Homing sequence methods
    void homingSequence();
    void retreatSequence();
    bool checkTriggerCondition();
    void handleTrigger();
    void handleError(uint32_t errorCode);
    void updatePosition();
    
    // Helper methods
    int32_t mmToSteps(float mm);
    float stepsToMm(int32_t steps);
    void setDirection(bool forward);
    void setFrequency(int32_t frequency);
    void stopMotion();
};

#endif // HOMING_H
