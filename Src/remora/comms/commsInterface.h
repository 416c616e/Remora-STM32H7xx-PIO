#ifndef COMMSINTERFACE_H
#define COMMSINTERFACE_H

#include <functional>
#include <cstdint>

#include "../modules/module.h"
#include "../modules/homing/homing.h"
#include "../modules/crashDetection/crashDetection.h"

/**
 * @brief Command codes for homing and crash detection
 */
enum class HomingCommand : uint8_t {
    CMD_HOMING_START = 0x50,
    CMD_HOMING_CANCEL = 0x51,
    CMD_HOMING_STATUS = 0x52,
    CMD_HOMING_SET_CONFIG = 0x53,
    CMD_HOMING_GET_CONFIG = 0x54,
};

enum class CrashCommand : uint8_t {
    CMD_CRASH_ENABLE = 0x60,
    CMD_CRASH_DISABLE = 0x61,
    CMD_CRASH_RESET = 0x62,
    CMD_CRASH_STATUS = 0x63,
    CMD_CRASH_GET_EVENT = 0x64,
    CMD_CRASH_SET_CONFIG = 0x65,
    CMD_CRASH_GET_CONFIG = 0x66,
};

enum class TMCCommand : uint8_t {
    CMD_TMC_SET_STALL_THRESHOLD = 0x70,
    CMD_TMC_GET_STALL_THRESHOLD = 0x71,
    CMD_TMC_GET_SG_RESULT = 0x72,
    CMD_TMC_GET_CURRENT = 0x73,
    CMD_TMC_ENABLE_LAYER = 0x74,
    CMD_TMC_DISABLE_LAYER = 0x75,
};

/**
 * @brief Homing request structure
 */
struct HomingRequest {
    uint8_t command;
    uint8_t axis;
    uint8_t mode;
    int32_t homingSpeed;
    int32_t fineSpeed;
    uint8_t stallThreshold;
    int32_t retreatDistance;
    int32_t retreatSpeed;
    uint32_t maxTravel;
    uint32_t debounceTime;
    bool invertDirection;
    bool limitSwitchInvert;
};

/**
 * @brief Homing response structure
 */
struct HomingResponse {
    uint8_t command;
    bool success;
    int32_t homePosition;
    uint8_t state;
    uint32_t errorCode;
    uint32_t elapsedTime;
    uint8_t axis;
    uint8_t mode;
    int32_t homingSpeed;
    int32_t fineSpeed;
    uint8_t stallThreshold;
    int32_t retreatDistance;
    int32_t retreatSpeed;
    uint32_t maxTravel;
    uint32_t debounceTime;
    bool invertDirection;
    bool limitSwitchInvert;
    bool isHomingActive;
    bool isHomingComplete;
};

/**
 * @brief Crash detection request structure
 */
struct CrashRequest {
    uint8_t command;
    bool enabled;
    bool requireMultipleLayers;
    bool enableImmediateStop;
    bool enableLogging;
    bool enableNotifications;
    bool layer1Enabled;
    bool layer2Enabled;
    bool layer3Enabled;
    uint32_t layer1DebounceTime;
    uint32_t layer1ConfirmationCount;
    uint32_t layer2DebounceTime;
    uint32_t layer2ConfirmationCount;
    uint32_t layer3DebounceTime;
    uint32_t layer3ConfirmationCount;
};

/**
 * @brief Crash detection response structure
 */
struct CrashResponse {
    uint8_t command;
    bool enabled;
    uint8_t state;
    bool isCrashDetected;
    bool isCrashConfirmed;
    bool isCrashTriggered;
    uint32_t eventCount;
    uint32_t layersTriggered;
    int16_t stallGuardResult;
    int16_t currentA;
    int16_t currentB;
    int32_t positionDeviation;
    bool success;
    uint32_t errorCode;
    bool layer1Enabled;
    bool layer2Enabled;
    bool layer3Enabled;
    bool requireMultipleLayers;
    bool enableImmediateStop;
    bool enableLogging;
    bool enableNotifications;
};

/**
 * @brief TMC request structure
 */
struct TMCRequest {
    uint8_t command;
    uint8_t axis;
    uint8_t stallThreshold;
    bool layer1Enabled;
    bool layer2Enabled;
    bool layer3Enabled;
};

/**
 * @brief TMC response structure
 */
struct TMCResponse {
    uint8_t command;
    uint8_t stallThreshold;
    int16_t stallGuardResult;
    int16_t currentA;
    int16_t currentB;
    bool layer1Enabled;
    bool layer2Enabled;
    bool layer3Enabled;
    bool success;
    uint32_t errorCode;
};

class CommsInterface : public Module {
private:
    std::function<void(bool)> dataCallback;
    
    // Homing module reference
    std::shared_ptr<Homing> homingModule;
    
    // Crash detection module reference
    std::shared_ptr<CrashDetection> crashDetectionModule;
    
    // TMC modules (indexed by axis)
    std::vector<std::shared_ptr<TMC>> tmcModules;

protected:
    void handleHomingCommand(const HomingRequest& request, HomingResponse& response);
    void handleCrashCommand(const CrashRequest& request, CrashResponse& response);
    void handleTMCCommand(const TMCRequest& request, TMCResponse& response);

public:
	CommsInterface();

	virtual void init(void);
	virtual void start(void);
	virtual void tasks(void);

    void setDataCallback(const std::function<void(bool)>& callback) {
        dataCallback = callback;
    }
    
    void setHomingModule(std::shared_ptr<Homing> module) {
        homingModule = module;
    }
    
    void setCrashDetectionModule(std::shared_ptr<CrashDetection> module) {
        crashDetectionModule = module;
    }
    
    void addTMCModule(std::shared_ptr<TMC> module) {
        tmcModules.push_back(module);
    }
};

#endif
