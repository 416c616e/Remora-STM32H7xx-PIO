#include "commsInterface.h"
#include <STM32H7_SPIComms.h>

CommsInterface::CommsInterface() : Module() {
    homingModule = nullptr;
    crashDetectionModule = nullptr;
}

void CommsInterface::init(void) {
    printf("CommsInterface initialized\n\r");
}

void CommsInterface::start(void) {
    printf("CommsInterface started\n\r");
}

void CommsInterface::tasks(void) {
    // Process incoming commands
    // This would typically be called from the main loop or a dedicated thread
}

void CommsInterface::handleHomingCommand(const HomingRequest& request, HomingResponse& response) {
    response.command = request.command;
    
    if (!homingModule) {
        response.success = false;
        response.errorCode = 1;  // Module not initialized
        return;
    }
    
    switch (request.command) {
        case static_cast<uint8_t>(HomingCommand::CMD_HOMING_START): {
            // Configure homing module
            Homing::HomingConfig config;
            config.axis = static_cast<Homing::Axis>(request.axis);
            config.mode = static_cast<Homing::HomingMode>(request.mode);
            config.homingSpeed = request.homingSpeed;
            config.fineSpeed = request.fineSpeed;
            config.stallThreshold = request.stallThreshold;
            config.retreatDistance = request.retreatDistance;
            config.retreatSpeed = request.retreatSpeed;
            config.maxTravel = request.maxTravel;
            config.debounceTime = request.debounceTime;
            config.invertDirection = request.invertDirection;
            config.limitSwitchInvert = request.limitSwitchInvert;
            
            homingModule->setConfig(config);
            homingModule->startHoming();
            
            // Wait for completion (simplified - in reality this would be async)
            while (homingModule->isHomingActive()) {
                homingModule->update();
            }
            
            Homing::HomingResult result = homingModule->getResult();
            response.success = result.success;
            response.homePosition = result.homePosition;
            response.state = static_cast<uint8_t>(result.state);
            response.errorCode = result.errorCode;
            response.elapsedTime = result.elapsedTime;
            break;
        }
        
        case static_cast<uint8_t>(HomingCommand::CMD_HOMING_CANCEL): {
            if (homingModule->isHomingActive()) {
                homingModule->cancelHoming();
            }
            response.success = true;
            response.errorCode = 0;
            break;
        }
        
        case static_cast<uint8_t>(HomingCommand::CMD_HOMING_STATUS): {
            response.success = true;
            response.state = static_cast<uint8_t>(homingModule->getState());
            response.isHomingActive = homingModule->isHomingActive();
            response.isHomingComplete = homingModule->isHomingComplete();
            response.homePosition = homingModule->getHomePosition();
            response.errorCode = homingModule->getLastError();
            break;
        }
        
        case static_cast<uint8_t>(HomingCommand::CMD_HOMING_SET_CONFIG): {
            Homing::HomingConfig config;
            config.axis = static_cast<Homing::Axis>(request.axis);
            config.mode = static_cast<Homing::HomingMode>(request.mode);
            config.homingSpeed = request.homingSpeed;
            config.fineSpeed = request.fineSpeed;
            config.stallThreshold = request.stallThreshold;
            config.retreatDistance = request.retreatDistance;
            config.retreatSpeed = request.retreatSpeed;
            config.maxTravel = request.maxTravel;
            config.debounceTime = request.debounceTime;
            config.invertDirection = request.invertDirection;
            config.limitSwitchInvert = request.limitSwitchInvert;
            
            homingModule->setConfig(config);
            response.success = true;
            response.errorCode = 0;
            break;
        }
        
        case static_cast<uint8_t>(HomingCommand::CMD_HOMING_GET_CONFIG): {
            Homing::HomingConfig config = homingModule->getConfig();
            response.axis = static_cast<uint8_t>(config.axis);
            response.mode = static_cast<uint8_t>(config.mode);
            response.homingSpeed = config.homingSpeed;
            response.fineSpeed = config.fineSpeed;
            response.stallThreshold = config.stallThreshold;
            response.retreatDistance = config.retreatDistance;
            response.retreatSpeed = config.retreatSpeed;
            response.maxTravel = config.maxTravel;
            response.debounceTime = config.debounceTime;
            response.invertDirection = config.invertDirection;
            response.limitSwitchInvert = config.limitSwitchInvert;
            response.success = true;
            response.errorCode = 0;
            break;
        }
        
        default:
            response.success = false;
            response.errorCode = 2;  // Unknown command
            break;
    }
}

void CommsInterface::handleCrashCommand(const CrashRequest& request, CrashResponse& response) {
    response.command = request.command;
    
    if (!crashDetectionModule) {
        response.enabled = false;
        response.isCrashDetected = false;
        return;
    }
    
    switch (request.command) {
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_ENABLE): {
            crashDetectionModule->enable(true);
            response.enabled = true;
            response.isCrashDetected = false;
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_DISABLE): {
            crashDetectionModule->enable(false);
            response.enabled = false;
            response.isCrashDetected = false;
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_RESET): {
            crashDetectionModule->reset();
            response.enabled = crashDetectionModule->isEnabled();
            response.isCrashDetected = false;
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_STATUS): {
            response.enabled = crashDetectionModule->isEnabled();
            response.isCrashDetected = crashDetectionModule->isCrashDetected();
            response.isCrashConfirmed = crashDetectionModule->isCrashConfirmed();
            response.isCrashTriggered = crashDetectionModule->isCrashTriggered();
            response.eventCount = crashDetectionModule->getEventCount();
            response.state = static_cast<uint8_t>(crashDetectionModule->getState());
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_GET_EVENT): {
            CrashDetection::CrashEvent event = crashDetectionModule->getLastEvent();
            response.layersTriggered = event.layersTriggered;
            response.stallGuardResult = event.stallGuardResult;
            response.currentA = event.currentA;
            response.currentB = event.currentB;
            response.positionDeviation = event.positionDeviation;
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_SET_CONFIG): {
            CrashDetection::CrashConfig config;
            config.layer1.enabled = request.layer1Enabled;
            config.layer1.debounceTime = request.layer1DebounceTime;
            config.layer1.confirmationCount = request.layer1ConfirmationCount;
            config.layer2.enabled = request.layer2Enabled;
            config.layer2.debounceTime = request.layer2DebounceTime;
            config.layer2.confirmationCount = request.layer2ConfirmationCount;
            config.layer3.enabled = request.layer3Enabled;
            config.layer3.debounceTime = request.layer3DebounceTime;
            config.layer3.confirmationCount = request.layer3ConfirmationCount;
            config.requireMultipleLayers = request.requireMultipleLayers;
            config.enableImmediateStop = request.enableImmediateStop;
            config.enableLogging = request.enableLogging;
            config.enableNotifications = request.enableNotifications;
            
            crashDetectionModule->setConfig(config);
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(CrashCommand::CMD_CRASH_GET_CONFIG): {
            CrashDetection::CrashConfig config = crashDetectionModule->getConfig();
            response.layer1Enabled = config.layer1.enabled;
            response.layer2Enabled = config.layer2.enabled;
            response.layer3Enabled = config.layer3.enabled;
            response.requireMultipleLayers = config.requireMultipleLayers;
            response.enableImmediateStop = config.enableImmediateStop;
            response.enableLogging = config.enableLogging;
            response.enableNotifications = config.enableNotifications;
            response.success = true;
            break;
        }
        
        default:
            response.success = false;
            response.errorCode = 2;  // Unknown command
            break;
    }
}

void CommsInterface::handleTMCCommand(const TMCRequest& request, TMCResponse& response) {
    response.command = request.command;
    
    if (tmcModules.empty()) {
        response.success = false;
        response.errorCode = 1;  // No TMC modules
        return;
    }
    
    // Get the appropriate TMC module based on axis
    std::shared_ptr<TMC> tmc = nullptr;
    if (request.axis < tmcModules.size()) {
        tmc = tmcModules[request.axis];
    }
    
    if (!tmc) {
        response.success = false;
        response.errorCode = 2;  // Invalid axis
        return;
    }
    
    switch (request.command) {
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_SET_STALL_THRESHOLD): {
            tmc->setStallThreshold(request.stallThreshold);
            response.stallThreshold = tmc->getStallThreshold();
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_GET_STALL_THRESHOLD): {
            response.stallThreshold = tmc->getStallThreshold();
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_GET_SG_RESULT): {
            response.stallGuardResult = tmc->getStallGuardResult();
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_GET_CURRENT): {
            response.currentA = tmc->getCurrentA();
            response.currentB = tmc->getCurrentB();
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_ENABLE_LAYER): {
            if (request.layer1Enabled) tmc->enableLayer1(true);
            if (request.layer2Enabled) tmc->enableLayer2(true);
            if (request.layer3Enabled) tmc->enableLayer3(true);
            response.layer1Enabled = tmc->isLayer1Enabled();
            response.layer2Enabled = tmc->isLayer2Enabled();
            response.layer3Enabled = tmc->isLayer3Enabled();
            response.success = true;
            break;
        }
        
        case static_cast<uint8_t>(TMCCommand::CMD_TMC_DISABLE_LAYER): {
            if (request.layer1Enabled) tmc->enableLayer1(false);
            if (request.layer2Enabled) tmc->enableLayer2(false);
            if (request.layer3Enabled) tmc->enableLayer3(false);
            response.layer1Enabled = tmc->isLayer1Enabled();
            response.layer2Enabled = tmc->isLayer2Enabled();
            response.layer3Enabled = tmc->isLayer3Enabled();
            response.success = true;
            break;
        }
        
        default:
            response.success = false;
            response.errorCode = 3;  // Unknown command
            break;
    }
}
