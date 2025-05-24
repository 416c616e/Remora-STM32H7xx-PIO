#include "remora.h"
#include "../irqHandlers.h"
#include "interrupt/interrupt.h"
#include "json/jsonConfigHandler.h"

// unions for TX and RX data
volatile txData_t txData;
volatile rxData_t rxData;

Remora::Remora(std::shared_ptr<CommsHandler> commsHandler,
               std::unique_ptr<pruTimer> baseTimer,
               std::unique_ptr<pruTimer> servoTimer,
               std::unique_ptr<pruTimer> serialTimer)
	: currentState(ST_SETUP),
	  prevState(ST_SETUP),
	  ptrTxData(&txData),
	  ptrRxData(&rxData),
	  reset(false),
	  configHandler(nullptr),
	  comms(std::move(commsHandler)),
	  baseThread(nullptr),
	  servoThread(nullptr),
	  serialThread(nullptr),
	  onLoad(),
	  baseFreq(baseTimer->getFrequency()),
	  servoFreq(servoTimer->getFrequency()),
	  serialFreq(serialTimer ? serialTimer->getFrequency() : 0),
	  threadsRunning(false)
{
	configHandler = std::make_unique<JsonConfigHandler>(this);
    comms->init();
    comms->start();

    baseThread = std::make_unique<pruThread>("BaseThread");
    baseTimer->setOwner(baseThread.get());
    baseThread->setTimer(std::move(baseTimer));

    servoThread = std::make_unique<pruThread>("ServoThread");
    servoTimer->setOwner(servoThread.get());
    servoThread->setTimer(std::move(servoTimer));

    if (serialTimer) {
        serialThread = std::make_unique<pruThread>("SerialThread");
        serialThread->setTimer(std::move(serialTimer));
        serialTimer->setOwner(serialThread.get());
    }

    servoThread->registerModule(comms);
}

void Remora::transitionToState(State newState)
{
    if (currentState != newState) {
        printStateEntry(newState);
        prevState = currentState;
        currentState = newState;
    }
}

void Remora::printStateEntry(State state)
{
    const char* stateNames[] = {
        "Setup", "Start", "Idle", "Running", "Stop", "Reset", "System Reset"
    };
    printf("## Transitioning to %s state\n\r", stateNames[state]);
}

void Remora::handleSetupState()
{
    loadModules();
    transitionToState(ST_START);
}

void Remora::handleStartState()
{
    for (const auto& module : onLoad) {
        if (module) {
            module->configure();
        }
    }

    if (!threadsRunning) {
        startThread(servoThread, "Servo");
        startThread(baseThread, "Base");
        threadsRunning = true;
    }

    transitionToState(ST_IDLE);
}

void Remora::handleIdleState()
{
    if (comms->getStatus()) {
        transitionToState(ST_RUNNING);
    }
}

void Remora::handleRunningState()
{
    if (!comms->getStatus()) {
        transitionToState(ST_RESET);
    }

    if (reset) {
        transitionToState(ST_SYSRESET);
    }
}

void Remora::handleResetState()
{
    printf("Resetting rxBuffer\n\r");
    resetBuffer(ptrRxData->rxBuffer, Config::dataBuffSize);
    transitionToState(ST_IDLE);
}

void Remora::handleSysResetState()
{
    HAL_NVIC_SystemReset();
}

void Remora::startThread(const std::unique_ptr<pruThread>& thread, const char* name)
{
    printf("Starting the %s thread\n\r", name);
    thread->startThread();
}

void Remora::resetBuffer(volatile uint8_t* buffer, size_t size)
{
    memset((void*)buffer, 0, size);
}

void Remora::run()
{
    while (true) {
        switch (currentState) {
            case ST_SETUP:
                handleSetupState();
                break;
            case ST_START:
                handleStartState();
                break;
            case ST_IDLE:
                handleIdleState();
                break;
            case ST_RUNNING:
                handleRunningState();
                break;
            case ST_RESET:
                handleResetState();
                break;
            case ST_SYSRESET:
                handleSysResetState();
                break;
            default:
                printf("Error: Invalid state\n\r");
                break;
        }
        comms->tasks();
    }
}

void Remora::loadModules()
{
    ModuleFactory* factory = ModuleFactory::getInstance();
    JsonArray modules = configHandler->getModules();
    if (modules.isNull()) {
      // Log something about missing modules or return early
    }

    for (size_t i = 0; i < modules.size(); i++) {
        if (modules[i]["Thread"].is<const char*>() && modules[i]["Type"].is<const char*>()) {
            const char* threadName = modules[i]["Thread"];
            const char* moduleType = modules[i]["Type"];
            uint32_t threadFreq = 0;

            // Determine the thread frequency based on the thread name
            if (strcmp(threadName, "Servo") == 0) {
                threadFreq = servoFreq;
            } else if (strcmp(threadName, "Base") == 0) {
                threadFreq = baseFreq;
            }

            // Add the "ThreadFreq" key and its value to the module's JSON object
            modules[i]["ThreadFreq"] = threadFreq;

            // Create module using factory
            std::shared_ptr<Module> _mod = factory->createModule(threadName, moduleType, modules[i], this);

            // Check if the module creation was successful
            if (!_mod) {
                printf("Error: Failed to create module of type '%s' for thread '%s'. Skipping registration.\n\r",
                        moduleType, threadName);
                continue; // Skip to the next iteration
            }

            bool _modPost = _mod->getUsesModulePost();

            if (strcmp(threadName, "Servo") == 0) {
                servoThread->registerModule(_mod);
                if (_modPost) {
                    servoThread->registerModulePost(_mod);
                }
            }
            else if (strcmp(threadName, "Base") == 0) {
                baseThread->registerModule(_mod);
                if (_modPost) {
                    baseThread->registerModulePost(_mod);
                }
            }
            else {
                onLoad.push_back(std::move(_mod));
            }
        }
    }
}
