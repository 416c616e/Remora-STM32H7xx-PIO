#ifndef TMCMODULE_H
#define TMCMODULE_H

#include <cstdint>
#include <string>

#include "../../remora.h"
#include "../../modules/module.h"
#include "../../drivers/TMCStepper/TMCStepper.h"

class TMC : public Module, public std::enable_shared_from_this<TMC>
{
protected:

	Remora* 	instance;
	float       Rsense;

	// Multi-layer protection member variables
	uint8_t stallThreshold;           // 0-255, lower = more sensitive
	bool layer1Enabled;               // StallGuard2
	bool layer2Enabled;               // Current monitoring
	bool layer3Enabled;               // Position tracking
	int32_t lastKnownPosition;        // For layer 3
	int32_t positionTolerance;        // Allowed deviation
	float currentSpikeThresholdA;     // Layer 2 threshold A
	float currentSpikeThresholdB;     // Layer 2 threshold B
	bool requireMultipleLayers;       // Multi-layer validation

public:

	TMC(Remora* _instance, float _Rsense)
		: instance(_instance), Rsense(_Rsense),
		  stallThreshold(64),
		  layer1Enabled(false),
		  layer2Enabled(false),
		  layer3Enabled(false),
		  lastKnownPosition(0),
		  positionTolerance(100),
		  currentSpikeThresholdA(1000.0f),
		  currentSpikeThresholdB(1000.0f),
		  requireMultipleLayers(false) {}

	virtual void update(void) = 0;           // Module default interface
	virtual void configure(void) = 0;

    std::shared_ptr<TMC> getShared() {
        return shared_from_this();
    }
    
    // Multi-layer protection interface (default implementations)
    virtual void setStallThreshold(uint8_t threshold);
    virtual uint8_t getStallThreshold();
    virtual int16_t getStallGuardResult();
    virtual void enableStallGuard(bool enable) {}
    virtual bool isStallGuardEnabled() { return true; }
    virtual void setNominalCurrent(float currentA, float currentB) {}
    virtual void setCurrentSpikeThreshold(float threshold) {}
    virtual bool checkCurrentSpike() { return false; }
    virtual int16_t getCurrentA();
    virtual int16_t getCurrentB();
    virtual void setLastKnownPosition(int32_t position);
    virtual int32_t getLastKnownPosition();
    virtual void setPositionTolerance(int32_t tolerance);
    virtual bool checkPositionDeviation(int32_t currentPosition, int32_t tolerance);
    virtual void enableLayer1(bool enable);
    virtual void enableLayer2(bool enable);
    virtual void enableLayer3(bool enable);
    virtual bool isLayer1Enabled();
    virtual bool isLayer2Enabled();
    virtual bool isLayer3Enabled();
    virtual bool validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition);
    virtual bool checkStallGuard();
    virtual bool checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB);
    virtual bool checkPositionDeviation(int32_t currentPosition, int32_t tolerance);
};

class TMC2208 : public TMC
{
protected:

	std::string rxtxPin;     // default to half duplex
	uint16_t    mA;
	uint16_t    microsteps;
	bool        stealth;

	std::unique_ptr<TMC2208Stepper> driver;

public:

	TMC2208(std::string, float, uint16_t, uint16_t, bool, Remora*);
	static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
	~TMC2208() = default;

    void update(void) override;
    void configure(void) override;

    // Multi-layer protection (TMC2208 does NOT support StallGuard2)
    void setStallThreshold(uint8_t threshold) override;
    uint8_t getStallThreshold() override;
    int16_t getStallGuardResult() override;
    void enableLayer1(bool enable) override;
    void enableLayer2(bool enable) override;
    void enableLayer3(bool enable) override;
    bool isLayer1Enabled() override;
    bool isLayer2Enabled() override;
    bool isLayer3Enabled() override;
    bool validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) override;
    int16_t getCurrentA() override;
    int16_t getCurrentB() override;
    bool checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB);
    bool checkStallGuard() override;
    void setLastKnownPosition(int32_t position) override;
    int32_t getLastKnownPosition() override;
    void setPositionTolerance(int32_t tolerance) override;
    bool checkPositionDeviation(int32_t currentPosition, int32_t tolerance) override;
};

class TMC2209 : public TMC
{
protected:

	std::string rxtxPin;     // default to half duplex
	uint8_t     addr;
	uint16_t    mA;
	uint16_t    microsteps;
	bool        stealth;
	uint16_t    stall;

	std::unique_ptr<TMC2209Stepper> driver;

public:

	TMC2209(std::string, float, uint8_t, uint16_t, uint16_t, bool, uint16_t, Remora*);
	static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
	~TMC2209() = default;

    void update(void) override;
    void configure(void) override;

    // Multi-layer protection
    void setStallThreshold(uint8_t threshold) override;
    uint8_t getStallThreshold() override;
    int16_t getStallGuardResult() override;
    void enableLayer1(bool enable) override;
    void enableLayer2(bool enable) override;
    void enableLayer3(bool enable) override;
    bool isLayer1Enabled() override;
    bool isLayer2Enabled() override;
    bool isLayer3Enabled() override;
    bool validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) override;
    int16_t getCurrentA() override;
    int16_t getCurrentB() override;
    bool checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB);
    bool checkStallGuard() override;
    void setLastKnownPosition(int32_t position) override;
    int32_t getLastKnownPosition() override;
    void setPositionTolerance(int32_t tolerance) override;
    bool checkPositionDeviation(int32_t currentPosition, int32_t tolerance) override;
};

class TMC2160 : public TMC
{
protected:

	std::string pinCS;
	std::string pinMOSI;
	std::string pinMISO;
	std::string pinSCK;
	uint8_t     addr;
	uint16_t    mA;
	uint16_t    microsteps;
	uint8_t     mode;
	uint16_t    stall;
	float     holdCurrent;

	std::unique_ptr<TMC2160Stepper> driver;

public:

	TMC2160(std::string, std::string, std::string, std::string, float, uint8_t, uint16_t, uint16_t, uint8_t, uint16_t, float, Remora*);
	static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
	~TMC2160() = default;

    void update(void) override;
    void configure(void) override;

    // Multi-layer protection
    void setStallThreshold(uint8_t threshold) override;
    uint8_t getStallThreshold() override;
    int16_t getStallGuardResult() override;
    void enableLayer1(bool enable) override;
    void enableLayer2(bool enable) override;
    void enableLayer3(bool enable) override;
    bool isLayer1Enabled() override;
    bool isLayer2Enabled() override;
    bool isLayer3Enabled() override;
    bool validateCrashCondition(int16_t currentA, int16_t currentB, int32_t currentPosition) override;
    int16_t getCurrentA() override;
    int16_t getCurrentB() override;
    bool checkCurrentSpike(int16_t currentA, int16_t currentB, int16_t thresholdA, int16_t thresholdB);
    bool checkStallGuard() override;
    void setLastKnownPosition(int32_t position) override;
    int32_t getLastKnownPosition() override;
    void setPositionTolerance(int32_t tolerance) override;
    bool checkPositionDeviation(int32_t currentPosition, int32_t tolerance) override;
};

class TMC5160 : public TMC
{
protected:

	std::string pinCS;
	std::string pinMOSI;
	std::string pinMISO;
	std::string pinSCK;
	uint8_t     addr;
	uint16_t    mA;
	uint16_t    microsteps;
	uint8_t     mode;
	uint16_t    stall;
	float     holdCurrent;

	std::unique_ptr<TMC5160Stepper> driver;

	// StallGuard2 configuration
	uint8_t stallThreshold;           // 0-255, lower = more sensitive
	bool sgEnabled;                   // Enable/disable StallGuard2

	// Current monitoring
	float nominalCurrentA;            // Nominal current for layer 2
	float nominalCurrentB;
	float currentSpikeThreshold;      // Multiplier (e.g., 2.0 = 2x nominal)

	// Position tracking
	int32_t lastKnownPosition;        // For layer 3
	int32_t positionTolerance;        // Allowed deviation

	// Runtime configuration flags
	bool sensorlessHomingEnabled;
	bool crashDetectionEnabled;
	bool layer1Enabled;               // StallGuard2
	bool layer2Enabled;               // Current monitoring
	bool layer3Enabled;               // Position tracking

public:

	TMC5160(std::string, std::string, std::string, std::string, float, uint8_t, uint16_t, uint16_t, uint8_t, uint16_t, float, Remora*,
			uint8_t stallThreshold = 64, bool sgEnabled = true, float currentSpikeThreshold = 2.0f,
			bool sensorlessHomingEnabled = false, bool crashDetectionEnabled = false,
			bool layer1Enabled = true, bool layer2Enabled = true, bool layer3Enabled = false);
	static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
	~TMC5160() = default;

    void update(void) override;
    void configure(void) override;

    // StallGuard2 configuration
    void setStallThreshold(uint8_t threshold);
    uint8_t getStallThreshold();
    int16_t getStallGuardResult();
    void enableStallGuard(bool enable);
    bool isStallGuardEnabled();

    // Current monitoring
    void setNominalCurrent(float currentA, float currentB);
    void setCurrentSpikeThreshold(float threshold);
    bool checkCurrentSpike();
    int16_t getCurrentA();
    int16_t getCurrentB();

    // Position tracking
    void setLastKnownPosition(int32_t position);
    int32_t getLastKnownPosition();
    void setPositionTolerance(int32_t tolerance);
    bool checkPositionDeviation();

    // Layer control
    void enableLayer1(bool enable);
    void enableLayer2(bool enable);
    void enableLayer3(bool enable);
    bool isLayer1Enabled();
    bool isLayer2Enabled();
    bool isLayer3Enabled();

    // Multi-layer validation
    bool validateCrashCondition();
};

class TMC_MODE
{
public:
	enum {
		COOLSTEP = 0,
		STEALTHCHOP = 1,
		STALLGUARD = 2
	};
};

#endif
