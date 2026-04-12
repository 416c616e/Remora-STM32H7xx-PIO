# TMC Sensorless Homing and Crash Detection - Multi-Driver Extension Plan

## Overview

This document outlines the plan to extend the TMC sensorless homing and crash detection capabilities from TMC5160 to other TMC drivers in the Remora system.

## Driver Capability Analysis

### TMC2209 - Full StallGuard2 Support
**Capabilities:**
- StallGuard2 (SG_RESULT register at 0x41)
- SGTHRS register (0x40) for sensitivity threshold
- Current monitoring via MSCURACT (0x6B)
- PWM_SCALE register for current scaling

**Implementation:**
- Add StallGuard2 reading methods to TMC2209Stepper driver
- Extend TMC2209 module with multi-layer protection
- Support sensorless homing with StallGuard2
- Support crash detection with all 3 layers

### TMC2160 - Full StallGuard2 Support
**Capabilities:**
- StallGuard2 (SG_RESULT register)
- SGTHRS register for sensitivity threshold
- Current monitoring via MSCURACT
- PWM_SCALE register for current scaling
- SPI interface (hardware or software)

**Implementation:**
- Add StallGuard2 reading methods to TMC2160Stepper driver
- Extend TMC2160 module with multi-layer protection
- Support sensorless homing with StallGuard2
- Support crash detection with all 3 layers

### TMC2208 - Limited Support (No StallGuard)
**Capabilities:**
- NO StallGuard2 (not available on TMC2208)
- Current monitoring via MSCURACT (0x6B)
- PWM_SCALE register for current scaling
- DRV_STATUS register for driver status

**Implementation:**
- Add current monitoring methods to TMC2208Stepper driver
- Extend TMC2208 module with Layer 2 (current spike) and Layer 3 (position deviation) only
- Support crash detection with 2 layers (no sensorless homing)
- Limit switch homing only (no sensorless option)

## Architecture Changes

### 1. Base TMC Class Extensions

The base `TMC` class in [`Src/remora/modules/tmc/tmc.h`](Src/remora/modules/tmc/tmc.h:1) already has virtual methods for multi-layer protection. These need to be:
- Properly implemented in each driver class
- Marked as not supported where applicable (TMC2208 Layer 1)

### 2. Driver-Specific Implementations

#### TMC2209 Module
```cpp
// Add to tmc2209.cpp
void TMC2209::setStallThreshold(uint8_t threshold) {
    driver->SGTHRS(threshold);
    stallThreshold = threshold;
}

int16_t TMC2209::getStallGuardResult() {
    return driver->SG_RESULT();
}

int16_t TMC2209::getCurrentA() {
    // Read from MSCURACT register
    return driver->mcurA();
}

int16_t TMC2209::getCurrentB() {
    // Read from MSCURACT register
    return driver->mcurB();
}
```

#### TMC2160 Module
```cpp
// Add to tmc2160.cpp
void TMC2160::setStallThreshold(uint8_t threshold) {
    driver->SGTHRS(threshold);
    stallThreshold = threshold;
}

int16_t TMC2160::getStallGuardResult() {
    return driver->SG_RESULT();
}

int16_t TMC2160::getCurrentA() {
    return driver->mcurA();
}

int16_t TMC2160::getCurrentB() {
    return driver->mcurB();
}
```

#### TMC2208 Module
```cpp
// Add to tmc2208.cpp
// Layer 1 (StallGuard2) - NOT SUPPORTED
bool TMC2208::checkStallGuard() {
    return false; // Not available
}

int16_t TMC2208::getStallGuardResult() {
    return 0; // Not available
}

// Layer 2 (Current monitoring) - SUPPORTED
int16_t TMC2208::getCurrentA() {
    return driver->mcurA();
}

int16_t TMC2208::getCurrentB() {
    return driver->mcurB();
}
```

### 3. Homing Module Updates

The Homing module needs to:
- Accept any TMC driver type (not just TMC5160)
- Check driver capabilities before enabling sensorless homing
- Fall back to limit switch mode for TMC2208

### 4. Crash Detection Module Updates

The Crash Detection module needs to:
- Accept any TMC driver type
- Enable only supported layers for each driver:
  - TMC5160: All 3 layers
  - TMC2209: All 3 layers
  - TMC2160: All 3 layers
  - TMC2208: Layers 2 and 3 only

## Implementation Steps

### Step 1: Extend TMCStepper Driver Library

Add missing register access methods to the TMCStepper driver classes:

1. **TMC2209Stepper** - Add SG_RESULT, MSCURACT access
2. **TMC2160Stepper** - Add SG_RESULT, MSCURACT access (may already exist in TMC2130 base)
3. **TMC2208Stepper** - Add MSCURACT access

### Step 2: Update TMC Module Classes

Extend each TMC module implementation:

1. **TMC2209** - Implement all multi-layer protection methods
2. **TMC2160** - Implement all multi-layer protection methods
3. **TMC2208** - Implement current monitoring and position tracking only

### Step 3: Update Homing Module

Modify Homing module to:
- Accept generic TMC interface
- Query driver capabilities
- Select appropriate homing mode

### Step 4: Update Crash Detection Module

Modify Crash Detection module to:
- Accept generic TMC interface
- Enable only supported layers
- Handle driver-specific limitations

### Step 5: Update JSON Configuration

Add driver-specific configuration options:
```json
{
  "TMC2209": {
    "Comment": "X Axis Driver",
    "RX pin": "PA2",
    "RSense": 0.075,
    "Address": 0,
    "Current": 1500,
    "Microsteps": 16,
    "Stall sensitivity": 10,
    "Stealth chop": "on",
    "Sensorless homing": { ... },
    "Crash detection": { ... }
  }
}
```

### Step 6: Testing

Test each driver type:
1. TMC2209 - Full sensorless homing and crash detection
2. TMC2160 - Full sensorless homing and crash detection
3. TMC2208 - Limit switch homing and 2-layer crash detection

## Register Map Reference

### TMC2209 Registers
| Register | Address | Description |
|----------|---------|-------------|
| SGTHRS | 0x40 | StallGuard threshold |
| SG_RESULT | 0x41 | StallGuard result (10-bit) |
| COOLCONF | 0x42 | CoolStep configuration |
| MSCURACT | 0x6B | Motor phase currents |

### TMC2160 Registers
| Register | Address | Description |
|----------|---------|-------------|
| SGTHRS | 0x40 | StallGuard threshold |
| SG_RESULT | 0x41 | StallGuard result (10-bit) |
| COOLCONF | 0x42 | CoolStep configuration |
| MSCURACT | 0x6B | Motor phase currents |

### TMC2208 Registers
| Register | Address | Description |
|----------|---------|-------------|
| MSCURACT | 0x6B | Motor phase currents |
| DRV_STATUS | 0x6F | Driver status |
| PWM_SCALE | 0x71 | PWM scale values |

## Risk Assessment

### High Risk
- TMC2208 sensorless homing attempts (must be disabled)
- Incorrect StallGuard threshold calibration per driver

### Medium Risk
- Current monitoring accuracy varies by driver
- Position tracking requires stepgen integration

### Low Risk
- Configuration validation
- Runtime capability detection

## Success Criteria

1. TMC2209 sensorless homing works correctly
2. TMC2160 sensorless homing works correctly
3. TMC2208 limit switch homing works correctly
4. Crash detection works on all drivers with appropriate layers
5. No crashes or undefined behavior on unsupported features
6. Configuration validation prevents invalid setups
