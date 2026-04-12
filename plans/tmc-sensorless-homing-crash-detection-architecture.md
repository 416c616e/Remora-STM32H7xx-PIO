# TMC Sensorless Homing and Crash Detection - Architecture Options

## Overview
This document outlines architectural options for implementing TMC sensorless homing and crash detection in the Remora STM32H7xx PIO system.

---

## Architecture Options

### Option 1: TMC5160 StallGuard2-Based Implementation

**Overview:** Utilize the TMC5160's built-in StallGuard2 sensorless load detection for both homing and crash detection.

**Key Features:**
- **Homing:** Drive axis until StallGuard2 threshold is exceeded (mechanical limit hit)
- **Crash Detection:** Monitor StallGuard2 readings during motion; trigger on sudden load spike
- **Driver:** TMC5160 (SPI interface, already in your codebase)

**Pros:**
- No additional hardware required (no limit switches)
- TMC5160 already integrated in your system
- High sensitivity and accuracy
- Can be tuned per-axis

**Cons:**
- Requires motor to be under load for detection (won't work at no-load)
- Needs careful tuning of `SGTHRS` (stall threshold)
- May have false positives with mechanical backlash

**Implementation Complexity:** Medium

---

### Option 2: Hybrid Approach (Sensorless + Limit Switches)

**Overview:** Combine sensorless detection with physical limit switches for more reliable homing.

**Key Features:**
- **Homing:** Use limit switch for coarse homing, sensorless for fine approach
- **Crash Detection:** Use StallGuard2 for real-time monitoring
- **Driver:** TMC5160 with GPIO-connected limit switches

**Pros:**
- More reliable homing (limit switch provides hard stop)
- Reduces mechanical wear (softer sensorless approach)
- Crash detection still works without limit switches

**Cons:**
- Requires additional wiring for limit switches
- More complex homing sequence

**Implementation Complexity:** Medium-High

---

### Option 3: TMC2209 UART-Based Implementation

**Overview:** Use TMC2209 drivers with UART communication for StallGuard2 functionality.

**Key Features:**
- **Homing:** StallGuard2 via UART interface
- **Crash Detection:** Real-time StallGuard2 monitoring
- **Driver:** TMC2209 (UART interface, requires SoftwareSerial)

**Pros:**
- Simpler wiring (UART vs SPI)
- StallGuard2 built-in (no spreadCycle needed)
- Lower cost than TMC5160

**Cons:**
- Requires driver replacement (TMC5160 → TMC2209)
- UART communication slower than SPI
- Current codebase is SPI-based

**Implementation Complexity:** High (driver replacement needed)

---

### Option 4: Multi-Layer Protection System

**Overview:** Implement a comprehensive system with multiple detection layers.

**Key Features:**
- **Layer 1:** StallGuard2 for immediate crash detection
- **Layer 2:** Current monitoring (MSCURACT register)
- **Layer 3:** Position tracking (XACTUAL) for unexpected movement
- **Layer 4:** Optional limit switches for hard stops

**Pros:**
- Most robust protection
- Multiple fail-safes
- Can be configured per-axis

**Cons:**
- Most complex implementation
- Higher CPU overhead for monitoring
- Requires careful tuning of all layers

**Implementation Complexity:** High

---

## Recommended Architecture (Option 1 + Layer 2)

For the current TMC5160-based system, the recommended approach is **Option 1 with current monitoring**.

### System Architecture Diagram

```
Motion Command
     |
     v
+------------------+
| StallGuard2      |
| Monitor          |
+--------+---------+
         |
    +----+----+
    |         |
    v         v
Normal    Stall Detected
Motion    |
    |     v
    | +------------------+
    | | Current Check    |
    | | (MSCURACT)       |
    | +--------+---------+
    |          |
    |    +-----+-----+
    |    |           |
    |    v           v
    | Normal    Spike Detected
    |    |           |
    |    v           v
    | Continue   Emergency
    | Motion     Stop
    |           +-----+------+
    |                 |
    v                 v
+-------+      +-----+------+
| Done  |      | Disable    |
+-------+      | Drivers    |
               +-----+------+
                     |
               +-----+------+
               | Log Event  |
               | Notify Host|
               +------------+
```

---

## Implementation Components

### 1. TMC Module Extensions

**File:** `Src/remora/modules/tmc/tmc.h`

Add the following methods to the `TMC5160` class:

```cpp
class TMC5160 : public TMC {
    // ... existing members ...
    
    // StallGuard2 configuration
    void setStallThreshold(uint8_t threshold);
    uint8_t getStallThreshold();
    int16_t getStallGuardResult();
    
    // Sensorless homing
    void enableSensorlessHoming(bool enable);
    bool isSensorlessHomingEnabled();
    
    // Crash detection
    void enableCrashDetection(bool enable);
    bool isCrashDetectionEnabled();
    bool checkCrashCondition();
    
    // Current monitoring
    int16_t getCurrentA();
    int16_t getCurrentB();
    bool checkCurrentSpike(float threshold);
};
```

### 2. Homing Module

**File:** `Src/remora/modules/homing/homing.h`

```cpp
class Homing : public Module {
public:
    enum class Axis { X, Y, Z, A, B, C };
    enum class HomingMode { SENSORLESS, LIMIT_SWITCH, HYBRID };
    
    struct HomingConfig {
        Axis axis;
        HomingMode mode;
        int32_t homingSpeed;        // mm/s
        int32_t homingFeedRate;     // mm/min for final approach
        uint8_t stallThreshold;     // 0-255 for TMC
        int32_t retreatDistance;    // mm to retreat after trigger
        bool invertDirection;
    };
    
    static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
    
    void homingSequence(const HomingConfig& config);
    void cancelHoming();
    bool isHomingComplete();
    int32_t getHomePosition();
};
```

### 3. Crash Detection Handler

**File:** `Src/remora/modules/crashDetection/crashDetection.h`

```cpp
class CrashDetection : public Module {
public:
    struct CrashConfig {
        uint8_t stallThreshold;         // StallGuard2 threshold
        float currentSpikeThreshold;    // Multiplier of nominal current
        uint32_t debounceTime;          // ms to confirm crash
        bool enableImmediateStop;       // True = disable drivers immediately
        bool enableLogging;             // Log crash events
    };
    
    static std::shared_ptr<Module> create(const JsonObject& config, Remora* instance);
    
    void enable(bool enable);
    bool isEnabled();
    bool isCrashDetected();
    void resetCrashState();
    void getCrashStats(uint32_t* count, uint32_t* lastEventTime);
};
```

### 4. Configuration Interface

**File:** `Src/remora/json/jsonConfigHandler.h`

Add configuration support for:

```json
{
  "TMC5160": {
    "Comment": "Z Axis Driver",
    "CS pin": "PC4",
    "MOSI pin": "PC12",
    "MISO pin": "PC11",
    "SCK pin": "PC10",
    "Address": 0,
    "RSense": 0.075,
    "Current": 2000,
    "Hold current": 0.5,
    "Microsteps": 16,
    "Driver mode": 2,
    "Stall sensitivity": 10,
    "Sensorless homing": {
      "enabled": true,
      "homingSpeed": 100,
      "retreatDistance": 5
    },
    "Crash detection": {
      "enabled": true,
      "stallThreshold": 64,
      "currentSpikeThreshold": 2.0,
      "debounceTime": 10,
      "enableImmediateStop": true
    }
  }
}
```

---

## TMC5160 Register Details

### StallGuard2 Configuration

| Register | Address | Bits | Description |
|----------|---------|------|-------------|
| `GCONF` | 0x00 | 32 | Enable `sg2cmp_en` for StallGuard2 output |
| `COOLCONF` | 0x14 | 32 | Configure `semin` (threshold), `seup`, `semax`, `sedn`, `seimin` |
| `SGTHRS` (TMC2209) / `COOLCONF.sgt` (TMC5160) | 0x14 | 8 | Stall threshold (0 = most sensitive, 255 = least) |

### Reading StallGuard2 Result

| Register | Address | Bits | Description |
|----------|---------|------|-------------|
| `DRV_STATUS` | 0x6F | 32 | `sg_result` bits [15:0] contain the StallGuard2 value |

### Current Monitoring

| Register | Address | Bits | Description |
|----------|---------|------|-------------|
| `MSCURACT` | 0x6B | 32 | `cur_a` [15:0], `cur_b` [15:0] - actual coil currents |

---

## Homing Sequence Algorithm

```
1. Enable StallGuard2 monitoring
2. Set homing speed (low for safety)
3. Drive axis in homing direction
4. Monitor SG_RESULT continuously
5. If SG_RESULT >= threshold:
   a. Stop motion immediately
   b. Retreat by retreatDistance
   c. Set home position
   d. Disable StallGuard2 for normal operation
6. Return success
```

---

## Crash Detection Algorithm

```
1. During motion, continuously monitor:
   - SG_RESULT
   - MSCURACT (coil currents)
2. If SG_RESULT >= threshold:
   a. Start debounce timer
   b. Check if current spike also detected
   c. If both conditions persist for debounceTime:
      - Trigger emergency stop
      - Disable all drivers
      - Log crash event
      - Notify host system
3. Reset on user acknowledgment
```

---

## Questions for Finalization

1. **Which option aligns best with your needs?** (1, 2, 3, or 4)
2. **Do you have limit switches available**, or should we go fully sensorless?
3. **What's your priority:** homing accuracy or crash detection sensitivity?
4. **Should this be configurable at runtime** or set at startup?

---

## Next Steps

Once the architecture is approved:
1. Create detailed implementation plan
2. Define all new classes and interfaces
3. Plan integration with existing Remora system
4. Create test plan for validation
