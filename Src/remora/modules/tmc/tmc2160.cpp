#include "tmc.h"
#include <cstdint>

// CHOPCONF
#define TMC2160_INTPOL              1   // Step interpolation: 0 = off, 1 = on
#define TMC2160_TOFF                5   // Off time: 1 - 15, 0 = MOSFET disable (8)
#define TMC2160_TBL                 1   // Blanking time: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2160_CHM                 0   // Chopper mode: 0 = spreadCycle, 1 = constant off time
// TMC2160_CHM 0 defaults
#define TMC2160_HSTRT               3   // Hysteresis start: 1 - 8
#define TMC2160_HEND                5   // Hysteresis end: -3 - 12
#define TMC2160_HMAX               16   // HSTRT + HEND
// TMC2160_CHM 1 defaults
#define TMC2160_TFD                 13  // fd3 & hstrt: 0 - 15

// IHOLD_IRUN
#define TMC2160_IHOLDDELAY          6

// TPOWERDOWN
#define TMC2160_TPOWERDOWN          128 // 0 - ((2^8)-1) * 2^18 tCLK

// TPWMTHRS
#define TMC2160_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// PWMCONF - StealthChop defaults
#define TMC2160_PWM_FREQ            1   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC2160_PWM_AUTOGRAD        1   // boolean (0 or 1)
#define TMC2160_PWM_GRAD            14  // 0 - 255
#define TMC2160_PWM_LIM             12  // 0 - 15
#define TMC2160_PWM_REG             8   // 1 - 15
#define TMC2160_PWM_OFS             36  // 0 - 255

// TCOOLTHRS
#define TMC2160_COOLSTEP_THRS       0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)

// COOLCONF - CoolStep defaults
#define TMC2160_SEMIN               5   // 0 = coolStep off, 1 - 15 = coolStep on
#define TMC2160_SEUP                0   // 0 - 3 (1 - 8)
#define TMC2160_SEMAX               2   // 0 - 15
#define TMC2160_SEDN                1   // 0 - 3
#define TMC2160_SEIMIN              0   // boolean (0 or 1)

std::shared_ptr<Module> TMC2160::create(const JsonObject& config, Remora* instance) {
    printf("Creating TMC2160 module\n\r");

    const char* comment = config["Comment"];
    printf("Comment: %s\n\r", comment);

    std::string pinCS = config["CS pin"];
    std::string pinMOSI = config["MOSI pin"];
    std::string pinMISO = config["MISO pin"];
    std::string pinSCK = config["SCK pin"];
    uint8_t address = config["Address"];

    float RSense = config["RSense"];
    uint16_t current = config["Current"];
    float holdCurrent = config["Hold current"];
    uint16_t microsteps = config["Microsteps"];
    uint8_t mode = config["Driver mode"];
    uint16_t stall = config["Stall sensitivity"];

    return std::make_shared<TMC2160>(std::move(pinCS), std::move(pinMOSI), std::move(pinMISO), std::move(pinSCK), RSense, address, current, microsteps, mode, stall, holdCurrent, instance);
}

TMC2160::TMC2160(std::string _pinCS, std::string _pinMOSI, std::string _pinMISO, std::string _pinSCK, float _Rsense, uint8_t _addr, uint16_t _mA, uint16_t _microsteps, uint8_t _mode, uint16_t _stall, float _holdCurrent, Remora* _instance)
    : TMC{_instance, _Rsense},  // Call base class constructor
      pinCS(std::move(_pinCS)),
	  pinMOSI(std::move(_pinMOSI)),
	  pinMISO(std::move(_pinMISO)),
	  pinSCK(std::move(_pinSCK)),
      addr(_addr),
      mA(_mA),
      microsteps(_microsteps),
      mode(_mode),
      stall(_stall),
      holdCurrent(_holdCurrent),
      driver(std::make_unique<TMC2160Stepper>(pinCS, _Rsense, pinMOSI, pinMISO, pinSCK)) {}


void TMC2160::configure()
{
    driver->begin();

    printf("Testing connection to TMC driver... ");
    uint16_t result = driver->test_connection();
    
    if (result) {
        printf("Failed!\nLikely cause: ");
        switch(result) {
            case 1: printf("Loose connection\n\r"); break;
            case 2: printf("No power\n\r"); break;
            default: printf("Unknown issue\n\r"); break;
        }
        printf("Fix the problem and reset the board.\n\r");
    } else {
        printf("OK - Driver Version: %i\n\r", driver->version());
    }

    driver->reset();
    driver->GSTAT();
    driver->reset();
    driver->microsteps(this->microsteps);
    driver->rms_current(mA, holdCurrent);

    // GCONF
    switch(this->mode)
    {
        case TMC_MODE::STALLGUARD:
            driver->en_pwm_mode(false);
            break;
        case TMC_MODE::STEALTHCHOP:
            driver->en_pwm_mode(true);
            break;
        case TMC_MODE::COOLSTEP:
        default:
            driver->en_pwm_mode(false);
            break;
    }

    // CHOPCONF
    driver->intpol(TMC2160_INTPOL);
    driver->toff(TMC2160_TOFF);
    driver->tbl(TMC2160_TBL);
    driver->chm(TMC2160_CHM);
    driver->hend(TMC2160_HEND + 3);

    // CHM
    #if TMC2160_CHM == 0
        driver->hstrt(TMC2160_HSTRT - 1);
    #else
        driver->fd3((TMC2160_TFD & 0x08) >> 3);
        driver->hstrt(TMC2160_TFD & 0x07);
    #endif

    // COOLCONF
    driver->semin(TMC2160_SEMIN);
    driver->seup(TMC2160_SEUP);
    driver->semax(TMC2160_SEMAX);
    driver->sedn(TMC2160_SEDN);
    driver->seimin(TMC2160_SEMIN);
    driver->TCOOLTHRS(TMC2160_COOLSTEP_THRS);
    
    // PWMCONF
    switch(this->mode)
    {
        case TMC_MODE::STALLGUARD:
            driver->pwm_autoscale(false);
            break;
        case TMC_MODE::STEALTHCHOP:
            driver->pwm_autoscale(true);
            break;
        case TMC_MODE::COOLSTEP:
        default:
            driver->pwm_autoscale(false);
            break;
    }
    driver->pwm_lim(TMC2160_PWM_LIM);
    driver->pwm_reg(TMC2160_PWM_REG);
    driver->pwm_autograd(TMC2160_PWM_AUTOGRAD);
    driver->pwm_freq(TMC2160_PWM_FREQ);
    driver->pwm_grad(TMC2160_PWM_GRAD);
    driver->pwm_ofs(TMC2160_PWM_OFS);

    // OTHERS
    driver->iholddelay(TMC2160_IHOLDDELAY);
    driver->TPOWERDOWN(TMC2160_TPOWERDOWN);
    driver->TPWMTHRS(TMC2160_TPWM_THRS);

    printf( "CHOPCONF reports %d\n\r", driver->CHOPCONF());
    printf( "drv_err reports %d\n\r", driver->drv_err());
    printf( "uv_cp reports %d\n\r", driver->uv_cp());
}

void TMC2160::update(){}
