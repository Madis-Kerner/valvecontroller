/**
 * Control the valve.
 *
 * Solution based on SiLabs Series2 ADC code examples.
 *
 * @author Madis Kerner
 * @license MIT
*/
#include "valvecontroller.h"

#include "cmsis_os2_ext.h"
#include "platform.h"

#include "em_device.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_cmu.h"

#include <stdio.h>
#include <inttypes.h>
#include <limits.h>

#include "loglevels.h"
#define __MODUUL__ "valve"
#define __LOG_LEVEL__ (LOG_LEVEL_valve_controller & BASE_LOG_LEVEL)
#include "log.h"

// valve pin definitions.
#define VALVE_OPEN_PIN                  1
#define VALVE_CLOSE_PIN                 2

// Set HFRCOEM23 to lowest frequency (1MHz)
#define HFRCOEM23_FREQ                  cmuHFRCOEM23Freq_1M0Hz

// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_ADC_FREQ                1000000     // CLK_SRC_ADC
#define CLK_ADC_FREQ                    10000       // CLK_ADC

// defines for current monitoring
#define VALVE_DRIVE_MAX_TIME            8000        // 8 sec
#define VALVE_DRIVE_MONITOR_DELAY       100
#define VALVE_DRIVE_IDLE_CURRENT        5           // mA

// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS                  CDBUSALLOC
#define IADC_INPUT_BUSALLOC_CLOSE       GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_BUSALLOC_OPEN        GPIO_CDBUSALLOC_CDODD0_ADC0

// pins to measure the current from
#define IADC_INPUT_PIN_OPEN             iadcPosInputPortDPin3
#define IADC_INPUT_PIN_CLOSE            iadcPosInputPortCPin4

// static functions
static int valvecontroller_adc_init(uint32_t measurementPin);
static int16_t valvecontroller_adc_read(void);
static int16_t valvecontroller_current_monitor(void);

void valvecontroller_init()
{
    PLATFORM_ClearGpioPin(VALVE_OPEN_PIN);
    PLATFORM_ClearGpioPin(VALVE_CLOSE_PIN);
}

static int valvecontroller_adc_init(uint32_t measurementPin)
{
    int err = 0;
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

    // Reset IADC to reset configuration in case it has been modified
    IADC_reset(IADC0);

    // Set clock frequency to defined value
    CMU_HFRCOEM23BandSet(HFRCOEM23_FREQ);

    // Configure IADC clock source for use while in EM2
    // Note that HFRCOEM23 is the lowest frequency source available for the IADC
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);

    CMU_ClockEnable(cmuClock_IADCCLK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Modify init structs and initialize
    init.warmup = iadcWarmupKeepWarm;

    // Set the HFSCLK prescale value here
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    // Configuration 0 is used by both scan and single conversions by default
    // Use unbuffered AVDD as reference
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2; //iadcCfgReferenceInt1V2;

    // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

    // Single initialization
    initSingle.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID1;

    initSingle.showId = true;

    // Set conversions to run continuously
    initSingle.triggerAction = iadcTriggerActionOnce;
    // initSingle.triggerAction = iadcTriggerActionContinuous;

    // Sample AVDD / 4
    initSingleInput.posInput = measurementPin; //iadcPosInputPortDPin3 iadcPosInputPortCPin4
    //When selecting SUPPLY for PORTPOS and GND for PORTNEG, PINNEG should be
    //configured for an odd number (1, 3, 5...) to avoid a polarity error.
    // iadcNegInputGnd in older SDKs however is even, it is odd in > 2.7.
    initSingleInput.negInput = iadcNegInputGnd | 1; // for compatibility, always set lowest bit to 1

    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize Scan
    IADC_initSingle(IADC0, &initSingle, &initSingleInput);

    // Allocate the analog bus for ADC0 inputs
    GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC_OPEN;
    GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC_CLOSE;

    return err;
}

static int16_t valvecontroller_adc_read(void)
{
    uint32_t avg = 0;
    uint32_t num = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        IADC_command(IADC0, iadcCmdStartSingle);

        uint32_t status = IADC_getStatus(IADC0);
        // debug1("start st %X", (unsigned int)status);

        uint32_t count = 0;
        while (!(status & IADC_STATUS_SINGLEFIFODV) && (count++ <= 1000000))
        {
            count++;
            status = IADC_getStatus(IADC0);
            //debug1("converting %lX", status);
        }

        uint32_t ints = IADC_getInt(IADC0);
        // debug("end st:%"PRIX32" c:%"PRIu32" IF:%"PRIX32, status, count, ints);

        if ((count > 1000000) || (IADC_IF_POLARITYERR & ints))
        {
            err1("c:%" PRIu32 " IF:%" PRIX32, count, ints);
            return 0;
        }
        else
        {
            IADC_Result_t sample = IADC_pullSingleFifoResult(IADC0);
            // debug1("smpl %lu %u", sample.data, sample.id);
            avg += sample.data;
            num++;
        }
    }
    // Calculate input voltage:
    //  AVDD/4 vs 12-bit 1.21 and then to millivolts
    // /10: millivolts / gain (50) / R (0.1)
    int16_t reading = (int16_t)(1000 * 1.21 * (avg / num) / 0xFFF / 5.0);
    debug("reading: %lu mA", reading);
    return reading;
}

static int16_t valvecontroller_current_monitor(void)
{
    int16_t current = INT16_MAX;
    int wait;
    int err = 0;

    for (wait = 0; \
        wait < VALVE_DRIVE_MAX_TIME/VALVE_DRIVE_MONITOR_DELAY && current > VALVE_DRIVE_IDLE_CURRENT; \
        ++wait)
    {
        osDelay(VALVE_DRIVE_MONITOR_DELAY);
        current = valvecontroller_adc_read();
    }

    if (wait >= VALVE_DRIVE_MAX_TIME/VALVE_DRIVE_MONITOR_DELAY)
    {
        // current did not drop below the limit
        logerr("valve is stuck");
        err = -1;
    }

    return err;
}

int valvecontroller_open(void)
{
    int err = 0;

    valvecontroller_adc_init(IADC_INPUT_PIN_OPEN);

    PLATFORM_ClearGpioPin(VALVE_CLOSE_PIN);
    PLATFORM_SetGpioPin(VALVE_OPEN_PIN);

    err = valvecontroller_current_monitor();

    PLATFORM_ClearGpioPin(VALVE_OPEN_PIN);

    return err;
}

int valvecontroller_close(void)
{
    int err = 0;

    valvecontroller_adc_init(IADC_INPUT_PIN_CLOSE);

    PLATFORM_ClearGpioPin(VALVE_OPEN_PIN);
    PLATFORM_SetGpioPin(VALVE_CLOSE_PIN);

    err = valvecontroller_current_monitor();

    PLATFORM_ClearGpioPin(VALVE_CLOSE_PIN);

    return err;
}

void valvecontroller_deinit()
{
    IADC_reset(IADC0);
    CMU_ClockEnable(cmuClock_IADCCLK, false);
}
