/**
 * @file    loadcell.c
 * @brief   HX711 bit-banged driver and force-based gait phase heuristic.
 */

#include "loadcell.h"
#include "system_state.h"

/* Calibration state -- set via LoadCell_Calibrate(), persisted in RAM.
 * A production build would load these from flash/EEPROM at boot instead
 * of defaulting to 1:1, but persistence is outside this module's scope. */
static int32_t s_zeroOffsetRaw       = 0;
static float   s_newtonsPerRawCount  = 1.0f;

/* Gait-phase heuristic thresholds. These are illustrative starting points
 * for bench testing, NOT clinically validated values -- tuning against
 * real subject data is required before any clinical/field use. */
#define STANCE_FORCE_THRESHOLD_N      50.0f   /* above this = weight-bearing */
#define SWING_FORCE_THRESHOLD_N       15.0f   /* below this = leg unloaded   */
#define RAPID_UNLOAD_RATE_N_PER_S   -200.0f   /* fast force drop = swing starting */
#define RAPID_LOAD_RATE_N_PER_S      200.0f   /* fast force rise = heel strike    */

void LoadCell_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = HX711_SCK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &gpio);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);

    gpio.Pin  = HX711_DOUT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP; /* HX711 DOUT idles high between conversions */
    HAL_GPIO_Init(HX711_DOUT_PORT, &gpio);
}

/* Microsecond-ish busy-wait spin. STM32H585 typically clocks well past
 * 200MHz; this loop count is intentionally conservative/approximate for
 * the >=1us HX711 SCK pulse-width requirement. For production timing
 * accuracy, retarget this to a hardware timer microsecond counter
 * (e.g. a free-running TIM with 1MHz tick) instead of a calibrated NOP
 * loop, since the NOP count is core-clock-dependent. */
static void DelayMicroseconds(uint32_t us)
{
    volatile uint32_t cycles = us * 100U; /* placeholder scaling, retune per clock config */
    while (cycles--)
    {
        __NOP();
    }
}

bool LoadCell_ReadRaw(int32_t *out_raw, uint32_t timeout_ms)
{
    if (out_raw == NULL)
    {
        return false;
    }

    /* DOUT goes low when a conversion is ready. Wait for that, bounded by
     * timeout_ms so a disconnected/dead load cell can be detected by the
     * caller rather than hanging the Load Cell task forever. */
    uint32_t startTick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - startTick) >= timeout_ms)
        {
            return false; /* HX711 not responding -- caller treats as fault */
        }
    }

    int32_t value = 0;

    /* Shift out 24 bits, MSB first. Each bit: drive SCK high, sample
     * DOUT, drive SCK low. The HX711 datasheet requires SCK high time
     * and low time both >=0.2us and <50us (else it powers down). */
    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        DelayMicroseconds(1);
        value = (value << 1) | (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET ? 1 : 0);
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        DelayMicroseconds(1);
    }

    /* 25th pulse: selects Channel A, gain 128 for the *next* conversion
     * (the most common HX711 wiring). If your module is wired for a
     * different channel/gain, change the pulse count here (25=ChA/128,
     * 26=ChB/32, 27=ChA/64) per the HX711 datasheet table. */
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    DelayMicroseconds(1);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    DelayMicroseconds(1);

    /* Sign-extend the 24-bit two's-complement result to 32-bit. */
    if (value & 0x00800000)
    {
        value |= 0xFF000000;
    }

    *out_raw = value;
    return true;
}

float LoadCell_RawToNewtons(int32_t raw)
{
    return (float)(raw - s_zeroOffsetRaw) * s_newtonsPerRawCount;
}

void LoadCell_Calibrate(int32_t zeroOffsetRaw, float newtonsPerRawCount)
{
    s_zeroOffsetRaw      = zeroOffsetRaw;
    s_newtonsPerRawCount = newtonsPerRawCount;
}

float LoadCell_FilterEMA(float previousFiltered, float newSample, float alpha)
{
    /* Standard exponential moving average: alpha closer to 1.0 tracks
     * the raw signal more closely (less smoothing, more noise); closer
     * to 0.0 smooths harder but lags real force changes more. A starting
     * alpha of ~0.3-0.4 at a 5ms sample period is a reasonable bench
     * default for a load-bearing signal like this. */
    return alpha * newSample + (1.0f - alpha) * previousFiltered;
}

KneeState_t LoadCell_InferGaitState(float forceNewtons, float forceDeltaPerSec, KneeState_t currentState)
{
    /* This is a deliberately simple, explainable state heuristic so it is
     * easy to reason about and tune on the bench. It does NOT attempt to
     * distinguish STAIR_ASCENT/STAIR_DESCENT or SITTING from force alone
     * -- those require additional sensing (e.g. knee angle trajectory,
     * an IMU, or explicit mode commands from the controller link) and
     * are left as TODOs / extension points rather than guessed at here. */

    if (currentState == STATE_FAULT)
    {
        return STATE_FAULT; /* Safety task owns fault entry/exit, not us */
    }

    bool weightBearing = (forceNewtons >= STANCE_FORCE_THRESHOLD_N);
    bool unloaded       = (forceNewtons <= SWING_FORCE_THRESHOLD_N);

    if (unloaded && forceDeltaPerSec <= RAPID_UNLOAD_RATE_N_PER_S)
    {
        /* Rapid unloading -> leg is being lifted into swing phase. */
        return STATE_WALKING;
    }

    if (weightBearing && forceDeltaPerSec >= RAPID_LOAD_RATE_N_PER_S)
    {
        /* Rapid loading -> heel strike, stance phase beginning. Still
         * classified as WALKING (stance is part of the walking cycle)
         * unless it then plateaus, handled below. */
        return STATE_WALKING;
    }

    if (weightBearing && forceDeltaPerSec > -20.0f && forceDeltaPerSec < 20.0f)
    {
        /* Sustained, steady weight-bearing with little change -> standing
         * still rather than mid-stride. */
        return STATE_STANDING;
    }

    if (unloaded && forceDeltaPerSec > -20.0f && forceDeltaPerSec < 20.0f)
    {
        /* Sustained near-zero load, not actively changing -> could be
         * IDLE (limb off the ground / sitting with leg relaxed). We do
         * not promote this to SITTING automatically since sitting vs.
         * idle-standing-on-other-leg are not distinguishable by force
         * alone; that decision is left to an explicit command or a
         * future IMU-fused implementation. */
        return STATE_IDLE;
    }

    /* Ambiguous reading -- hold the current state rather than guessing,
     * to avoid chattering. */
    return currentState;
}
