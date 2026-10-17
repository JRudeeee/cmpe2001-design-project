/**
 ******************************************************************************
 * @file    sss_detect.h
 * @brief   Erratic heart-rate detection (sick sinus syndrome warning sign)
 *          from BPM readings, e.g. a MAX30102 library's "current BPM".
 *
 * INPUT : SSS_OnReading(time, bpm) each time the sensor gives a new BPM
 *         (about once per second), and SSS_Update() every loop.
 * OUTPUT: SSS_GetStatus() -> alarm on/off, whether the piezo should sound;
 *         SSS_GetText()   -> 16-character line for the LCD.
 *
 * Rule: the last 10 readings (~10 s) are checked. Glitch readings are
 * removed and single-reading spikes smoothed out, then the heart rate is
 * ERRATIC when EITHER
 *   - there are 2+ sudden jumps (> 12 % of the typical rate) between
 *     consecutive readings, OR
 *   - the spread (highest - lowest) is more than 35 % of the typical rate.
 * A healthy heart only drifts a few bpm, so neither happens.
 *
 * The alarm latches (stays on) until the user acknowledges it.
 ******************************************************************************
 */
#ifndef SSS_DETECT_H
#define SSS_DETECT_H

#include <stdint.h>
#include <stdbool.h>

/* ----------------------------- Tuning ----------------------------------- */
#define SSS_READING_PERIOD_MS     4000U   /**< use at most one reading per second   */
#define SSS_WINDOW                10U     /**< readings analysed (~10 s)            */
#define SSS_MIN_VALID_BPM         30.0f   /**< below = sensor gave no valid rate     */
#define SSS_MAX_VALID_BPM         220.0f  /**< above = sensor gave no valid rate     */
#define SSS_GLITCH_LOW            0.6f    /**< < 0.6 x typical rate = glitch         */
#define SSS_GLITCH_HIGH           1.7f    /**< > 1.7 x typical rate = glitch         */
#define SSS_MIN_READINGS          6U      /**< readings needed before judging        */
#define SSS_JUMP_FRAC             0.12f   /**< sudden jump = > 12 % of typical rate  */
#define SSS_ERRATIC_JUMPS         2U      /**< this many jumps = erratic             */
#define SSS_SPREAD_FRAC           0.35f   /**< max - min > 35 % of typical = erratic */
#define SSS_STALE_MS              10000U   /**< no valid reading this long = restart  */
#define SSS_ACK_MUTE_MS           30000U  /**< piezo silenced after acknowledge      */

/** Everything the rest of the program needs to know. */
typedef struct
{
    bool     alarm;         /**< erratic heart rate detected (latched)          */
    bool     erratic_now;   /**< erratic right now (not latched)                */
    bool     sound;         /**< true = piezo should be beeping                 */
    bool     muted;         /**< alarm acknowledged, piezo silenced for now     */
    bool     has_rate;      /**< enough readings to show a heart rate           */
    uint16_t bpm;           /**< typical heart rate (median of the window)      */
    uint8_t  jumps;         /**< sudden jumps in the window                     */
    uint8_t  spread_pct;    /**< (max - min) / typical rate, in %               */
    uint8_t  readings;      /**< readings currently in the window               */
} SSS_Status;

/** Clear everything. Call once at start-up. */
void              SSS_Init(void);

/**
 * @brief  Give the detector a new heart-rate reading.
 * @param  t_ms  Time of the reading in ms (HAL_GetTick()).
 * @param  bpm   Current heart rate from the sensor. Values outside
 *               30-220 bpm (e.g. 0 when no rate) are ignored.
 */
void              SSS_OnReading(uint32_t t_ms, float bpm);

/**
 * @brief  Re-evaluate. Call every loop iteration.
 * @param  now_ms     Current time (HAL_GetTick()).
 * @param  finger_on  false when no finger is on the sensor
 *                    (pass true if your code cannot tell).
 */
void              SSS_Update(uint32_t now_ms, bool finger_on);

/** Latest result. */
const SSS_Status *SSS_GetStatus(void);

/** User pressed the acknowledge button: clear the alarm, mute piezo 30 s. */
void              SSS_Acknowledge(uint32_t now_ms);

/** Fill out[17] with a 16-character LCD line, e.g. "!ERRATIC HR". */
void              SSS_GetText(char *out);

#endif /* SSS_DETECT_H */