/* The last SSS_WINDOW readings are kept in a ring buffer. On every update:
 *   1. median        -> the typical heart rate right now
 *   2. glitch filter -> drop readings < 0.6x or > 1.7x the median
 *   3. smoothing     -> median of each 3 neighbours removes one-off spikes
 *   4. erratic check -> count sudden jumps and measure the spread
 ******************************************************************************
 */
#include "sss_detect.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


static float s_bpm[SSS_WINDOW];   /* last readings array                      */
static uint8_t s_head;              /* where the next reading is written      */
static uint8_t s_count;             /* how many slots are filled              */


static bool s_have_reading;      /* at least one reading accepted          */
static uint32_t s_last_accept_ms;    /* last reading used (1 per second)       */
static uint32_t s_last_valid_ms;     /* last reading with a valid BPM          */

// Alarm
static bool s_mute_active;
static uint32_t s_mute_until_ms;
static bool s_finger_on;
static SSS_Status s_st;

/** Forget the stored readings (finger removed or readings stopped). */
static void clear_readings(void)
{
    s_head = 0;
    s_count = 0;
    s_have_reading = false;
}

// Comparator of 2 floats. -ve if a<b, 0 if equal, +ve if a>b
static int compare_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

// Uses built in quicksort to get median from the array
static float median(const float *v, uint8_t n)
{
    float s[SSS_WINDOW];
    memcpy(s, v, n * sizeof(float));
    qsort(s, n, sizeof(float), compare_float);
    return (n % 2U) ? s[n / 2U] : 0.5f * (s[n / 2U - 1U] + s[n / 2U]);
}

/**
 * Check the stored readings. Fills in bpm, jumps, spread.
 * @return true if the heart rate is erratic.
 */
static bool check_erratic(void)
{
    float   b[SSS_WINDOW];   /* readings, oldest first */
    float   v[SSS_WINDOW];   /* readings after removing glitches */
    uint8_t nv = 0;

    s_st.has_rate   = false;
    s_st.jumps      = 0;
    s_st.spread_pct = 0;
    s_st.readings   = s_count;

    if (s_count < 3U)
    {
        return false;        /* not enough readings yet */
    }

    // The rolling array is flattened into chronological order
    uint8_t start = (uint8_t)((s_head + SSS_WINDOW - s_count) % SSS_WINDOW);
    for (uint8_t i = 0; i < s_count; i++)
    {
        b[i] = s_bpm[(start + i) % SSS_WINDOW];
    }

    /* 1. Typical heart rate = median (one bad reading barely moves it). */
    float med = median(b, s_count);
    s_st.bpm      = (uint16_t)(med + 0.5f);
    s_st.has_rate = true;

    /* 2. Drop glitch readings far from the typical rate. */
    for (uint8_t i = 0; i < s_count; i++)
    {
        if (b[i] >= SSS_GLITCH_LOW * med && b[i] <= SSS_GLITCH_HIGH * med)
        {
            v[nv++] = b[i];
        }
    }
    if (nv < SSS_MIN_READINGS)
    {
        return false;
    }

    /* 3. Smooth out single-reading spikes: replace each reading with the
          median of itself and its two neighbours. A one-off glitch
          disappears, but a heart rate that keeps jumping still shows. */
    float   m[SSS_WINDOW];
    uint8_t nm = 0;
    for (uint8_t i = 1; i + 1U < nv; i++)
    {
        m[nm++] = median(&v[i - 1U], 3U);
    }

    /* 4. Count sudden jumps and find the lowest / highest value. */
    float   lo = m[0], hi = m[0];
    uint8_t jumps = 0;
    for (uint8_t i = 1; i < nm; i++)
    {
        if (fabsf(m[i] - m[i - 1U]) > SSS_JUMP_FRAC * med)
        {
            jumps++;
        }
        if (m[i] < lo) lo = m[i];
        if (m[i] > hi) hi = m[i];
    }
    float spread = (hi - lo) / med;

    s_st.jumps      = jumps;
    s_st.spread_pct = (uint8_t)((spread > 2.55f ? 2.55f : spread) * 100.0f + 0.5f);

    return (jumps >= SSS_ERRATIC_JUMPS) || (spread > SSS_SPREAD_FRAC);
}


void SSS_Init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_mute_active = false;
    s_finger_on   = false;
    clear_readings();
}

void SSS_OnReading(uint32_t t_ms, float bpm)
{
    /* At most one reading per second, so calling this every loop with the
       same value does not fill the window with copies. */
    if (s_have_reading && (t_ms - s_last_accept_ms) < SSS_READING_PERIOD_MS)
    {
        return;
    }
    s_have_reading   = true;
    s_last_accept_ms = t_ms;

    /* Ignore readings the sensor could not calculate (e.g. 0). */
    if (!(bpm >= SSS_MIN_VALID_BPM && bpm <= SSS_MAX_VALID_BPM))
    {
        return;
    }
    s_last_valid_ms = t_ms;

    /* Store in the ring buffer, overwriting the oldest when full. */
    s_bpm[s_head] = bpm;
    s_head = (uint8_t)((s_head + 1U) % SSS_WINDOW);
    if (s_count < SSS_WINDOW)
    {
        s_count++;
    }
}

void SSS_Update(uint32_t now_ms, bool finger_on)
{
    s_finger_on = finger_on;

    if (!finger_on)
    {
        clear_readings();
        s_st.erratic_now = false;
        s_st.has_rate    = false;
        s_st.readings    = 0;
    }
    else
    {
        /* Readings stopped arriving: the old ones are out of date. */
        if (s_count > 0U && (now_ms - s_last_valid_ms) > SSS_STALE_MS)
        {
            clear_readings();
        }
        s_st.erratic_now = check_erratic();
    }

    /* Latch: once erratic, the alarm stays on until acknowledged. */
    if (s_st.erratic_now)
    {
        s_st.alarm = true;
    }

    /* Piezo: on during the alarm unless recently acknowledged. */
    if (s_mute_active && (int32_t)(now_ms - s_mute_until_ms) >= 0)
    {
        s_mute_active = false;
    }
    s_st.muted = s_mute_active;
    s_st.sound = s_st.alarm && !s_mute_active;
}

const SSS_Status *SSS_GetStatus(void)
{
    return &s_st;
}

void SSS_Acknowledge(uint32_t now_ms)
{
    s_st.alarm      = false;
    s_mute_active   = true;
    s_mute_until_ms = now_ms + SSS_ACK_MUTE_MS;
    /* If still erratic, the alarm comes back on the next update (LED
       flashes) but the piezo stays quiet until the mute time is over. */
}

void SSS_GetText(char *out)
{
    char     line[32];
    unsigned bpm = (s_st.bpm > 999U) ? 999U : s_st.bpm;

    if (!s_finger_on)
    {
        snprintf(line, sizeof(line), "Place finger");
    }
    else if (s_st.alarm)
    {
        snprintf(line, sizeof(line), "!ERRATIC HR");
    }
    else if (s_st.has_rate)
    {
        snprintf(line, sizeof(line), "HR %3u bpm", bpm);
    }
    else
    {
        snprintf(line, sizeof(line), "Measuring...");
    }

    memcpy(out, line, 16);
    out[16] = '\0';
}