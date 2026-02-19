#pragma once
#include <Arduino.h>

/*
 * Project: Offline Speaker-Aware Voice Control on ESP32 (TinyML)
 * Hardware: ESP32 DevKit + INMP441 (I2S) + 2 LEDs
 *
 * Notes:
 * - Thresholds below are tuned for security-first behavior.
 * - The system prefers False Reject over False Accept.
 */

// ---------------- GPIO ----------------
static constexpr int LED_GREEN_PIN = 2;
static constexpr int LED_RED_PIN   = 4;  // change if needed

// ------------- INMP441 pins ------------
static constexpr int I2S_SCK = 14;
static constexpr int I2S_WS  = 25;
static constexpr int I2S_SD  = 32;

// ------------ I2S settings -------------
static constexpr int SHIFT_BITS    = 13;  // I2S 32-bit -> PCM16
static constexpr int DMA_BUF_COUNT = 10;
static constexpr int DMA_BUF_LEN   = 256;

// -------- Decision thresholds ----------
// Candidate thresholds (enter "candidate" state)
static constexpr float THR_ME_CAND      = 0.65f;
static constexpr float THR_OTHER_CAND   = 0.75f;
static constexpr float THR_OFF_CAND     = 0.65f;

// Confirm thresholds (used with margins)
static constexpr float THR_ME_CONFIRM    = 0.78f;
static constexpr float THR_OTHER_CONFIRM = 0.88f;  // stricter
static constexpr float THR_OFF_CONFIRM   = 0.78f;

// Margin thresholds: prob must beat the best competitor by this margin
static constexpr float MARGIN_ME    = 0.20f;
static constexpr float MARGIN_OTHER = 0.30f;  // stricter for other
static constexpr float MARGIN_OFF   = 0.20f;

// Instant thresholds: bypass streak requirement if extremely confident
static constexpr float INSTANT_ME    = 0.95f;
static constexpr float INSTANT_OTHER = 0.97f;
static constexpr float INSTANT_OFF   = 0.95f;

// Timing
static constexpr uint32_t RED_WARN_MS = 2000; // red LED warning duration
static constexpr uint32_t REFRACT_MS  = 400;  // refractory to prevent double-trigger

// Debug
static constexpr bool DEBUG_NN = false;
static constexpr bool PRINT_PROBABILITIES = true; // set false for silent mode
