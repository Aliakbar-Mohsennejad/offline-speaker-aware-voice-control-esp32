/*
 * Offline Speaker-Aware Voice Control on ESP32 (TinyML / Edge Impulse)
 *
 * Hardware:
 *  - ESP32 DevKit (ESP-WROOM-32)
 *  - INMP441 I2S Microphone (32-bit RX ONLY_LEFT)
 *  - Green LED (authorized on)
 *  - Red LED (unauthorized warning)
 *
 * Pipeline:
 *  I2S 32-bit RX -> PCM16 (SHIFT_BITS) -> Edge Impulse continuous inference
 *
 * Labels:
 *  - switch_on_me
 *  - switch_on_other
 *  - unknown
 *  - noise
 *  - power_off (sometimes exported as "Power Off")
 *
 * Behavior:
 *  - switch_on_me    => GREEN latched ON until power_off
 *  - switch_on_other => RED warning for 2 seconds (no state change)
 *  - unknown/noise   => no action
 *
 * Decision Logic (security-first):
 *  - Mutual exclusion: only one action per decision
 *  - Two-stage confirmation: 2 consecutive confirms OR instant high confidence
 *  - Stricter rules for unauthorized detection to reduce false positives
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

#include "config.h"
#include "esp32_voice_onoff_aliakbar_inferencing.h"

// ---------------- Inference buffers ----------------
typedef struct {
  int16_t *buffers[2];
  uint8_t  buf_select;
  volatile uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;

static const uint32_t sample_buffer_size = EI_CLASSIFIER_SLICE_SIZE;
static int32_t sampleBuffer32[sample_buffer_size];
static int16_t sampleBuffer16[sample_buffer_size];

static int  print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
static volatile bool record_status = true;

// ---------------- System state ----------------
static bool power_on = false;
static uint32_t red_until_ms = 0;
static uint32_t last_fire_ms = 0;

// Two-stage confirm streaks (consecutive confirms)
static uint8_t me_confirm_streak = 0;
static uint8_t other_confirm_streak = 0;
static uint8_t off_confirm_streak = 0;

// ---------------- Utilities ----------------
static inline int16_t clamp16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

static inline int16_t i2s32_to_pcm16(int32_t x) {
  return clamp16(x >> SHIFT_BITS);
}

static inline void update_leds() {
  digitalWrite(LED_GREEN_PIN, power_on ? HIGH : LOW);
  digitalWrite(LED_RED_PIN, (millis() < red_until_ms) ? HIGH : LOW);
}

static inline float max4(float a, float b, float c, float d) {
  return max(max(a, b), max(c, d));
}

static inline void reset_streaks() {
  me_confirm_streak = 0;
  other_confirm_streak = 0;
  off_confirm_streak = 0;
}

// ---------------- Audio pipeline ----------------
static void audio_inference_callback(uint32_t n_samples) {
  for (uint32_t i = 0; i < n_samples; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer16[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static void capture_samples(void *arg) {
  const uint32_t n_samples_to_read = (uint32_t)arg;
  size_t bytes_read = 0;

  while (record_status) {
    i2s_read(I2S_NUM_0, (void*)sampleBuffer32,
             n_samples_to_read * sizeof(int32_t),
             &bytes_read,
             portMAX_DELAY);

    uint32_t samples_read = bytes_read / sizeof(int32_t);
    for (uint32_t i = 0; i < samples_read; i++) {
      sampleBuffer16[i] = i2s32_to_pcm16(sampleBuffer32[i]);
    }

    audio_inference_callback(samples_read);
  }

  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (int16_t*)malloc(n_samples * sizeof(int16_t));
  inference.buffers[1] = (int16_t*)malloc(n_samples * sizeof(int16_t));

  if (!inference.buffers[0] || !inference.buffers[1]) {
    Serial.println("ERROR: Could not allocate inference buffers");
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count  = 0;
  inference.n_samples  = n_samples;
  inference.buf_ready  = 0;

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = EI_CLASSIFIER_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_SD
  };

  esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    Serial.print("Error in i2s_driver_install: ");
    Serial.println(ret);
    return false;
  }

  ret = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (ret != ESP_OK) {
    Serial.print("Error in i2s_set_pin: ");
    Serial.println(ret);
    return false;
  }

  i2s_zero_dma_buffer(I2S_NUM_0);

  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 16, (void*)n_samples, 10, NULL);

  return true;
}

static bool microphone_inference_record() {
  while (inference.buf_ready == 0) delay(1);
  inference.buf_ready = 0;
  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

// ---------------- Classification helpers ----------------
static void read_probabilities(const ei_impulse_result_t &result,
                               float &p_me, float &p_other, float &p_unk, float &p_noise, float &p_off) {
  p_me = p_other = p_unk = p_noise = p_off = 0.0f;

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    const char* label = result.classification[ix].label;
    float value = result.classification[ix].value;

    if (strcmp(label, "switch_on_me") == 0) p_me = value;
    else if (strcmp(label, "switch_on_other") == 0) p_other = value;
    else if (strcmp(label, "unknown") == 0) p_unk = value;
    else if (strcmp(label, "noise") == 0) p_noise = value;
    else if (strcmp(label, "Power Off") == 0 || strcmp(label, "power_off") == 0 || strcmp(label, "Power_Off") == 0) p_off = value;
  }
}

static inline bool is_refractory(uint32_t now_ms) {
  return (now_ms - last_fire_ms) < REFRACT_MS;
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  Serial.println("Offline Speaker-Aware Voice Control (ESP32 + INMP441 + Edge Impulse)");

  run_classifier_init();
  delay(300);

  if (!microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE)) {
    Serial.println("ERR: Audio buffer allocation / I2S init failed");
    while (1) delay(1000);
  }

  Serial.println("Recording...");
  update_leds();
}

void loop() {
  update_leds();

  if (!microphone_inference_record()) {
    Serial.println("ERR: Failed to record audio...");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, DEBUG_NN);
  if (r != EI_IMPULSE_OK) {
    Serial.print("ERR: run_classifier_continuous returned: ");
    Serial.println(r);
    return;
  }

  float p_me, p_other, p_unk, p_noise, p_off;
  read_probabilities(result, p_me, p_other, p_unk, p_noise, p_off);

  uint32_t now = millis();
  const bool refractory = is_refractory(now);

  // competitors for margin checks
  float comp_me    = max4(p_other, p_unk, p_noise, p_off);
  float comp_other = max4(p_me,    p_unk, p_noise, p_off);
  float comp_off   = max4(p_me,    p_other, p_unk, p_noise);

  // candidate states
  bool me_cand    = (p_me    >= THR_ME_CAND);
  bool other_cand = (p_other >= THR_OTHER_CAND);
  bool off_cand   = (p_off   >= THR_OFF_CAND);

  // confirm checks (instant OR confirm+margin)
  bool me_confirm =
      (p_me >= INSTANT_ME) ||
      ((p_me >= THR_ME_CONFIRM) && ((p_me - comp_me) >= MARGIN_ME));

  bool other_confirm =
      (p_other >= INSTANT_OTHER) ||
      ((p_other >= THR_OTHER_CONFIRM) && ((p_other - comp_other) >= MARGIN_OTHER) &&
       (p_other > p_unk + 0.10f) && (p_other > p_noise + 0.10f));

  bool off_confirm =
      (p_off >= INSTANT_OFF) ||
      ((p_off >= THR_OFF_CONFIRM) && ((p_off - comp_off) >= MARGIN_OFF));

  // update streaks (2-stage confirmation)
  if (me_confirm) me_confirm_streak = min<uint8_t>(me_confirm_streak + 1, 3);
  else if (!me_cand) me_confirm_streak = 0;

  if (other_confirm) other_confirm_streak = min<uint8_t>(other_confirm_streak + 1, 3);
  else if (!other_cand) other_confirm_streak = 0;

  if (off_confirm) off_confirm_streak = min<uint8_t>(off_confirm_streak + 1, 3);
  else if (!off_cand) off_confirm_streak = 0;

  bool fire_me    = (me_confirm_streak >= 2);
  bool fire_other = (other_confirm_streak >= 2);
  bool fire_off   = (off_confirm_streak >= 2);

  // mutual exclusion: score = prob - competitor
  float score_me    = p_me    - comp_me;
  float score_other = p_other - comp_other;
  float score_off   = p_off   - comp_off;

  if (!refractory) {
    // OFF has priority only if currently ON
    if (power_on && fire_off) {
      power_on = false;
      last_fire_ms = now;
      reset_streaks();
      Serial.println(">>> POWER OFF (GREEN=OFF)");
    } else {
      // resolve ME vs OTHER if both fire
      if (fire_me && fire_other) {
        if (score_me >= score_other) fire_other = false;
        else fire_me = false;
      }

      if (!power_on && fire_me) {
        power_on = true;
        last_fire_ms = now;
        reset_streaks();
        Serial.println(">>> SWITCH_ON_ME (GREEN=ON)");
      } else if (!fire_me && fire_other) {
        red_until_ms = now + RED_WARN_MS;
        last_fire_ms = now;
        reset_streaks();
        Serial.println(">>> SWITCH_ON_OTHER (RED WARNING 2s)");
      }
    }
  }

  update_leds();

  // throttled debug output
  if (PRINT_PROBABILITIES && (++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
    Serial.print("me=");     Serial.print(p_me, 3);
    Serial.print(" other="); Serial.print(p_other, 3);
    Serial.print(" unk=");   Serial.print(p_unk, 3);
    Serial.print(" noise="); Serial.print(p_noise, 3);
    Serial.print(" off=");   Serial.println(p_off, 3);

    Serial.print("score_me=");     Serial.print(score_me, 3);
    Serial.print(" score_other="); Serial.print(score_other, 3);
    Serial.print(" score_off=");   Serial.println(score_off, 3);

    print_results = 0;
  }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
