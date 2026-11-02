// Stage C: Stage B plus per-ID gap statistics, to verify timing against the send rate.
#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"
#include "esp_timer.h"

// ---- Feather V2 + TK109: A5 (GPIO4) -> CTX, SCK (GPIO5) <- CRX ----
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

// ---- Task settings ----
constexpr uint32_t    RX_TASK_STACK_BYTES = 4096;
constexpr UBaseType_t RX_TASK_PRIORITY    = 10;
constexpr BaseType_t  RX_TASK_CORE        = 1;

// ---- Test settings ----
constexpr bool     PRINT_EVERY_FRAME = false;  // false for timing tests: printing every frame delays the timestamps
constexpr uint32_t REPORT_PERIOD_MS  = 1000;   // how often loop() prints the summary
constexpr int      MAX_IDS           = 16;     // how many different IDs we track

// One row of statistics per CAN ID.
struct IdStats {
  bool     used;
  uint32_t id;
  int64_t  lastUs;     // timestamp of the most recent frame with this ID
  uint32_t frames;     // frames seen in this report window
  uint32_t gaps;       // gaps measured in this report window
  int64_t  sumGapUs;   // sum of gaps, for the average
  int64_t  minGapUs;
  int64_t  maxGapUs;
};

static IdStats stats[MAX_IDS];                                  // shared by both tasks
static portMUX_TYPE statsLock = portMUX_INITIALIZER_UNLOCKED;   // protects stats[]

// Called only from the receive task, with statsLock held.
static void recordFrame(uint32_t id, int64_t t_us) {
  IdStats *row = nullptr;
  for (int i = 0; i < MAX_IDS; i++) {
    if (stats[i].used && stats[i].id == id) { row = &stats[i]; break; }
  }
  if (row == nullptr) {                       // new ID: claim an empty row
    for (int i = 0; i < MAX_IDS; i++) {
      if (!stats[i].used) {
        row = &stats[i];
        row->used = true;
        row->id = id;
        row->lastUs = -1;                     // -1 means "no previous frame yet"
        break;
      }
    }
    if (row == nullptr) return;               // table full: ignore this ID
  }

  row->frames++;
  if (row->lastUs >= 0) {
    int64_t gap = t_us - row->lastUs;
    if (row->gaps == 0 || gap < row->minGapUs) row->minGapUs = gap;
    if (row->gaps == 0 || gap > row->maxGapUs) row->maxGapUs = gap;
    row->sumGapUs += gap;
    row->gaps++;
  }
  row->lastUs = t_us;
}

static void canRxTask(void *arg) {
  twai_message_t msg;
  char line[96];

  for (;;) {
    if (twai_receive(&msg, portMAX_DELAY) != ESP_OK) continue;
    int64_t t_us = esp_timer_get_time();

    portENTER_CRITICAL(&statsLock);
    recordFrame(msg.identifier, t_us);
    portEXIT_CRITICAL(&statsLock);

    if (PRINT_EVERY_FRAME) {
      uint8_t len = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
      int n = snprintf(line, sizeof(line), "%12lld us  ID 0x%03lX  [%u]",
                       (long long)t_us, (unsigned long)msg.identifier, len);
      for (uint8_t i = 0; i < len; i++) {
        n += snprintf(line + n, sizeof(line) - n, " %02X", msg.data[i]);
      }
      Serial.println(line);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 32;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("ERROR: TWAI setup failed");
    while (true) delay(1000);
  }

  if (xTaskCreatePinnedToCore(canRxTask, "canRx", RX_TASK_STACK_BYTES, nullptr,
                              RX_TASK_PRIORITY, nullptr, RX_TASK_CORE) != pdPASS) {
    Serial.println("ERROR: could not create canRx task");
    while (true) delay(1000);
  }
  Serial.println("Hub running: canRx on core 1, gap statistics enabled");
}

void loop() {
  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs < REPORT_PERIOD_MS) { delay(10); return; }
  lastReportMs = millis();

  // 1) Copy the table and reset the window, holding the lock as briefly as possible.
  IdStats snapshot[MAX_IDS];
  portENTER_CRITICAL(&statsLock);
  memcpy(snapshot, stats, sizeof(stats));
  for (int i = 0; i < MAX_IDS; i++) {
    stats[i].frames = 0;
    stats[i].gaps = 0;
    stats[i].sumGapUs = 0;
    // lastUs is kept, so the first gap of the next window is still measured.
  }
  portEXIT_CRITICAL(&statsLock);

  // 2) Print from the copy, with the lock released.
  Serial.println("---- gap report ----");
  for (int i = 0; i < MAX_IDS; i++) {
    const IdStats &r = snapshot[i];
    if (!r.used || r.gaps == 0) continue;
    double avgMs = (double)r.sumGapUs / r.gaps / 1000.0;
    Serial.printf("ID 0x%03lX  frames %lu  avg %.3f ms  min %.3f ms  max %.3f ms  (~%.1f Hz)\n",
                  (unsigned long)r.id, (unsigned long)r.frames, avgMs,
                  r.minGapUs / 1000.0, r.maxGapUs / 1000.0, 1000.0 / avgMs);
  }

  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK) {
    Serial.printf("bus: missed=%lu overrun=%lu bus_errors=%lu\n",
                  (unsigned long)s.rx_missed_count,
                  (unsigned long)s.rx_overrun_count,
                  (unsigned long)s.bus_error_count);
  }
}
