// Stage D: Stage C (timestamped CAN receive + gap statistics) plus SD card logging.
// Hub = Adafruit Feather ESP32 V2 + Lonely Binary TK109 (CAN) + WWZMDiB 3.3 V microSD module (SPI).
//
// Data path:
//   canRx task (core 1, high priority): wait for a frame -> timestamp -> gap stats -> copy into logQueue
//   sdTask     (core 0, low priority):  take records from logQueue -> CSV text -> RAM buffer -> SD card
// An SD card can stall for tens of milliseconds while it writes. The queue soaks that up,
// so the SD card can never delay a CAN timestamp.
//
// Button SW38: press to STOP logging and close the file (safe to remove the card).
//              Press again to start a new file. Red LED (D13) = logging.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <atomic>
#include <string.h>
#include "driver/twai.h"
#include "esp_timer.h"

// ---- CAN pins (TK109) ----
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;    // Feather A5  -> TK109 CTX
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_33;   // Feather D33 <- TK109 CRX  (moved off SCK: the SD card needs SCK)

// ---- SD card pins (SPI) ----
constexpr int      SD_SCK_PIN  = 5;          // Feather SCK -> module CLK/SCK
constexpr int      SD_MISO_PIN = 21;         // Feather MI  <- module MISO (DO)
constexpr int      SD_MOSI_PIN = 19;         // Feather MO  -> module MOSI (DI)
constexpr int      SD_CS_PIN   = 32;         // Feather D32 -> module CS
constexpr uint32_t SD_SPI_HZ   = 4000000;    // 4 MHz: far more than we need, and forgiving of breadboard wires

// ---- Other board pins ----
constexpr int BUTTON_PIN = 38;   // SW38 user button: input only, pull-up on the board, pressed = LOW
constexpr int LED_PIN    = 13;   // red LED: on = logging to a file

// ---- Task settings ----
constexpr uint32_t    RX_TASK_STACK = 4096;
constexpr UBaseType_t RX_TASK_PRIO  = 10;
constexpr BaseType_t  RX_TASK_CORE  = 1;
constexpr uint32_t    SD_TASK_STACK = 8192;
constexpr UBaseType_t SD_TASK_PRIO  = 3;
constexpr BaseType_t  SD_TASK_CORE  = 0;

// ---- Logging settings ----
constexpr int      LOG_QUEUE_LEN     = 1024;   // records; at ~112 frames/s that is ~9 s of buffering
constexpr size_t   WRITE_CHUNK_BYTES = 4096;   // collect this much text before each SD write
constexpr uint32_t FLUSH_PERIOD_MS   = 1000;   // push data onto the card at least this often
constexpr uint32_t MOUNT_RETRY_MS    = 2000;   // if there's no card, try again this often

// ---- Report settings ----
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr int      MAX_IDS          = 16;

// ===================== Logging: shared data =====================
// NOTE: these types come BEFORE the first function on purpose. The Arduino IDE automatically
// writes declarations (prototypes) for every function and puts them just above the first
// function in the file, so any type a function uses must already be defined by then.
// One received frame, as it travels from the CAN task to the SD task.
struct LogRecord {
  int64_t  t_us;      // hub timestamp, microseconds since boot
  uint32_t id;        // CAN ID
  uint8_t  dlc;       // number of data bytes (0-8)
  uint8_t  data[8];
};
static QueueHandle_t logQueue = nullptr;

// Status numbers, shown in the once-a-second report.
struct LogStatus {
  uint32_t queued;      // records the CAN task put in the queue
  uint32_t dropped;     // records lost because the queue was full
  uint32_t written;     // CSV lines handed to the card
  uint32_t sdErrors;    // failed mounts, opens or writes
  uint32_t maxWaiting;  // most records ever waiting in the queue at once
  bool     cardOk;      // card mounted?
  bool     logging;     // file open and being written?
  bool     wantLogging; // false after the button stops logging
  char     fileName[16];
};
static LogStatus logStat = {};
static portMUX_TYPE logLock = portMUX_INITIALIZER_UNLOCKED;

static std::atomic<bool> toggleRequest{false};   // set by loop() when the button is pressed

// ===================== Gap statistics (same as Stage C) =====================
struct IdStats {
  bool     used;
  uint32_t id;
  int64_t  lastUs;
  uint32_t frames;
  uint32_t gaps;
  int64_t  sumGapUs;
  int64_t  minGapUs;
  int64_t  maxGapUs;
};
static IdStats stats[MAX_IDS];
static portMUX_TYPE statsLock = portMUX_INITIALIZER_UNLOCKED;

static void recordFrame(uint32_t id, int64_t t_us) {
  IdStats *row = nullptr;
  for (int i = 0; i < MAX_IDS; i++) {
    if (stats[i].used && stats[i].id == id) { row = &stats[i]; break; }
  }
  if (row == nullptr) {
    for (int i = 0; i < MAX_IDS; i++) {
      if (!stats[i].used) {
        row = &stats[i];
        row->used = true;
        row->id = id;
        row->lastUs = -1;
        break;
      }
    }
    if (row == nullptr) return;
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

// ===================== CAN receive task =====================
static void canRxTask(void *arg) {
  twai_message_t msg;

  for (;;) {
    if (twai_receive(&msg, portMAX_DELAY) != ESP_OK) continue;
    int64_t t_us = esp_timer_get_time();   // stamp first, before anything else

    portENTER_CRITICAL(&statsLock);
    recordFrame(msg.identifier, t_us);
    portEXIT_CRITICAL(&statsLock);

    LogRecord rec;
    rec.t_us = t_us;
    rec.id   = msg.identifier;
    rec.dlc  = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
    memcpy(rec.data, msg.data, 8);

    // Timeout 0: never wait for the SD side. If the queue is full, count the loss and move on.
    bool ok = (xQueueSend(logQueue, &rec, 0) == pdTRUE);

    portENTER_CRITICAL(&logLock);
    if (ok) logStat.queued++; else logStat.dropped++;
    portEXIT_CRITICAL(&logLock);
  }
}

// ===================== SD task =====================
static File   logFile;
static char   outBuf[WRITE_CHUNK_BYTES + 128];   // +128 so one more line always fits
static size_t outLen = 0;

static void bumpSdErrors() {
  portENTER_CRITICAL(&logLock);
  logStat.sdErrors++;
  portEXIT_CRITICAL(&logLock);
}

// Open the first unused name: /LOG0001.CSV, /LOG0002.CSV, ... so old logs are never overwritten.
static bool openNextLogFile() {
  char name[16];
  for (int n = 1; n <= 9999; n++) {
    snprintf(name, sizeof(name), "/LOG%04d.CSV", n);
    if (SD.exists(name)) continue;

    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) return false;
    logFile.print("t_us,id,dlc,data\n");   // CSV header row
    logFile.flush();

    portENTER_CRITICAL(&logLock);
    strncpy(logStat.fileName, name, sizeof(logStat.fileName) - 1);
    portEXIT_CRITICAL(&logLock);
    return true;
  }
  return false;   // card already has 9999 logs
}

// Turn one record into a CSV line, e.g. "12345678,101,4,0000007B\n", and add it to outBuf.
static void appendCsv(const LogRecord &r) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  int n = snprintf(outBuf + outLen, sizeof(outBuf) - outLen, "%lld,%lX,%u,",
                   (long long)r.t_us, (unsigned long)r.id, (unsigned)r.dlc);
  outLen += n;
  for (uint8_t i = 0; i < r.dlc; i++) {          // each byte -> two hex characters
    outBuf[outLen++] = HEX_DIGITS[r.data[i] >> 4];
    outBuf[outLen++] = HEX_DIGITS[r.data[i] & 0x0F];
  }
  outBuf[outLen++] = '\n';
}

// Hand outBuf to the card. Returns false if the card didn't take all of it.
static bool writeBuffer() {
  if (outLen == 0) return true;
  size_t n = logFile.write(reinterpret_cast<const uint8_t *>(outBuf), outLen);
  bool ok = (n == outLen);
  outLen = 0;
  return ok;
}

static void stopLogging(bool cardStillGood) {
  if (cardStillGood) { writeBuffer(); logFile.flush(); }
  logFile.close();
  SD.end();
  outLen = 0;
  digitalWrite(LED_PIN, LOW);
  portENTER_CRITICAL(&logLock);
  logStat.logging = false;
  logStat.cardOk  = false;
  portEXIT_CRITICAL(&logLock);
}

static void sdTask(void *arg) {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  bool     logging      = false;
  bool     wantLogging  = true;
  uint32_t lastTryMs    = 0;
  uint32_t lastFlushMs  = millis();
  uint32_t linesWritten = 0;
  LogRecord rec;

  for (;;) {
    // 1) Button pressed? Flip between "log" and "stop".
    if (toggleRequest.exchange(false)) {
      wantLogging = !wantLogging;
      if (!wantLogging && logging) {
        stopLogging(true);
        logging = false;
      }
    }

    // 2) Want to log but no file open? Mount the card and open a new file (retry every 2 s).
    if (wantLogging && !logging && millis() - lastTryMs >= MOUNT_RETRY_MS) {
      lastTryMs = millis();
      SD.end();
      bool mounted = SD.begin(SD_CS_PIN, SPI, SD_SPI_HZ);
      if (mounted && openNextLogFile()) {
        logging = true;
        lastFlushMs = millis();
        digitalWrite(LED_PIN, HIGH);
      } else {
        bumpSdErrors();
      }
      portENTER_CRITICAL(&logLock);
      logStat.cardOk = mounted;
      logStat.logging = logging;
      portEXIT_CRITICAL(&logLock);
    }

    // 3) Take every record that's waiting (sleep up to 50 ms if there are none).
    UBaseType_t waiting = uxQueueMessagesWaiting(logQueue);
    TickType_t  wait    = pdMS_TO_TICKS(50);
    while (xQueueReceive(logQueue, &rec, wait) == pdTRUE) {
      wait = 0;                                   // after the first record, don't wait: just drain
      if (!logging) continue;                     // no file: throw it away (still counted as queued)
      appendCsv(rec);
      linesWritten++;
      if (outLen >= WRITE_CHUNK_BYTES) {
        if (!writeBuffer()) {                     // card removed or failed
          bumpSdErrors();
          stopLogging(false);
          logging = false;
        }
      }
    }

    // 4) At least once a second, push everything onto the card so a power cut loses at most ~1 s.
    if (logging && millis() - lastFlushMs >= FLUSH_PERIOD_MS) {
      lastFlushMs = millis();
      if (writeBuffer()) {
        logFile.flush();
      } else {
        bumpSdErrors();
        stopLogging(false);
        logging = false;
      }
    }

    // 5) Publish status for the report.
    portENTER_CRITICAL(&logLock);
    logStat.logging     = logging;
    logStat.wantLogging = wantLogging;
    logStat.written     = linesWritten;
    if (waiting > logStat.maxWaiting) logStat.maxWaiting = waiting;
    portEXIT_CRITICAL(&logLock);
  }
}

// ===================== Button (polled from loop) =====================
static void pollButton() {
  static bool     lastState  = HIGH;
  static uint32_t lastEdgeMs = 0;
  bool now = digitalRead(BUTTON_PIN);
  if (now != lastState && millis() - lastEdgeMs > 50) {   // ignore contact bounce within 50 ms
    lastEdgeMs = millis();
    lastState  = now;
    if (now == LOW) toggleRequest = true;                  // act on the press, not the release
  }
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BUTTON_PIN, INPUT);       // GPIO38 is input-only; the board already has the pull-up
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogRecord));
  if (logQueue == nullptr) {
    Serial.println("ERROR: could not create log queue");
    while (true) delay(1000);
  }

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

  if (xTaskCreatePinnedToCore(canRxTask, "canRx", RX_TASK_STACK, nullptr,
                              RX_TASK_PRIO, nullptr, RX_TASK_CORE) != pdPASS ||
      xTaskCreatePinnedToCore(sdTask, "sdLog", SD_TASK_STACK, nullptr,
                              SD_TASK_PRIO, nullptr, SD_TASK_CORE) != pdPASS) {
    Serial.println("ERROR: could not create tasks");
    while (true) delay(1000);
  }
  Serial.println("Hub Stage D running: canRx on core 1, SD logging on core 0. SW38 = stop/start logging.");
}

void loop() {
  static uint32_t lastReportMs = 0;
  pollButton();
  if (millis() - lastReportMs < REPORT_PERIOD_MS) { delay(10); return; }
  lastReportMs = millis();

  // ---- Gap report (same as Stage C) ----
  IdStats snapshot[MAX_IDS];
  portENTER_CRITICAL(&statsLock);
  memcpy(snapshot, stats, sizeof(stats));
  for (int i = 0; i < MAX_IDS; i++) {
    stats[i].frames = 0;
    stats[i].gaps = 0;
    stats[i].sumGapUs = 0;
  }
  portEXIT_CRITICAL(&statsLock);

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

  // ---- Logging report ----
  portENTER_CRITICAL(&logLock);
  LogStatus L = logStat;
  portEXIT_CRITICAL(&logLock);

  const char *state = L.logging          ? "LOGGING"
                    : !L.wantLogging     ? "STOPPED - safe to remove card"
                    : L.cardOk           ? "CARD OK, NO FILE"
                                         : "NO CARD (retrying)";
  Serial.printf("log: %s  file=%s  written=%lu  queued=%lu  dropped=%lu  max_waiting=%lu  sd_errors=%lu\n",
                state, L.logging ? L.fileName : "-",
                (unsigned long)L.written, (unsigned long)L.queued, (unsigned long)L.dropped,
                (unsigned long)L.maxWaiting, (unsigned long)L.sdErrors);
}
