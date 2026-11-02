// CANTwoNode: two-node bench test for the XIAO C3 + TK109 test nodes.
// Sends the traffic pattern the hub's Stage C gap report expects:
//   Node 1: data frame 0x101 every 10 ms      Node 2: data frame 0x202 every 100 ms
//   Both:   heartbeat 0x700 + NODE_ID every 1000 ms
// Every frame carries a 4-byte counter, so the receiver can tell if any frames were lost.
// (For debugging a noisy breadboard bus, TWAI_TIMING_CONFIG_1MBITS() in setup() can be
//  swapped for TWAI_TIMING_CONFIG_500KBITS(). Every node on the bus must use the same rate.)
#include <Arduino.h>
#include "driver/twai.h"

// ================= CHANGE THIS BEFORE FLASHING EACH BOARD =================
constexpr uint8_t NODE_ID = 1;   // 1 on one board, 2 on the other
// ==========================================================================

static_assert(NODE_ID == 1 || NODE_ID == 2, "NODE_ID must be 1 or 2");

// ---- Pins (XIAO C3) ----
constexpr gpio_num_t TX_PIN = GPIO_NUM_5;   // D3 -> TK109 CTX
constexpr gpio_num_t RX_PIN = GPIO_NUM_4;   // D2 <- TK109 CRX

// ---- What this node sends, and what it expects from the other node ----
constexpr uint8_t  PEER_ID         = (NODE_ID == 1) ? 2 : 1;
constexpr uint32_t DATA_ID         = 0x101 * NODE_ID;               // 0x101 or 0x202
constexpr uint32_t DATA_PERIOD_MS  = (NODE_ID == 1) ? 10 : 100;
constexpr uint32_t HB_ID           = 0x700 + NODE_ID;               // 0x701 or 0x702
constexpr uint32_t HB_PERIOD_MS    = 1000;
constexpr uint32_t PEER_DATA_ID    = 0x101 * PEER_ID;
constexpr uint32_t PEER_HB_ID      = 0x700 + PEER_ID;
constexpr uint32_t PEER_TIMEOUT_MS = 2500;                          // no heartbeat for this long = peer lost

// ---- Counters shared between the tasks ----
struct Counters {
  uint32_t sentData;      // data frames accepted into the TX queue
  uint32_t sentHb;        // heartbeats accepted into the TX queue
  uint32_t txRejected;    // twai_transmit refused (queue full, or bus-off)
  uint32_t rxPeerData;    // peer's data frames received
  uint32_t rxPeerHb;      // peer's heartbeats received
  uint32_t rxOther;       // any other ID (e.g. the hub, later)
  uint32_t peerLost;      // gaps in the peer's counter = frames that never arrived
  uint32_t busOffCount;   // how many times we went bus-off and recovered
  uint32_t lastPeerHbMs;  // millis() of the most recent peer heartbeat
  bool     peerSeen;      // have we ever heard a peer heartbeat?
};
static Counters cnt = {};
static portMUX_TYPE cntLock = portMUX_INITIALIZER_UNLOCKED;

// ---- Helpers: put a 32-bit number into 4 bytes (most significant first), and back ----
static void putU32(uint8_t *d, uint32_t v) {
  d[0] = (v >> 24) & 0xFF;
  d[1] = (v >> 16) & 0xFF;
  d[2] = (v >> 8) & 0xFF;
  d[3] = v & 0xFF;
}
static uint32_t getU32(const uint8_t *d) {
  return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
}

static const char *stateName(twai_state_t st) {
  switch (st) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

// Queue one 4-byte frame. Timeout 0: if there's no room, give up immediately and report it.
static bool sendFrame(uint32_t id, uint32_t value) {
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = 4;
  putU32(m.data, value);
  return twai_transmit(&m, 0) == ESP_OK;
}

// ---- Sender task: wakes every DATA_PERIOD_MS, sends data, and a heartbeat once a second ----
static void senderTask(void *arg) {
  uint32_t dataCounter = 0, hbCounter = 0, msSinceHb = 0;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DATA_PERIOD_MS));   // fixed rate, no drift

    bool dataOk = sendFrame(DATA_ID, dataCounter);
    if (dataOk) dataCounter++;

    bool hbSent = false, hbOk = false;
    msSinceHb += DATA_PERIOD_MS;
    if (msSinceHb >= HB_PERIOD_MS) {
      msSinceHb = 0;
      hbSent = true;
      hbOk = sendFrame(HB_ID, hbCounter);
      if (hbOk) hbCounter++;
    }

    portENTER_CRITICAL(&cntLock);
    if (dataOk) cnt.sentData++; else cnt.txRejected++;
    if (hbSent) { if (hbOk) cnt.sentHb++; else cnt.txRejected++; }
    portEXIT_CRITICAL(&cntLock);
  }
}

// ---- Receiver task: sleeps until a frame arrives, then sorts and counts it ----
static void receiverTask(void *arg) {
  twai_message_t m;
  bool     haveLast = false;
  uint32_t lastPeerCounter = 0;

  for (;;) {
    if (twai_receive(&m, pdMS_TO_TICKS(100)) != ESP_OK) continue;   // timeout: just wait again

    uint32_t nowMs = millis();
    uint32_t value = (m.data_length_code >= 4) ? getU32(m.data) : 0;

    portENTER_CRITICAL(&cntLock);
    if (m.identifier == PEER_DATA_ID) {
      cnt.rxPeerData++;
      // The peer's counter should go up by exactly 1 each frame. A bigger jump = lost frames.
      // A smaller value means the peer rebooted, so just start tracking again.
      if (haveLast && value > lastPeerCounter + 1) cnt.peerLost += value - lastPeerCounter - 1;
      lastPeerCounter = value;
      haveLast = true;
    } else if (m.identifier == PEER_HB_ID) {
      cnt.rxPeerHb++;
      cnt.lastPeerHbMs = nowMs;
      cnt.peerSeen = true;
    } else {
      cnt.rxOther++;
    }
    portEXIT_CRITICAL(&cntLock);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 10;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("ERROR: TWAI setup failed");
    while (true) delay(1000);
  }

  xTaskCreate(receiverTask, "canRx", 4096, nullptr, 6, nullptr);   // higher priority: never miss a frame
  xTaskCreate(senderTask,   "canTx", 4096, nullptr, 5, nullptr);

  Serial.printf("Node %u running: sends 0x%03lX every %lu ms + heartbeat 0x%03lX, listens for node %u\n",
                NODE_ID, (unsigned long)DATA_ID, (unsigned long)DATA_PERIOD_MS,
                (unsigned long)HB_ID, PEER_ID);
}

void loop() {
  static uint32_t lastReportMs = 0;
  static uint32_t prevRxPeerData = 0;
  static uint32_t prevBusErr = 0;

  // ---- Bus-off recovery: rejoin automatically, but count it so it can't hide ----
  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK) {
    if (s.state == TWAI_STATE_BUS_OFF) {
      portENTER_CRITICAL(&cntLock);
      cnt.busOffCount++;
      portEXIT_CRITICAL(&cntLock);
      twai_initiate_recovery();   // waits for 128 x 11 recessive bits, then the driver goes to STOPPED
    } else if (s.state == TWAI_STATE_STOPPED) {
      twai_start();               // recovery finished: rejoin the bus
    }
  }

  if (millis() - lastReportMs < 1000) { delay(10); return; }
  lastReportMs = millis();

  // ---- Copy the counters under the lock, then print with the lock released ----
  portENTER_CRITICAL(&cntLock);
  Counters c = cnt;
  portEXIT_CRITICAL(&cntLock);

  bool peerAlive = c.peerSeen && (millis() - c.lastPeerHbMs < PEER_TIMEOUT_MS);
  uint32_t rxPerSec = c.rxPeerData - prevRxPeerData;
  prevRxPeerData = c.rxPeerData;
  uint32_t busErrPerSec = s.bus_error_count - prevBusErr;   // errors in the last second
  prevBusErr = s.bus_error_count;

  Serial.printf("[N%u] %-10s TEC=%lu REC=%lu bus_err=%lu (+%lu/s) bus_off=%lu | "
                "sent 0x%03lX=%lu hb=%lu rejected=%lu | "
                "rx 0x%03lX=%lu (%lu/s) hb=%lu lost=%lu other=%lu | N%u %s\n",
                NODE_ID, stateName(s.state),
                (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
                (unsigned long)s.bus_error_count, (unsigned long)busErrPerSec, (unsigned long)c.busOffCount,
                (unsigned long)DATA_ID, (unsigned long)c.sentData, (unsigned long)c.sentHb,
                (unsigned long)c.txRejected,
                (unsigned long)PEER_DATA_ID, (unsigned long)c.rxPeerData, (unsigned long)rxPerSec,
                (unsigned long)c.rxPeerHb, (unsigned long)c.peerLost, (unsigned long)c.rxOther,
                PEER_ID, peerAlive ? "ALIVE" : "NOT HEARD");
}
