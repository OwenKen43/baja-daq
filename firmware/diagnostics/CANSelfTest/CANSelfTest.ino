// CAN self-test: checks ONE node's whole path without needing any other node:
//   ESP32 TX pin -> transceiver -> CANH/CANL -> transceiver -> ESP32 RX pin
// Uses NO_ACK mode + self-reception, so no second node is needed to ACK.
#include <Arduino.h>
#include "driver/twai.h"

// ---- Set these to match THIS board's actual wiring ----
// The test nodes use TX = GPIO5 (XIAO C3 "D3"), RX = GPIO4 (XIAO C3 "D2").
constexpr gpio_num_t TX_PIN = GPIO_NUM_5;   // ESP32 output -> transceiver TX / CTX
constexpr gpio_num_t RX_PIN = GPIO_NUM_4;   // transceiver RX / CRX -> ESP32 input (through divider on 5 V modules)

const char *stateName(twai_state_t st) {
  switch (st) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NO_ACK);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("TWAI init failed");
    while (true) delay(1000);
  }
  Serial.printf("Self-test running: TX=GPIO%d RX=GPIO%d\n", (int)TX_PIN, (int)RX_PIN);
}

void loop() {
  static uint8_t  n = 0;
  static uint32_t ok = 0, bad = 0;

  // 1) Send one frame, and ask the controller to also put it in our own RX queue.
  twai_message_t tx = {};
  tx.identifier = 0x123;
  tx.self = 1;                 // self-reception request
  tx.data_length_code = 1;
  tx.data[0] = n;
  esp_err_t txErr = twai_transmit(&tx, pdMS_TO_TICKS(50));

  // 2) Wait up to 50 ms for it to come back in through the transceiver.
  twai_message_t rx;
  esp_err_t rxErr = twai_receive(&rx, pdMS_TO_TICKS(50));
  bool good = (rxErr == ESP_OK && rx.identifier == 0x123 && rx.data[0] == n);
  if (good) ok++; else bad++;

  // 3) Report.
  twai_status_info_t s;
  twai_get_status_info(&s);
  Serial.printf("%s  tx=%s rx=%s  ok=%lu bad=%lu  state=%s TEC=%lu REC=%lu bus_err=%lu\n",
                good ? "PASS" : "FAIL",
                esp_err_to_name(txErr), esp_err_to_name(rxErr),
                (unsigned long)ok, (unsigned long)bad,
                stateName(s.state),
                (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
                (unsigned long)s.bus_error_count);
  n++;
  delay(500);
}
