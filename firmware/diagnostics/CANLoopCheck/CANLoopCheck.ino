// CAN loop check WITHOUT the CAN controller.
// Drives the transceiver's TX input by hand and reads its RX output back,
// so each piece of the path can be checked with a multimeter.
//
//   D3 (GPIO5) --> transceiver TX/CTX --> CANH/CANL --> transceiver RX/CRX --> (divider) --> D2 (GPIO4)
#include <Arduino.h>

constexpr int TX_PIN = 5;   // XIAO C3 D3 -> transceiver TX / CTX
constexpr int RX_PIN = 4;   // XIAO C3 D2 <- transceiver RX / CRX (through divider on the TJA1050)

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  digitalWrite(TX_PIN, HIGH);   // HIGH = recessive (transceiver lets go of the bus)
  Serial.println("Loop check running: D3 drives TX, D2 reads RX");
}

void loop() {
  // ---- Phase A: one quick logic check ----
  digitalWrite(TX_PIN, HIGH);          // recessive
  delayMicroseconds(50);
  int rxWhenTxHigh = digitalRead(RX_PIN);   // expect 1

  digitalWrite(TX_PIN, LOW);           // dominant, kept short (well under the transceiver's TXD time-out)
  delayMicroseconds(20);
  int rxWhenTxLow = digitalRead(RX_PIN);    // expect 0
  digitalWrite(TX_PIN, HIGH);

  bool ok = (rxWhenTxHigh == 1 && rxWhenTxLow == 0);
  Serial.printf("TX high -> RX=%d   TX low -> RX=%d   %s\n",
                rxWhenTxHigh, rxWhenTxLow, ok ? "LOOP OK" : "LOOP BROKEN");

  // ---- Phase B: ~1 s of 50% square wave so a multimeter shows average levels ----
  uint32_t start = millis();
  while (millis() - start < 1000) {
    digitalWrite(TX_PIN, LOW);  delayMicroseconds(50);
    digitalWrite(TX_PIN, HIGH); delayMicroseconds(50);
  }
}
