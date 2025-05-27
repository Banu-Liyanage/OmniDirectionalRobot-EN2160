// --------- RS485 wiring ----------
#include <Arduino.h>
HardwareSerial &rs485 = Serial1; // TX1/RX1
const uint8_t DE_PIN = 2;        // Direction pin (DE+RE)

void setup()
{
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  rs485.begin(1000000); // 1 Mbit/s; drop to 500000 if needed
}

// ---------- CRC-16 (Modbus) -----------
uint16_t crc16(const uint8_t *buf, size_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--)
  {
    crc ^= *buf++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// ---------- Send command --------------
void sendSpeed(int16_t rpm[4], uint8_t seq)
{
  uint8_t frame[14];
  frame[0] = 0xAA;
  frame[1] = 0x01;
  frame[2] = 9; // payload bytes
  memcpy(&frame[3], rpm, 8);
  frame[11] = seq;
  uint16_t crc = crc16(&frame[1], 11);
  frame[12] = crc & 0xFF;
  frame[13] = 0x55;

  digitalWrite(DE_PIN, HIGH);
  rs485.write(frame, 14);
  rs485.flush();
  digitalWrite(DE_PIN, LOW);
}

// ---------- Example loop -------------
int16_t target[4] = {1500, 1500, 1500, 1500};
uint8_t seq = 0;

void loop()
{
  sendSpeed(target, seq++);
  // Optionally readTelemetry() here (state-machine parsing omitted for brevity)
  delayMicroseconds(1000); // 1 kHz loop
}
