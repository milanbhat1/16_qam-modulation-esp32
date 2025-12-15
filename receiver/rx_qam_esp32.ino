#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int qamLevels[4] = { -3, -1, 1, 3 };
const int MAX_SYMBOLS_PER_PACKET = 4;

typedef struct {
  uint16_t packetId;
  uint8_t  numSymbols;
  int8_t   I[MAX_SYMBOLS_PER_PACKET];
  int8_t   Q[MAX_SYMBOLS_PER_PACKET];
} QAMPacket;

// ================= QAM HELPERS =================

int8_t nearestLevel(int8_t v)
{
  if (v < -2) return -3;
  else if (v < 0) return -1;
  else if (v < 2) return 1;
  else return 3;
}

void qamToBits(int8_t I, int8_t Q,
               uint8_t &b1, uint8_t &b2,
               uint8_t &b3, uint8_t &b4)
{
  int iIndex = (nearestLevel(I) + 3) / 2;
  int qIndex = (nearestLevel(Q) + 3) / 2;

  b1 = (iIndex >> 1) & 1;
  b2 = iIndex & 1;
  b3 = (qIndex >> 1) & 1;
  b4 = qIndex & 1;
}

// ================= RECEIVE CALLBACK =================

void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len)
{
  QAMPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));

  uint16_t rxWord = 0;

  for (int s = 0; s < 4; s++) {
    uint8_t b1, b2, b3, b4;
    qamToBits(pkt.I[s], pkt.Q[s], b1, b2, b3, b4);

    rxWord = (rxWord << 1) | b1;
    rxWord = (rxWord << 1) | b2;
    rxWord = (rxWord << 1) | b3;
    rxWord = (rxWord << 1) | b4;
  }

  Serial.print("[RX] Packet ");
  Serial.print(pkt.packetId);
  Serial.print(" | Data = 0x");
  Serial.println(rxWord, HEX);
}

// ================= SETUP =================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  Serial.print("[RX] MAC: ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[RX] Ready");
}

void loop()
{
  delay(1000);
}
