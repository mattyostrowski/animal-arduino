#include "DFRobot_C4001.h"
#include <SPI.h>
#include <Ethernet.h>

#define SENSOR_A_SERIAL Serial2
#define SENSOR_B_SERIAL Serial3

DFRobot_C4001_UART radarA(&SENSOR_A_SERIAL, 9600);
DFRobot_C4001_UART radarB(&SENSOR_B_SERIAL, 9600);

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAF, 0x05, 0xA3 };
IPAddress ip(192, 168, 2, 2);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

IPAddress remoteIP(192, 168, 2, 1);
const uint16_t remotePort = 54321;

EthernetClient tcpClient;

int prevStableA = 0;
int lastReadA = 0;
unsigned long stateChangeMillisA = 0;

int prevStableB = 0;
int lastReadB = 0;
unsigned long stateChangeMillisB = 0;

const unsigned long stableDelay = 1000; // 1s

bool radarA_ok = false;
bool radarB_ok = false;

const uint8_t pinsCount = 8;
const uint8_t inputPins[pinsCount] = {2,3,4,5,6,7,8,9};
const unsigned long debounceMs = 50;

uint8_t stableState[pinsCount];
uint8_t lastRead[pinsCount];
unsigned long lastChangeTime[pinsCount];

bool sentInitialPins = false;
const unsigned long startupDelayMs = 2000;

// recheck every 30 seconds
const unsigned long recheckIntervalMs = 30000;
unsigned long lastRecheckMillis = 0;

bool beginAndValidateRadar(DFRobot_C4001_UART &radar, const char *name);
bool ensureTcpConnected(unsigned long timeoutMs = 500);
void sendSensorStateA(int aState);
void sendSensorStateB(int bState);
void sendPinStateTCP(uint8_t pinIndex, int stateHighLow);

void setup() {
  Serial.begin(115200);
  Serial.println("setup start");

  SENSOR_A_SERIAL.begin(9600);
  SENSOR_B_SERIAL.begin(9600);

  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  delay(300);
  Serial.print("IP ");
  Serial.println(Ethernet.localIP());

  radarA_ok = beginAndValidateRadar(radarA, "A");
  radarB_ok = beginAndValidateRadar(radarB, "B");

  if (radarA_ok) {
    radarA.setSensorMode(eSpeedMode);
    radarA.setDetectThres(11, 1200, 10);
    radarA.setFrettingDetection(eON);
  }

  if (radarB_ok) {
    radarB.setSensorMode(eSpeedMode);
    radarB.setDetectThres(11, 1200, 10);
    radarB.setFrettingDetection(eON);
  }

  for (uint8_t i = 0; i < pinsCount; ++i) {
    pinMode(inputPins[i], INPUT_PULLUP);
    lastRead[i] = digitalRead(inputPins[i]);
    stableState[i] = lastRead[i];
    lastChangeTime[i] = millis();
  }

  Serial.println("setup complete");
  lastRecheckMillis = millis();
}

void loop() {
  unsigned long now = millis();

  if (recheckIntervalMs > 0 && (now - lastRecheckMillis) >= recheckIntervalMs) {
    lastRecheckMillis = now;
    if (!radarA_ok) {
      radarA_ok = beginAndValidateRadar(radarA, "A");
      Serial.print("Radar A: ");
      Serial.println(radarA_ok ? "found" : "not found");
      if (radarA_ok) {
        radarA.setSensorMode(eSpeedMode);
        radarA.setDetectThres(11, 1200, 10);
        radarA.setFrettingDetection(eON);
      }
    }
    if (!radarB_ok) {
      radarB_ok = beginAndValidateRadar(radarB, "B");
      Serial.print("Radar B: ");
      Serial.println(radarB_ok ? "found" : "not found");
      if (radarB_ok) {
        radarB.setSensorMode(eSpeedMode);
        radarB.setDetectThres(11, 1200, 10);
        radarB.setFrettingDetection(eON);
      }
    }
  }

  int currentA = 0;
  if (radarA_ok) {
    int tnumA = radarA.getTargetNumber();
    currentA = (tnumA == 0) ? 0 : 1;
  }
  if (currentA != lastReadA) {
    stateChangeMillisA = now;
    lastReadA = currentA;
  }
  if (currentA != prevStableA && (now - stateChangeMillisA) >= stableDelay) {
    prevStableA = currentA;
    sendSensorStateA(prevStableA);
  }

  int currentB = 0;
  if (radarB_ok) {
    int tnumB = radarB.getTargetNumber();
    currentB = (tnumB == 0) ? 0 : 1;
  }
  if (currentB != lastReadB) {
    stateChangeMillisB = now;
    lastReadB = currentB;
  }
  if (currentB != prevStableB && (now - stateChangeMillisB) >= stableDelay) {
    prevStableB = currentB;
    sendSensorStateB(prevStableB);
  }

  for (uint8_t i = 0; i < pinsCount; ++i) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastRead[i]) {
      lastChangeTime[i] = now;
      lastRead[i] = reading;
    } else if ((now - lastChangeTime[i]) >= debounceMs && reading != stableState[i]) {
      stableState[i] = reading;
      sendPinStateTCP(i, stableState[i]);
    }
  }

  if (!sentInitialPins && (now >= startupDelayMs)) {
    sentInitialPins = true;
    for (uint8_t i = 0; i < pinsCount; ++i) {
      sendPinStateTCP(i, stableState[i]);
      delay(10);
    }
  }

  delay(10);
}

bool beginAndValidateRadar(DFRobot_C4001_UART &radar, const char *name) {
  bool began = radar.begin();
  sSensorStatus_t st = radar.getStatus();
  int tnum = radar.getTargetNumber();

  bool statusAllZero = (st.workStatus == 0 && st.workMode == 0 && st.initStatus == 0);
  if (statusAllZero && tnum == 0) return false;

  if (!began) return false;
  if (st.initStatus < 0 || st.initStatus > 10) return false;
  if (tnum < -1 || tnum > 10) return false;
  return true;
}

bool ensureTcpConnected(unsigned long timeoutMs) {
  if (tcpClient.connected()) return true;
  tcpClient.stop();
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (tcpClient.connect(remoteIP, remotePort)) return true;
    delay(50);
  }
  return tcpClient.connected();
}

void sendSensorStateA(int aState) {
  const uint8_t headerA = 100;
  if (!ensureTcpConnected()) return;
  uint8_t bufA[2] = { headerA, (uint8_t)(aState ? 1 : 0) };
  tcpClient.write(bufA, 2);
  tcpClient.flush();
}

void sendSensorStateB(int bState) {
  const uint8_t headerB = 101;
  if (!ensureTcpConnected()) return;
  uint8_t bufB[2] = { headerB, (uint8_t)(bState ? 1 : 0) };
  tcpClient.write(bufB, 2);
  tcpClient.flush();
}

void sendPinStateTCP(uint8_t pinIndex, int stateHighLow) {
  uint8_t value = (stateHighLow == HIGH) ? 0 : 1;
  uint8_t packet[2] = { pinIndex, value };
  if (!ensureTcpConnected()) return;
  tcpClient.write(packet, 2);
  tcpClient.flush();
}
