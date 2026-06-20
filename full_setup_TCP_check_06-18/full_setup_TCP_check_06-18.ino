#include "DFRobot_C4001.h"
#include <SPI.h>
#include <Ethernet.h>
#include <math.h>

#define SENSOR_A_SERIAL Serial2
#define SENSOR_B_SERIAL Serial3

struct RadarConfig {
  uint16_t minR, maxR, thresR;
  uint8_t trigSens, keepSens;
};

void applyConfig(DFRobot_C4001_UART *r, const RadarConfig &c);
void printReadback(const char *name, DFRobot_C4001_UART *r, const RadarConfig &c);
void printCommands();
void handleCmd(String s);
bool setupRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg);
bool tryReconnectRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg);
void printStatus();
bool ensureTcpConnected();
bool writeTcpPacket(const uint8_t *buf, size_t len);
void sendSensorStateA(int aState);
void sendSensorStateB(int bState);
void sendPinStateTCP(uint8_t pinIndex, int stateHighLow);
void sendAllCurrentStates();

DFRobot_C4001_UART radarA(&SENSOR_A_SERIAL, 9600);
DFRobot_C4001_UART radarB(&SENSOR_B_SERIAL, 9600);

RadarConfig cfgA = {30, 1000, 5, 1, 2};    //initial sensor ranges
RadarConfig cfgB = {100, 750, 15, 1, 2};

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAF, 0x05, 0xA3 };
IPAddress ip(192, 168, 0, 12);
IPAddress gateway(192, 168, 0, 11);
IPAddress subnet(255, 255, 255, 0);
IPAddress remoteIP(192, 168, 0, 11);
const uint16_t remotePort = 54321;

EthernetClient tcpClient;

bool radarAConnected = false;
bool radarBConnected = false;
const unsigned long radarSetupTimeoutMs = 5000;
const unsigned long radarReconnectIntervalMs = 5000;
unsigned long lastReconnectAttemptA = 0;
unsigned long lastReconnectAttemptB = 0;

int prevStableA = 0, lastReadA = 0;
int prevStableB = 0, lastReadB = 0;
unsigned long stateChangeMillisA = 0, stateChangeMillisB = 0;
unsigned long radarOnDebounceMs = 250;
unsigned long radarOffDebounceMs = 480000;

const uint8_t pinsCount = 8;
const uint8_t inputPins[pinsCount] = {2, 3, 4, 5, 6, 7, 8, 9};
const unsigned long debounceMs = 50;

uint8_t stableState[pinsCount];
uint8_t lastRead[pinsCount];
unsigned long lastChangeTime[pinsCount];

bool sentInitialPins = false;
const unsigned long startupDelayMs = 1000;

bool tcpWasConnected = false;
unsigned long lastTcpReconnectCheck = 0;
const unsigned long tcpReconnectCheckMs = 10000;

bool ensureTcpConnected() {
  if (Ethernet.linkStatus() == LinkOFF) {
    tcpClient.stop();
    return false;
  }

  if (tcpClient.connected()) {
    return true;
  }

  tcpClient.stop();
  if (!tcpClient.connect(remoteIP, remotePort)) {
    return false;
  }

  Serial.println("TCP connected");
  return true;
}

bool writeTcpPacket(const uint8_t *buf, size_t len) {
  if (!ensureTcpConnected()) {
    return false;
  }

  if (tcpClient.write(buf, len) == (int)len) {
    tcpClient.flush();
    return true;
  }

  tcpClient.stop();
  delay(50);

  if (!ensureTcpConnected()) {
    return false;
  }

  if (tcpClient.write(buf, len) == (int)len) {
    tcpClient.flush();
    return true;
  }

  tcpClient.stop();
  return false;
}

void printStatus() {
  if (radarAConnected) {
    int tnumA = radarA.getTargetNumber();
    Serial.print("A targets="); Serial.print(tnumA);
    Serial.print(" state="); Serial.println(tnumA > 0 ? "PRESENT" : "ABSENT");
  } else {
    Serial.println("A not connected");
  }

  if (radarBConnected) {
    int tnumB = radarB.getTargetNumber();
    Serial.print("B targets="); Serial.print(tnumB);
    Serial.print(" state="); Serial.println(tnumB > 0 ? "PRESENT" : "ABSENT");
  } else {
    Serial.println("B not connected");
  }

  Serial.print("Radar ON debounce ms="); Serial.println(radarOnDebounceMs);
  Serial.print("Radar OFF debounce ms="); Serial.println(radarOffDebounceMs);
  Serial.println();
}

void sendSensorStateA(int aState) {
  const uint8_t headerA = 100;
  uint8_t bufA[2] = { headerA, (uint8_t)(aState ? 1 : 0) };

  if (!writeTcpPacket(bufA, 2)) {
    Serial.println("Failed to send sensor A");
    return;
  }

  Serial.print("Sent sensor A: ");
  Serial.println(aState ? "present" : "absent");
}

void sendSensorStateB(int bState) {
  const uint8_t headerB = 101;
  uint8_t bufB[2] = { headerB, (uint8_t)(bState ? 1 : 0) };

  if (!writeTcpPacket(bufB, 2)) {
    Serial.println("Failed to send sensor B");
    return;
  }

  Serial.print("Sent sensor B: ");
  Serial.println(bState ? "present" : "absent");
}

void sendPinStateTCP(uint8_t pinIndex, int stateHighLow) {
  uint8_t value = (stateHighLow == HIGH) ? 0 : 1;
  uint8_t packet[2] = { pinIndex, value };

  if (!writeTcpPacket(packet, 2)) {
    Serial.print("Failed to send pin index ");
    Serial.println(pinIndex);
    return;
  }

  Serial.print("Sent pin index ");
  Serial.print(pinIndex);
  Serial.print(": ");
  Serial.println(value ? "1 (pressed)" : "0 (open)");
}

void sendAllCurrentStates() {
  if (radarAConnected) sendSensorStateA(prevStableA);
  if (radarBConnected) sendSensorStateB(prevStableB);

  for (uint8_t i = 0; i < pinsCount; ++i) {
    sendPinStateTCP(i, stableState[i]);
    delay(10);
  }
}

bool setupRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg) {
  unsigned long start = millis();

  while (!radar.begin()) {
    Serial.print("NO Device ");
    Serial.println(name);

    if (millis() - start >= radarSetupTimeoutMs) {
      Serial.print("Timeout waiting for ");
      Serial.println(name);
      return false;
    }

    delay(1000);
  }

  Serial.print("Device ");
  Serial.print(name);
  Serial.println(" connected!");

  applyConfig(&radar, cfg);
  printReadback(name, &radar, cfg);
  return true;
}

bool tryReconnectRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg) {
  if (!radar.begin()) {
    return false;
  }

  Serial.print("Reconnected ");
  Serial.println(name);

  applyConfig(&radar, cfg);
  printReadback(name, &radar, cfg);
  return true;
}

void applyConfig(DFRobot_C4001_UART *r, const RadarConfig &c) {
  r->setSensor(eStopSen); delay(500);
  r->setSensorMode(eSpeedMode); delay(100);
  r->setDetectThres(c.minR, c.maxR, c.thresR); delay(100);
  r->setTrigSensitivity(c.trigSens); delay(100);
  r->setKeepSensitivity(c.keepSens); delay(100);
  r->setFrettingDetection(eON); delay(100);
  r->setSensor(eStartSen); delay(500);
}

void printReadback(const char *name, DFRobot_C4001_UART *r, const RadarConfig &c) {
  sSensorStatus_t st = r->getStatus();
  int targets = r->getTargetNumber();
  int present = (targets > 0) ? 1 : 0;

  Serial.print(name); Serial.println(":");
  Serial.print("  state="); Serial.println(present ? "PRESENT" : "ABSENT");
  Serial.print("  targets="); Serial.println(targets);

  Serial.print("  workStatus(raw)="); Serial.print(st.workStatus);
  Serial.print(" workMode(raw)="); Serial.print(st.workMode);
  Serial.print(" init(raw)="); Serial.println(st.initStatus);

  Serial.print("  cfg min="); Serial.print(c.minR);
  Serial.print(" max="); Serial.print(c.maxR);
  Serial.print(" thres="); Serial.println(c.thresR);

  Serial.print("  dev min="); Serial.print(r->getTMinRange());
  Serial.print(" max="); Serial.print(r->getTMaxRange());
  Serial.print(" thres="); Serial.println(r->getThresRange());

  Serial.print("  cfg trigSens="); Serial.print(c.trigSens);
  Serial.print(" keepSens="); Serial.println(c.keepSens);

  Serial.print("  dev trigSens="); Serial.print(r->getTrigSensitivity());
  Serial.print(" keepSens="); Serial.println(r->getKeepSensitivity());

  Serial.print("  onDebounceMs="); Serial.println(radarOnDebounceMs);
  Serial.print("  offDebounceMs="); Serial.println(radarOffDebounceMs);

  Serial.println();
}

void handleCmd(String s) {
  s.trim();
  Serial.print("CMD: [");
  Serial.print(s);
  Serial.println("]");

  if (s.length() == 0) {
    printCommands();
    return;
  }

  String lower = s;
  lower.toLowerCase();

  if (lower == "help" || lower == "commands" || lower == "?") {
    printCommands();
    return;
  }

  if (lower == "status") {
    printStatus();
    return;
  }

  if (lower.startsWith("radarondebounce ")) {
    unsigned long v = (unsigned long) max(0, lower.substring(16).toInt());
    radarOnDebounceMs = v;
    Serial.print("Radar ON debounce set to ");
    Serial.print(radarOnDebounceMs);
    Serial.println(" ms");
    return;
  }

  if (lower.startsWith("radaroffdebounce ")) {
    unsigned long v = (unsigned long) max(0, lower.substring(17).toInt());
    radarOffDebounceMs = v;
    Serial.print("Radar OFF debounce set to ");
    Serial.print(radarOffDebounceMs);
    Serial.println(" ms");
    return;
  }

  if (s.length() < 2) {
    Serial.println("Ignored: too short");
    printCommands();
    return;
  }

  char id = toupper(s.charAt(0));
  if (id != 'A' && id != 'B') {
    Serial.println("Bad sensor id. Use A or B");
    printCommands();
    return;
  }

  DFRobot_C4001_UART *r = (id == 'A') ? &radarA : &radarB;
  RadarConfig *c = (id == 'A') ? &cfgA : &cfgB;
  const char *name = (id == 'A') ? "Sensor A" : "Sensor B";
  bool connected = (id == 'A') ? radarAConnected : radarBConnected;

  if (!connected) {
    Serial.print(name);
    Serial.println(" not connected");
    return;
  }

  s = s.substring(1);
  s.trim();

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String args = (sp < 0) ? "" : s.substring(sp + 1);

  cmd.toLowerCase();
  args.trim();

  Serial.print("Sensor: "); Serial.println(name);
  Serial.print("Command: "); Serial.println(cmd);
  Serial.print("Args: "); Serial.println(args);

  if (cmd == "status") {
    printReadback(name, r, *c);
    return;
  }

  if (cmd == "setrange") {
    int a = 0, b = 0, d = 0;
    if (sscanf(args.c_str(), "%d %d %d", &a, &b, &d) == 3) {
      c->minR = a;
      c->maxR = b;
      c->thresR = d;
      applyConfig(r, *c);
      Serial.println("setRange applied");
      printReadback(name, r, *c);
    } else {
      Serial.println("Bad setRange. Use: A setRange min max thres");
      printCommands();
    }
    return;
  }

  if (cmd == "settrig") {
    c->trigSens = (uint8_t)constrain(args.toInt(), 0, 9);
    applyConfig(r, *c);
    Serial.println("setTrig applied");
    printReadback(name, r, *c);
    return;
  }

  if (cmd == "setkeep") {
    c->keepSens = (uint8_t)constrain(args.toInt(), 0, 9);
    applyConfig(r, *c);
    Serial.println("setKeep applied");
    printReadback(name, r, *c);
    return;
  }

  Serial.println("Unknown command");
  printCommands();
}

void printCommands() {
  Serial.println("Commands:");
  Serial.println("  status");
  Serial.println("  radarOnDebounce ms");
  Serial.println("  radarOffDebounce ms");
  Serial.println("  A/B status");
  Serial.println("  A/B setRange min max thres");
  Serial.println("  A/B setTrig n");
  Serial.println("  A/B setKeep n");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {}

  SENSOR_A_SERIAL.begin(9600);
  SENSOR_B_SERIAL.begin(9600);

  Serial.println("Initializing Ethernet...");
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  delay(500);
  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());

  tcpWasConnected = tcpClient.connected();
  lastTcpReconnectCheck = millis();

  radarAConnected = setupRadar(radarA, "Sensor A", cfgA);
  radarBConnected = setupRadar(radarB, "Sensor B", cfgB);

  lastReadA = prevStableA = radarAConnected ? ((radarA.getTargetNumber() > 0) ? 1 : 0) : 0;
  lastReadB = prevStableB = radarBConnected ? ((radarB.getTargetNumber() > 0) ? 1 : 0) : 0;
  stateChangeMillisA = stateChangeMillisB = millis();

  if (radarAConnected) sendSensorStateA(prevStableA);
  if (radarBConnected) sendSensorStateB(prevStableB);

  for (uint8_t i = 0; i < pinsCount; ++i) {
    pinMode(inputPins[i], INPUT_PULLUP);
    lastRead[i] = digitalRead(inputPins[i]);
    stableState[i] = lastRead[i];
    lastChangeTime[i] = millis();

    Serial.print("Pin ");
    Serial.print(inputPins[i]);
    Serial.print(": ");
    Serial.println(stableState[i] == HIGH ? "HIGH (open)" : "LOW (pressed)");
  }

  printStatus();
  Serial.println("Both devices ready!");
  printCommands();
}

void loop() {
  unsigned long nowLoop = millis();

  if ((nowLoop - lastTcpReconnectCheck) >= tcpReconnectCheckMs) {
    lastTcpReconnectCheck = nowLoop;

    bool tcpNowConnected = tcpClient.connected();

    if (!tcpNowConnected) {
      tcpClient.stop();
      if (ensureTcpConnected()) {
        sendAllCurrentStates();
        tcpWasConnected = true;
      } else {
        tcpWasConnected = false;
      }
    } else if (!tcpWasConnected) {
      sendAllCurrentStates();
      tcpWasConnected = true;
    }
  }

  if (!radarAConnected && (nowLoop - lastReconnectAttemptA) >= radarReconnectIntervalMs) {
    lastReconnectAttemptA = nowLoop;
    radarAConnected = tryReconnectRadar(radarA, "Sensor A", cfgA);
    if (radarAConnected) {
      lastReadA = prevStableA = (radarA.getTargetNumber() > 0) ? 1 : 0;
      stateChangeMillisA = nowLoop;
      sendSensorStateA(prevStableA);
    }
  }

  if (!radarBConnected && (nowLoop - lastReconnectAttemptB) >= radarReconnectIntervalMs) {
    lastReconnectAttemptB = nowLoop;
    radarBConnected = tryReconnectRadar(radarB, "Sensor B", cfgB);
    if (radarBConnected) {
      lastReadB = prevStableB = (radarB.getTargetNumber() > 0) ? 1 : 0;
      stateChangeMillisB = nowLoop;
      sendSensorStateB(prevStableB);
    }
  }

  int currentA = prevStableA;
  if (radarAConnected) {
    int tnumA = radarA.getTargetNumber();
    currentA = (tnumA > 0) ? 1 : 0;
    if (currentA != lastReadA) {
      stateChangeMillisA = nowLoop;
      lastReadA = currentA;
    }
  }

  int currentB = prevStableB;
  if (radarBConnected) {
    int tnumB = radarB.getTargetNumber();
    currentB = (tnumB > 0) ? 1 : 0;
    if (currentB != lastReadB) {
      stateChangeMillisB = nowLoop;
      lastReadB = currentB;
    }
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      handleCmd(line);
    }
  }

  if (radarAConnected && currentA != prevStableA) {
    unsigned long requiredA = currentA ? radarOnDebounceMs : radarOffDebounceMs;
    if ((nowLoop - stateChangeMillisA) >= requiredA) {
      prevStableA = currentA;
      sendSensorStateA(prevStableA);
    }
  }

  if (radarBConnected && currentB != prevStableB) {
    unsigned long requiredB = currentB ? radarOnDebounceMs : radarOffDebounceMs;
    if ((nowLoop - stateChangeMillisB) >= requiredB) {
      prevStableB = currentB;
      sendSensorStateB(prevStableB);
    }
  }

  for (uint8_t i = 0; i < pinsCount; ++i) {
    int reading = digitalRead(inputPins[i]);

    if (reading != lastRead[i]) {
      lastChangeTime[i] = nowLoop;
      lastRead[i] = reading;
    } else if ((nowLoop - lastChangeTime[i]) >= debounceMs && reading != stableState[i]) {
      stableState[i] = reading;

      Serial.print("Pin ");
      Serial.print(inputPins[i]);
      Serial.print(" -> ");
      Serial.println(stableState[i] == HIGH ? "HIGH (open)" : "LOW (pressed)");

      sendPinStateTCP(i, stableState[i]);
    }
  }

  if (!sentInitialPins && nowLoop >= startupDelayMs) {
    sentInitialPins = true;
    for (uint8_t i = 0; i < pinsCount; ++i) {
      Serial.print("Initial send pin ");
      Serial.print(inputPins[i]);
      Serial.print(": ");
      Serial.println(stableState[i] == HIGH ? "HIGH (open)" : "LOW (pressed)");
      sendPinStateTCP(i, stableState[i]);
      delay(10);
    }
  }

  delay(100);
}
