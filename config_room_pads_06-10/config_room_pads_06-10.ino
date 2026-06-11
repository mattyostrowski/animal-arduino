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
void setupRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg);
void printStatus();
bool ensureTcpConnected();
void sendSensorStateA(int aState);
void sendSensorStateB(int bState);
void sendPinStateTCP(uint8_t pinIndex, int stateHighLow);

DFRobot_C4001_UART radarA(&SENSOR_A_SERIAL, 9600);
DFRobot_C4001_UART radarB(&SENSOR_B_SERIAL, 9600);

RadarConfig cfgA = {11, 1200, 10, 1, 2};
RadarConfig cfgB = {31, 1234, 1000, 1, 2};


byte mac[] = { 0xA8, 0x61, 0x0A, 0xAF, 0x05, 0xA3 };
IPAddress ip(192, 168, 0, 12);             // device static IP - change if needed
IPAddress gateway(192, 168, 0, 11);        // remote gateway (your target device IP here)
IPAddress subnet(255, 255, 255, 0);
IPAddress remoteIP(192, 168, 0, 11);       // this computer
const uint16_t remotePort = 54321;        

EthernetClient tcpClient;

int prevStableA = 0, lastReadA = 0;
int prevStableB = 0, lastReadB = 0;
unsigned long stateChangeMillisA = 0, stateChangeMillisB = 0;
const unsigned long stableDelay = 1000;

const uint8_t pinsCount = 8;
const uint8_t inputPins[pinsCount] = {2, 3, 4, 5, 6, 7, 8, 9};
const unsigned long debounceMs = 50;

uint8_t stableState[pinsCount];
uint8_t lastRead[pinsCount];
unsigned long lastChangeTime[pinsCount];

bool sentInitialPins = false;
const unsigned long startupDelayMs = 2000;

bool ensureTcpConnected() {
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

void printStatus() {
  int tnumA = radarA.getTargetNumber();
  int tnumB = radarB.getTargetNumber();

  Serial.print("A targets="); Serial.print(tnumA);
  Serial.print(" state="); Serial.println(tnumA > 0 ? "PRESENT" : "ABSENT");

  Serial.print("B targets="); Serial.print(tnumB);
  Serial.print(" state="); Serial.println(tnumB > 0 ? "PRESENT" : "ABSENT");
  Serial.println();
}

void sendSensorStateA(int aState) {
  const uint8_t headerA = 100;

  if (!ensureTcpConnected()) {
    Serial.println("TCP connect failed (A)");
    return;
  }

  uint8_t bufA[2] = { headerA, (uint8_t)(aState ? 1 : 0) };
  if (tcpClient.write(bufA, 2) != 2) {
    Serial.println("Failed to send sensor A");
    return;
  }

  tcpClient.flush();
  Serial.print("Sent sensor A: ");
  Serial.println(aState ? "present" : "absent");
}

void sendSensorStateB(int bState) {
  const uint8_t headerB = 101;

  if (!ensureTcpConnected()) {
    Serial.println("TCP connect failed (B)");
    return;
  }

  uint8_t bufB[2] = { headerB, (uint8_t)(bState ? 1 : 0) };
  if (tcpClient.write(bufB, 2) != 2) {
    Serial.println("Failed to send sensor B");
    return;
  }

  tcpClient.flush();
  Serial.print("Sent sensor B: ");
  Serial.println(bState ? "present" : "absent");
}

void sendPinStateTCP(uint8_t pinIndex, int stateHighLow) {
  uint8_t value = (stateHighLow == HIGH) ? 0 : 1;
  uint8_t packet[2] = { pinIndex, value };

  if (!ensureTcpConnected()) {
    Serial.print("TCP connect failed for pin index ");
    Serial.println(pinIndex);
    return;
  }

  if (tcpClient.write(packet, 2) != 2) {
    Serial.print("Failed to send pin index ");
    Serial.println(pinIndex);
    return;
  }

  tcpClient.flush();
  Serial.print("Sent pin index ");
  Serial.print(pinIndex);
  Serial.print(": ");
  Serial.println(value ? "1 (pressed)" : "0 (open)");
}

void setupRadar(DFRobot_C4001_UART &radar, const char *name, const RadarConfig &cfg) {
  while (!radar.begin()) {
    Serial.print("NO Device ");
    Serial.println(name);
    delay(1000);
  }

  Serial.print("Device ");
  Serial.print(name);
  Serial.println(" connected!");

  applyConfig(&radar, cfg);
  printReadback(name, &radar, cfg);
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
  Serial.println("  A/B status");
  Serial.println("  A/B setRange min max thres");
  Serial.println("  A/B setTrig n");
  Serial.println("  A/B setKeep n");
  Serial.println("  A/B setDelay trig_s keep_s");
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

  setupRadar(radarA, "Sensor A", cfgA);
  setupRadar(radarB, "Sensor B", cfgB);

  lastReadA = prevStableA = (radarA.getTargetNumber() > 0) ? 1 : 0;
  lastReadB = prevStableB = (radarB.getTargetNumber() > 0) ? 1 : 0;
  stateChangeMillisA = stateChangeMillisB = millis();

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

  int tnumA = radarA.getTargetNumber();
  int currentA = (tnumA > 0) ? 1 : 0;
  if (currentA != lastReadA) {
    stateChangeMillisA = nowLoop;
    lastReadA = currentA;
  }

  int tnumB = radarB.getTargetNumber();
  int currentB = (tnumB > 0) ? 1 : 0;
  if (currentB != lastReadB) {
    stateChangeMillisB = nowLoop;
    lastReadB = currentB;
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      if (line == "status") {
        printStatus();
      } else {
        handleCmd(line);
      }
    }
  }

if (currentA != prevStableA && (nowLoop - stateChangeMillisA) >= stableDelay) {
  prevStableA = currentA;
  sendSensorStateA(prevStableA);
}

if (currentB != prevStableB && (nowLoop - stateChangeMillisB) >= stableDelay) {
  prevStableB = currentB;
  sendSensorStateB(prevStableB);
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
