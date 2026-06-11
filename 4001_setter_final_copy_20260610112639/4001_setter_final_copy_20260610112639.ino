#include "DFRobot_C4001.h"
#include <math.h>

#define SENSOR_A_SERIAL Serial2
#define SENSOR_B_SERIAL Serial3

DFRobot_C4001_UART radarA(&SENSOR_A_SERIAL, 9600);
DFRobot_C4001_UART radarB(&SENSOR_B_SERIAL, 9600);

const unsigned long POLL_MS = 150;

struct Params {
  uint16_t minRange = 30;
  uint16_t maxRange = 2000;
  uint16_t thresRange = 1000;
  uint8_t trigSens = 1;
  uint8_t keepSens = 2;
  uint16_t trigDelay = 5;
  uint16_t keepDelay = 4;
  uint8_t pwm1 = 50;
  uint8_t pwm2 = 0;
  uint16_t pwmTimer = 10;
  uint8_t ioPol = 1;
};

Params paramsA, paramsB;

void applyParams(DFRobot_C4001_UART *r, const Params &p);
void initRoutine(DFRobot_C4001_UART *r, const char *name, Params &p);
void handleCommand(String line);
void pollSensors();
void handleCommandLineForSensor(DFRobot_C4001_UART *r, Params &p, const String &cmd, const String &args);
void printStartupSummary();

unsigned long lastPoll = 0;

sSensorStatus_t lastStatusA = {255, 255, 255};
sSensorStatus_t lastStatusB = {255, 255, 255};

int8_t lastPresenceA = -1;
int8_t lastPresenceB = -1;

void printSensorReadback(DFRobot_C4001_UART *r, const char *name) {
  Serial.println(name);

  sSensorStatus_t st = r->getStatus();
  Serial.print("  workStatus="); Serial.print(st.workStatus);
  Serial.print(" workMode="); Serial.print(st.workMode);
  Serial.print(" init="); Serial.println(st.initStatus);

  Serial.print("  minRange="); Serial.println(r->getMinRange());
  Serial.print("  maxRange="); Serial.println(r->getMaxRange());
  Serial.print("  trigRange="); Serial.println(r->getTrigRange());
  Serial.print("  trigSens="); Serial.println(r->getTrigSensitivity());
  Serial.print("  keepSens="); Serial.println(r->getKeepSensitivity());
  Serial.print("  trigDelay="); Serial.println(r->getTrigDelay());
  Serial.print("  keepTimeout="); Serial.println(r->getKeepTimerout());
  Serial.print("  ioPol="); Serial.println(r->getIoPolaity());

  sPwmData_t pwm = r->getPwm();
  Serial.print("  pwm1="); Serial.print(pwm.pwm1);
  Serial.print(" pwm2="); Serial.print(pwm.pwm2);
  Serial.print(" timer="); Serial.println(pwm.timer);

  Serial.println();
}

void printStartupSummary() {
  Serial.println("Configurable parameters (commands):");
  Serial.println("  A/B setRange min max thres");
  Serial.println("  A/B setTrig n");
  Serial.println("  A/B setKeep n");
  Serial.println("  A/B setDelay trig_s keep_s");
  Serial.println("  A/B setPwm pwm1 pwm2 timer");
  Serial.println("  A/B setIoPol n");
  Serial.println("  A/B setMode presence|speed|0|1");
  Serial.println("  A/B status");
  Serial.println("  A/B initstart");
  Serial.println("  A/B printall");
  Serial.println();

  printSensorReadback(&radarA, "Sensor A readback:");
  printSensorReadback(&radarB, "Sensor B readback:");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SENSOR_A_SERIAL.begin(9600);
  SENSOR_B_SERIAL.begin(9600);

  if (!radarA.begin()) Serial.println("Radar A init failed");
  if (!radarB.begin()) Serial.println("Radar B init failed");

  initRoutine(&radarA, "Sensor A", paramsA);
  initRoutine(&radarB, "Sensor B", paramsB);

  printStartupSummary();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) handleCommand(line);
  }

  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();

   // pollSensors();

    postOnPresentToAbsent(&radarA, "Sensor A", lastPresenceA);
    postOnPresentToAbsent(&radarB, "Sensor B", lastPresenceB);
  }
}


void applyParams(DFRobot_C4001_UART *r, const Params &p) {
  r->setSensorMode(eExitMode);
  r->setDetectionRange(p.minRange, p.maxRange, p.thresRange);
  r->setTrigSensitivity(p.trigSens);
  r->setKeepSensitivity(p.keepSens);
  r->setDelay(p.trigDelay, p.keepDelay);
  r->setPwm(p.pwm1, p.pwm2, p.pwmTimer);
  r->setIoPolaity(p.ioPol);
}

void initRoutine(DFRobot_C4001_UART *r, const char *name, Params &p) {
  Serial.print("Init "); Serial.println(name);
  applyParams(r, p);
  r->setSensor(eStartSen);
  delay(200);
  sSensorStatus_t st = r->getStatus();
  Serial.print(name);
  Serial.print(" status: workStatus=");
  Serial.println(st.workStatus);
}

void pollSensors() {
  sSensorStatus_t stA = radarA.getStatus();
  if (stA.workStatus != lastStatusA.workStatus ||
      stA.workMode != lastStatusA.workMode ||
      stA.initStatus != lastStatusA.initStatus) {
    Serial.println("Sensor A status changed:");
    Serial.print("  workStatus: "); Serial.print(stA.workStatus);
    Serial.print(" ("); Serial.print(stA.workStatus ? "start" : "stop"); Serial.println(")");
    Serial.print("  workMode:   "); Serial.print(stA.workMode);
    Serial.print(" ("); Serial.print(stA.workMode == 0 ? "presence" : (stA.workMode == 1 ? "speed/range" : "unknown")); Serial.println(")");
    Serial.print("  initStatus: "); Serial.print(stA.initStatus);
    Serial.print(" ("); Serial.print(stA.initStatus ? "init success" : "not init"); Serial.println(")");
    lastStatusA = stA;
  }

  sSensorStatus_t stB = radarB.getStatus();
  if (stB.workStatus != lastStatusB.workStatus ||
      stB.workMode != lastStatusB.workMode ||
      stB.initStatus != lastStatusB.initStatus) {
    Serial.println("Sensor B status changed:");
    Serial.print("  workStatus: "); Serial.print(stB.workStatus);
    Serial.print(" ("); Serial.print(stB.workStatus ? "start" : "stop"); Serial.println(")");
    Serial.print("  workMode:   "); Serial.print(stB.workMode);
    Serial.print(" ("); Serial.print(stB.workMode == 0 ? "presence" : (stB.workMode == 1 ? "speed/range" : "unknown")); Serial.println(")");
    Serial.print("  initStatus: "); Serial.print(stB.initStatus);
    Serial.print(" ("); Serial.print(stB.initStatus ? "init success" : "not init"); Serial.println(")");
    lastStatusB = stB;
  }
}

void postOnPresentToAbsent(DFRobot_C4001_UART *r, const char *name, int8_t &lastState) {
  int8_t current = r->motionDetection() ? 1 : 0;

  if (lastState == 1 && current == 0) {
    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] POST -> ");
    Serial.print(name);
    Serial.println(": ABSENT");
  }

    if (lastState == 0 && current == 1) {
    Serial.print('[');
    Serial.print(millis());
    Serial.print(" ms] POST -> ");
    Serial.print(name);
    Serial.println(": PRESENT");
  }

  lastState = current;
}

void stopSetRangeSaveStart(DFRobot_C4001_UART *r, uint16_t minRange, uint16_t maxRange, uint16_t thresRange) {
  r->setSensor(eStopSen);
  delay(300);
  r->setDetectionRange(minRange, maxRange, thresRange);
  delay(100);
  r->setSensor(eSaveParams);
  delay(2000);
  r->setSensor(eStartSen);
  delay(2000);
}

void stopSetTrigSaveStart(DFRobot_C4001_UART *r, uint8_t trigSens) {
  r->setSensor(eStopSen);
  delay(300);
  r->setTrigSensitivity(trigSens);
  delay(100);
  r->setSensor(eSaveParams);
  delay(2000);
  r->setSensor(eStartSen);
  delay(2000);
}

void stopSetKeepSaveStart(DFRobot_C4001_UART *r, uint8_t keepSens) {
  r->setSensor(eStopSen);
  delay(300);
  r->setKeepSensitivity(keepSens);
  delay(100);
  r->setSensor(eSaveParams);
  delay(2000);
  r->setSensor(eStartSen);
  delay(2000);
}

void stopSetDelaySaveStart(DFRobot_C4001_UART *r, uint16_t trigDelay, uint16_t keepDelay) {
  r->setSensor(eStopSen);
  delay(300);
  r->setDelay(trigDelay, keepDelay);
  delay(100);
  r->setSensor(eSaveParams);
  delay(2000);
  r->setSensor(eStartSen);
  delay(2000);
}

void stopSetPwmSaveStart(DFRobot_C4001_UART *r, uint8_t pwm1, uint8_t pwm2, uint16_t pwmTimer) {
  r->setSensor(eStopSen);
  delay(300);
  r->setPwm(pwm1, pwm2, pwmTimer);
  delay(100);
  r->setSensor(eSaveParams);
  delay(500);
  r->setSensor(eStartSen);
  delay(500);
}

void stopSetIoPolSaveStart(DFRobot_C4001_UART *r, uint8_t ioPol) {
  r->setSensor(eStopSen);
  delay(300);
  r->setIoPolaity(ioPol);
  delay(100);
  r->setSensor(eSaveParams);
  delay(2000);
  r->setSensor(eStartSen);
  delay(2000);
}

//-----------------trigger reset test

void stopSetTrigSaveReset(DFRobot_C4001_UART *r, uint8_t trigSens) {
  r->setSensor(eStopSen);
  delay(1000);

  r->setKeepSensitivity(trigSens);   // swapped workaround
  delay(500);

  Serial.print("before save trigSens=");
  Serial.println(r->getTrigSensitivity());

  r->setSensor(eSaveParams);
  Serial.println("save sent");
  delay(8000);

  Serial.println("waiting extra before reset...");
  delay(5000);

  // r->setSensor(eResetSen);
  // Serial.println("reset sent");
  // delay(3000);

r->setSensor(eStartSen);
delay(2000);

  Serial.print("after reset trigSens=");
  Serial.println(r->getTrigSensitivity());
}


void stopSetKeepSaveReset(DFRobot_C4001_UART *r, uint8_t keepSens) {
  r->setSensor(eStopSen);
  delay(1000);

  r->setTrigSensitivity(keepSens);
  delay(300);

  r->setSensor(eSaveParams);
  delay(3000);

  r->setSensor(eResetSen);
  delay(2000);

  Serial.print("readback trigSens=");
  Serial.println(r->getTrigSensitivity());
  Serial.print("readback keepSens=");
  Serial.println(r->getKeepSensitivity());
}


void handleCommand(String line) {
  line.trim();
  if (line.length() < 2) return;

  char t = toupper(line.charAt(0));
  if (t != 'A' && t != 'B') {
    Serial.println("Start command with A or B");
    return;
  }

  String rest = line.substring(1);
  rest.trim();

  int sp = rest.indexOf(' ');
  String cmd = (sp == -1) ? rest : rest.substring(0, sp);
  String args = (sp == -1) ? "" : rest.substring(sp + 1);
  cmd.toLowerCase();
  args.trim();

  DFRobot_C4001_UART *r = (t == 'A') ? &radarA : &radarB;
  Params &p = (t == 'A') ? paramsA : paramsB;

  handleCommandLineForSensor(r, p, cmd, args);
}

void handleCommandLineForSensor(DFRobot_C4001_UART *r, Params &p, const String &cmd, const String &args) {
if (cmd == "setrange") {
  int a, b, c;
  if (sscanf(args.c_str(), "%d %d %d", &a, &b, &c) == 3) {
    p.minRange = a;
    p.maxRange = b;
    p.thresRange = c;

    stopSetRangeSaveStart(r, p.minRange, p.maxRange, p.thresRange);

    Serial.print("setRange -> min="); Serial.print(p.minRange);
    Serial.print(" max="); Serial.print(p.maxRange);
    Serial.print(" thres="); Serial.println(p.thresRange);
  } else {
    Serial.println("Bad setRange. Use: A setRange min max thres");
  }
  return;
}


if (cmd == "settrig") {
  int v = atoi(args.c_str());
  p.trigSens = (uint8_t)constrain(v, 0, 9);

  stopSetTrigSaveReset(r, p.trigSens);

  Serial.print("requested trigSens -> "); Serial.println(p.trigSens);
  return;
}

if (cmd == "setkeep") {
  int v = atoi(args.c_str());
  p.keepSens = (uint8_t)constrain(v, 0, 9);

  stopSetKeepSaveReset(r, p.keepSens);

  Serial.print("requested keepSens -> "); Serial.println(p.keepSens);
  return;
}



if (cmd == "setdelay") {
  String s = args;
  s.trim();

  int sp = s.indexOf(' ');
  if (sp == -1) {
    Serial.println("Bad setDelay. Use: A setDelay trig_s keep_s");
    return;
  }

  String aStr = s.substring(0, sp);
  String bStr = s.substring(sp + 1);
  aStr.trim();
  bStr.trim();

  if (aStr.length() == 0 || bStr.length() == 0) {
    Serial.println("Bad setDelay. Use: A setDelay trig_s keep_s");
    return;
  }

  float a = aStr.toFloat();
  float b = bStr.toFloat();

  p.trigDelay = (uint16_t)round(a * 1.f);
  p.keepDelay = (uint16_t)round(b * 1.f);

  stopSetDelaySaveStart(r, p.trigDelay, p.keepDelay);

  Serial.print("setDelay -> trig(0.01s)="); Serial.print(p.trigDelay);
  Serial.print(" keep(0.5s)="); Serial.println(p.keepDelay);
  return;
}




if (cmd == "setpwm") {
  int a, b, c;
  if (sscanf(args.c_str(), "%d %d %d", &a, &b, &c) == 3) {
    p.pwm1 = (uint8_t)constrain(a, 0, 100);
    p.pwm2 = (uint8_t)constrain(b, 0, 100);
    p.pwmTimer = (uint16_t)max(c, 0);

    stopSetPwmSaveStart(r, p.pwm1, p.pwm2, p.pwmTimer);

    Serial.print("setPwm -> pwm1="); Serial.print(p.pwm1);
    Serial.print(" pwm2="); Serial.print(p.pwm2);
    Serial.print(" timer="); Serial.println(p.pwmTimer);
  } else {
    Serial.println("Bad setPwm. Use: A setPwm pwm1 pwm2 timer");
  }
  return;
}


if (cmd == "setiopol") {
  int v = atoi(args.c_str());
  p.ioPol = (uint8_t)(v ? 1 : 0);

  stopSetIoPolSaveStart(r, p.ioPol);

  Serial.print("setIoPol -> "); Serial.println(p.ioPol);
  return;
}



  if (cmd == "status") {
    sSensorStatus_t st = r->getStatus();
    Serial.print("workStatus="); Serial.print(st.workStatus);
    Serial.print(" workMode="); Serial.print(st.workMode);
    Serial.print(" init="); Serial.println(st.initStatus);
    return;
  }

  if (cmd == "initstart") {
    Serial.println("initstart -> reset/start sequence...");
    r->setSensor(eResetSen);
    delay(500);
    r->setSensor(eStartSen);
    delay(200);

    sSensorStatus_t st = r->getStatus();
    Serial.print("after start: workStatus="); Serial.print(st.workStatus);
    Serial.print(" workMode="); Serial.print(st.workMode);
    Serial.print(" init="); Serial.println(st.initStatus);
    return;
  }

  if (cmd == "printall") {
    printStartupSummary();
    return;
  }

  Serial.println("Unknown cmd");
}
