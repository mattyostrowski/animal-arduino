#include "DFRobot_C4001.h"
#include <SPI.h>
#include <Ethernet.h>
#include <math.h>

#define SENSOR_A_SERIAL Serial2
#define SENSOR_B_SERIAL Serial3

DFRobot_C4001_UART radarA(&SENSOR_A_SERIAL, 9600);
DFRobot_C4001_UART radarB(&SENSOR_B_SERIAL, 9600);

// Poll interval
const unsigned long POLL_MS = 150;

// Parameter struct
struct Params {
  uint16_t minRange = 30;
  uint16_t maxRange = 1000;
  uint16_t thresRange = 1000;
  uint8_t trigSens = 1;
  uint8_t keepSens = 2;
  uint16_t trigDelay = 0;   // in 0.01s units when passed to setDelay
  uint16_t keepDelay = 4;   // in 0.5s units when passed to setDelay
  uint8_t pwm1 = 50;
  uint8_t pwm2 = 0;
  uint16_t pwmTimer = 10;
  uint8_t ioPol = 1;
} paramsA, paramsB;

// Forward declarations
void applyParams(DFRobot_C4001_UART *r, const Params &p);
void initRoutine(DFRobot_C4001_UART *r, const char *name, Params &p);
void handleCommand(String line);
void pollSensors();
void handleCommandLineForSensor(DFRobot_C4001_UART *r, Params &p, char t, const String &cmd, const String &args);

unsigned long lastPoll = 0;

sSensorStatus_t lastStatusA = {255,255,255};
sSensorStatus_t lastStatusB = {255,255,255};

void printStartupSummary() {
  Serial.println("Configurable parameters (commands):");
  Serial.println("  setRange min max thres");
  Serial.println("  setTrig n        (0-9)");
  Serial.println("  setKeep n        (0-9)");
  Serial.println("  setDelay trig_s keep_s   (seconds)");
  Serial.println("  setPwm pwm1 pwm2 timer");
  Serial.println("  setIoPol n       (0|1)");
  Serial.println("  setMode presence|speed|0|1");
  Serial.println("  commit            (stop -> save -> start)");
  Serial.println("  status            (print sensor status)");
  Serial.println();
  // Print current status + parameter values for both sensors
  Serial.println("Sensor A current status & parameters:");
  sSensorStatus_t sa = radarA.getStatus();
  Serial.print("  workStatus="); Serial.print(sa.workStatus); Serial.print(" workMode="); Serial.print(sa.workMode); Serial.print(" init="); Serial.println(sa.initStatus);
  Serial.print("  range: min="); Serial.print(paramsA.minRange); Serial.print(" max="); Serial.print(paramsA.maxRange); Serial.print(" thres="); Serial.println(paramsA.thresRange);
  Serial.print("  trigSens="); Serial.print(paramsA.trigSens); Serial.print(" keepSens="); Serial.print(paramsA.keepSens);
  Serial.print("  delay_trig="); Serial.print(paramsA.trigDelay); Serial.print(" (0.01s units)"); Serial.print(" keep="); Serial.println(paramsA.keepDelay);
  Serial.print("  pwm1="); Serial.print(paramsA.pwm1); Serial.print(" pwm2="); Serial.print(paramsA.pwm2); Serial.print(" timer="); Serial.println(paramsA.pwmTimer);
  Serial.print("  ioPol="); Serial.println(paramsA.ioPol);
  Serial.println();

  Serial.println("Sensor B current status & parameters:");
  sSensorStatus_t sb = radarB.getStatus();
  Serial.print("  workStatus="); Serial.print(sb.workStatus); Serial.print(" workMode="); Serial.print(sb.workMode); Serial.print(" init="); Serial.println(sb.initStatus);
  Serial.print("  range: min="); Serial.print(paramsB.minRange); Serial.print(" max="); Serial.print(paramsB.maxRange); Serial.print(" thres="); Serial.println(paramsB.thresRange);
  Serial.print("  trigSens="); Serial.print(paramsB.trigSens); Serial.print(" keepSens="); Serial.print(paramsB.keepSens);
  Serial.print("  delay_trig="); Serial.print(paramsB.trigDelay); Serial.print(" (0.01s units)"); Serial.print(" keep="); Serial.println(paramsB.keepDelay);
  Serial.print("  pwm1="); Serial.print(paramsB.pwm1); Serial.print(" pwm2="); Serial.print(paramsB.pwm2); Serial.print(" timer="); Serial.println(paramsB.pwmTimer);
  Serial.print("  ioPol="); Serial.println(paramsB.ioPol);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

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
    pollSensors();
  }
}

void applyParams(DFRobot_C4001_UART *r, const Params &p) {
  // Use public APIs to configure sensor
  r->setSensorMode(eExitMode); // exit any running mode to configure
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
  Serial.print(name); Serial.print(" status: workStatus="); Serial.println(st.workStatus);
}

void pollSensors() {
  sSensorStatus_t stA = radarA.getStatus();
  if (stA.workStatus != lastStatusA.workStatus || stA.workMode != lastStatusA.workMode || stA.initStatus != lastStatusA.initStatus) {
    Serial.println("Sensor A status changed:");
    Serial.print("  workStatus: "); Serial.print(stA.workStatus); Serial.print(" ("); Serial.print(stA.workStatus ? "start" : "stop"); Serial.println(")");
    Serial.print("  workMode:   "); Serial.print(stA.workMode); Serial.print(" ("); Serial.print(stA.workMode==0 ? "presence" : (stA.workMode==1 ? "speed/range" : "unknown")); Serial.println(")");
    Serial.print("  initStatus: "); Serial.print(stA.initStatus); Serial.print(" ("); Serial.print(stA.initStatus ? "init success" : "not init"); Serial.println(")");
    lastStatusA = stA;
  }

  sSensorStatus_t stB = radarB.getStatus();
  if (stB.workStatus != lastStatusB.workStatus || stB.workMode != lastStatusB.workMode || stB.initStatus != lastStatusB.initStatus) {
    Serial.println("Sensor B status changed:");
    Serial.print("  workStatus: "); Serial.print(stB.workStatus); Serial.print(" ("); Serial.print(stB.workStatus ? "start" : "stop"); Serial.println(")");
    Serial.print("  workMode:   "); Serial.print(stB.workMode); Serial.print(" ("); Serial.print(stB.workMode==0 ? "presence" : (stB.workMode==1 ? "speed/range" : "unknown")); Serial.println(")");
    Serial.print("  initStatus: "); Serial.print(stB.initStatus); Serial.print(" ("); Serial.print(stB.initStatus ? "init success" : "not init"); Serial.println(")");
    lastStatusB = stB;
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() < 2) return;
  char t = toupper(line.charAt(0));
  if (t != 'A' && t != 'B') { Serial.println("Start with A or B"); return; }

  String rest = line.substring(1);
  rest.trim();
  int sp = rest.indexOf(' ');
  String cmd = (sp == -1) ? rest : rest.substring(0, sp);
  String args = (sp == -1) ? "" : rest.substring(sp + 1);
  cmd.toLowerCase(); args.trim();

  DFRobot_C4001_UART *r = (t == 'A') ? &radarA : &radarB;
  Params &p = (t == 'A') ? paramsA : paramsB;

  handleCommandLineForSensor(r, p, t, cmd, args);
}

void handleCommandLineForSensor(DFRobot_C4001_UART *r, Params &p, char t, const String &cmd, const String &args) {
if (cmd == "setrange") {
  int a,b,c;
  if (sscanf(args.c_str(), "%d %d %d", &a,&b,&c) == 3) {
    p.minRange = a; p.maxRange = b; p.thresRange = c;
    applyParams(r,p);
    Serial.print("setRange -> min="); Serial.print(p.minRange);
    Serial.print(" max="); Serial.print(p.maxRange);
    Serial.print(" thres="); Serial.println(p.thresRange);
  } else Serial.println("Bad setRange. Use: A setRange min max thres");
  return;
}

if (cmd == "settrig") {
  int v = atoi(args.c_str());
  p.trigSens = (uint8_t)constrain(v,0,9);
  applyParams(r,p);
  Serial.print("setTrig -> "); Serial.println(p.trigSens);
  return;
}


  if (cmd == "setkeep") {
  int v = atoi(args.c_str());
  p.keepSens = (uint8_t)constrain(v,0,9);
  applyParams(r,p);
  Serial.print("setKeep -> "); Serial.println(p.keepSens);
  return;
}

if (cmd == "setdelay") {
  float a,b;
  if (sscanf(args.c_str(), "%f %f", &a,&b) == 2) {
    p.trigDelay = (uint16_t)round(a * 100.0f);
    p.keepDelay = (uint16_t)round(b / 0.5f);
    applyParams(r,p);
    Serial.print("setDelay -> trig(0.01s)="); Serial.print(p.trigDelay);
    Serial.print(" keep(0.5s)="); Serial.println(p.keepDelay);
  } else Serial.println("Bad setDelay. Use: A setDelay trig_s keep_s");
  return;
}

if (cmd == "setmode") {
  String s = args; s.trim(); s.toLowerCase();
  eMode_t mode;
  if (isDigit(s.charAt(0))) mode = (eMode_t)atoi(s.c_str());
  else if (s == "presence") mode = (eMode_t)0;
  else if (s == "speed") mode = (eMode_t)1;
  else { Serial.println("Bad setMode. Use presence|speed|0|1"); return; }
  if (r->setSensorMode(mode)) {
    Serial.print("setMode -> "); Serial.println((int)mode);
    sSensorStatus_t st = r->getStatus();
    Serial.print("workMode reported: "); Serial.println(st.workMode);
  } else Serial.println("setMode failed");
  return;
}

  if (cmd == "commit") {
  r->setSensor(eStopSen); delay(300);
  r->setSensor(eSaveParams); delay(300);
  r->setSensor(eStartSen); delay(200);
  Serial.println("commit -> stop/save/start executed");
  sSensorStatus_t st = r->getStatus();
  Serial.print("post-commit status workStatus="); Serial.println(st.workStatus);
  return;
}

  if (cmd == "status") {
    sSensorStatus_t st = r->getStatus();
    Serial.print("workStatus="); Serial.print(st.workStatus);
    Serial.print(" workMode="); Serial.print(st.workMode);
    Serial.print(" init="); Serial.println(st.initStatus);
    return;
  }

 
void handleCommandLineForSensor(DFRobot_C4001_UART *r, Params &p, char t, const String &cmd, const String &args) {
  if (cmd == "setrange") {
    int a,b,c;
    if (sscanf(args.c_str(), "%d %d %d", &a,&b,&c) == 3) {
      p.minRange = a; p.maxRange = b; p.thresRange = c;
      applyParams(r,p);
      Serial.print("setRange -> min="); Serial.print(p.minRange);
      Serial.print(" max="); Serial.print(p.maxRange);
      Serial.print(" thres="); Serial.println(p.thresRange);
    } else Serial.println("Bad setRange. Use: A setRange min max thres");
    return;
  }

  if (cmd == "settrig") {
    int v = atoi(args.c_str());
    p.trigSens = (uint8_t)constrain(v,0,9);
    applyParams(r,p);
    Serial.print("setTrig -> "); Serial.println(p.trigSens);
    return;
  }

  if (cmd == "setkeep") {
    int v = atoi(args.c_str());
    p.keepSens = (uint8_t)constrain(v,0,9);
    applyParams(r,p);
    Serial.print("setKeep -> "); Serial.println(p.keepSens);
    return;
  }

  if (cmd == "setdelay") {
    float a,b;
    if (sscanf(args.c_str(), "%f %f", &a,&b) == 2) {
      p.trigDelay = (uint16_t)round(a * 100.0f);
      p.keepDelay = (uint16_t)round(b / 0.5f);
      applyParams(r,p);
      Serial.print("setDelay -> trig(0.01s)="); Serial.print(p.trigDelay);
      Serial.print(" keep(0.5s)="); Serial.println(p.keepDelay);
    } else Serial.println("Bad setDelay. Use: A setDelay trig_s keep_s");
    return;
  }

  if (cmd == "setmode") {
    String s = args; s.trim(); s.toLowerCase();
    eMode_t mode;
    if (isDigit(s.charAt(0))) mode = (eMode_t)atoi(s.c_str());
    else if (s == "presence") mode = (eMode_t)0;
    else if (s == "speed") mode = (eMode_t)1;
    else { Serial.println("Bad setMode. Use presence|speed|0|1"); return; }

    if (r->setSensorMode(mode)) {
      Serial.print("setMode -> "); Serial.println((int)mode);
      sSensorStatus_t st = r->getStatus();
      Serial.print("workMode reported: "); Serial.println(st.workMode);
    } else {
      Serial.println("setMode failed");
    }
    return;
  }

  if (cmd == "commit") {
    r->setSensor(eStopSen); delay(300);
    r->setSensor(eSaveParams); delay(300);
    r->setSensor(eStartSen); delay(200);
    Serial.println("commit -> stop/save/start executed");
    sSensorStatus_t st = r->getStatus();
    Serial.print("post-commit status workStatus="); Serial.println(st.workStatus);
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
    Serial.println("initstart -> initializing sensor...");
    if (!r->setSensor(eInitSen)) {
      Serial.println("setSensor(eInitSen) failed");
    } else {
      delay(500);
      sSensorStatus_t st = r->getStatus();
      Serial.print("after init: initStatus="); Serial.println(st.initStatus);
    }

    Serial.println("Starting sensor...");
    if (!r->setSensor(eStartSen)) {
      Serial.println("setSensor(eStartSen) failed");
    } else {
      delay(200);
      sSensorStatus_t st2 = r->getStatus();
      Serial.print("after start: workStatus="); Serial.print(st2.workStatus);
      Serial.print(" workMode="); Serial.print(st2.workMode);
      Serial.print(" init="); Serial.println(st2.initStatus);
    }
    return;
  }

  if (cmd == "printall") {
    printStartupSummary();
    return;
  }

  Serial.println("Unknown cmd");
}

