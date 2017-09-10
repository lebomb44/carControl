#include <Cmd.h>
#include <Servo.h>
#include <EEPROM.h>

/* *****************************
 *  Pin allocation
 * *****************************
 */
#define FL_SERVO_PIN 2
#define FR_SERVO_PIN 3
#define RL_SERVO_PIN 4
#define RR_SERVO_PIN 5

#define POWER_SERVO_PIN 6
#define POWER_VOLTAGE_PIN 7
#define BATT_VOLTAGE_PIN 9
#define CAM_SERVO_PIN 8

/* *****************************
 *  Global variables
 * *****************************
 */
Servo fl;
uint8_t flStop = 90;
Servo fr;
uint8_t frStop = 90;
Servo rl;
uint8_t rlStop = 90;
Servo rr;
uint8_t rrStop = 90;

uint8_t speedStep = 10;

Servo power;
uint8_t powerON = 110;
uint8_t powerOFF = 70;
Servo cam;
uint8_t camFront = 110;
uint8_t camRear = 70;

#define FLSTOP_EEADDR 10
#define FRSTOP_EEADDR 11
#define RLSTOP_EEADDR 12
#define RRSTOP_EEADDR 13
#define SPEEDSTEP_EEADDR 14
#define POWERON_EEADDR 15
#define POWEROFF_EEADDR 16
#define CAMFRONT_EEADDR 17
#define CAMREAR_EEADDR 18

/* *****************************
 *  Debug Macros
 * *****************************
 */
bool carControl_printIsEnabled = true;
#define CARCONTROL_PRINT(m) if(true == carControl_printIsEnabled) { m }

/* *****************************
 *  Command lines functions
 * *****************************
 */
void front (int arg_cnt, char **args) {
  int16_t speed = max(abs(fl.read() - flStop), abs(fr.read() - frStop));
  speed = speed + speedStep;
  fl.write(flStop + speed); fr.write(frStop + speed);
  rl.write(rlStop + speed); rr.write(rrStop + speed);
  Serial.print("front="); Serial.println(speed);
}
void back (int arg_cnt, char **args) {
  int16_t speed = max(abs(fl.read() - flStop), abs(fr.read() - frStop));
  speed = speed - speedStep;
  fl.write(flStop + speed); fr.write(frStop + speed);
  rl.write(rlStop + speed); rr.write(rrStop + speed);
  Serial.print("back="); Serial.println(speed);
}
void left (int arg_cnt, char **args) {
  fl.write(fl.read() - speedStep); fr.write(fr.read() + speedStep);
  rl.write(rl.read() - speedStep); rr.write(rr.read() + speedStep);
  Serial.println("left=OK");
}
void right (int arg_cnt, char **args) {
  fl.write(fl.read() + speedStep); fr.write(fr.read() - speedStep);
  rl.write(rl.read() + speedStep); rr.write(rr.read() - speedStep);
  Serial.println("right=OK");
}
void stop (int arg_cnt, char **args) { fl.write(flStop); fr.write(frStop); rl.write(rlStop); rr.write(rrStop); Serial.println("Stop"); }
void flStopTrimP (int arg_cnt, char **args) { if(180>flStop) { flStop++; EEPROM.write(FLSTOP_EEADDR, flStop); } Serial.print("flStop="); Serial.println(flStop); }
void flStopTrimM (int arg_cnt, char **args) { if(0<flStop) { flStop--; EEPROM.write(FLSTOP_EEADDR, flStop); } Serial.print("flStop="); Serial.println(flStop); }
void frStopTrimP (int arg_cnt, char **args) { if(180>frStop) { frStop++; EEPROM.write(FRSTOP_EEADDR, frStop);} Serial.print("frStop="); Serial.println(frStop); }
void frStopTrimM (int arg_cnt, char **args) { if(0<frStop) { frStop--; EEPROM.write(FRSTOP_EEADDR, frStop); } Serial.print("frStop="); Serial.println(frStop); }
void rlStopTrimP (int arg_cnt, char **args) { if(180>rlStop) { rlStop++; EEPROM.write(RLSTOP_EEADDR, rlStop); } Serial.print("rlStop="); Serial.println(rlStop); }
void rlStopTrimM (int arg_cnt, char **args) { if(0<rlStop) { rlStop--; EEPROM.write(RLSTOP_EEADDR, rlStop); } Serial.print("rlStop="); Serial.println(rlStop); }
void rrStopTrimP (int arg_cnt, char **args) { if(180>rrStop) { rrStop++; EEPROM.write(RRSTOP_EEADDR, rrStop); } Serial.print("rrStop="); Serial.println(rrStop); }
void rrStopTrimM (int arg_cnt, char **args) { if(0<rrStop) { rrStop--; EEPROM.write(RRSTOP_EEADDR, rrStop); } Serial.print("rrStop="); Serial.println(rrStop); }

void go2powerON (int arg_cnt, char **args) { power.write(powerON); Serial.print("powerON="); Serial.println(powerON); }
void powerONTrimP (int arg_cnt, char **args) { if(180>powerON) { powerON++; EEPROM.write(POWERON_EEADDR, powerON); } Serial.print("powerON="); Serial.println(powerON); }
void powerONTrimM (int arg_cnt, char **args) { if(0<powerON) { powerON--; EEPROM.write(POWERON_EEADDR, powerON); } Serial.print("powerON="); Serial.println(powerON); }
void go2powerOFF (int arg_cnt, char **args) { power.write(powerOFF); Serial.print("powerOFF="); Serial.println(powerOFF); }
void powerOFFTrimP (int arg_cnt, char **args) { if(180>powerOFF) { powerOFF++; EEPROM.write(POWEROFF_EEADDR, powerOFF); } Serial.print("powerOFF="); Serial.println(powerOFF); }
void powerOFFTrimM (int arg_cnt, char **args) { if(0<powerOFF) { powerOFF--; EEPROM.write(POWEROFF_EEADDR, powerOFF); } Serial.print("powerOFF="); Serial.println(powerOFF); }
void powerVoltage (int arg_cnt, char **args) { Serial.print("powerVoltage="); Serial.println(analogRead(POWER_VOLTAGE_PIN)); }
void battVoltage (int arg_cnt, char **args) { Serial.print("battVoltage="); Serial.println(analogRead(BATT_VOLTAGE_PIN)); }

void camUP (int arg_cnt, char **args) { cam.write(cam.read() + 10); Serial.print("camUP="); Serial.println(cam.read()); }
void camDOWN (int arg_cnt, char **args) { cam.write(cam.read() - 10); Serial.print("camDOWN="); Serial.println(cam.read()); }
void go2camFront (int arg_cnt, char **args) {  cam.write(camFront); Serial.print("camFront="); Serial.println(camFront); }
void camFrontTrimP (int arg_cnt, char **args) { if(180>camFront) { camFront++; EEPROM.write(CAMFRONT_EEADDR, camFront); } Serial.print("camFront="); Serial.println(camFront); }
void camFrontTrimM (int arg_cnt, char **args) { if(0<camFront) { camFront--; EEPROM.write(CAMFRONT_EEADDR, camFront); } Serial.print("camFront="); Serial.println(camFront); }
void go2camRear (int arg_cnt, char **args) { cam.write(camRear); Serial.print("camRear="); Serial.println(camRear); }
void camRearTrimP (int arg_cnt, char **args) { if(180>camRear) { camRear++; EEPROM.write(CAMREAR_EEADDR, camRear); } Serial.print("camRear="); Serial.println(camRear); }
void camRearTrimM (int arg_cnt, char **args) { if(0<camRear) { camRear--; EEPROM.write(CAMREAR_EEADDR, camRear); } Serial.print("camRear="); Serial.println(camRear); }

void carControlEnablePrint(int arg_cnt, char **args) { carControl_printIsEnabled = true; Serial.println("carControl print ENABLED"); }
void carControlDisablePrint(int arg_cnt, char **args) { carControl_printIsEnabled = false; Serial.println("carControl print DISABLED"); }

void setup() {
  /* ****************************
   *  Pin configuration
   * ****************************
   */
  pinMode(FL_SERVO_PIN, OUTPUT); digitalWrite(FL_SERVO_PIN, LOW);
  pinMode(FR_SERVO_PIN, OUTPUT); digitalWrite(FR_SERVO_PIN, LOW);
  pinMode(RL_SERVO_PIN, OUTPUT); digitalWrite(RL_SERVO_PIN, LOW);
  pinMode(RR_SERVO_PIN, OUTPUT); digitalWrite(RR_SERVO_PIN, LOW);

  pinMode(POWER_SERVO_PIN, OUTPUT); digitalWrite(POWER_SERVO_PIN, LOW);
  pinMode(POWER_VOLTAGE_PIN, INPUT);
  pinMode(BATT_VOLTAGE_PIN, INPUT);
  pinMode(CAM_SERVO_PIN, OUTPUT); digitalWrite(CAM_SERVO_PIN, LOW);

  Serial.begin(115200);
  Serial.println("INFO: carControl Starting...");

  flStop    = EEPROM.read(FLSTOP_EEADDR);
  frStop    = EEPROM.read(FRSTOP_EEADDR);
  rlStop    = EEPROM.read(RLSTOP_EEADDR);
  rrStop    = EEPROM.read(RRSTOP_EEADDR);
  speedStep = EEPROM.read(SPEEDSTEP_EEADDR);
  powerON   = EEPROM.read(POWERON_EEADDR);
  powerOFF  = EEPROM.read(POWEROFF_EEADDR);
  camFront  = EEPROM.read(CAMFRONT_EEADDR);
  camRear   = EEPROM.read(CAMREAR_EEADDR);

  fl.attach(FL_SERVO_PIN);
  fr.attach(FR_SERVO_PIN);
  rl.attach(RL_SERVO_PIN);
  rr.attach(RR_SERVO_PIN);
  stop(0, NULL);
  power.attach(POWER_SERVO_PIN);
  go2powerOFF(0, NULL);
  cam.attach(CAM_SERVO_PIN);
  go2camFront(0, NULL);
  
  cmdInit();

  cmdAdd("enablePrint", "Enable print", carControlEnablePrint);
  cmdAdd("disablePrint", "Disable print", carControlDisablePrint);
  cmdAdd("help", "List commands", cmdList);

  Serial.println("INFO: carControl Init done");
}

void loop() {
  /* Poll for new command line */
  cmdPoll();
}

