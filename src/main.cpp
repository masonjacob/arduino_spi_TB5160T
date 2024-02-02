#include <Arduino.h>
#include <TMCStepper.h>
#include <Servo.h>

#define MOSI_PIN 11 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN 12 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  13 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)

#define WRIST_CS_PIN 10
#define WRIST_STEP_PIN 9
#define WRIST_CURRENT 800

#define WRIST_ROTATION_CS_PIN 8
#define WRIST_ROTATION_STEP_PIN 7
#define WRIST_ROTATION_CURRENT 400

#define ELBOW_CS_PIN 6
#define ELBOW_STEP_PIN 5
#define ELBOW_CURRENT 1680

// #define BASE_ROTATION_CS_PIN 10
// #define BASE_ROTATION_STEP_PIN 9
// #define BASE_ROTATION_CURRENT 2000

// #define LEFT_SHOULDER_CS_PIN 8
// #define RIGHT_SHOULDER_CS_PIN A1
// #define SHOULDER_STEP_PIN A0
// #define SHOULDER_CURRENT 3000

#define END_EFFECTOR_SIGNAL_PIN 4
#define END_EFFECTOR_SWITCH_PIN 3

#define R_SENSE   0.075f //TMC5160: 0.075

//function declarations:
void create_drivers();
void setup_driver_pins(int* pins);
void setup_driver(TMC5160Stepper* driver, int current);


enum DRIVERS {
    WRIST, 
    WRIST_ROTATION,
    ELBOW
};

enum PINS {
    CS_PIN,
    STEP_PIN
};

TMC5160Stepper* drivers[3];

Servo end_effector;

int pins[3][2] = {{WRIST_CS_PIN, WRIST_STEP_PIN},
                 {WRIST_ROTATION_CS_PIN, WRIST_ROTATION_STEP_PIN},
                 {ELBOW_CS_PIN, ELBOW_STEP_PIN}
                 };

int currents[3] = {WRIST_CURRENT, WRIST_ROTATION_CURRENT, ELBOW_CURRENT};

void create_drivers() {
    for (int i = 0; i < sizeof(drivers)/sizeof(TMC5160Stepper*); i++) {
        drivers[i] = new TMC5160Stepper(pins[i][0], R_SENSE, MOSI_PIN, MISO_PIN, SCK_PIN);
        setup_driver_pins(pins[i]);
        setup_driver(drivers[i], currents[i]);
    }
};

void setup_driver_pins(int* pins) {
    pinMode(pins[CS_PIN], OUTPUT);
    digitalWrite(pins[CS_PIN], LOW);
    pinMode(pins[STEP_PIN], OUTPUT);
    digitalWrite(pins[STEP_PIN], LOW);
}

void setup_driver(TMC5160Stepper* driver, int current) {
    driver->begin();
    driver->toff(4); //off time
    driver->microsteps(16); //16 microsteps
    driver->rms_current(current); //400mA RMS
    driver->en_pwm_mode(true);
    driver->pwm_autoscale(true);
	driver->AMAX(500);
	driver->VMAX(200000);
	driver->DMAX(700);
	driver->VSTOP(10);
	driver->RAMPMODE(0);
	driver->XTARGET(-51200);
}

#define MOTION_TEST

#ifdef CALIBRATE_SPREADCYCLE
// You can define starting values here:
struct {
    uint8_t blank_time = 24;        // [16, 24, 36, 54]
    uint8_t off_time = 3;           // [1..15]
    uint8_t hysteresis_start = 1;   // [1..8]
    int8_t hysteresis_end = 12;     // [-3..12]
} config;

void initPins();

// ISR(TIMER1_COMPA_vect){
//     STEP_PIN |= 1 << STEP_BIT;
//     STEP_PIN &= ~(1 << STEP_BIT);
// }

void reportCurrentSettings() {
    Serial.print("Off time = ");
    Serial.print(config.off_time);
    Serial.print(" Hysteresis end = ");
    Serial.print(config.hysteresis_end);
    Serial.print(" Hysteresis start = ");
    Serial.println(config.hysteresis_start);
}

void setup() {
    initPins();
    Serial.begin(250000);
    Serial.println(F("Starting calibration of TMC spreadCycle parameters."));
    Serial.println(F("This example by default uses X axis tmc on a RAMPS1.4."));
    Serial.println(F("First make sure your serial terminal sends newline ending only!"));
    for (uint8_t i = 0; i < 60; i++) { Serial.print('.'); delay(100); }
    Serial.println(F("\nThen make sure the belt is disconnected"));
    Serial.println(F("as we will not respect endstops or physical limits"));
    while(1) {
        Serial.println(F("Is the belt disconnected? Send 'yes' to confirm."));
        while(!Serial.available());
        String yn = Serial.readStringUntil('\n');
        Serial.println(yn);
        if (yn == "yes") {
            break;
        } else {
            Serial.println(F("Belt still connected."));
            Serial.println(F("Please disconnect belt."));
        };
    }

    Serial.println(F("\nNow make sure the tmc has 12V (or greater) power turned on."));
    while(1) {
        Serial.println(F("Is VMOT power on? Send 'yes' to confirm"));
        while(!Serial.available());
        String yn = Serial.readStringUntil('\n');
        Serial.println(yn);
        if (yn == "yes") {
            break;
        } else {
            Serial.println(F("Please turn on power to the tmc."));
        };
    }

    Serial.print(F("\nTesting connection..."));
    uint8_t result = left_shoulder.test_connection();
    if (result) {
        Serial.println(F("failed!"));
        Serial.print(F("Likely cause: "));
        switch(result) {
            case 1: Serial.println(F("loose connection")); break;
            case 2: Serial.println(F("Likely cause: no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        abort();
    }
    Serial.println(F("OK"));

    left_shoulder.push();
    left_shoulder.microsteps(256);
    tmc.irun(10);
    tmc.ihold(10);

    Serial.print(F("Setting tmc blank time to "));
    Serial.print(config.blank_time);
    Serial.println(F(" cycles."));
    tmc.blank_time(config.blank_time);

    Serial.print(F("Setting tmc off time to "));
    Serial.println(config.off_time);
    tmc.toff(config.off_time);

    Serial.print(F("Setting hysteresis end value to "));
    Serial.println(config.hysteresis_end);
    tmc.hysteresis_end(config.hysteresis_end);

    Serial.print(F("Setting hysteresis start value to "));
    Serial.println(config.hysteresis_start);
    tmc.hysteresis_start(config.hysteresis_start);

    Serial.print(F("Effective hysteresis = "));
    Serial.print(config.hysteresis_end);
    Serial.print(F(" + "));
    Serial.print(config.hysteresis_start);
    Serial.print(F(" = "));
    Serial.println(config.hysteresis_end + config.hysteresis_start);

    Serial.println(F("\nNow we start decreasing the hysteresis end setting."));
    Serial.println(F("You should first hear your motor making clearly audible noise."));
    Serial.println(F("As we tune the timings the noise will get higher pitched."));
    Serial.println(F("When the noise is no longer audible we have reached a good setting."));
    Serial.println(F("You can tune the setting by sending - (minus) character"));
    Serial.println(F("or you can go back to previous parameter by sending + (plus) character."));
    Serial.println(F("Sending = (equal) character move to the next phase."));

    digitalWrite(EN_PIN, LOW);
    while (tmc.cur_a() < 240) { // Use cur_b if measuring from B coil
        digitalWrite(STEP_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        delay(3);
    }

    while(1) {
        if (Serial.available() > 0) {
            uint8_t c = Serial.read();
            if (c == '+') {
                if (config.hysteresis_end == 12) Serial.println(F("Reached MAX setting already!"));
                else {
                    config.hysteresis_end++;
                    reportCurrentSettings();
                    tmc.hysteresis_end(config.hysteresis_end);
                }
            } else if (c == '-') {
                if (config.hysteresis_end == -3) Serial.println(F("Reached MIN setting already!"));
                else {
                    config.hysteresis_end--;
                    reportCurrentSettings();
                    tmc.hysteresis_end(config.hysteresis_end);
                }
            }
            else if (c == '=') break;
        }
    }

    Serial.print(F("Final effective hysteresis = "));
    Serial.print(config.hysteresis_end);
    Serial.print(F(" + "));
    Serial.print(config.hysteresis_start);
    Serial.print(F(" = "));
    Serial.println(config.hysteresis_end + config.hysteresis_start);
    Serial.println(F("Your configuration parameters in Marlin are:"));
    Serial.print(F("#define CHOPPER_TIMING { "));
    Serial.print(config.off_time);
    Serial.print(F(", "));
    Serial.print(config.hysteresis_end);
    Serial.print(F(", "));
    Serial.print(config.hysteresis_start);
    Serial.println(F(" }"));
}

void initPins() {
    pinMode(EN_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH); //deactivate tmc (LOW active)
    digitalWrite(DIR_PIN, LOW); //LOW or HIGH
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
}

void loop() {}
#endif

#ifdef MOTION_TEST

bool dir1 = false;
bool dir2 = false;


void driver_status(TMC5160Stepper* driver) {
    if(driver->diag0_error()){ Serial.println(F("DIAG0 error"));    }
    if(driver->ot())         { Serial.println(F("Overtemp."));      }
    if(driver->otpw())       { Serial.println(F("Overtemp. PW"));   }
    if(driver->s2ga())       { Serial.println(F("Short to Gnd A")); }
    if(driver->s2gb())       { Serial.println(F("Short to Gnd B")); }
    if(driver->ola())        { Serial.println(F("Open Load A"));    }
    if(driver->olb())        { Serial.println(F("Open Load B"));    }
}

void setup()
{
  pinMode(MOSI_PIN, OUTPUT);
  digitalWrite(MOSI_PIN, LOW);
  pinMode(MISO_PIN, INPUT);
  digitalWrite(MISO_PIN, HIGH);
  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW);

  pinMode(END_EFFECTOR_SIGNAL_PIN, OUTPUT);
  pinMode(END_EFFECTOR_SWITCH_PIN, INPUT);

  //init serial port
  Serial.begin(9600); //init serial port and set baudrate
  while(!Serial); //wait for serial port to connect (needed for Leonardo only)
  Serial.println("\nStart...");
  
  create_drivers();
  end_effector.attach(END_EFFECTOR_SIGNAL_PIN);
}

void loop()
{
// SPI only
    delay(1000);

     auto xactual = drivers[WRIST_ROTATION]->XACTUAL();
     auto xtarget = drivers[WRIST_ROTATION]->XTARGET();

     char buffer[256];
     sprintf(buffer, "ioin=%#-10lx xactual=%7ld\n",
         drivers[WRIST_ROTATION]->IOIN(),
         xactual
         );
     Serial.print(buffer);

     if (xactual == xtarget) 
        {
         drivers[WRIST_ROTATION]->XTARGET(-xactual);
        }


//   static uint32_t timer_1=0;
//   static uint32_t timer_2=0;
//   uint32_t ms = millis();


//   if ((ms-timer_2) > 4000) {
//     // delay(1000);
//     // timer_1 = ms;
//     timer_2 = ms;
//     // dir1 = !dir1;
//     dir2 = !dir2;
//     drivers[ELBOW]->shaft(dir2);
//     // drivers[WRIST]->shaft(dir1);
//     // drivers[WRIST_ROTATION]->shaft(dir1);
//   }

//   if((ms-timer_1) > 2000) //run every 1s
//   {
//     timer_1 = ms;
//     dir1 = !dir1;
//     drivers[WRIST]->shaft(dir1);
//     drivers[WRIST_ROTATION]->shaft(dir1);

//     driver_status(drivers[WRIST]);
//   }

//   if (digitalRead(END_EFFECTOR_SWITCH_PIN) == HIGH) {
//     end_effector.write(180);
//   } else {
//     end_effector.write(90);
//   }



//   //make steps
//   digitalWrite(pins[WRIST][STEP_PIN], HIGH);
//   digitalWrite(pins[WRIST_ROTATION][STEP_PIN], HIGH);
//   digitalWrite(pins[ELBOW][STEP_PIN], HIGH);

//   delayMicroseconds(250);
//   digitalWrite(pins[WRIST][STEP_PIN], LOW);

//   delayMicroseconds(250);
//   digitalWrite(pins[WRIST][STEP_PIN], HIGH);
//   digitalWrite(pins[WRIST_ROTATION][STEP_PIN], LOW);
//   digitalWrite(pins[ELBOW][STEP_PIN], LOW);

//   delayMicroseconds(250);
//   digitalWrite(pins[WRIST][STEP_PIN], LOW);

//   delayMicroseconds(250);
  
}
#endif

#ifdef SPI_TEST
#include "TMC5160.h"
TMC5160_SPI motor = TMC5160_SPI(SPI_CS); //Use default SPI peripheral and SPI settings.


void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);

  pinMode(SPI_DRV_ENN, OUTPUT);
  digitalWrite(SPI_DRV_ENN, LOW);
  pinMode(CLOCK, OUTPUT); 
  digitalWrite(CLOCK, LOW);
  pinMode(STEP, OUTPUT); 
  digitalWrite(STEP, LOW);
  pinMode(DIR, OUTPUT); 
  digitalWrite(DIR, LOW); // Active low

  // This sets the motor & tmc parameters /!\ run the configWizard for your tmc and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 98; // Adapt to your tmc and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 2;
  motorParams.ihold = 1;

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  // ramp definition
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(400);
  motor.setAcceleration(500);

  Serial.println("starting up");

  delay(1000); // Standstill for automatic tuning
}

void loop()
{
  uint32_t now = millis();
  static unsigned long t_dirchange, t_echo;
  static bool dir;

  // every n seconds or so...
  if ( now - t_dirchange > 3000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    motor.setTargetPosition(dir ? 200 : 0);  // 1 full rotation = 200s/rev
  }

  // print out current position
  if( now - t_echo > 100 )
  {
    t_echo = now;

    // get the current target position
    float xactual = motor.getCurrentPosition();
    float vactual = motor.getCurrentSpeed();
    Serial.print("current position : ");
    Serial.print(xactual);
    Serial.print("\tcurrent speed : ");
    Serial.println(vactual);
  }
}
#endif

