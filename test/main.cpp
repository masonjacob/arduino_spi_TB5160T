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
#define WRIST_ROTATION_CURRENT 2000

#define ELBOW_CS_PIN 6
#define ELBOW_STEP_PIN 5
#define ELBOW_CURRENT 1680


#define BASE_ROTATION_CS_PIN 10
#define BASE_ROTATION_STEP_PIN 9
#define BASE_ROTATION_CURRENT 2000

#define LEFT_SHOULDER_CS_PIN 8
#define RIGHT_SHOULDER_CS_PIN A1
#define SHOULDER_STEP_PIN A0
#define SHOULDER_CURRENT 3000

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
        drivers[i] = new TMC5160Stepper(pins[i][0], R_SENSE);
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
	// driver->AMAX(500);
    // driver->VSTART(10);
	// driver->VMAX(2000);
	// driver->DMAX(700);
	// driver->VSTOP(10);
	// driver->RAMPMODE(1);
	// driver->XTARGET(-51200);
    driver->shaft(true);
}

void driver_status(TMC5160Stepper* driver) {
    if(driver->diag0_error()){ Serial.println(F("DIAG0 error"));    }
    if(driver->ot())         { Serial.println(F("Overtemp."));      }
    if(driver->otpw())       { Serial.println(F("Overtemp. PW"));   }
    if(driver->s2ga())       { Serial.println(F("Short to Gnd A")); }
    if(driver->s2gb())       { Serial.println(F("Short to Gnd B")); }
    if(driver->ola())        { Serial.println(F("Open Load A"));    }
    if(driver->olb())        { Serial.println(F("Open Load B"));    }
}

#define STEP_DIR_SPI_TEST

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

#ifdef STEP_DIR_SPI_TEST

bool dir1 = false;
bool dir2 = false;

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
/////////////////////////////////
// Working SPI + STEP PIN TEST //
/////////////////////////////////

  static uint32_t timer_1=0;
  static uint32_t timer_2=0;
  uint32_t ms = millis();


  if ((ms-timer_2) > 4000) {
    // delay(1000);
    // timer_1 = ms;
    timer_2 = ms;
    // dir1 = !dir1;
    dir2 = !dir2;
    drivers[ELBOW]->shaft(dir2);
    // drivers[WRIST]->shaft(dir1);
    // drivers[WRIST_ROTATION]->shaft(dir1);
  }

  if((ms-timer_1) > 2000) //run every 1s
  {
    timer_1 = ms;
    dir1 = !dir1;
    drivers[WRIST]->shaft(dir1);
    drivers[WRIST_ROTATION]->shaft(dir1);

    driver_status(drivers[WRIST]);
  }

  if (digitalRead(END_EFFECTOR_SWITCH_PIN) == HIGH) {
    end_effector.write(180);
  } else {
    end_effector.write(90);
  }



  //make steps
  digitalWrite(pins[WRIST][STEP_PIN], HIGH);
  digitalWrite(pins[WRIST_ROTATION][STEP_PIN], HIGH);
  digitalWrite(pins[ELBOW][STEP_PIN], HIGH);

  delayMicroseconds(250);
  digitalWrite(pins[WRIST][STEP_PIN], LOW);

  delayMicroseconds(250);
  digitalWrite(pins[WRIST][STEP_PIN], HIGH);
  digitalWrite(pins[WRIST_ROTATION][STEP_PIN], LOW);
  digitalWrite(pins[ELBOW][STEP_PIN], LOW);

  delayMicroseconds(250);
  digitalWrite(pins[WRIST][STEP_PIN], LOW);

  delayMicroseconds(250);
}
#endif

#ifdef TMC_LIBRARY_SPI_TEST

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
  for (int i = 0; i < sizeof(drivers)/sizeof(TMC5160Stepper*); i++) {
    drivers[i]->AMAX(500);
    drivers[i]->VSTART(10);
	drivers[i]->VMAX(2000);
	drivers[i]->DMAX(700);
	drivers[i]->VSTOP(10);
	drivers[i]->RAMPMODE(1);
	drivers[i]->XTARGET(-51200);
  }
  end_effector.attach(END_EFFECTOR_SIGNAL_PIN);
}

void loop()
{
/////////////////////////////////
// Previously Working Full SPI //
/////////////////////////////////

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
}

#endif

#ifdef TMC5160_SPI_TEST
#include "TMC5160.h"
TMC5160_SPI motor = TMC5160_SPI(WRIST_ROTATION_CS_PIN); //Use default SPI peripheral and SPI settings.


void setup()
{
  // USB/debug serial coms
  pinMode(WRIST_ROTATION_CS_PIN, OUTPUT);
  digitalWrite(WRIST_ROTATION_CS_PIN, HIGH);

  Serial.begin(115200);

//   pinMode(SPI_DRV_ENN, OUTPUT);
//   digitalWrite(SPI_DRV_ENN, LOW);
//   pinMode(CLOCK, OUTPUT); 
//   digitalWrite(CLOCK, LOW);
//   pinMode(STEP, OUTPUT); 
//   digitalWrite(STEP, LOW);
//   pinMode(DIR, OUTPUT); 
//   digitalWrite(DIR, LOW); // Active low

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

#ifdef TRINAMIC_EXAMPLE


/* The trinamic TMC5160 motor controller and driver operates through an
* SPI interface. Each datagram is sent to the device as an address byte
* followed by 4 data bytes. This is 40 bits (8 bit address and 32 bit word).
* Each register is specified by a one byte (MSB) address: 0 for read, 1 for
* write. The MSB is transmitted first on the rising edge of SCK.
*
* Arduino Pins Eval Board Pins
* 51 MOSI 32 SPI1_SDI
* 50 MISO 33 SPI1_SDO
* 52 SCK 31 SPI1_SCK
* 25 CS 30 SPI1_CSN
*
* 17 DIO 8 DIO0 (DRV_ENN)
* 11 DIO REFL_STEP pin header
* 4 DIO 22 SPI_MODE * 5 DIO 20 SD_MODE
* 20 DIO DIAG0 pin header * GND 2 GND
* +5V 5 +5V
*/


#define chipCS 8
const byte STEPOUT = 11;
int enable = 17;
int SPImode = 4;
int SDmode = 5;
int DIAG0interruptpin = 20;
unsigned long SGvalue = 0;
unsigned long SGflag = 0;
int cnt = 0;
void setup() {
//put your setup code here, to run once:
pinMode(SPImode, OUTPUT);
pinMode(SDmode, OUTPUT);
pinMode(chipCS, OUTPUT);
pinMode(STEPOUT, OUTPUT);
pinMode(enable, OUTPUT);
digitalWrite(SPImode, HIGH);
digitalWrite(SDmode, HIGH);
digitalWrite(chipCS, HIGH);
digitalWrite(enable, HIGH);
#ifdef diag0_interrupt
pinMode(DIAG0interruptpin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(DIAG0interruptpin), stallguardstop, FALLING);
#endif set up Timer1
//for step generation
TCCR1A = bit(COM1A0);
//toggle OC1A on Compare Match TCCR1B = bit(WGM12) CTC, no prescaling | bit(CS10) | bit(CS12);
//Pre scaler set to 1024 resulting in 7.8125 kHz
OCR1A = 0;
//output every cycle
SPI.setBitOrder(MSBFIRST);
SPI.setClockDivider(SPI_CLOCK_DIV8);
SPI.setDataMode(SPI_MODE3);
SPI.begin();
Serial.begin(9600);
sendData(0x80, 0x00000080);
//GCONF -> Activate diag0_stall(Datasheet Page 31)
sendData(0xED, 0x00000000);
//SGT -> Needs to be adapted to get a StallGuard2 event
sendData(0x94, 0x00000040);
// TCOOLTHRS -> TSTEP based threshold = 55(Datasheet Page 38)
sendData(0x89, 0x00010606);
//SHORTCONF
sendData(0x8A, 0x00080400);
//DRV_CONF
sendData(0x90, 0x00080303);
//IHOLD_IRUN
sendData(0x91, 0x0000000A);
//TPOWERDOWN
sendData(0xAB, 0x00000001);
//VSTOP
sendData(0xBA, 0x00000001);
//ENC_CONST
sendData(0xEC, 0x15410153);
//CHOPCONF
sendData(0xF0, 0xC40C001E);
//PWMCONF
digitalWrite(enable, LOW);
}
void loop() {
SGvalue = sendData(0x6F, 0x00000000);
//Read DRV_STATUS register
SGflag = (SGvalue & 0x1000000) >> 24;
//Check SG2 flag
SGvalue = (SGvalue & 0x3FF);
//Extract SG2 value bits
Serial.print("StallGuard2 value: ");
Serial.println(SGvalue, DEC);
Serial.print("StallGuard2 event:");
Serial.println(SGflag);
}
unsigned long sendData(unsigned long address, unsigned long datagram) {
//TMC5130 takes 40 bit data: 8 address and 32 data
unsigned long i_datagram = 0;
digitalWrite(chipCS, LOW);
delayMicroseconds(10);
SPI.transfer(address);
i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
i_datagram <<= 8;
i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
i_datagram <<= 8;
i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
i_datagram <<= 8;
i_datagram |= SPI.transfer((datagram) & 0xff);
digitalWrite(chipCS, HIGH);
Serial.print("Received: ");
Serial.println(i_datagram, HEX);
Serial.print(" from register: ");
Serial.println(address, HEX);
return i_datagram;
}
void stallguardstop() {
cnt++;
// To avoid false detection.
// When motor starts diag0 pin goes low.
if (cnt > 1) {
TCCR1B &= (0 << CS12);
TCCR1B &= (0 << CS11);
TCCR1B &= (0 << CS10);
}
}
#endif