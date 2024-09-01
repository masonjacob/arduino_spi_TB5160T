#include <SPI.h>
#include "TMC5130_registers.h"

/* The trinamic TMC5130 motor controller and driver operates through an 
 *  SPI interface.  Each datagram is sent to the device as an address byte
 *  followed by 4 data bytes.  This is 40 bits (8 bit address and 32 bit word).
 *  Each register is specified by a one byte (MSB) address: 0 for read, 1 for 
 *  write.  The MSB is transmitted first on the rising edge of SCK.
 *  
 * Arduino Pins   Eval Board Pins
 * 51 MOSI        32 SPI1_SDI
 * 50 MISO        33 SPI1_SDO
 * 52 SCK         31 SPI1_SCK
 * 25 CS          30 SPI1_CSN
 * 17 DIO         8 DIO0 (DRV_ENN)
 * 11 DIO         23 CLK16
 * GND            2 GND
 * +5V            5 +5V
 */

int chipCS = 10;
const byte CLOCKOUT = 13;
int enable = 17;
const float r_sense = 0.075;


void sendData(unsigned long address, unsigned long datagram) {
  //TMC5130 takes 40 bit data: 8 address and 32 data
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  delay(100);
  unsigned long i_datagram;

  digitalWrite(chipCS,LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(chipCS,HIGH);

  Serial.print("Received: ");
  Serial.println(i_datagram,HEX);
  Serial.print(" from register: ");
  Serial.println(address,HEX);

  SPI.endTransaction();
}

/*
  Requested current = mA = I_rms/1000
  Equation for current:
  I_rms = GLOBALSCALER/256 * (CS+1)/32 * V_fs/R_sense * 1/sqrt(2)
  Solve for GLOBALSCALER ->
  
                 32 * 256 * sqrt(2) * I_rms * R_sense    |
  GLOBALSCALER = ------------------------------------    |
                           (CS + 1) * V_fs               | V_fs = 0.325
  
*/
void print_current_settings(uint16_t mA) {
  uint32_t V_fs = 325; // x1000
  uint8_t CS = 31;
  float holdMultiplier = 0.3;
  uint32_t numerator = 1414UL * mA;
  numerator *= r_sense*1000UL;
  uint32_t scaler = numerator / V_fs; // (CS+1) is always 32
  scaler <<= (8); // Multiply by 256
  scaler /= 1000UL;
  scaler /= 1000UL;
  if (scaler < 32) scaler = 32; // Not allowed for operation
  Serial.println("scalar: " + String(scaler));
  Serial.println("IRUN: " + String(CS));
  Serial.println("IHOLD: " + String(CS*holdMultiplier));
}

void send_setup_commands() {
  sendData(0x80,0x00000000);      //GCONF
  sendData(0x8b,0x00000000);      //Global Scalar

  sendData(0xEC,0x000101D5);      //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0x90,0x00061F0A);      //IHOLD_IRUN: IHOLD=10, IRUN=31 (max.current), IHOLDDELAY=6
  sendData(0x8B,0x000000A7);      //GlobalScaler=167
  sendData(0x91,0x0000000A);      //TPOWERDOWN=10

  sendData(0xF0,0x00000000);      // PWMCONF
  //sendData(0xF0,0x000401C8);      //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amp limit=200, grad=1

  sendData(0xA4,0x000003E8);     //A1=1000
  sendData(0xA5,0x000186A0);     //V1=100000
  sendData(0xA6,0x0000C350);     //AMAX=50000
  sendData(0xA7,0x000186A0);     //VMAX=100000
  sendData(0xAA,0x00000578);     //D1=1400
  sendData(0xAB,0x0000000A);     //VSTOP=10

  sendData(0xA0,0x00000000);     //RAMPMODE=0

  sendData(0xA1,0x00000000);     //XACTUAL=0
  sendData(0xAD,0x00000000);     //XTARGET=0
  sendData(0xAD,0x0007D000);    //XTARGET=512000 | 10 revolutions with micro step = 256
}  

void setup() {
  // put your setup code here, to run once:
  pinMode(chipCS,OUTPUT);
  pinMode(CLOCKOUT,OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(chipCS,HIGH);
  digitalWrite(enable,LOW);

  //set up Timer1
  TCCR1A = bit (COM1A0);                //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);    //CTC, no prescaling
  OCR1A = 0;                            //output every cycle

  SPI.begin();

  // SPI.setBitOrder(MSBFIRST);
  // SPI.setClockDivider(SPI_CLOCK_DIV8);
  // SPI.setDataMode(SPI_MODE3);

  Serial.begin(9600);
  send_setup_commands();
  print_current_settings(2000);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming message until newline character
    String message = Serial.readStringUntil('\n');

    // Extract hex address and command from the message
    long address = 0;
    long command = 0;
    int commaIndex = message.indexOf(',');
    if (commaIndex != -1) {
      String addressStr = message.substring(0, commaIndex);
      String commandStr = message.substring(commaIndex + 1);

      // Convert hex strings to long integers
      address = strtol(addressStr.c_str(), NULL, 16);
      command = strtol(commandStr.c_str(), NULL, 16);

      // Call the function with extracted address and command
      sendData(address, command);
    }
  }

  // sendData(0x6F,0x00000000);
  // delay(1000);
  // // put your main code here, to run repeatedly:
  // // sendData(0xAD,0x0007D000);     //XTARGET=512000 | 10 revolutions with micro step = 256
  
  // // delay(1000);
  // // //sendData(0x01,0x00000111);
  // // // sendData(0xAD,0x00000000);     //XTARGET=0
  // // // delay(20000);
  // // sendData(0x01,0x00000000);
}
