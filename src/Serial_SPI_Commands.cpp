#include <hidboot.h>
#include <usbhub.h>
#include <SPI.h>
// Satisfy the IDE, which needs to see the include statement in the ino too.

#define MAX_INPUT_LENGTH 40 // Maximum length for input command
#define RESPONSE_LENGTH 40 // Length of response from SPI slave
class KbdRptParser : public KeyboardReportParser
{
    char inputBuffer[MAX_INPUT_LENGTH]; // Buffer to store input command
    int bufferIndex = 0; // Current position in buffer
    void PrintKey(uint8_t mod, uint8_t key);
  protected:
    void OnControlKeysChanged(uint8_t before, uint8_t after);
    void OnKeyDown(uint8_t mod, uint8_t key);
    void OnKeyUp(uint8_t mod, uint8_t key);
    void OnKeyPressed(uint8_t key);
    void processCommand(const char *command); // Process and transmit the command via SPI
    void receiveSPIResponse(); // Receive and display the SPI response
};
void KbdRptParser::PrintKey(uint8_t m, uint8_t key)
{
  MODIFIERKEYS mod;
  *((uint8_t*)&mod) = m;
  Serial.print((mod.bmLeftCtrl   == 1) ? "C" : " ");
  Serial.print((mod.bmLeftShift  == 1) ? "S" : " ");
  Serial.print((mod.bmLeftAlt    == 1) ? "A" : " ");
  Serial.print((mod.bmLeftGUI    == 1) ? "G" : " ");

  Serial.print(" >");
  PrintHex<uint8_t>(key, 0x80);
  Serial.print("< ");

  Serial.print((mod.bmRightCtrl   == 1) ? "C" : " ");
  Serial.print((mod.bmRightShift  == 1) ? "S" : " ");
  Serial.print((mod.bmRightAlt    == 1) ? "A" : " ");
  Serial.println((mod.bmRightGUI    == 1) ? "G" : " ");
}
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
 Serial.print("DN ");
  PrintKey(mod, key);
  uint8_t c = OemToAscii(mod, key);

  if (c)
    OnKeyPressed(c);
}
void KbdRptParser::OnControlKeysChanged(uint8_t before, uint8_t after)
{
  MODIFIERKEYS beforeMod;
  *((uint8_t*)&beforeMod) = before;

  MODIFIERKEYS afterMod;
  *((uint8_t*)&afterMod) = after;

  if (beforeMod.bmLeftCtrl != afterMod.bmLeftCtrl) {
    Serial.println("LeftCtrl changed");
  }
  if (beforeMod.bmLeftShift != afterMod.bmLeftShift) {
    Serial.println("LeftShift changed");
  }
  if (beforeMod.bmLeftAlt != afterMod.bmLeftAlt) {
    Serial.println("LeftAlt changed");
  }
  if (beforeMod.bmLeftGUI != afterMod.bmLeftGUI) {
    Serial.println("LeftGUI changed");
  }

  if (beforeMod.bmRightCtrl != afterMod.bmRightCtrl) {
    Serial.println("RightCtrl changed");
  }
  if (beforeMod.bmRightShift != afterMod.bmRightShift) {
    Serial.println("RightShift changed");
  }
  if (beforeMod.bmRightAlt != afterMod.bmRightAlt) {
    Serial.println("RightAlt changed");
  }
  if (beforeMod.bmRightGUI != afterMod.bmRightGUI) {
    Serial.println("RightGUI changed");
  }
}
void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key)
{
    Serial.print("UP ");
    PrintKey(mod, key);
}
void KbdRptParser::OnKeyPressed(uint8_t key)
{
  if (key == '\n') { // Assuming newline character indicates end of command
    inputBuffer[bufferIndex] = 0; // Null-terminate the string
    processCommand(inputBuffer); // Process the command
    bufferIndex = 0; // Reset buffer index for next command
  } else if (bufferIndex < MAX_INPUT_LENGTH - 1) {
    inputBuffer[bufferIndex++] = key; // Add key to buffer
  }
}
void KbdRptParser::processCommand(const char *command)
{
  Serial.print("Transmitting SPI command: ");
  Serial.println(command);

  // Begin SPI transaction
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW); // Assume SS pin is defined elsewhere as the SPI slave select

  // Transmit the command byte by byte and receive response
  for (const char *p = command; *p; p++) {
    uint8_t receivedVal = SPI.transfer((uint8_t)*p);
    Serial.print("Received Byte: ");
    Serial.println(receivedVal, HEX); // Display each received byte
  }

  digitalWrite(SS, HIGH); // End transmission
  SPI.endTransaction();

  // Optionally, you can receive a fixed-length response after sending the command
  // receiveSPIResponse();
}

void KbdRptParser::receiveSPIResponse()
{
  char response[RESPONSE_LENGTH];
  digitalWrite(SS, LOW); // Begin response reception
  for (int i = 0; i < RESPONSE_LENGTH; i++) {
    response[i] = SPI.transfer(0x00); // Send dummy bytes to receive response
  }
  digitalWrite(SS, HIGH); // End reception

  // Print the received response
  Serial.print("SPI Response: ");
  for (int i = 0; i < RESPONSE_LENGTH; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

USB Usb;
//USBHub Hub(&Usb);
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);
KbdRptParser Prs;
void setup()
{
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
  }
  SPI.begin(); // Initialize SPI
  pinMode(SS, OUTPUT); // Set the SS pin as output
  HidKeyboard.SetReportParser(0, &Prs);
  Serial.println("Start");
}
void loop()
{
  Usb.Task();
}