/*
 Example sketch for the Xbox One S Bluetooth library - developed by Kristian Sloth Lauszus
 For more information visit the Github repository: github.com/felis/USB_Host_Shield_2.0 or
 send me an e-mail:  lauszus@gmail.com
 */
//#include "ps4_update.h"
#include "common_operator.h"
#include <XBOXONESBT.h>
#include <usbhub.h>
// Satisfy the IDE, which needs to see the include statement in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <mcp2515.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the XBOXONESBT class in two ways */
// This will start an inquiry and then pair with the Xbox One S controller - you only have to do this once
// You will need to hold down the Sync and Xbox button at the same time, the Xbox One S controller will then start to blink rapidly indicating that it is in pairing mode
XBOXONESBT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the Xbox button on the device
//XBOXONESBT Xbox(&Btd);
struct can_frame Txmsg;
MCP2515 mcp2515(7);
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
volatile int16_t L2_X;
volatile int16_t R2_Y;
volatile int16_t TX;
volatile int8_t trig = '0';
volatile int8_t up = '0';
volatile int8_t reload = '0';

void setup() {
  // put your setup code here, to run once:
SPI.begin();
  Txmsg.can_id  = 0x036;
    Serial.begin(115200);
#if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
    if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start, debug the hardware connection."));
      while (1);
    }
    Serial.print(F("\r\nPS4 Bluetooth Library Started, press the logo for connection or press teh logo with share to pair the controller"));
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  // put your main code here, to run repeatedly:
Usb.Task();
if (PS4.connected()) {
    int16_t LL2_X = PS4.getAnalogHat(LeftHatX);
    L2_X = map(LL2_X, -32768, 32768,0,255);
    int16_t RR2_Y = PS4.getAnalogHat(RightHatY);
    R2_Y = map(RR2_Y, -32768, 32768,0,255);
    Serial.print(F("\r\nLeftHatX: "));
    Serial.print(L2_X);
    Serial.print(F("\tRightHatY: "));
    Serial.print(R2_Y);
    Serial.print(F("\tElevate: "));
    Serial.print(up);
    
    
    if (PS4.getButtonPress(RT)) { // These are the only analog buttons on the PS4 controller
      Serial.print(F("\tR2: "));
      volatile int16_t TX_R =PS4.getButtonPress(RT);
      TX = map(TX_R,0, 1023,0,255);
      if(TX>200){
      Serial.print(TX);
      trig = '1';
      }
    }
    if (PS4.getButtonClick(UP)) {
        up = 'A';
        Serial.print(F("\elevate: "));
        Serial.print(up);
        }
    if (PS4.getButtonClick(DOWN)) {
        up = 'B';
        Serial.print(F("\elevate: "));
        Serial.print(up);
        }
     if(PS4.getButtonClick(X)){
        reload = 'Y';
        Serial.print(F("\tReload: "));
        Serial.print(reload);
      }
     else{
        reload = '0';
        up = '0';
      }
    Txmsg.can_dlc = 8;
    Txmsg.data[0] = L2_X >> 8;
    Txmsg.data[1] = L2_X & 0xFF;
    Txmsg.data[2] = R2_Y >> 8;
    Txmsg.data[3] = R2_Y & 0xFF;
    Txmsg.data[4] = up;
    Txmsg.data[5] = reload;
    Txmsg.data[6] = trig;
    mcp2515.sendMessage(&Txmsg);
//    up = '0';
//    trig = '0';
  }
}
