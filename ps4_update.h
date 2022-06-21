#ifndef PS4_UPDATE_H
#define PS4_UPDATE_H

#include <PS4BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include <SPI.h>

//USB Usb;
//BTD Btd(&Usb);
//PS4BT PS4(&Btd, PAIR);    //for pairing
//PS4BT PS4(&Btd);

bool button[20];
int8_t axis[6];

float softening = 0.2;
float max_speed = 1;
float max_rotation_speed = 1;
int16_t deadband = 100; 

void ps4_update() {
  //analog button
  axis[0] = (PS4.getAnalogButton(L2));
  axis[1] = (PS4.getAnalogButton(R2));
  if (PS4.getAnalogHat(LeftHatX) < 0) {
    axis[2] = -(PS4.getAnalogHat(LeftHatX)) - 126;
  }
  else {
    axis[2] = 127 - (PS4.getAnalogHat(LeftHatX));
  }
  if (PS4.getAnalogHat(LeftHatY) < 0) {
    axis[3] = -(PS4.getAnalogHat(LeftHatY)) - 126;
  }
  else {
    axis[3] = 127 - (PS4.getAnalogHat(LeftHatY));
  }
  if (PS4.getAnalogHat(RightHatX) < 0) {
    axis[4] = -(PS4.getAnalogHat(RightHatX)) - 126;
  }
  else {
    axis[4] = 127 - (PS4.getAnalogHat(RightHatX));
  }

  if (PS4.getAnalogHat(RightHatY) < 0) {
    axis[5] = -(PS4.getAnalogHat(RightHatY)) - 126;
  }
  else {
    axis[5] = 127 - (PS4.getAnalogHat(RightHatY));
  }


  //buttonstate
  if (PS4.getButtonClick(UP)) {
    button[0] = !button[0];
    if (max_speed <= 3.0)
    {
      max_speed += 0.5;
    }
    else{
      PS4.setRumbleOn(button[0],button[1]);
    }
  }
  if (PS4.getButtonClick(DOWN)) {
    button[1] = !button[1];
    if (max_speed >= 1.0)
    {
      max_speed -= 0.5;
    }
    else{
      PS4.setRumbleOn(button[1],button[0]);
    }
  }
  if (PS4.getButtonClick(LEFT)) {
    button[2] = !button[2];
  }
  if (PS4.getButtonClick(RIGHT)) {
    button[3] = !button[3];
  }

  button[4] = PS4.getButtonClick(SQUARE);
  button[5] = PS4.getButtonClick(CROSS);
  button[6] = PS4.getButtonClick(CIRCLE);
  button[7] = PS4.getButtonClick(TRIANGLE);

  button[8] = PS4.getButtonClick(SHARE);
  button[9] = PS4.getButtonClick(OPTIONS);
  if (PS4.getButtonClick(PS)){
    button[10] = !button[10];
  }
  button[11] = PS4.getButtonClick(TOUCHPAD);

  if (PS4.getButtonClick(L1)) {
    button[12] = !button[12];
    if (softening <= 0.8)
    {
      softening += 0.2;
    }
    else{
      PS4.setRumbleOn(button[12],button[13]);
    }
  }
  if (PS4.getButtonClick(R1)) {
    button[13] = !button[13];
    if (softening >= 0.05)
    {
      softening -= 0.2;
    }
    else{
      PS4.setRumbleOn(button[13],button[12]);
    }
  }
  button[14] = PS4.getAnalogButton(L3);
  button[15] = PS4.getAnalogButton(R3);

    if (max_speed >= 2.5){
      PS4.setLed(Red);
    }
    else if (max_speed < 2.5 && max_speed >= 1.5){
      PS4.setLed(Yellow);
    }
    else if (max_speed < 1.5){
      PS4.setLed(Green);
    }
    else{
      PS4.setLed(Blue);
    }
}
   //vabration
    //PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    //PS4.setRumbleOn(RumbleLow);
    //PS4.setRumbleOn(RumbleHigh);

    //blink
    //PS4.setLedFlash(10, 10);
    //PS4.setLedFlash(0, 0);
    

#endif
