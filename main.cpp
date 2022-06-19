#include "ThisThread.h"
#include <algorithm>
#include "mbed.h"
#include "vesc.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
#include "c620/c620.h"
#define MAXIMUM_BUFFER_SIZE   32   
#include"Sender.h"
using namespace std::chrono;
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

Ticker ticker;
int32_t can_baud = 1000000;
CAN can(PB_5,PB_6,can_baud);    //can2, for arduino
CAN can2(PA_11,PA_12,can_baud); //can1, for bldc
int can_id = 80;
int can_id2 =90;
volatile uint16_t LX_L;
volatile uint16_t RX_R;
volatile int8_t elva;
volatile int8_t trig;
volatile int speed = 0;
volatile int dis;
const int sens = 100;
volatile int8_t mm;
int speedOffSet = 0;
volatile bool normal = true;
volatile bool reloadd = false;

DigitalIn upbutton(PC_13); // User Button
DigitalOut led(LED1);
bool state = false;
vesc _vesc1;
vesc _vesc2;
c620 _m3508;
DigitalOut fire(PB_3);
int convert(float x);

char var[]= {'1','2','3'};
// main() runs in its own thread in the OS
const char head[] = {'A'};
const char foot[] = {'B'};
int Is_digit(int speed);


int convert(float x){
    dis = x/sens;
    return dis;
}

int Is_digit(int speed){
        int digit = 1;
        while(speed > 10){
        speed = speed/10;
        digit++;
        }
        return digit;
}

void checkInputChar(char inputChar){
    static char inputString[32] = {0};
    static int inputStringCounter = 0;
    static int inputValue = 0;
    bool valueIsEnter = false;
        switch(inputChar){
        case 's':
            inputValue = atoi(inputString);
            inputStringCounter = 0;
            memset(inputString, 0, 8);
            valueIsEnter = true;
            speedOffSet = inputValue;
        break;
        default:
            inputString[inputStringCounter] = inputChar;
            inputStringCounter++;
            if (inputStringCounter >= 8) {
                inputStringCounter = 0;
                memset(inputString, 0, 8);
            }
        break;
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(){

  //printf("main running\r\n");
  CANMessage msg;
  _vesc1.vesc_init(&can, can_baud);
  _vesc1.set_monitor_id(can_id); 
  _vesc2.vesc_init(&can, can_baud);
  _vesc2.set_monitor_id(can_id2); 
  
  _m3508.c620_init(&can2);
  _m3508.set_i_pid_param(0, 1.0, 0.000, 0.000000); // Torque PID W1
  _m3508.set_v_pid_param(0, 1.0, 0.000, 0.000);  // Velocity PID W1
  _m3508.set_p_pid_param(0, 5.0, 5.0, 0.0010);     // position PID W1
  
  // set the LP filter of the desire control
  _m3508.profile_velocity_CCW[0] = 7000;//Maximum is 12000 for c620 and 10000 for c610
  _m3508.profile_velocity_CW[0] = -7000;
  _m3508.profile_torque_CCW[0] = 8000; //Maximum is 16000 for c620 and 10000 for c610
  _m3508.profile_torque_CW[0] = -8000;
  // set the current limit, overcurrent may destory the driver
  _m3508.motor_max_current = 8000; // 10000 max for c610+2006 16000 max for c620
  // output position = motor rotation(deg) * gear ratio.  2006 = 1:36 , 3508
  // = 1:19 rotate 180 deg
  //_m3508.set_position(0, 0);// set starting position as 0
  _m3508.set_velocity(0, 100);
  Timer pidTimer, printerTimer, espTimer, keyboardTimer;
  pidTimer.start();
  printerTimer.start();
  espTimer.start();
  keyboardTimer.start();
  int yaw = 0;
  Sender<int> sender(PA_9, PA_10);
    while(1){
        if(espTimer.elapsed_time() > 100ms){
            *sender.original = speed;
            sender.send();
            espTimer.reset();
        }
        if(pidTimer.elapsed_time() > 1ms){
            if(normal ==true){
            //printf("normal State\n"); 
            _m3508.c620_read();
            _m3508.c620_calc();
            _vesc1.set_rpm(can_id, -speed);
            _vesc2.set_rpm(can_id2, speed - speedOffSet);
            /*if (abs(_m3508.global_angle[0]) > 30000 && sgn(yaw) == sgn(_m3508.global_angle[0])) {
                yaw = 0;
            }*/
            _m3508.set_velocity(0, yaw);
            pidTimer.reset();
            }
            if(reloadd == true){
                printf("reloading\n");
                _m3508.set_position(0, 100);
                _m3508.c620_read();
                _m3508.c620_calc();
                normal = true;
                reloadd = false;
                //ThisThread::sleep_for(50);
                printf("end loading\n");
                pidTimer.reset();
            }
        }
        // if (printerTimer.elapsed_time() > 200ms){
        //     if (serial_port.writable()){
        //         printf("yaw speed: %d, yaw angle: %d, speed: %d off set: %d\r\n", yaw, int(_m3508.global_angle[0]), speed, speedOffSet);
        //     }
        //     printerTimer.reset();
        // }
        if (keyboardTimer.elapsed_time() > 100ms){
            if(serial_port.readable()){
                static char buffer[32];
                static int numberOfCharReceived = 0;
                numberOfCharReceived += serial_port.read(buffer, sizeof(buffer));
                for (int i = 0; i < numberOfCharReceived; i++){
                    checkInputChar(buffer[i]);
                    numberOfCharReceived--;
                }
            }
        }
        if(can.read(msg) && msg.id == 0x036){
             uint16_t LX_L = (uint16_t)(msg.data[0] << 8) | (msg.data[1]);
             uint16_t RX_R = (uint16_t)(msg.data[2] << 8) | (msg.data[3]);
             int8_t elva = (int8_t) msg.data[4];
             int8_t reload = (int8_t)msg.data[5];
            int8_t trig = (int8_t) msg.data[6];
            yaw = LX_L - 125;
            if (abs(yaw) < 15){
                yaw = 0;
            }
            //printf(" RX_R: %d  ", RX_R);
            //printf(" speed: %d\n ", speed);
            //printf(" trig: %c  \n", trig);
            //printf("pos: %d\t", yaw);
            //printf(" speed: %d \n", speed);

            if(elva == 'A'){
                speed += 1000;
            }
            if(elva == 'B'){
                speed -= 1000;
            }
            if(speed <= 0){
                    speed = 0; 
                }
            if(trig == '1'){
               fire = 1;
               printf("fire: \n");
                }
            if(trig != '1'){
                fire = 0;
            }
            if(reload == 'Y'){
                normal = false;
                reloadd = true;
            }
            

            }
        }
    }


