#include <TimerOne.h>
#include "fix16.h"

/*     ENCODERS      */
#define encoder1IntPin 2
#define encoder1Enc1  22
#define encoder1Enc2  24

#define encoder2IntPin 3
#define encoder2Enc1  26
#define encoder2Enc2  28

#define encoder3IntPin 18
#define encoder3Enc1  17
#define encoder3Enc2  16

#define encoder4IntPin 19
#define encoder4Enc1  21
#define encoder4Enc2  20

int16_t encoder1Pos = 0;
uint8_t encoder1State;
int16_t encoder2Pos = 0;
uint8_t encoder2State;
int16_t encoder3Pos = 0;
uint8_t encoder3State;
int16_t encoder4Pos = 0;
uint8_t encoder4State;


/*    MOTORS    */
#define motor1DirPin 5
#define motor1PWMPin 4
fix16_t motor1Current;
#define motor1CurentPin 12

#define motor2DirPin 7
#define motor2PWMPin 6
fix16_t motor2Current;
#define motor2CurentPin 13

#define motor3DirPin 9
#define motor3PWMPin 8
fix16_t motor3Current;
#define motor3CurentPin 14

#define motor4DirPin 11
#define motor4PWMPin 10
fix16_t motor4Current;
#define motor4CurentPin 15

#define MAX_SPEED 4194304

fix16_t target_speed_left;
fix16_t target_speed_right;
fix16_t speed1,speed2,speed3,speed4;
fix16_t motor1state,motor2state,motor3state,motor4state;
uint8_t trigger;
uint8_t moving;

/*    LINE SENSORS    */
#define sensor1Pin 0
uint16_t sensor1val;
#define sensor2Pin 1
uint16_t sensor2val;
#define sensor3Pin 2
uint16_t sensor3val;


void processEncoder(uint8_t &state, uint8_t enc1_pin, uint8_t enc2_pin, int16_t &pos);
void moveMotor(uint8_t motor, fix16_t vel);
void sendControl(uint8_t ctrl, uint8_t value);

void setup() {

  /*   Initialize encoders */
  pinMode(encoder1Enc1, INPUT);
  pinMode(encoder1Enc2, INPUT);
  pinMode(encoder1IntPin, INPUT);
  digitalWrite(encoder1Enc1, HIGH);       // turn on pullup resistor
  digitalWrite(encoder1Enc2, HIGH);       // turn on pullup resistor
  digitalWrite(encoder1IntPin, HIGH);       // turn on pullup resistor

  pinMode(encoder2Enc1, INPUT);
  pinMode(encoder2Enc2, INPUT);
  pinMode(encoder2IntPin, INPUT);
  digitalWrite(encoder2Enc1, HIGH);       // turn on pullup resistor
  digitalWrite(encoder2Enc2, HIGH);       // turn on pullup resistor
  digitalWrite(encoder2IntPin, HIGH);       // turn on pullup resistor

  pinMode(encoder3Enc1, INPUT);
  pinMode(encoder3Enc2, INPUT);
  pinMode(encoder3IntPin, INPUT);
  digitalWrite(encoder3Enc1, HIGH);       // turn on pullup resistor
  digitalWrite(encoder3Enc2, HIGH);       // turn on pullup resistor
  digitalWrite(encoder3IntPin, HIGH);       // turn on pullup resistor

  pinMode(encoder4Enc1, INPUT);
  pinMode(encoder4Enc2, INPUT);
  pinMode(encoder4IntPin, INPUT);
  digitalWrite(encoder4Enc1, HIGH);       // turn on pullup resistor
  digitalWrite(encoder4Enc2, HIGH);       // turn on pullup resistor
  digitalWrite(encoder4IntPin, HIGH);       // turn on pullup resistor

  encoder1State = digitalRead(encoder1Enc1)<<1|digitalRead(encoder1Enc2);
  encoder2State = digitalRead(encoder2Enc1)<<1|digitalRead(encoder2Enc2);
  encoder3State = digitalRead(encoder3Enc1)<<1|digitalRead(encoder3Enc2);
  encoder4State = digitalRead(encoder4Enc1)<<1|digitalRead(encoder4Enc2);

  attachInterrupt(0, processEncoder1, CHANGE);
  attachInterrupt(1, processEncoder2, CHANGE);
  attachInterrupt(5, processEncoder3, CHANGE);
  attachInterrupt(4, processEncoder4, CHANGE);


  /*   Initialize motors   */

  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor1PWMPin, OUTPUT);

  pinMode(motor2DirPin, OUTPUT);
  pinMode(motor2PWMPin, OUTPUT);

  pinMode(motor3DirPin, OUTPUT);
  pinMode(motor3PWMPin, OUTPUT);

  pinMode(motor4DirPin, OUTPUT);
  pinMode(motor4PWMPin, OUTPUT);

  Timer1.initialize(100000);
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

  motor1state = 0;
  motor2state = 0;
  motor3state = 0;
  motor4state = 0;

  TCCR0B = (TCCR0B & 0xF8) | 2;
  TCCR2B = (TCCR2B & 0xF8) | 2 ;
  TCCR4B = (TCCR4B & 0xF8) | 2 ;

  target_speed_left = 0;
  target_speed_right = 0;
  Serial.begin (115200);
}

void saturate(fix16_t &v){
  v = v>MAX_SPEED?MAX_SPEED:(v< -MAX_SPEED? -MAX_SPEED:v);
}

void loop(){
  uint8_t inByte;
  uint8_t ctrl,val,midi_state=0;
  int8_t sval;
  fix16_t abs_vel1,abs_vel2;

  while(1)
  {
    if(Serial.available()>0)
    {
      inByte = Serial.read();
      if(midi_state==0 && inByte==0xB0)
      {
        midi_state = 1;
      }
      else if(midi_state==1){
        ctrl = inByte;
        midi_state = 2;
      }
      else if(midi_state==2){
        val = inByte;
        sval = val;
        sval = sval-64;
        midi_state = 0;
        if(ctrl==0x50){
          target_speed_left = fix16_from_int(sval);
        }
        if(ctrl==0x51){
          target_speed_right = fix16_from_int(sval);
        }
      }
      else
        midi_state = 0;
    }

    abs_vel1 = target_speed_left>0?target_speed_left:-target_speed_left;
    abs_vel2 = target_speed_right>0?target_speed_right:-target_speed_right;
    if(abs_vel1>fix16_one || abs_vel2>fix16_one)
      moving = 1;
    else
      moving = 0;

    if(trigger!=0){
       fix16_t diff;
       fix16_t currentMatch_left,speedMatch_left,currentMatch_right,speedMatch_right;

       trigger = 0;

       #define CURRENT_P_GAIN 32768   // 0.3
       #define CURRENT_I_GAIN 19660   // 0.7

       #define SPEED_P_GAIN 45875   // 0.7
       #define SPEED_I_GAIN 32768   // 0.5

       // Motor 1 follows the current of motor 3
       diff = fix16_sub((motor3state>0?motor3Current:-motor3Current),motor1state>0?motor1Current:-motor1Current);
       motor1state = fix16_add(fix16_mul(diff,CURRENT_I_GAIN),motor1state);
       //saturate(motor1state);
       currentMatch_left = fix16_mul(diff,CURRENT_P_GAIN) + motor1state;


       diff = fix16_sub(target_speed_left,speed3);
       motor3state = fix16_add(fix16_mul(diff,SPEED_I_GAIN),motor3state);
       //saturate(motor3state);
       speedMatch_left = fix16_mul(diff,SPEED_P_GAIN) + motor3state;

       moveMotor(1,currentMatch_left);
       moveMotor(3,speedMatch_left);

       // Motor 2 follows the current of motor 4
       diff = fix16_sub(motor4state>0?motor4Current:-motor4Current,motor2state>0?motor2Current:-motor2Current);
       motor2state = fix16_add(fix16_mul(diff,CURRENT_I_GAIN),motor2state);
       //saturate(motor2state);
       currentMatch_right = fix16_mul(diff,CURRENT_P_GAIN) + motor2state;

       diff = fix16_sub(target_speed_right,speed4);
       motor4state = fix16_add(fix16_mul(diff,SPEED_I_GAIN),motor4state);
       //saturate(motor4state);
       speedMatch_right = fix16_mul(diff,SPEED_P_GAIN) + motor4state;

       moveMotor(2,currentMatch_right);
       moveMotor(4,speedMatch_right);

       // Send information
       sendControl(0x20,abs(fix16_to_int(speed1)));
       sendControl(0x21,abs(fix16_to_int(speed2)));
       sendControl(0x22,abs(fix16_to_int(speed3)));
       sendControl(0x23,abs(fix16_to_int(speed4)));

       sendControl(0x30,fix16_to_int(motor1Current));
       sendControl(0x31,fix16_to_int(motor2Current));
       sendControl(0x32,fix16_to_int(motor3Current));
       sendControl(0x33,fix16_to_int(motor4Current));
       
       sendControl(0x40,sensor1val);
       sendControl(0x41,sensor2val);
       sendControl(0x42,sensor3val);

    }
  }
}

void timerIsr()
{
    int v;
    speed1 = fix16_from_int(encoder1Pos);
    v = analogRead(motor1CurentPin);
    delayMicroseconds(20);

    motor1Current = fix16_from_int(v);
    speed2 = fix16_from_int(encoder2Pos);
    v = analogRead(motor2CurentPin);
    delayMicroseconds(20);

    motor2Current = fix16_from_int(v);
    speed3 = fix16_from_int(encoder3Pos);
    v = analogRead(motor3CurentPin);
    delayMicroseconds(20);


    motor3Current = fix16_from_int(v);
    speed4 = fix16_from_int(encoder4Pos);
    v = analogRead(motor4CurentPin);
    motor4Current = fix16_from_int(v);


    sensor1val = analogRead(sensor1Pin) >> 2;delayMicroseconds(20);
    sensor2val = analogRead(sensor2Pin) >> 2;delayMicroseconds(20);
    sensor3val = analogRead(sensor3Pin) >> 2;delayMicroseconds(20);


    encoder1Pos=0;
    encoder2Pos=0;
    encoder3Pos=0;
    encoder4Pos=0;

    trigger=1;
}

void moveMotor(uint8_t motor, fix16_t vel)
{
  uint8_t dir,dir_fixed;
  uint8_t vel_int;
  fix16_t fixed_vel;
  dir = vel>0?1:0;
  fixed_vel = vel>0?vel:fix16_sub(0,vel);
  vel_int = fix16_to_int(fixed_vel);

  if(moving==0) vel_int = 0;
  if(motor==1 || motor==2)
    dir_fixed = dir;
  else
    dir_fixed = dir!=0?0:1;
  switch(motor)
  {
    case 1:
      digitalWrite(motor1DirPin,dir_fixed);
      analogWrite(motor1PWMPin,vel_int);
      break;
    case 2:
      digitalWrite(motor2DirPin,dir_fixed);
      analogWrite(motor2PWMPin,vel_int);
      break;
    case 3:
      digitalWrite(motor3DirPin,dir_fixed);
      analogWrite(motor3PWMPin,vel_int);
      break;
    case 4:
      digitalWrite(motor4DirPin,dir_fixed);
      analogWrite(motor4PWMPin,vel_int);
      break;
  }
}

void processEncoder1() {
  processEncoder(encoder1State,encoder1Enc1,encoder1Enc2,encoder1Pos);
}

void processEncoder2() {
  processEncoder(encoder2State,encoder2Enc1,encoder2Enc2,encoder2Pos);
}

void processEncoder3() {
  processEncoder(encoder3State,encoder3Enc1,encoder3Enc2,encoder3Pos);
}

void processEncoder4() {
  processEncoder(encoder4State,encoder4Enc1,encoder4Enc2,encoder4Pos);
}



void processEncoder(uint8_t &state, uint8_t enc1_pin, uint8_t enc2_pin, int16_t &pos) {

  uint8_t enc1 = digitalRead(enc1_pin);
  uint8_t enc2 = digitalRead(enc2_pin);
  switch(state)
  {
    case 0:
      if(enc2!=0){
         pos++;
         state = 1;
       }
      else {
        pos--;
        state = 2;
      }
      break;
     case 1:
      if(enc1!=0){
         pos++;
         state = 3;
       }
      else {
        pos--;
        state = 0;
      }
      break;
     case 2:
      if(enc2!=0){
         pos--;
         state = 3;
       }
      else {
        pos++;
        state = 0;
      }
      break;
     case 3:
      if(enc1==0){
         pos--;
         state = 1;
       }
      else {
        pos++;
        state = 2;
      }
      break;
  }
}

void sendControl(uint8_t ctrl, uint8_t value)
{
  Serial.write(0xB0);
  Serial.write(ctrl);
  Serial.write(value);
}

