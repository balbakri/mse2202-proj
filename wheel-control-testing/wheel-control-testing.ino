#include <Servo.h>
#include <EEPROM.h>
#include <I2CEncoder.h>

const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
unsigned long ul_Echo_Time;

//wheel motors
byte b_LowByte;
byte b_HighByte;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;
const int stop_motor = 1500;
unsigned int ui_Motors_Speed = 1900; 
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

Servo servo_RightMotor;
Servo servo_LeftMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//#define DEBUG_ENCODERS
//#define DEBUG_MOTORS


void setup() {
  Serial.begin(9600);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // setup wheel motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);



  /*encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
*/

  /*b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);*/
}
void loop() {

  //ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
  //ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);


  servo_LeftMotor.writeMicroseconds(1650);
  servo_RightMotor.writeMicroseconds(1650);

  /*EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
    EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
    EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
    EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));*/




#ifdef DEBUG_ENCODERS
  l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

  Serial.print("Encoders L: ");
  Serial.print(l_Left_Motor_Position);
  Serial.print(", R: ");
  Serial.println(l_Right_Motor_Position);
# endif

#ifdef DEBUG_MOTORS
          //Serial.print("Motors enabled: ");
          //Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif


}
