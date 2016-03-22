//mode 1 code
#include <CharliePlexM.h>
#include <Servo.h>
#include <EEPROM.h>
#include <I2CEncoder.h>

const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
unsigned long ul_Echo_Time;

const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;


// Charlieplexing LED assignments
/*const int ci_Heartbeat_LED = 1;
  const int ci_Indicator_LED = 4;*/

const int ci_servo_scan = 10;

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
Servo servo_scan;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//#define DEBUG_ENCODERS
//#define DEBUG_MOTORS
//#define DEBUG_ULTRASONIC

/*************************SETUP**********************************/
void setup() {
  Serial.begin(9600);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // setup wheel motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  servo_scan.attach(ci_servo_scan);

}
/*************************END SETUP**********************************/

int previous = 0;
int current = 0;
bool drive = true;
int pos = 0;

void loop() {
  current = millis();
  //Ping();
  Serial.print("Previous: ");
  Serial.print(previous);
  Serial.print("Current: ");
  Serial.println(current);
  if (drive == true) {//drive forward
    CharliePlexM::Write(3, HIGH);
    //servo_LeftMotor.writeMicroseconds(1650);
    //servo_RightMotor.writeMicroseconds(1650);

    if ((current - previous) > 2000) { //stop after one second
      //servo_LeftMotor.writeMicroseconds(1500);
      //servo_RightMotor.writeMicroseconds(1500);
      drive = false;
    }
  }

  else if (drive == false) {
    CharliePlexM::Write(3, LOW);
    scan();
    drive = true;//after finished scanning continue driving
    current = millis();
    previous = current;
  }

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

bool clockwise = true;

void scan() {//rotates arm to scan for good tesseracks
  if (clockwise == false) {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo_scan.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    clockwise = true;
  }
  else {
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo_scan.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    clockwise = false;
  }
}

void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

#ifdef DEBUG_ULTRASONIC
  // Print Sensor Readings
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 24); //divide time by 58 to get distance in cm
#endif
  //indicate LED on Charlieplex when reached below 20 cm
  if ((ul_Echo_Time / 24) < 20) {
    CharliePlexM::Write(12, HIGH);
  }
  else {
    CharliePlexM::Write(12, LOW);
  }

}
