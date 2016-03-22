//mode 1 code
#include <CharliePlexM.h>

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

boolean bt_Heartbeat = true;

void setup() {
  Serial.begin(9600);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

}
void loop() {
  Ping();


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

  // Print Sensor Readings
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 24); //divide time by 58 to get distance in cm

  //indicate LED on Charlieplex when reached below 20 cm
  if (ul_Echo_Time / 24 < 20){
    CharliePlexM::Write(12, HIGH);
  }
  else{
    CharliePlexM::Write(12, LOW);
  }
}
