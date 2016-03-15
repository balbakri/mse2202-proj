//code for testing hall effect sensor
void setup() {
Serial.begin(9600);

}

int mag_field = 0;

void loop() {

mag_field = analogRead(A3);
Serial.println(mag_field);

}
