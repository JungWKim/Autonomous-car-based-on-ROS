#define trig 37
#define echo 33

// detectable distance : 2 ~ 230 cm

float duration, distance;

void setup()
{
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    Serial.begin(57600);
}

void loop()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = ((340 * duration) / 10000) / 2;
    Serial.print("distance :");
    Serial.println(distance);
    delay(1000);
}

//const int pingPin = 5; // Trigger Pin of Ultrasonic Sensor
//const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
//
//void setup() {
//   Serial.begin(9600); // Starting Serial Terminal
//}
//
//void loop() {
//   long duration, inches, cm;
//   pinMode(pingPin, OUTPUT);
//   digitalWrite(pingPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(pingPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(pingPin, LOW);
//   pinMode(echoPin, INPUT);
//   duration = pulseIn(echoPin, HIGH);
//   inches = microsecondsToInches(duration);
//   cm = microsecondsToCentimeters(duration);
//   Serial.print(inches);
//   Serial.print("in, ");
//   Serial.print(cm);
//   Serial.print("cm");
//   Serial.println();
//   delay(1000);
//}
//
//long microsecondsToInches(long microseconds) {
//   return microseconds / 74 / 2;
//}
//
//long microsecondsToCentimeters(long microseconds) {
//   return microseconds / 29 / 2;
//}
