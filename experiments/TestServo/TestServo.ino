#include "Pin.h"
#include "Servo.h"


Servo servo;

PinTrigger<FALLING> pinDown (8, HIGH);
PinTrigger<FALLING> pinUp   (9, HIGH);


// Modelcraft Y-3009
//   cw:   550
//   ccw: 2400

int microsServo = 1500;


void setup()
{
  Serial.begin(9600);

  servo.attach(2);
  updateServo();
}


void updateServo()
{
  Serial.print(microsServo);
  Serial.println(F(" microseconds"));

  servo.writeMicroseconds(microsServo);
}


void loop()
{
  if (pinDown) {
    microsServo -= 50;
    updateServo();
    delay(200);
  }

  if (pinUp) {
    microsServo += 50;
    updateServo();
    delay(200);
  }
}
