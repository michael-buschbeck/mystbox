#include "Pin.h"
#include "Servo.h"
#include "Timer.h"

#include "avr/pgmspace.h"


uint8_t  const EASING_LENGTH =    64;
uint16_t const EASING_FINAL  = 32768;


uint16_t const easingRegular[EASING_LENGTH] PROGMEM = {
      0,    20,    40,    68,   134,   201,   295,   415,
    539,   721,   903,  1140,  1393,  1695,  2028,  2413,
   2833,  3319,  3833,  4435,  5064,  5778,  6540,  7356,
   8250,  9186, 10226, 11265, 12304, 13462, 14631, 15799,
  16968, 18136, 19305, 20463, 21502, 22541, 23581, 24517,
  25411, 26227, 26989, 27703, 28332, 28934, 29448, 29934,
  30354, 30739, 31072, 31374, 31627, 31864, 32046, 32228,
  32352, 32472, 32566, 32633, 32699, 32727, 32747, 32768,
};

uint16_t const easingOverswing[EASING_LENGTH] PROGMEM = {
      0,    18,    36,    62,   124,   186,   283,   400,
    541,   726,   934,  1205,  1515,  1889,  2349,  2870,
   3513,  4283,  5180,  6229,  7450,  8858, 10515, 12172,
  14083, 16000, 17903, 19803, 21702, 23510, 25198, 26886,
  28479, 29925, 31372, 32690, 33885, 34971, 35930, 36744,
  37426, 37970, 38370, 38627, 38742, 38723, 38580, 38328,
  37955, 37560, 37068, 36566, 36063, 35560, 35087, 34617,
  34217, 33844, 33506, 33238, 33036, 32897, 32809, 32768,
};

uint16_t const easingTwitch[EASING_LENGTH] PROGMEM = {
      0,    47,    96,   245,   416,   679,  1005,  1393,
   1912,  2520,  3238,  4091,  5110,  6320,  7765,  9496,
  11433, 13620, 16104, 18692, 21292, 23691, 25786, 27586,
  28986, 30097, 30963, 31620, 32103, 32443, 32655, 32734,
  32734, 32655, 32443, 32103, 31620, 30963, 30097, 28986,
  27586, 25786, 23691, 21292, 18692, 16104, 13620, 11433,
   9496,  7765,  6320,  5110,  4091,  3238,  2520,  1912,
   1393,  1005,   679,   416,   245,    96,    47,     0,
};


Servo servoScanner;
Servo servoLatch;

PinTrigger<FALLING> pinTriggerScan;

PinAnalog<INPUT> pinSensor[6];
PinDigital<OUTPUT> pinLight[6];


void moveServo(Servo& servo, int microsServoTo, unsigned long millisDuration)
{
  int microsServoFrom = servo.readMicroseconds();
  int microsServoDelta = microsServoTo - microsServoFrom;

  unsigned long millisTimeStart = millis();

  for (;;) {
    unsigned long millisTimeNow = millis();
    unsigned long millisElapsed = millisTimeNow - millisTimeStart;

    if (millisElapsed >= millisDuration)
      break;

    // interpolation must be performed as
    // - signed because microsServoDelta might be negative
    // - long because the multiplication might exceed int

    int microsServoInterpolated =
      microsServoFrom + static_cast<int>(static_cast<long>(millisElapsed) * microsServoDelta / static_cast<long>(millisDuration));

    servo.writeMicroseconds(microsServoInterpolated);
  }

  servo.writeMicroseconds(microsServoTo);
}


void moveServo(Servo& servo, int microsServoTo, unsigned long millisDuration, uint16_t const easing[])
{
  int microsServoFrom = servo.readMicroseconds();
  int microsServoDelta = microsServoTo - microsServoFrom;

  unsigned long millisTimeStart = millis();

  for (uint8_t iStep = 0; iStep < EASING_LENGTH; ++iStep) {
    unsigned long millisTimeNow = millis();
    unsigned long millisElapsed = millisTimeNow - millisTimeStart;

    if (millisElapsed >= millisDuration)
      break;

    unsigned long millisElapsedAfter = millisDuration * (iStep + 1) / EASING_LENGTH;

    if (millisElapsed < millisElapsedAfter) {
      uint16_t step = pgm_read_word(&easing[iStep]);

      // interpolation must be performed as
      // - signed because microsServoDelta might be negative
      // - long because the multiplication might exceed int

      int microsServoStep =
        microsServoFrom + static_cast<int>(static_cast<long>(step) * microsServoDelta / static_cast<long>(EASING_FINAL));

      unsigned long millisDurationStep = millisElapsedAfter - millisElapsed;

      moveServo(servo, microsServoStep, millisDurationStep);
    }
  }
}


int microsServoScanner[7] = {
  1940,  // position 0
  1820,  // position 1
  1660,  // position 2
  1500,  // position 3 (neutral)
  1330,  // position 4
  1180,  // position 5
  1040,  // position 6
};


void moveServoScanner(uint8_t position, int8_t adjust = 0)
{
  if (position >= 7)
    return;

  int microsServoScannerFrom = servoScanner.readMicroseconds();
  int microsServoScannerTo = microsServoScanner[position];

  if (adjust == 0) {
    // no adjustment
  }
  else if (adjust < 0) {
    // move slightly towards neutral position
    if (microsServoScannerTo < microsServoScanner[3])
           microsServoScannerTo += 15;
      else microsServoScannerTo -= 15;
  }
  else if (adjust > 0) {
    // move slightly away from neutral position
    if (microsServoScannerTo < microsServoScanner[3])
           microsServoScannerTo -= 15;
      else microsServoScannerTo += 15;
  }

  unsigned long millisDuration = abs(microsServoScannerTo - microsServoScannerFrom) * 2;

  if (   (microsServoScannerTo > microsServoScanner[3] && microsServoScannerTo > microsServoScannerFrom)
      || (microsServoScannerTo < microsServoScanner[3] && microsServoScannerTo < microsServoScannerFrom)) {

    // moving away from neutral position:
    // use overswing to keep servo from grumbling
    moveServo(servoScanner, microsServoScannerTo, millisDuration, easingOverswing);
  }
  else {
    // moving towards neutral position
    moveServo(servoScanner, microsServoScannerTo, millisDuration, easingRegular);
  }
}


int microsServoLatch[8] = {
  1040,  // position 0 (closed)
  1140,  // position 1
  1240,  // position 2
  1340,  // position 3
  1440,  // position 4
  1540,  // position 5
  1640,  // position 6
  1950,  // position 7 (open)
};


void moveServoLatch(uint8_t position, bool twitch = false)
{
  if (position >= 8)
    return;

  int microsServoLatchFrom = servoLatch.readMicroseconds();
  int microsServoLatchTo = microsServoLatch[position];

  if (twitch)
    microsServoLatchTo += 100;

  unsigned long millisDuration = abs(microsServoLatchTo - microsServoLatchFrom) * 2;

  if (twitch)
         moveServo(servoLatch, microsServoLatchTo, millisDuration, easingTwitch);
    else moveServo(servoLatch, microsServoLatchTo, millisDuration, easingRegular);
}


void writeLights(char bits)
{
  for (uint8_t iLight = 5; ; --iLight) {
    uint8_t valueLight;

    if (bits & (1 << 0))
             valueLight = HIGH;
        else valueLight = LOW;

    pinLight[iLight] = valueLight;

    bits >>= 1;

    if (iLight == 0)
      break;
  }
}


void signalErrorInvalid()
{
  Serial.println(F("invalid!"));

  writeLights(000);  delay(100);
  writeLights(052);  delay(100);
  writeLights(025);  delay(100);
  writeLights(052);  delay(100);
  writeLights(025);  delay(100);
  writeLights(052);  delay(100);
  writeLights(025);  delay(100);
  writeLights(000);  delay(100);
}


void signalErrorDuplicate()
{
  Serial.println(F("duplicate!"));

  writeLights(000);  delay(100);
  writeLights(077);  delay(200);
  writeLights(000);  delay(200);
  writeLights(077);  delay(200);
  writeLights(000);  delay(100);
}


int const valueSensorDeltaThreshold[7][6] = {
  {  80,  70,  50,  60,  50, 100 },
  { 100,  80,  60, 100,  70, 130 },
  {  80,  70,  60,  90,  70, 110 },
  {  90,  70,  60,  90,  70, 120 },
  { 100,  80,  50,  80,  60, 120 },
  {  90,  80,  60, 100,  80, 130 },
  { 110,  90,  70, 110,  80, 130 },
};


char readScanner(uint8_t position)
{
  char letter = 0;

  for (int8_t adjust = 1; adjust >= -1; --adjust) {
    moveServoScanner(position, adjust);

    for (uint8_t iLight = 0; iLight < 6; ++iLight)
      pinLight[iLight] = LOW;

    char letterCandidate = 0;

    for (uint8_t iSensor = 0; iSensor < 6; ++iSensor) {
      pinLight[iSensor] = LOW;
      delay(30);

      int valueSensorLow = pinSensor[iSensor];

      pinLight[iSensor] = HIGH;
      delay(30);

      int valueSensorHigh = pinSensor[iSensor];
      int valueSensorDelta = valueSensorLow - valueSensorHigh;

      Serial.print(valueSensorDelta);
      Serial.print('/');
      Serial.print(valueSensorDeltaThreshold[position][iSensor]);
      Serial.print(':');
      if (valueSensorDelta > valueSensorDeltaThreshold[position][iSensor])
             Serial.print('H');
        else Serial.print('L');
      Serial.print(' ');

      letterCandidate <<= 1;
      if (valueSensorDelta > valueSensorDeltaThreshold[position][iSensor])
        letterCandidate |= (1 << 0);

      pinLight[iSensor] = LOW;
    }

    if (adjust > -1)
      Serial.println(F(" ..."));

    letter |= letterCandidate;
  }

  Serial.print(F(" -> "));

  // compensate for missing seventh bit
  if ((letter & (1 << 5)) == 0)
    letter |= (1 << 6);

  Serial.print(F("position "));
  Serial.print(position);
  Serial.print(F(" is \""));
  Serial.print(letter);
  Serial.println(F("\""));

  writeLights(letter);

  return letter;
}


bool readScanner(char* string)
{
  for (uint8_t iLetter = 0; iLetter < 8; ++iLetter)
    string[iLetter] = 0;

  for (uint8_t position = 0; position < 7; ++position) {
    char letter = readScanner(position);

    if (   (letter < '0' || letter > '9')
        && (letter < 'A' || letter > 'Z')) {

      signalErrorInvalid();
      return false;
    }
    else {
      string[position] = letter;

      for (uint8_t iLetter = 0; iLetter < position; ++iLetter) {
        if (string[iLetter] == letter) {
          signalErrorDuplicate();
          return false;
        }
      }
    }
  }

  moveServoScanner(3);
  return true;
}


uint8_t countLettersCorrect(char const* input, char const* expected)
{
  uint8_t nLettersCorrect = 0;

  for (uint8_t iLetter = 0; input[iLetter] && expected[iLetter]; ++iLetter) {
    if (input[iLetter] == expected[iLetter])
      ++nLettersCorrect;
  }

  return nLettersCorrect;
}


uint8_t countLettersMisplaced(char const* input, char const* expected)
{
  uint8_t nLettersMisplaced = 0;

  for (uint8_t iLetter = 0; input[iLetter] && expected[iLetter]; ++iLetter) {
    if (input[iLetter] == expected[iLetter])
      continue;

    for (uint8_t iLetterExpected = 0; expected[iLetterExpected]; ++iLetterExpected) {
      if (iLetter != iLetterExpected && input[iLetter] == expected[iLetterExpected]) {
        ++nLettersMisplaced;
        break;
      }
    }
  }

  return nLettersMisplaced;
}


void setup()
{
  Serial.begin(9600);
  Serial.println(F("ready."));

  servoScanner.attach(8);
  moveServoScanner(3);

  servoLatch.attach(9);
  moveServoLatch(0);

  pinTriggerScan.begin(11, HIGH);

  pinSensor[0].begin(A0);
  pinSensor[1].begin(A1);
  pinSensor[2].begin(A2);
  pinSensor[3].begin(A3);
  pinSensor[4].begin(A4);
  pinSensor[5].begin(A5);

  pinLight[0].begin(2, LOW);
  pinLight[1].begin(3, LOW);
  pinLight[2].begin(4, LOW);
  pinLight[3].begin(5, LOW);
  pinLight[4].begin(6, LOW);
  pinLight[5].begin(7, LOW);
}


char const* const expected = "GC319QK";

Timer timerAnimationIdle (Timer::STARTED | Timer::REPEAT, 30);
uint8_t animationIdle = 0;


void loop()
{
  if (timerAnimationIdle.due()) {
    switch (animationIdle) {
      case 0:   writeLights((0x0F << 5) & 077);  break;
      case 1:   writeLights((0x0F << 4) & 077);  break;
      case 2:   writeLights((0x0F << 3) & 077);  break;
      case 3:   writeLights((0x0F << 2) & 077);  break;
      case 4:   writeLights((0x0F << 1) & 077);  break;
      case 5:   writeLights((0x0F     ) & 077);  break;
      case 6:   writeLights((0x0F >> 1) & 077);  break;
      case 7:   writeLights((0x0F >> 2) & 077);  break;
      case 8:   writeLights((0x0F >> 3) & 077);  break;
      default:  writeLights(0);                  break;
    }

    animationIdle = (animationIdle + 1) % 32;
  }

  if (pinTriggerScan) {
    Serial.println(F("trigger!"));

    writeLights(0);
    moveServoLatch(0);
    moveServoScanner(3);

    char input[8];

    if (readScanner(input)) {
      delay(200);

      uint8_t nLettersCorrect   = countLettersCorrect  (input, expected);
      uint8_t nLettersMisplaced = countLettersMisplaced(input, expected);

      Serial.print(F("input \""));
      Serial.print(input);
      Serial.print(F("\" (expected \""));
      Serial.print(expected);
      Serial.print(F("\"), "));
      Serial.print(nLettersCorrect);
      Serial.print(F(" correct, "));
      Serial.print(nLettersMisplaced);
      Serial.println(F(" misplaced"));

      if (nLettersCorrect == 7) {
        writeLights(077);
        moveServoLatch(7);
      }
      else {
        uint8_t lights = 0;

        for (uint8_t iLetterCorrect = 0; iLetterCorrect < nLettersCorrect; ++iLetterCorrect) {
          lights = (lights << 1) | (1 << 0);
          writeLights(lights);
          moveServoLatch(iLetterCorrect + 1);
          delay(100);
        }

        for (uint8_t iLetterMisplaced = 0; iLetterMisplaced < nLettersMisplaced; ++iLetterMisplaced) {
          moveServoLatch(nLettersCorrect, true);
          delay(100);
        }

        delay(1000);

        moveServoLatch(0);
        writeLights(0);
      }
    }

    delay(2000);

    timerAnimationIdle.start();
    animationIdle = 0;
  }

  if (Serial.available()) {
    int target = Serial.read();
    while (Serial.peek() == 0x20)
      Serial.read();
    
    if (target != 's' && target != 'l') {
      Serial.println(F("invalid target (expected \"s\" or \"l\")"));
    }
    else {
      int position = Serial.parseInt();
      while (Serial.peek() == 0x20)
        Serial.read();

      switch (target) {
        case 's':  if (position >= 7) position = -1;  break;
        case 'l':  if (position >= 8) position = -1;  break;
      }

      int microsServoNew = Serial.parseInt();
      while (Serial.peek() >= 0)
        Serial.read();

      if (position < 0) {
        Serial.println(F("cannot set invalid position"));
      }
      else if (microsServoNew == 0) {
        Serial.print(F("moving to position "));
        Serial.print(position);
        Serial.print(F(" at "));

        switch (target) {
          case 's':  Serial.print(microsServoScanner[position]);  break;
          case 'l':  Serial.print(microsServoLatch  [position]);  break;
        }

        Serial.println(F(" micros"));

        switch (target) {
          case 's':  readScanner   (position);  break;
          case 'l':  moveServoLatch(position);  break;
        }

      }
      else if (microsServoNew < 800 || microsServoNew > 2200) {
        Serial.print(F("cannot set position "));
        Serial.print(position);
        Serial.print(F(" to invalid "));
        Serial.print(microsServoNew);
        Serial.println(F(" micros"));
      }
      else {
        Serial.print(F("setting position "));
        Serial.print(position);
        Serial.print(F(" from "));

        switch (target) {
          case 's':  Serial.print(microsServoScanner[position]);  break;
          case 'l':  Serial.print(microsServoLatch  [position]);  break;
        }

        Serial.print(F(" to "));
        Serial.print(microsServoNew);
        Serial.println(F(" micros"));
        
        switch (target) {
          case 's':  microsServoScanner[position] = microsServoNew;  moveServoScanner(position);  break;
          case 'l':  microsServoLatch  [position] = microsServoNew;  moveServoLatch  (position);  break;
        }
      }
    }
  }
}
