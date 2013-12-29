void setup()
{
  Serial.begin(9600);
}


void loop()
{
  Serial.print(F("A0: "));
  Serial.print(analogRead(A0));

  Serial.print(F("  A1: "));
  Serial.print(analogRead(A1));

  Serial.print(F("  A2: "));
  Serial.print(analogRead(A2));

  Serial.print(F("  A3: "));
  Serial.print(analogRead(A3));

  Serial.print(F("  A4: "));
  Serial.print(analogRead(A4));

  Serial.print(F("  A5: "));
  Serial.print(analogRead(A5));

  Serial.println();

  delay(500);
}
