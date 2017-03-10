// Code for Hardware Presentation 1, 2/23/2017
// Junior Design, EGR 3380
// Team 15 - Ricky Bobby and the Washing Machine

// operates motor with h-bridge using button

// initially motor is stoppped, 
// pressing button causes motor to push washers forward in arm
// pressing button again stops motor
// pressing again causes motor to rotate in opposite direction
// pressing again causes motor to stop
// repeats

#define buttonIn 10
#define hBin1 11
#define hBin2 12
#define hBen 13

void motOpChange(int enVal, int hBoutPinVal1, int hBoutPinVal2);

void setup() 
{
  pinMode(3, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
  /*
  int enVal;
  int hBoutPinVal1;
  int hBoutPinVal2;

  enVal = HIGH;
  hBoutPinVal1 = LOW;
  hBoutPinVal2 = HIGH;
  
  digitalWrite(hBen, enVal);
  digitalWrite(hBin1, hBoutPinVal1);
  digitalWrite(hBin2, hBoutPinVal2);
  delay(1000);

  enVal = HIGH;
  hBoutPinVal1 = HIGH;
  hBoutPinVal2 = LOW;

  digitalWrite(hBen, enVal);
  digitalWrite(hBin1, hBoutPinVal1);
  digitalWrite(hBin2, hBoutPinVal2);
  delay(1000);
  */
  /*
  digitalWrite(hBen, HIGH);
  digitalWrite(hBin1, HIGH);
  digitalWrite(hBin2, LOW);
  delay(1000);
  
  digitalWrite(hBen, HIGH);
  digitalWrite(hBin1, LOW);
  digitalWrite(hBin2, HIGH);
  delay(1000);
  */

/*
  motOpChange(LOW, HIGH, LOW);
  motOpChange(HIGH, LOW, HIGH);
  motOpChange(LOW, LOW, HIGH);
  motOpChange(HIGH, HIGH, LOW);
*/
/*
  motOpChange(LOW, HIGH, LOW);
  motOpChange(HIGH, HIGH, LOW);
  motOpChange(LOW, LOW, HIGH);
  motOpChange(HIGH, HIGH, LOW);
  */
/*
  digitalWrite(hBen, HIGH);
  digitalWrite(hBin1, LOW);
  digitalWrite(hBin2, HIGH);

  // wait for next pressing of button
  while (digitalRead(buttonIn) == LOW)
  {
    if (digitalRead(buttonIn) == LOW)
    {
      break;
    }
  }
 
  // wait for button to be released
  while (digitalRead(buttonIn) == HIGH)
  {
    if (digitalRead(buttonIn) == HIGH)
    {
      break;
    }
  }
  digitalWrite(hBen, LOW);
  digitalWrite(hBin1, HIGH);
  digitalWrite(hBin2, LOW);

  // wait for next pressing of button
  while(digitalRead(buttonIn) == LOW)
  {
  }
 
  // wait for button to be released
  while(digitalRead(buttonIn) == HIGH)
  {
  }
*/


/*
if(digitalRead(4) == HIGH)
{
  digitalWrite(3, HIGH);
}
else
{
  digitalWrite(3, LOW);
}
  */
  
/*
  if(digitalRead(buttonIn))
  {
    
  }
  */
      digitalWrite(13, LOW);
      delay(10);
      digitalWrite(13, HIGH);
      digitalWrite(11, HIGH);
      digitalWrite(12, LOW);
      delay(10);
      digitalWrite(13, HIGH);
      digitalWrite(11, LOW);
      digitalWrite(12, HIGH);
      delay(10);

/*
    if(digitalRead(10) == HIGH)
    {
      digitalWrite(13, LOW);
    }
    else
    {
      digitalWrite(13, HIGH);
    }
    */
}

void motOpChange(int enVal, int hBoutPinVal1, int hBoutPinVal2)
{
  // set desired motor operation
  digitalWrite(hBen, enVal);
  digitalWrite(hBin1, hBoutPinVal1);
  digitalWrite(hBin2, hBoutPinVal2);

  // wait for next pressing of button
  if(enVal == HIGH)
  {
    while(digitalRead(buttonIn) == HIGH)
    {
      digitalWrite(hBen, HIGH);
/*      delay(5);
      digitalWrite(hBen, LOW);
      delay(5);*/
    }
  }
  else
  {
    while(digitalRead(buttonIn) == HIGH)
    {
    }
  }
  
  // wait for button to be released
  while(digitalRead(buttonIn) == LOW)
  {
    delay(100);
  }
}


