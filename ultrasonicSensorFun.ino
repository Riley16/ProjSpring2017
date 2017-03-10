/* Code for testing the HC-SR04 ultrasonic sensor
 * 
 * Sets sensor to ping periodically and to display the returned
 * distance in cm
 */

#include <NewPing.h>
 
#define TRIGGER_PIN  12
#define ECHO_PIN     13
#define MAX_DISTANCE 30 // cm
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
void setup()
{
  Serial.begin(115200);
}
 
void loop()
{
  double usTime;
  delay(0);
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.print(" cm    ");
  usTime = (double)sonar.ping_median(45);
  Serial.print(usTime);
  Serial.print(" us    ");
  Serial.print(usTime*0.34/2);
  Serial.println(" mm    ");
}

