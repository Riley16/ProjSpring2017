// Code for complete system control of the 
// washer dropping robot for Junior Design, EGR 3308

// Control system requires input for start command, input 
// for ultrasonic sensor, output for two servo motors,
// output for a stepper motor controller, and output for 
// DC motors controlled using relays (and possibly with
// power signal modulators as well).

#include <Servo.h>
#include <NewPing.h>

// associate ultrasonic sensor with pins
#define TRIGGER_PIN  12
#define ECHO_PIN     13
#define MAX_DIST_SENSE 4 // cm
#define LED_PIN      11
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DIST_SENSE);

#define startPin        0
#define servoPinConst   1
#define servoPinVar     2
#define washActConstEn  3
#define washActConstDir 4
#define washActVarEn    5
#define washActVarDir   6
#define STEPPER_CONT    7
#define STEPPER_DIR     8
#define baseMotPinEn    9
#define baseMotPinDir   10

void initialization();
void stepperMotMove(int steps);
void DCmotMove(int motEnPin, int motDirPin, int en, int dir);
void dropProcedure();
void firstPegSetProcedure();
void middlePegSetProcedures();
void finalPegSetProcedure();

Servo servoConst;
Servo servoVar;

void setup() {
  // Pin for start command
  pinMode(startPin, INPUT_PULLUP);

  // Servo motor pins
  // Attach servo motor for constant elevation dropping cylinder
  servoConst.attach(servoPinConst);
  // Attach servo motor for variable elevation dropping cylinder
  servoVar.attach(servoPinVar);

  // DC motor pins
  // Pins for base motor
  pinMode(baseMotPinEn, OUTPUT);
  pinMode(baseMotPinDir, OUTPUT);
  // Pins for washer-moving actuator of arm 
  // with constant elevation dropping cylinder
  pinMode(washActConstEn, OUTPUT);
  pinMode(washActConstDir, OUTPUT);
  // Pins for washer-moving actuator of arm 
  // with variable elevation dropping cylinder
  pinMode(washActVarEn, OUTPUT);
  pinMode(washActVarDir, OUTPUT);

  // control pins for the stepper motor
  pinMode(STEPPER_CONT, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  
  // LED pin for sensor testing and validation
  pinMode(LED_PIN, OUTPUT);
}

// Variable definitions
// count of the number of steps the stepper motor has rotated
// the initial position at which the variable elevation dropping
// cylinder is raised to the maximum height
int stepCount = 0;

// number of microseconds between pulse transmission and 
// reception returned by the ultrasonic sensor
double usTime = 0;

// array of stepper motor step numbers to raise or lower the 
// variable elevation dropping cylinder in the order appearing on the course
// compute values in MATLAB from stepper motor and gear radius parameters and heights of variable height pegs
int stepMotStepNums[6] = {};

void loop() {
  initialization();
  // NEED TO CREATE A FUNCTION TO RESET THE STEPPER MOTOR AND THE ARM POSITIONS TO THEIR INITIAL 
  // POSITIONS FOR THE RACE, FOR TESTING, FOR RESETTING AFTER THE RACE
  // NEED TO WRITE POSITIONING FUNCTION FOR THE STEPPER MOTOR TO ALLOW INITIALIZATION OF THE
  // STEPPER MOTOR POSITION IF THE STEPPING MOTOR IS MOVED OUT OF POSITION OR DOES NOT START
  // AT THE CORRECT POSITION

  // NEED TO PROGRAM SEPARATE PROCEDURES FOR THE DROPPING PROCEDURES FOR THE FIRST AND LAST PEG SETS
  // FIRST PEG SET PROCEDURE NEEDS TO ENSURE AVOIDANCE OF DETECTING THE WOODEN BLOCK ON THE FRONT OF
  // BOARD, SECOND PEG SET PROCEDURE DOES NOT REQUIRE MOVEMENT OF THE VARIABLE ELEVATION DROPPING CYLINDER

  firstPegSetProcedure();
  middlePegSetProcedures();
  finalPegSetProcedure();  
  // wait indefinitely once competition is finished
  while(1)
  {
  }
}

void initialization()
{
  // wait for start command to be given with push button
  while(startPin == HIGH)
  {
  }
  // wait for push button to be released
  while(startPin == LOW)
  {
  }
  servoConst.write(3);
  servoVar.write(3);
}

void DCmotMove(int motEnPin, int motDirPin, int en, int dir)
{
  // MUST VERIFY MOTOR DIRECTIONS, GENERALLY TRY TO MAKE THE "FORWARD" DIRECTION LOW
  // AND THE "BACKWARD" DIRECTION HIGH
  digitalWrite(motEnPin, en);
  digitalWrite(motDirPin, dir);
}

void stepperMotMove(int steps)
{
  // moves the stepper motor "steps" steps in either direction
  
  // iteration variable
  int i;

  // rotate backwards for a negative input steps value
  if (steps < 0)
  {
    digitalWrite(STEPPER_DIR, HIGH);
  }
  // rotate forwards otherwise
  else
  {
    digitalWrite(STEPPER_DIR, LOW);
  }
  

  if (steps < 0)
  {
    for (i = 0; i < steps; i++)
    {
      stepCount--;
      // send step signal to stepper motor driver
      digitalWrite(STEPPER_CONT, HIGH);
      delay(1);
      digitalWrite(STEPPER_CONT, LOW);
      delay(20);
    }
  }
  else
  {
    for (i = 0; i < steps; i++)
    {
      stepCount++;
      // send step signal to stepper motor driver
      digitalWrite(STEPPER_CONT, HIGH);
      delay(1);
      digitalWrite(STEPPER_CONT, LOW);
      delay(20);
    }
  }
}

void dropProcedure()
{
  // open claws to drop washers
  // CHECK IF THE SERVOS ARE OPERATED SIMULTANEOUSLY
  servoConst.write(90);
  servoConst.write(90);
  
  // wait for washers to fall 
  // MAY REQUIRE SOME CALIBRATION
  delay(500);
  // close claws for next dropping procedure
  servoConst.write(3);
  servoConst.write(3);
}

void middlePegSetProcedures()
{
  // procedure for the middle five sets of pegs, which are structurally identical
  int pegSetNum;
  for(pegSetNum = 0; pegSetNum < 5; pegSetNum++)
  {
    // MEASURE APPROXIMATE MOVEMENT TIMES FOR EACH MOTION
    // BASE MOVEMENT BETWEEN PEGS:
    // DROPPING WASHER STACKS INTO CYLINDERS:
    // MOVING VARIABLE ELEVATION CYLINDER TO LOWEST PEG POSITION:
    
    // move base forward
    DCmotMove(baseMotPinEn, baseMotPinDir, HIGH, LOW);

    // move washer stacks into dropping cylinders
    DCmotMove(washActConstEn, washActConstDir, HIGH, LOW);
    DCmotMove(washActVarEn, washActVarDir, HIGH, LOW);
    // DELAY REQUIRES CALIBRATION TO ENSURE WASHER STACKS ARE COMPLETELY DROPPED WHILE
    // MINIMIZING TIME IN WHICH MOTOR IS DRIVING WASHERS INTO DROPPING CYLINDER WALL
    delay(300);
    // relax motors driving washers into dropping cylinders to drop washer stacks into cylinders
    DCmotMove(washActConstEn, washActConstDir, LOW, LOW);
    DCmotMove(washActVarEn, washActVarDir, LOW, LOW);

    // MUST VERIFY THAT MOVING THE STEPPER MOTOR AND THE WASHER STACKS INTO POSITION WILL TAKE IDEALLY 
    // AT LEAST 0.5 s LESS THAN THE TIME REQUIRED FOR THE BASE MOTOR TO REACH THE NEXT PEG SET
    stepperMotMove(stepMotStepNums[pegSetNum]);

    // wait for ultrasonic sensor to sense board peg
    while(usTime == 0)
    {
      // take the average of multiple ultrasonic sensor readings
      // THE NUMBER OF AVERAGED READINGS MAY NEED TIMING AND SENSITIVITY CALIBRATION
      usTime = (double)sonar.ping_median(15);

      // light LED if peg is sensed (if any object is sensed by the LED
      if(usTime == 0) // THIS VALUE MAY NEED TO BE RAISED TO SOME VALUE GREATER
                      // THAN ZERO TO ACCOUNT FOR NOISE AND RANDOM READINGS
      {
        // use LED for testing and operation validation
        digitalWrite(LED_PIN, LOW);
      }
      else
      {
        digitalWrite(LED_PIN, HIGH);
      }
    }
    
    // stop the base, ensure base is aligned
    DCmotMove(baseMotPinEn, baseMotPinDir, LOW, LOW); // REQUIRES SIGNIFICANT CALIBRATION TO ENSURE PEG AND ARM ARE ALIGNED
    // ALIGNMENT FUNCTION HERE

    // drop the washers
    dropProcedure();
    
    // MOVING THE STEPPER BACK TO THE INITIAL HEIGHT MAY NEED TO BE DONE WHILE THE ROBOT IS MOVING IF THERE IS TIME
    // AVAILABLE. HOWEVER WITH LITTLE LOAD ON THE STEPPER MOTOR, RETURNING TO THE INITIAL POSITION SHOULD TAKE A 
    // SMALL AMOUNT OF TIME
    // move the stepper motor back to its starting height
    stepperMotMove(-stepMotStepNums[pegSetNum]);
    
  }

}

void firstPegSetProcedure()
{
  // procedure for first peg set; same as procedures for middle five peg sets but must not sense the wooden block at the end of the board
  
}

void finalPegSetProcedure()
{
  // procedure for final peg set; same as procedures for middle five peg sets but does not need to move the variable elevation dropping cylinder 
  // or its rack and pinion
  
}


