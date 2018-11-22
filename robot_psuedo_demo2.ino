/*  -------------------------------------------------------------------
 *  PURPOSE: The purpose of this script is to demonstrate the use of
 *  rotary encoders to determine the angular position and
 *  the distanceFromOrigin travelled of a robot. Canibalized from previous code with the
 *  intention of adapting it as necessary for Demo I
 *  -------------------------------------------------------------------
 *  WRITTEN BY: Jay Drobnick
 *  ASSISTED BY: Arduino Playground
 *  SECTION: C&D (1:00 pm)
 *  TEAM #: 13
 *  -----------------------------------------------------------------*/
 

/* --------------------------------------------------------------------
 * -----STILL NEEDED: (Please update this list as we go)---------------
 * -----(I'm sure I'm missing stuff, please add anything necessary)----
 * --------------------------------------------------------------------
 * I. variable for desired distanceFromOrigin (DONE)
 * II. variable for desired rotation (DONE)
 * III. a method to input the desired distanceFromOrigin and rotation (DONE)
 *    A. Bluetooth to Arduino OR
 *    B. Serial from Pi via SSH
 *    C. manual input between tests
 * IV. motor control (DONE)
 *    A. cannibalize from mini project (DONE)
 * V. check the variables and stop the motor when criteria is met (DONE)
 * VI. Feedback control
 */

/* -------------------------------------------------
 * --------------INCLUDE LIBRARIES------------------
 * ------------------------------------------------*/
#include <Encoder.h> //Include the 4x encoder library 
#include <math.h> // Include the math library
#include <Wire.h>

/* -------------------------------------------------
 * --------------------DEFINES----------------------
 * ------------------------------------------------*/
#define COUNT2RAD (3.14/1600) // This is a ratio to convert counts to radians. The motor encoders have a resolution
                            // of 64 CPR (counts per rotation)
#define pi 3.14         // Self-explanatory
#define motor1_encA 2   // pin # for motor1 encoder channel A (yellow)
#define motor1_encB 5   // pin # for motor1 encoder channel B (white)
#define motor2_encA 3   // pin # for motor2 encoder channel A (yellow)
#define motor2_encB 6   // pin # for motor2 encoder channel B (white)
#define LMOTOR_PWM 9    // pin # for motor1 pwm input (0-255)
#define LMOTOR_DIR 7    // pin # for motor1 dir input (true or false)
#define RMOTOR_PWM 10   // pin # for motor2 pwm input (0-255)
#define RMOTOR_DIR 8    // pin # for motor2 dir input (true or false)
#define M_ENABLE 4      // pin # for motor enable, both. 
#define FAULT 12        // pin # for motor fault detection
#define VECSIZE 4      
#define SLAVE_ADDRESS 0x04

/* -------------------------------------------------
 * --------------Encoder objects--------------------
 * ------------------------------------------------*/
Encoder motorL(motor1_encA, motor1_encB); // Sets up an encoder object for left wheel
Encoder motorR(motor2_encA, motor2_encB); // Sets up an encoder object for right wheel

/* -------------------------------------------------
 * --------------GLOBAL VARIABLES-------------------
 * ------------------------------------------------*/
const float r = 0.0715;                                 // Radius of each wheel (in meters)
const float d = 0.2921;                                 // Wheel baseline (distanceFromOrigin between wheels); 
                                                        // necessary for angle calculations
volatile float x_old, x_new, y_old, y_new, phi_old, phi_new, delta_r, delta_l, distanceFromOrigin, angle; 
volatile float delta_r_tot, delta_l_tot;                // total rotation (radians) of the wheels since beginning
volatile float old_right, old_left;                     // The rotation values, in radians, for the tires. 
volatile float countLeft_old  = 0;                      // old left count; used for encoder functionality
volatile float countRight_old = 0;                      // old right count; used for encoder functionality
volatile float desireddistanceFromOrigin, desiredAngle; // variables for the desired distanceFromOrigin and rotation
volatile long countLeft, countRight;                    // encoder count variables, updated when entering encoder ISR
int left_motor_pwm = 25;                                // declares and sets the initial value of the left motor pwm signal
int right_motor_pwm = 25;                               // declares and sets the initial value of the right motor pwm signal
boolean turnFlag, driveFlag, readyFlag;                 // flags used to determine when to execuse turn and drive functionality
int i = 0;                                              // loop variable i
int n = 0;                                              // loop variable n
float t = 0;                                            // time variable for testing
float startTime, endTime, currentTime;                  // used in the feedback system
float target_omega;                                     // the target wheel speed (rad/s)
float omega_left, omega_right;                          // the actual wheel speed of the left and right wheels (rad/s)
float duration;                                         // duration variable used in determinging angular velocity of wheels
float dist_vect[VECSIZE];                          // vector containing desired distances
float phi_vect[VECSIZE];                 // vector containing desired angles
float delta_x, delta_y, delta_phi, delta_distance;       // Will be used for multiple input coordinate management (TESTING PHASE)
float temp_distance;
float phi, dist;
float deg, ft;

/* -------------------------------------------------
 * --------------SETUP FUNCTION---------------------
 * ------------------------------------------------*/
void setup() {
  Serial.begin(9600);
    // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  x_old = 0; // Set starting x-coordinate
  y_old = 0; // Set starting y-coordinate
  phi_old = 0; // Set starting angle (relative to x-axis)
  delta_l = 0; // Set starting rotation for left wheel to 0
  delta_r = 0; // Set starting rotation for right wheel to 0
  old_right = 0; // Sets the intial "old" rotation value for right tire
  old_left = 0; // Sets the initial "old" rotation value for left tire
  
  // set up the motor pins and initial motor state
  pinMode(M_ENABLE, OUTPUT); //Enable pin: disables the outputs of both motors when LOW
  pinMode(LMOTOR_DIR, OUTPUT); //This pin controls the "sign" of motor voltage
  pinMode(LMOTOR_PWM, OUTPUT); //This pin controls whether the voltage to the motor is "on" or "off"
  pinMode(RMOTOR_DIR, OUTPUT); //This pin controls the "sign" of motor voltage
  pinMode(RMOTOR_PWM, OUTPUT); //This pin controls whether the voltage to the motor is "on" or "off"
  pinMode(FAULT, INPUT); //This pin looks for faults. LOW indicates a fault
  //It will be used with AnalogWrite to send a PWM signal to the motor
  digitalWrite(M_ENABLE, HIGH); //Sets the enable pin high
  analogWrite(LMOTOR_PWM, 0); //This should set the motor intially to 'off'
  analogWrite(RMOTOR_PWM, 0); //Sets the right motor initially to 'off'

  // Setup variables for testing. Only given a value here once.
  distanceFromOrigin = 0; // sets the robots initial distance from the origin
  readyFlag = false; // set the readyFlag to false to start
  temp_distance = 0; // sets the temp_distance variable to 0 to begin
}

/* -------------------------------------------------
 * --------------MAIN LOOP FUNCTION-----------------
 * ------------------------------------------------*/
void loop() { //begin loop function

//--- testLoc() creates test variables for dist_vect and phi_vect, then sets the readyFlag to true
//--- Should be commented out most of the time (useful only for testing motion without Pi)

float curr_x, curr_y, curr_dist;
n = 0;

Serial.println("----Wheel position encoder test----");
Serial.println("Waitng for instructions from Pi...");

while(readyFlag == false){ // While we don't have sight of a beacon (i.e. until readyFlag is set to true from the Pi)
  turnRight(); // Set the robot to turn right
  feedbackDriveLoop(); // start the motors
  //-- delay only for testing ---
  delay(2000);  // simulates searching for a beacon for 2 seconds
  testLoc();    // simulates what happens when the Pi sends coordinate requests
  stopMotors();   // added thsi for accuracy improvement
  delay(1000);
}

resetPos();     // Some rotation will have accumulated prior to the distance and angle request
                // this sets it back to 0.



if(readyFlag == true){
  target_omega = 0.7; // desired wheel speed in radians per second (rad/s)
//  for(n = 0; n < VECSIZE; n++){
    if(phi_vect[n] != 0) turnFlag = true;
    else turnFlag == false;
    if(turnFlag == true){
      Serial.print("Turning ... ");
      if(phi_new < phi_vect[n]){
//        while(phi_old < (phi_vect[n]) && (abs(phi_old < phi_vect[n]) > 0.035 || abs(phi_vect[n] - phi_old) > 0.035)){
        while(phi_new < phi_vect[n]){
          turnLeft();
          feedbackDriveLoop();
          Serial.print(" phi ");
          Serial.println(phi_old);
          Serial.print(" desired ");
          Serial.println(phi_vect[n]);
          if(abs(phi_old-phi_vect[n]) < (0.5*phi_vect[n]) || abs(phi_old-phi_vect[n]) < (0.5*phi_vect[n])){
            target_omega = 0.02;
          }
        }
      }
//      else if (phi_old > phi_vect[n] && (abs(phi_old < phi_vect[n]) > 0.035 || abs(phi_vect[n] - phi_old) > 0.035)){
      else if (phi_old > phi_vect[n]){
        while(phi_old > phi_vect[n]){
          turnRight();
          feedbackDriveLoop();
          Serial.print(" phi ");
          Serial.println(phi_old);
          if(abs(phi_old-phi_vect[n]) < (0.5*phi_vect[n]) || abs(phi_old-phi_vect[n]) < (0.5*phi_vect[n])){
            target_omega = 0.02;
          }
        }
      }
      turnFlag = false;
      Serial.println("done turning");
      stopMotors();
    } // end if turnFlag == true line
    
      delay(300); // let the motor settle after turning
//      left_motor_pwm = 50;
//      right_motor_pwm = 50;
  
      if(dist_vect[n] != 0) driveFlag = true;
      if(driveFlag == true){
        target_omega = 1.59;
        Serial.print("Driving straight ... ");
        if(distanceFromOrigin < dist_vect[n]){
          while (distanceFromOrigin < (dist_vect[n] - 0.1)){
            forward();
            feedbackDriveLoop();
            if(abs(distanceFromOrigin - dist_vect[n]) < (0.5*dist_vect[n]) || abs(dist_vect[n] - distanceFromOrigin) < (0.5*dist_vect[n])){
              target_omega = 0.7;
            }
          }
        }
        else if(distanceFromOrigin > dist_vect[n]){
          while (distanceFromOrigin > dist_vect[n] - 0.1){
            reverse();
            feedbackDriveLoop();
            if(abs(distanceFromOrigin - dist_vect[n]) < (0.5*dist_vect[n]) || abs(dist_vect[n] - distanceFromOrigin) < (0.5*dist_vect[n])){
              target_omega = 0.7;
            }
          }
        }
        Serial.println("done driving straight");
      }
      driveFlag = false;
      stopMotors();
      
          // -- DIAGNOSTIC PRINT STATEMENTS --

            Serial.print(" distanceFromOrigin traveled (m)"); 
            Serial.println(distanceFromOrigin);
            Serial.print(" x_new , y_new (");
            Serial.print(x_new);
            Serial.print(" ,");
            Serial.print(y_new);
            Serial.println(")");
            Serial.print(" phi_new (rads) ");
            Serial.println(phi_new);
            Serial.print(" duration ");
            Serial.println(duration);
            Serial.print(" omega left ");
            Serial.println(omega_left);
            Serial.print(" omega right ");  
            Serial.println(omega_right);
            Serial.print(" pwm ");
            Serial.print(left_motor_pwm);
            Serial.print("  ");
            Serial.println(right_motor_pwm);
  
        delay(200);
        if(n == 0){ // when on the first element of the dist_vect(desired distances) and phi_vect (desired angles)
          curr_x = x_old; // 
          curr_y = y_old;
        }
        else if(n != 0){
          curr_x = curr_x + x_old;
          curr_y = curr_y + y_old;
        }
        curr_dist = sqrt(sq(curr_x) + sq(curr_y)); 
    }
    Serial.print(" Final position (x, y) = (");
    Serial.print(curr_x);
    Serial.print(", ");
    Serial.print(curr_y);
    Serial.println(")");
    Serial.print(" Current distance from the original origin (m): ");
    Serial.println(curr_dist);
    readyFlag = false;
    resetPos();
//  } // end for loop
} // end loop function

/* -------------------------------------------------
 * --------------READ ENCODERS FUNC-----------------
 * ------------------------------------------------*/
void readEncoders(){ 
  countRight = motorL.read();
  countLeft = -motorR.read();
  if (countLeft != countLeft_old || countRight != countRight_old) { 
    
    delta_r = countRight*COUNT2RAD - countRight_old*COUNT2RAD; // Compute the rotation of the right wheel (radians)
    delta_l = countLeft*COUNT2RAD - countLeft_old*COUNT2RAD; // Compute rotation of the left wheel (radians)
    delta_r_tot = countRight*COUNT2RAD; // Compute the rotation of the right wheel (radians)
    delta_l_tot = countLeft*COUNT2RAD; // Compute rotation of the left wheel (radians)
    x_new = x_old + cos(phi_old)*(r/2)*(delta_r + delta_l); // Compute the new x-coord
    y_new = y_old + sin(phi_old)*(r/2)*(delta_r + delta_l); // Compute the new y-coord
    distanceFromOrigin = sqrt(sq(x_new)+sq(y_new)); // Compute travelled distanceFromOrigin based off of x and y [d = sqrt(x^2+y^2)]
    phi_new = phi_old + (r/d)*(delta_r - delta_l); // Compute the new angle (relative to x-axis)

    phi_old = phi_new; // update the value of phi_old
    x_old = x_new; // update the value of x_old
    y_old = y_new; // update the value of y_old
    countLeft_old = countLeft; // update the countLeft_old value
    countRight_old = countRight; // update the countRight_old value
    old_right = delta_r; //update tthe right wheels angular rotation
    old_left = delta_l; //update the left wheels angular rotation
    
  }// end if
}// end readEncoders

/*----------------------------------------------------------------
 * -----------MOTOR DRIVE AND TURN FUNCTIONS----------------------
 * -------------------------------------------------------------*/
void runMotors(){
  analogWrite(LMOTOR_PWM, left_motor_pwm);  // these pwm values are determined by feedbackDriveLoop();
  analogWrite(RMOTOR_PWM, right_motor_pwm); 
}

void stopMotors(){
  analogWrite(LMOTOR_PWM, 0);
  analogWrite(RMOTOR_PWM, 0);
}

void forward(){
  target_omega = 3.14; // desired wheel speed in radians per second (rad/s)
  digitalWrite(LMOTOR_DIR, true);
  digitalWrite(RMOTOR_DIR, true);
}

void reverse(){
  target_omega = 3.14; // desired wheel speed in radians per second (rad/s)
  digitalWrite(LMOTOR_DIR, false);
  digitalWrite(RMOTOR_DIR, false);
}

void turnLeft(){
  target_omega = 1.59; // desired wheel speed in radians per second (rad/s)
  digitalWrite(LMOTOR_DIR, false); // left wheel backwards
  digitalWrite(RMOTOR_DIR, true);  // right wheel forwards
}

void turnRight(){
  target_omega = 1.59; // desired wheel speed in radians per second (rad/s)
  digitalWrite(LMOTOR_DIR, true);
  digitalWrite(RMOTOR_DIR, false);
}

void resetPos(){

    motorL.write(0);
    motorR.write(0);
    
    phi_old = 0;
    phi_new = 0;
    x_old = 0;
    y_old = 0; // update the value of y_old
    x_new = 0;
    y_new = 0;
    countLeft_old = 0; // update the countLeft_old value
    countRight_old = 0; // update the countRight_old value
    old_right = 0; //update tthe right wheels angular rotation
    old_left = 0; //update the left wheels angular rotation
    countLeft = 0;
    countRight = 0;

    readEncoders();

    Serial.print(" the position has been reset to (x,y, phi) = (");
    Serial.print(x_new);
    Serial.print(", ");
    Serial.print(y_new);
    Serial.print(", ");
    Serial.print(phi_new);
    Serial.println(")");
    Serial.print(" distance = ");
    Serial.println(distanceFromOrigin);
}

//------------------------------------------------
//------ This function controls motor speed ------
//------ and attempts to match it to a target ----
//------------------------------------------------
void feedbackDriveLoop(){ // begin feedbackDriveLoop
  
    startTime = millis(); // begin time check
    runMotors(); // start the motors
    delay(5); // wait 5 ms
    readEncoders(); // read the encoders
    endTime = millis(); // stop the timer
    
    duration = (endTime - startTime)/1000; // calculate the time elapsed
    omega_left = abs(delta_l)/duration; // this is the rotational velocity of the left wheel
    omega_right = abs(delta_r)/duration; // this is the rotational velocity of the right wheel
     
    if(omega_left < target_omega && left_motor_pwm < 250){
      left_motor_pwm++;
    }
    else if (omega_left == target_omega) left_motor_pwm = left_motor_pwm;
    else if (left_motor_pwm > 15) left_motor_pwm--;
   
    if(omega_right < target_omega && right_motor_pwm < 250){
      right_motor_pwm++;
    }
    else if (omega_right == target_omega) right_motor_pwm = right_motor_pwm; 
    else if (right_motor_pwm > 15) right_motor_pwm--;
} // end feedbackDriveLoop


//--- callback for received data ---
void receiveData(int byteCount) {
  //Receive phi and dist values from Pi from Camera
  int temp = Wire.read();
  phi_vect[0] = Wire.read()*(pi/180.0 );
  dist_vect[0] = Wire.read()/100.0;
  int temp_direction = Wire.read();
  if (temp_direction == 2) {
    phi_vect[0] = -phi_vect[0];
  }
  readyFlag = true;
  Serial.print(" distance requested from Pi " );
  Serial.println(dist_vect[0]);
}

// --- callback for sending data ---
void sendData() {
//Send ready flag value to Pi
 Wire.write(readyFlag); 
}

void testLoc(){ // just used for a test location without pi
  int i;
  if(readyFlag == false){
    i = 0;
    phi_vect[0] = 2*(pi/180.0); // desired angle
    dist_vect[0] = 50/100.0;   // desired distance
    readyFlag = true;   // set the readyFlag to true
    delay(200);
    Serial.print(" test requests are : ");
    Serial.print(phi_vect[0]);
    Serial.print("  ");
    Serial.println(dist_vect[0]);
  }
}

void si2imp(float rads, float dist){
  deg = phi*(180/pi);
  ft = dist*(3.28084); 
}


