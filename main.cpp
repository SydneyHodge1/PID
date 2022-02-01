/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sydney Gabe Hodge                                         */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  excuse me sir? yes, you. your mother. adios!              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> // allows (std::abs)

using namespace vex;

// A global instance of competition
competition Competition;
vex::controller Controller1(controllerType::primary);
vex::motor FR(vex::PORT7, ratio18_1, true);
vex::motor FL(vex::PORT9, ratio18_1, false);
vex::motor BL(vex::PORT10, ratio18_1, true);
vex::motor BR(vex::PORT5, ratio18_1, false);
vex::motor Lift(vex::PORT13, ratio36_1, true);
vex::motor Forklift(vex::PORT20, ratio36_1, false);
vex::motor TL(vex::PORT8, ratio18_1, false);
vex::motor TR(vex::PORT19, ratio18_1, true);
vex::motor_group Drivetrain(FR, FL, BL, BR, TL, TR);
vex::motor_group Left(FL, BL, TL);
vex::motor_group Right(FR, BR, TR);
inertial Inertial3 = inertial(PORT3);
triport  myThreeWirePort(PORT22);
  pneumatics claw = pneumatics( myThreeWirePort.A );





///////////////////////
//    Brake Types   //
/////////////////////


//brakes drive in either hold or coast
void set_hold() {
  FR.setStopping(hold); TR.setStopping(hold); BR.setStopping(hold); FL.setStopping(hold); TL.setStopping(hold); BL.setStopping(hold); }

void set_coast() {
  FR.setStopping(coast); TR.setStopping(coast); BR.setStopping(coast); FL.setStopping(coast); TL.setStopping(coast); BL.setStopping(coast); }

void coast_drive(){
FR.stop(coast); TR.stop(coast); BR.stop(coast); FL.stop(coast); TL.stop(coast); BL.stop(coast); }

void brake_drive(){
FR.stop(hold); TR.stop(hold); BR.stop(hold); FL.stop(hold); TL.stop(hold); BL.stop(hold); }

void reset_rotation() {
  FR.resetRotation(); TR.resetRotation(); BR.resetRotation(); FL.resetRotation(); TL.resetRotation(); BL.resetRotation(); Lift.resetRotation(); 
  }

void set_position(int pos){
  FR.setPosition(pos, deg); TR.setPosition(pos, deg); BR.setPosition(pos, deg); FL.setPosition(pos, deg); TR.setPosition(pos, deg); BL.setPosition(pos, deg); }












     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //    Driver Control Global Statements    //
 //                                        //
//                                        //
///////////////////////////////////////////











////////////////////////////
//    Lift Statements    //
//////////////////////////

void lift() { 
  Lift.spin(directionType::fwd, 11, voltageUnits::volt);
  }
//change the number to alter the speed of the lift when going up (both numbers must be the same!!!)

void liftRev() {
  Lift.spin(directionType::rev, 11, voltageUnits::volt);
  }
//change the number to alter the speed of the lift when going down (both numbers must be the same!!!)

void liftBrake() {
  Lift.stop(brakeType::hold);
  }
//Lift brake types (hold,coast,brake)

  //////////////////////////
  //     Forklift        //
  ////////////////////////

  void forklift(){
  Forklift.spin(directionType::fwd, 11, voltageUnits::volt);
  }
  
  void forkliftRev(){
    Forklift.spin(directionType::rev, 11, voltageUnits::volt);
  }

  void forkliftBrake(){
    Forklift.stop(brakeType::hold);
  }
  








     ////////////////////////////////////////
    //                                    //
   //                                    //
  //    Autonomous Global Statements    //
 //                                    //
//                                    //
///////////////////////////////////////



void turn_left(int pos, int speed, bool stopping) {
  FR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  TR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

void turn_right(int pos, int speed, bool stopping) {
  FL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  TL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}
void lift_up(int pos, int speed, bool stopping) {
  Lift.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

void forklift_up(int pos, int speed, bool stopping) {
  Forklift.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}


const int HALF_SPEED = 50;
const int FULL_SPEED = 100;
const int QUARTER_SPEED = 25;
const int NO_MIN = 0;

///////////////////////////
// PID for auto balance // 
/////////////////////////

void autoBalance(){
  //rotation will happen on x axis
  // calibrate to create reference point when robot is flat. would be using the gyro
  // maybe use gyroRate?
  // orientation?
  // changed
  // pitch -- USE THIS
  // Make it a PID ?
  double targetAngle = 0;

  double motorspeed = 0, currentAngle = Inertial3.pitch(),
         error = (targetAngle - currentAngle), errorSum = 0, deltaE = 0,
         lastError = 0, maxAllowedError = 1.0, errorTimerMax = 50;
  double kP = 10, kI = 0.0, kD = 15;
  bool timerExpired = false;
  timer errorTimer = timer();
  errorTimer.clear();

  while ((fabs(error) > maxAllowedError) && (timerExpired==false)){
    currentAngle = Inertial3.pitch();
    error = targetAngle - currentAngle;
    errorSum += error;
    deltaE = (error - lastError) / 5000;
    motorspeed = (error * kP) + (errorSum * kI) + (deltaE * kD);

    Drivetrain.spin(directionType::rev, motorspeed, velocityUnits::pct);
    

    lastError = error;

    if (fabs(error) > maxAllowedError) {
      errorTimer.clear();
    }
    else {
      if (errorTimer.time() > errorTimerMax) {
        timerExpired = true;
      }
    }
    vex::task::sleep(20);
  }
 Drivetrain.stop(brake);
}


/////////////////////////////////////
//    PID for Turning Movement    //
///////////////////////////////////

void turn_drive(int turnTarget, int speed){


int turnProportion;
int turnDerivative;

set_position(0); // ^ zeros the Motors and the Inertial Sensor

double turnKp = 0.2; //speed control (error) 
double turnKi = 0.2; //increase precision with error left over from kp 
double turnKd = 0.2; //makes sure that the speed is not too fast or too slow
// ^ constants (increase the kp until error is small, then increase kd until overshoot is minimal, increase ki until error is gone)

int turnError; //error (target - the actual values)
int turnPrevError; //error from the previous loop

int proportion; //error
int integral = 0.0; //total error
int derivative; //error minus previous error (speed)

int volts = (speed*.12); //converts the set speed above into an interger to volts (100 *.12 = 12)
  //voltageUnits::volts goes from 0-12 volts, 12 being the maximum voltage
int rawPow; //power calculated from summing the PID and their corresponding constants
int leftPower; //rawPow added with the difference between FL and FR
int rightPower; //rawPow subtracted with the difference between FL and FR
int sign; //interger that can only be 1 or -1 (used for the speed cap)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);

while(1){ //while loop, this will continue to run until the specific parameters are met to break the loop



  turnProportion = turnError;
  integral += turnError;
  turnDerivative = (turnError - turnPrevError);
    rawPow = turnProportion *turnKp + integral *turnKi + turnDerivative *turnKd;




  sign = turnError/abs(int(turnError)); //calulates the sign of error (1 or -1)

if(abs(rawPow)>=abs(volts)){ // if the left power is greater than the desired speed, the left power equals the desired speed in volts times the sign of the error
  rawPow = volts*sign; // power left side = speed times sign of the error (direction)
}
 



if(abs(turnError) <= 4){ //breaks the loop if the error is less than 4 for more than 60 msec
    errorTimer +=DELAY_TIME;
  if (errorTimer > 60){
    errorTimer = 0;
    break;
  }
}
  else{
    errorTimer = 0;
  }

//sets motors to move with their corresponding powers
FL.spin(forward, rawPow, vex::voltageUnits::volt);
FR.spin(reverse, rawPow, vex::voltageUnits::volt);
TL.spin(forward, rawPow, vex::voltageUnits::volt);
TR.spin(reverse, rawPow, vex::voltageUnits::volt);
BL.spin(forward, rawPow, vex::voltageUnits::volt);
BR.spin(reverse, rawPow, vex::voltageUnits::volt);

  turnPrevError = turnError; 
    wait(DELAY_TIME,msec); // waits 10msec before repeating the while loop
} 
 


//coasts the motors when while loop broken
brake_drive();
  wait(20,msec); //waits 20msec before going to the next line of code
}





double kp = 0.2; //speed control (error) 0.185
double kd = 1.2; //makes sure that the speed is not too fast or too slow 0.17
//double ki = 0.;
double turnKp = 0.025;
double turnKd = 0.011875;
// ^ constants


///////////////////////////////
//    PD Linear Movement    //
/////////////////////////////

void move_drive(int target, int max_speed, int min_speed){

set_position(0);
  // ^ zeros the Motors



int error; //error (target - average position)
int prevError; //the error from the previous loop

int proportion; //error
int derivative; //error minus previous error (speed)


int rawPow; //power calculated from summing the PID and their corresponding constants
int curPos;
int turnRawPow;
int turnDiff;
int leftPow;
int rightPow;

//double curPos = curDeg * 0.047269;
int max_volts = (max_speed * .12); //converts the speed into voltage (0-12)
int min_volts = (min_speed * .12);

int sign; //sign (1 or -1) error/abs(error)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);


while(1){

curPos = ((FL.position(degrees) + FR.position(degrees))/2); //average position between FL and FR
//turnDiff = (FL.position(degrees) - FR.position(degrees));

error = target - curPos;

//PD calculations using the average position of the motors
  proportion = error;
  derivative = (error - prevError);
    rawPow = proportion *kp + derivative *kd;
printf("%i", rawPow);



      sign = error / abs(int(error)); //calulates the sign of error (1 or -1)

if (abs(rawPow) <= abs(min_volts)){
  rawPow = min_volts*sign;
}

if (abs(rawPow) >= abs(max_volts)){ // if the rawPower is greater than the desired speed, the rawPower equals the desired speed in volts times the sign of the error
  rawPow = max_volts*sign;
}


if(abs(error) <= 5 ){   // && abs(turnError) <= 4
    errorTimer +=DELAY_TIME;
  if (errorTimer > 60){
    errorTimer = 0;
      break;
  }
}
  else{
    errorTimer = 0;
  }



//sets motors to move with the rawPower calculated from the PID controller
  Drivetrain.spin(fwd, rawPow, vex::voltageUnits::volt);


  prevError = error;
//  turnPrevError = turnError;
    wait(DELAY_TIME,msec); // waits 10 msec before repeating the while loop
} 


brake_drive();
  wait(20,msec);  //waits 20msec before going to the next line of code
}

//out
//of
//a
//hill
//of
//despair
//,
//a
//rock
//of
//hope
//-Zain


//moves lift (moves in degrees)
void move_lift(int pos, int speed, bool stopping) {
  Lift.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}


void one_mogo_constants(double KP, double KD){
  kp = KP;
  kd = KD;
}


     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //        Pre-Autonomous Functions        //
 //                                        //
//                                        //
///////////////////////////////////////////

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
 claw.open();
 Drivetrain.setVelocity(100, percent);



}




     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //             Autonomous Task            //
 //                                        //
//                                        //
///////////////////////////////////////////

void autonomous(void) {
   forklift_up(1000, 100, 0);
   wait(2, sec);
   move_drive(-465, 60, 0);
   forklift_up(-700, 100, 0);
   wait(1.2,sec);
   turn_right(-600, 60, 0);
   wait(1, sec);
  move_drive(-300, 60, 0);
  turn_left(-610, 60, 0);
  wait (2, sec);
  move_drive(-1600, 60, 0);
  move_drive(300, 60, 0);
  forklift_up(700, 100, 0);
  wait (1, sec);
  move_drive (1000, 60, 0);
  turn_left (460, 60, 0);
  wait (1, sec);
  move_drive(-2000, 60, 0);
turn_left (-610, 100, 0);
move_drive (2000, 60, 0);
move_drive (-2000, 60, 0);
turn_right (200, 60, 0);
move_drive (2000, 60, 0);
move_drive (-2000, 60, 0);
turn_right (200, 60, 0);
move_drive (2000, 60, 0);
move_drive (-2000, 60, 0);
turn_right (200, 60, 0);
}







     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //             Driver Control             //
 //                                        //
//                                        //
///////////////////////////////////////////

void usercontrol(void) {
  

//Drivetrain
double turnImportance = 0.5;

//Claw stay closed after auton
  claw.open();

  //More claw things
    bool claw_up = true;
    int claw_lock = 0;

  bool slow_drive = false;
  int slow_lock = 0;

  // Everybody go fast
  Lift.setVelocity(100, percent);
  Forklift.setVelocity(100, percent);
  Drivetrain.setVelocity(100, percent);

  //stay stupid
  Lift.setStopping(brakeType::hold);
  Forklift.setStopping(brakeType::brake);
  //Controller Axis Code
  while (true) {
    double turnPct = Controller1.Axis4.position();
double forwardPct = Controller1.Axis2.position();

double turnVolts = turnPct * 0.12;
double forwardVolts = forwardPct * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);


  if (Controller1.ButtonA.pressing() && slow_lock==0){
    slow_drive = !slow_drive;
    slow_lock = 1;}

    else if (!Controller1.ButtonA.pressing()){
      slow_lock = 0;}

        if (slow_drive){
FR.spin(forward, (forwardVolts - turnVolts)/2, voltageUnits::volt);
FL.spin(forward, (forwardVolts + turnVolts)/2, voltageUnits::volt);
BR.spin(forward, (forwardVolts - turnVolts)/2, voltageUnits::volt);
BL.spin(forward, (forwardVolts + turnVolts)/2, voltageUnits::volt);
TR.spin(forward, (forwardVolts - turnVolts)/2, voltageUnits::volt);
TL.spin(forward, (forwardVolts + turnVolts)/2, voltageUnits::volt);
}

        else{
FR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
FL.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
BR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
BL.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
TR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
TL.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
}
    
  ///////////////////////
  //     Claw         //
  /////////////////////
      
      if (Controller1.ButtonL2.pressing() && claw_lock==0){
  claw_up = !claw_up;
  claw_lock = 1;}

  else if (!Controller1.ButtonL2.pressing()){
    claw_lock = 0;}

    if(claw_up)
      claw.close();

    else
      claw.open();

  /////////////////////
  //    Lift        //
  ///////////////////

  if(Controller1.ButtonR2.pressing()){
    lift();
}

    else if(Controller1.ButtonR1.pressing()){
      liftRev();
}

      else{
        liftBrake();
}
    /////////////////////
    //   Forklift     //
    ///////////////////

  if(Controller1.ButtonY.pressing()){
    forklift();
}

    else if(Controller1.ButtonRight.pressing()){
      forkliftRev();
}

      else{
        forkliftBrake();
}


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
