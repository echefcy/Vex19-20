/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftFrontMotor       motor         11              
// RightFrontMotor      motor         19              
// RightBackMotor       motor         14              
// LeftBackMotor        motor         13              
// LeftLiftMotor        motor         15              
// RightLiftMotor       motor         16              
// LeftRollerMotor      motor         17              
// RightRollerMotor     motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <algorithm>
#include <ctime>
using namespace std;

// A global instance of competition
competition Competition;
controller Controller1 = controller(primary);
motor LeftFrontMotor = motor(PORT11,ratio18_1,false);
motor RightFrontMotor = motor(PORT5,ratio18_1, true);
motor LeftBackMotor = motor(PORT12,ratio18_1, false);
motor RightBackMotor = motor(PORT14,ratio18_1, true);
motor LeftLiftMotor = motor(PORT8,ratio36_1, false);
motor RightLiftMotor = motor(PORT6,ratio36_1, true);
motor LeftRollerMotor = motor(PORT17,ratio18_1, false);
motor RightRollerMotor = motor(PORT20,ratio18_1, true);
inertial inertial_sensor = inertial(PORT7);
// motor_group Lift = motor_group(LeftLiftMotor,RightLiftMotor);

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertial_sensor.startCalibration();
  while(inertial_sensor.isCalibrating()){
    wait(2,msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void InertialPIDturnCW(double goal){
  inertial_sensor.resetRotation();
  double drivepower = 0;
  
  double Kp = 0.55;
  double Kd = 0.23;
  // double Ki = 0;
  double Ki = 0.002;
  
  double proportional;
  double derivative;
  double last_error = goal - inertial_sensor.yaw();
  double integral;
  double integral_active_zone = 0.25*std::abs(goal);
  double integral_limit = 30;
  
  double error = goal - inertial_sensor.yaw();

  bool timerBool = true;
  timer t = timer();
  t.clear();
  double duration;

  while(timerBool){

    proportional = Kp*error;

    derivative = Kd*(error-last_error);
    last_error = error;
    if(error == 0){
      derivative = 0;
    }

    if(std::abs(error)<integral_active_zone && error != 0){
      integral += Ki*error;
    }else{
      integral = 0;
    }
    if(integral > integral_limit){
      integral = integral_limit;
    }else if(integral < -integral_limit){
      integral = -integral_limit;
    }

    drivepower = proportional+integral+derivative;
    LeftBackMotor.spin(directionType::fwd, drivepower, velocityUnits::pct);
    RightBackMotor.spin(directionType::rev, drivepower, velocityUnits::pct);
    LeftFrontMotor.spin(directionType::fwd, drivepower, velocityUnits::pct);
    RightFrontMotor.spin(directionType::rev, drivepower, velocityUnits::pct);
    wait(10,msec);

    error = goal - inertial_sensor.yaw();

    if(std::abs(error)<0.5){
      duration = t.time();
    }else{
      t.clear();
    }
    if(duration > 40){
      timerBool = false;
    }
  }
  LeftBackMotor.stop(hold);
  RightBackMotor.stop(hold);
  LeftFrontMotor.stop(hold);
  RightFrontMotor.stop(hold);
  wait(200,msec);
  LeftBackMotor.stop(coast);
  RightBackMotor.stop(coast);
  LeftFrontMotor.stop(coast);
  RightFrontMotor.stop(coast);
}

void InertialPIDturnCCW(double goal){
  inertial_sensor.resetRotation();
  double drivepower = 0;
  
  double Kp = 0.55;
  double Kd = 0.23;
  // double Ki = 0;
  double Ki = 0.002;
  
  double proportional;
  double derivative;
  double last_error = goal - inertial_sensor.yaw();
  double integral;
  double integral_active_zone = 0.25*std::abs(goal);
  double integral_limit = 30;
  
  double error = goal - inertial_sensor.yaw();

  bool timerBool = true;
  timer t = timer();
  t.clear();
  double duration;

  while(timerBool){

    proportional = Kp*error;

    derivative = Kd*(error-last_error);
    last_error = error;
    if(error == 0){
      derivative = 0;
    }

    if(std::abs(error)<integral_active_zone && error != 0){
      integral += Ki*error;
    }else{
      integral = 0;
    }
    if(integral > integral_limit){
      integral = integral_limit;
    }else if(integral < -integral_limit){
      integral = -integral_limit;
    }

    drivepower = proportional+integral+derivative;
    LeftBackMotor.spin(directionType::rev, drivepower, velocityUnits::pct);
    RightBackMotor.spin(directionType::fwd, drivepower, velocityUnits::pct);
    LeftFrontMotor.spin(directionType::rev, drivepower, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, drivepower, velocityUnits::pct);
    wait(10,msec);

    error = goal - inertial_sensor.yaw();

    if(std::abs(error)<0.5){
      duration = t.time();
    }else{
      t.clear();
    }
    if(duration > 40){
      timerBool = false;
    }
  }
  LeftBackMotor.stop(hold);
  RightBackMotor.stop(hold);
  LeftFrontMotor.stop(hold);
  RightFrontMotor.stop(hold);
  wait(200,msec);
  LeftBackMotor.stop(coast);
  RightBackMotor.stop(coast);
  LeftFrontMotor.stop(coast);
  RightFrontMotor.stop(coast);
}

void roller(double speed){
  LeftRollerMotor.spin(directionType::fwd,speed,velocityUnits::pct);
  RightRollerMotor.spin(directionType::fwd,speed,velocityUnits::pct);
}

void stop_roller(){
  LeftRollerMotor.stop(brakeType::hold);
  RightRollerMotor.stop(brakeType::hold);
}

void straight(double rot, double speed){
  // drive straight
  // rot defines the amount of rotations and direction
  // speed defines speed
  LeftBackMotor.startRotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct);
  RightBackMotor.startRotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct);
  RightFrontMotor.startRotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct);
  LeftFrontMotor.rotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct,true);
  LeftBackMotor.stop(hold);
  RightBackMotor.stop(hold);
  LeftFrontMotor.stop(hold);
  RightFrontMotor.stop(hold);
  wait(200,msec);
  LeftBackMotor.stop(coast);
  RightBackMotor.stop(coast);
  LeftFrontMotor.stop(coast);
  RightFrontMotor.stop(coast);
}

void turnCW(double rot, double speed){
  // turn clockwise
  // rot defines the amount of rotations and direction (- is CCW)
  // speed defines speed
  LeftBackMotor.startRotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct);
  RightBackMotor.startRotateFor(-rot,rotationUnits::rev,speed,velocityUnits::pct);
  RightFrontMotor.startRotateFor(-rot,rotationUnits::rev,speed,velocityUnits::pct);
  LeftFrontMotor.rotateFor(rot,rotationUnits::rev,speed,velocityUnits::pct,true);
  LeftBackMotor.stop(brakeType::hold);
  RightBackMotor.stop(brakeType::hold);
  RightFrontMotor.stop(brakeType::hold);
  LeftFrontMotor.stop(brakeType::hold);
}

void lift_absolute(double rot_pos, double speed){
  // moves lift to an absolute position
  // rot_pos defines position (>= 0)
  // speed defines speed
  LeftLiftMotor.startRotateTo(rot_pos, rotationUnits::rev, speed, velocityUnits::pct);
  RightLiftMotor.rotateTo(rot_pos, rotationUnits::rev, speed, velocityUnits::pct,true);
  LeftLiftMotor.stop(brakeType::hold);
  RightLiftMotor.stop(brakeType::hold);
}

void lift_relative(double relative_rot, double speed){
  // moves lift relative to the current position
  // relative_rot defines how much the lift is to be moved by (+ up, - down)
  // speed defines speed
  LeftLiftMotor.startRotateFor(relative_rot, rotationUnits::rev, speed, velocityUnits::pct);
  RightLiftMotor.startRotateFor(relative_rot, rotationUnits::rev, speed, velocityUnits::pct);
  LeftLiftMotor.stop(brakeType::hold);
  RightLiftMotor.stop(brakeType::hold);
}

void stacking(double cubes, double lift_speed){
  //uses the number of cubes to determine how tall to stack
  double height = cubes * 30; 
  RightLiftMotor.startRotateTo(height, rotationUnits::rev, lift_speed, velocityUnits::pct);
  LeftLiftMotor.startRotateTo(height, rotationUnits::rev, lift_speed, velocityUnits::pct);
  RightRollerMotor.rotateFor(height,rotationUnits::rev);
  LeftRollerMotor.rotateFor(-height,rotationUnits::rev);
}

void collect(double cubes, double roller_speed){
  double height = cubes * 30;
  double lift_speed = 20;
  double time = cubes * 1.2;
  RightLiftMotor.startRotateTo(height, rotationUnits::rev, lift_speed, velocityUnits::pct);
  LeftLiftMotor.startRotateTo(height , rotationUnits::rev, lift_speed, velocityUnits::pct);
  RightRollerMotor.rotateFor(time,timeUnits::sec, roller_speed,velocityUnits::pct);
  LeftRollerMotor.rotateFor(time,rotationUnits::rev,roller_speed, velocityUnits::pct);
}

void autonomous(void) {
  pre_auton();
  InertialPIDturnCW(90);
  // straight(4.1,50);
  // InertialPIDturnCCW(270);
  // lift_absolute(-1.75/1.4,50);
  // straight(0.95, 20);


  // straight(0.32,20);
  // lift_absolute(-1.75/4, 50);
  // roller(-100);
  // straight(0.4,20);
  // lift_absolute(-1.75/4+1.75/6, 50);
  // wait(200,msec);
  // straight(1.7,50);
  // stop_roller();
  // lift_absolute(-1.75/1.4,50);
  // straight(0.95, 20);
  // roller(-100);
  // lift_absolute(-1.75/4+1.75/6, 50);
  // straight(-1.7,50);
  // InertialPIDturnCW(90);

  // vexcodeInit();
  // straight(4.1,50);
  // InertialPIDturnCW(88);
  // lift_absolute(-1.75/1.4, 50);
  // straight(1.2,20);
  // roller()


  // double four_height = 100;
  // double one_height = 20;
  // double bot_speed = 50;
  // double collect_height = 10;
  // double intake = 30;
  // double outtake = 40;
  // vexcodeInit();
  // straight(bot_speed,25);
  // lift_absolute(four_height,30);
  // straight(10, bot_speed);
  // lift_absolute(collect_height,intake);
  // lift_absolute(one_height, 30);
  // turnCW(40, bot_speed);
  // straight(100, bot_speed);
  // lift_absolute(collect_height, intake);
  // lift_absolute(one_height, 30);
  // straight(30, bot_speed);
  // stacking(6, outtake);

  // 1 cube
  // LeftBackMotor.setVelocity(100,velocityUnits::pct);
  // RightBackMotor.setVelocity(100,velocityUnits::pct);
  // LeftFrontMotor.setVelocity(100,velocityUnits::pct);
  // RightFrontMotor.setVelocity(100,velocityUnits::pct);
  // LeftBackMotor.rotateFor(-500,rotationUnits::deg,false);
  // RightBackMotor.rotateFor(-500,rotationUnits::deg,false);
  // LeftFrontMotor.rotateFor(-500,rotationUnits::deg,false);
  // RightFrontMotor.rotateFor(-500,rotationUnits::deg,true);

  // test
  // straight(1,50);
  // stop_drive();
  // wait(1000,timeUnits::msec);
  // straight(-1,50);
  // stop_drive();
  // wait(1000,timeUnits::msec);
  // turnCW(1.5, 50);
  // stop_drive();
  // wait(1000,timeUnits::msec);
  // turnCW(-1.5,50);
  // stop_drive();
}

void driving(){
  // double scaling_constant = 10;
  // double lift_activation = 0.875;
  double left_speed = 0;
  double right_speed = 0;
  left_speed = Controller1.Axis3.position(percentUnits::pct)+0.45*Controller1.Axis1.position(percentUnits::pct);
  right_speed = Controller1.Axis3.position(percentUnits::pct)-0.45*Controller1.Axis1.position(percentUnits::pct);
  if(std::abs(left_speed)<3||std::abs(right_speed)<3){
    LeftFrontMotor.stop(brakeType::coast);
    RightFrontMotor.stop(brakeType::coast);
    LeftBackMotor.stop(brakeType::coast);
    RightBackMotor.stop(brakeType::coast);
  }else{
    // double lift_loc = (-LeftLiftMotor.rotation(rotationUnits::rev)+RightLiftMotor.rotation(rotationUnits::rev))/2;
    // if(lift_loc > lift_activation){
    //   left_speed = left_speed/max<double>(1,((lift_loc-lift_activation)*scaling_constant));
    //   right_speed = right_speed/max<double>(1,((lift_loc-lift_activation)*scaling_constant));
    // }
    LeftFrontMotor.spin(directionType::fwd, left_speed, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, right_speed, velocityUnits::pct);
    LeftBackMotor.spin(directionType::fwd, left_speed, velocityUnits::pct);
    RightBackMotor.spin(directionType::fwd, right_speed, velocityUnits::pct);
  }
}

void lifting(double speed){
  if(Controller1.ButtonL1.pressing()&&Controller1.ButtonL2.pressing()){
    LeftLiftMotor.stop(brakeType::hold);
    RightLiftMotor.stop(brakeType::hold);
  }else if(Controller1.ButtonL1.pressing()){
    LeftLiftMotor.spin(directionType::fwd, -speed, velocityUnits::pct);
    RightLiftMotor.spin(directionType::fwd, -speed, velocityUnits::pct);
  }else if(Controller1.ButtonL2.pressing()){
    LeftLiftMotor.spin(directionType::fwd, speed, velocityUnits::pct);
    RightLiftMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  }else{
    LeftLiftMotor.stop(brakeType::hold);
    RightLiftMotor.stop(brakeType::hold);
  }
}

void lifting(double speed, double pos){
  if(Controller1.ButtonL1.pressing()&&Controller1.ButtonL2.pressing()){
    LeftLiftMotor.stop(brakeType::hold);
    RightLiftMotor.stop(brakeType::hold);
  }else if(Controller1.ButtonL1.pressing()){
    LeftLiftMotor.spin(directionType::fwd, -speed, velocityUnits::pct);
    RightLiftMotor.spin(directionType::fwd, -speed, velocityUnits::pct);
  }else if(Controller1.ButtonL2.pressing()){
    LeftLiftMotor.spin(directionType::fwd, speed, velocityUnits::pct);
    RightLiftMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  }else{
    LeftLiftMotor.startRotateTo(-pos, rotationUnits::rev, speed, velocityUnits::pct);
    RightLiftMotor.startRotateTo(-pos, rotationUnits::rev, speed, velocityUnits::pct);
  }
}

void rollering(double out_speed){
  if(Controller1.ButtonR1.pressing()&&Controller1.ButtonR2.pressing()){
    LeftRollerMotor.stop(brakeType::coast);
    RightRollerMotor.stop(brakeType::coast);
  }else if(Controller1.ButtonR1.pressing()){
    LeftRollerMotor.spin(directionType::fwd,-100,velocityUnits::pct);
    RightRollerMotor.spin(directionType::fwd,-100,velocityUnits::pct);
  }else if(Controller1.ButtonR2.pressing()){
    LeftRollerMotor.spin(directionType::fwd,out_speed,velocityUnits::pct);
    RightRollerMotor.spin(directionType::fwd,out_speed,velocityUnits::pct);
  }else{
    LeftRollerMotor.stop(brakeType::coast);
    RightRollerMotor.stop(brakeType::coast);
  }
}

void auto_stacker(double out_speed, double lift_speed){
  if(Controller1.ButtonA.pressing()){
    LeftRollerMotor.spin(directionType::fwd,out_speed,velocityUnits::pct);
    RightRollerMotor.spin(directionType::fwd,out_speed,velocityUnits::pct);
    LeftLiftMotor.spin(directionType::fwd,lift_speed,velocityUnits::pct);
    RightLiftMotor.spin(directionType::fwd,lift_speed,velocityUnits::pct);
  }else{
    LeftRollerMotor.stop(brakeType::coast);
    RightRollerMotor.stop(brakeType::coast);
    LeftLiftMotor.stop(brakeType::hold);
    RightLiftMotor.stop(brakeType::hold);
  }
}

void normal_drive(){
  driving();
  if(Controller1.ButtonA.pressing()){
    auto_stacker(55,-23);
  }else{
    lifting(50);
    rollering(100);
  }
}

void intake_drive(){
  driving();
  lifting(50,1.75/6.5);
  LeftRollerMotor.spin(directionType::fwd,-75,velocityUnits::pct);
  RightRollerMotor.spin(directionType::fwd,-75,velocityUnits::pct);
}

void usercontrol(void) {
  // User control code here, inside the loop
  bool flag = false;
  while (1) {
    if(!flag){
      normal_drive();
    }else{
      intake_drive();
    }
    if(Controller1.ButtonX.pressing()==1){
      wait(20,msec);
      if(Controller1.ButtonX.pressing()==0){
        flag = !flag;
      }
    }

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
