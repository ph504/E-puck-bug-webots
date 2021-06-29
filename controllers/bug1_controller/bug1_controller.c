/*
 * File:          epuck_controller_bug2.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/utils/ansi_codes.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * You may want to add macros here.
 */
#define PI 3.142857
 // define robot properties
#define MAX_SPEED 6.28
// the angular velocity of the wheel when we want to rotate the robot in constant place.
#define ROTATION_WHEEL_AV 1.57
#define TIME_STEP 32
#define NUM_OF_SENSORS 8
#define ANGULAR_VELOCITY_MULTIPLIER 1.7823
// shaft/axle between the wheels
#define AXLE_LENGTH_MM 52
// radius of wheel
#define RADIUS_MM 20.5 

// angular velocity of the robot in radian.
#define THETA_ANGULAR_VELOCITY 0.035254
// define steps which the robot takes to turn. and the time step is assumed to be 32ms. 
// the steps of time which takes the robot to rotate less than 45 degrees.
#define DEG_45_STEPS (int)( (theta_offset/THETA_ANGULAR_VELOCITY) - 1)
// the steps of time which takes the robot to rotate more than 45 degrees.
#define DEG_90_STEPS (int)( (theta_offset/THETA_ANGULAR_VELOCITY))

// define states here
#define CALCULATE_TARGET_ANGLE     1
#define ROTATE_HEAD_TOWARD_GOAL    2
#define MOVE_TOWARD_TARGET         3
#define BOUNDARY_TANGENT_DIRECTION 4
//#define BOUNDARY_WALL_FOLLWING     5
#define OBTSACLE_FULL_SCAN         5
#define GOTO_LEAVE_POINT           6
#define END                        7

// define transition functions
int rotate_head(WbDeviceTag,WbDeviceTag,double);
double calculate_line_slope(double,double,double,double);
double calculate_point2point_distance(double,double,double,double);
double update_current_angle(double,double);

// global variables
double left_motor_speed = 0;
double right_motor_speed = 0;
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
   /* necessary to initialize webots stuff */
   wb_robot_init();

    // bot state 
     int current_state = CALCULATE_TARGET_ANGLE;
     int rotation_steps = -1;
     int steps_needed = 0;

     double left_motor_speed = 0.0;
     double right_motor_speed = 0.0;

     //double currentVX = 0.0;
     //double previuseVX = 0.0;

     //double currentVZ = 0.0;
     //double previuseVZ = 0.0;
     double total_distance_walked = 0.0;
     double previousX;
     double previousZ;
     double originX = 0.0,             originZ = 0.4;
     double currentX,                  currentZ;

     double targetX = 0.3,             targetZ = -0.3;
     double target_distance;
     double min_obst_leaveX=INFINITY,  min_obst_leaveZ=INFINITY; 
     
     double hitpointX=INFINITY,        hitpointZ=INFINITY;
     double hitpoint_distance = 0.0;
     
     //double targetTheta = 0.0; // currently not needed. // might be the angle we want to have in the target point when the robot reaches.

     // offset angle from the target point.
     double heading_angle = 0.0;
     double theta_offset = 0.0; 
     double angle_accumulator = 0.0;

    // define transition conditions here
     bool _is_faced_target      = false;
     bool _meet_hit_point       = false;
     bool _is_in_tan_direc      = false;
     bool _left_hit_point       = false;
     bool _reached_hit_point    = false;
     bool _reached_leave_point  = false;
     bool _is_reached_goal      = false;
     
    // define threshhold values here
     const double distance_threshold = 0.03;
     const double threshold_offset = 0.1;
     const double leave_point_reach_threshhold = distance_threshold;
     const double hitpoint_reach_threshhold = distance_threshold;
     const double goal_reach_threshhold = distance_threshold;
     const double obstacle_detect_threshold = 100;
    // define distance sensor values here
     bool left_wall;
     bool left_corner;
     bool front_wall;

   WbDeviceTag ps[NUM_OF_SENSORS];
   char ps_names[NUM_OF_SENSORS][4] = {
     "ps0", "ps1", "ps2", "ps3",
     "ps4", "ps5", "ps6", "ps7"
   };
   double ps_value[NUM_OF_SENSORS];
   for (int i = 0; i < NUM_OF_SENSORS ; i++) {
      ps[i] = wb_robot_get_device(ps_names[i]);
      wb_distance_sensor_enable(ps[i], TIME_STEP);
   }

    // Get the accelerometer and enable it.
    WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(accelerometer, TIME_STEP); //possible time_step might be correct.
    // Get and enable gyro
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);
    // get and enable GPS device
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
    // enable the devices
    WbDeviceTag compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, TIME_STEP);

    // actuators init
     /*WbDeviceTag left_wheel_sensor = wb_robot_get_device("left wheel sensor");
     WbDeviceTag right_wheel_sensor = wb_robot_get_device("right wheel sensor");
     wb_position_sensor_enable(left_wheel_sensor, TIME_STEP);
     wb_position_sensor_enable(right_wheel_sensor, TIME_STEP);*/
     WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
     WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
     wb_motor_set_position(left_motor, INFINITY);
     wb_motor_set_position(right_motor, INFINITY);
     wb_motor_set_velocity(left_motor, 0);
     wb_motor_set_velocity(right_motor, 0);


     targetZ = -targetZ;
     originZ = -originZ;
     currentX = originX;
     currentZ = originZ;  
     previousX = originX;
     previousZ = originZ;
    
    /*
     * You should declare here WbDeviceTag variables for storing
     * robot devices like this:
     *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
     *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
     */
  
    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
//    ANSI_CLEAR_CONSOLE();
//    printf("rotation axes: [ x y z ] = [ %+.2f %+.2f %+.2f ]\n", vel[0], vel[1], vel[2]);
    while (wb_robot_step(TIME_STEP) != -1) {
    
     /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */      
      // Read gyro sensor data : vel[0], vel[1], vel[2]
      const double *vel = wb_gyro_get_values(gyro);
    
      // Get the acceleration vector, which is close the gravity vector.
      // acceleration[0..2] for ax...az respectively
      const double *acceleration = wb_accelerometer_get_values(accelerometer);

      // read GPS values
      const double *gps_values = wb_gps_get_values(gps);

      // read compass and rotate arrow accordingly
      const double *north = wb_compass_get_values(compass);
      
      // get value of proximity sensors
      for (int i = 0; i < NUM_OF_SENSORS; i++){
        ps_value[i] = wb_distance_sensor_get_value(ps[i]);
      }         
      
      /* Process sensor data here */
      // find state of the robot

      // update values

      // measure the location  
       /*printf("aZ=%+.3f \n",acceleration[1]);
       printf("aX=%+.3f \n",acceleration[0]);*/
      heading_angle = -atan2(north[2], north[0]);
      heading_angle = heading_angle<0?heading_angle+2*PI:heading_angle;
      //printf("heading_angle=%.3f\n",heading_angle);
      //printf("north[0]=%.3f\tnorth[1]=%.3f\tnorth[2]=%.3f\n",north[0], north[1], north[2]);
      
      // update the location of the bot
       previousX = currentX;
       previousZ = currentZ;
       currentX = gps_values[0];
       currentZ = -gps_values[2];
       total_distance_walked += calculate_point2point_distance(previousX, previousZ, currentX, currentZ);
      // use distance sensors
       left_wall = ps_value[5] > 80;
       left_corner = ps_value[6] > 80;
       front_wall = ps_value[7] > 80;

       hitpoint_distance = calculate_point2point_distance(currentX,currentZ, hitpointX, hitpointZ);

      // check transition conditions here
       /*printf("steps_needed=%d\n",steps_needed);
       printf("steps=%d\n",rotation_steps);*/
       _is_faced_target     = rotation_steps >= steps_needed;
       _meet_hit_point      = ps_value[6]>obstacle_detect_threshold || ps_value[7]>obstacle_detect_threshold || ps_value[0]>obstacle_detect_threshold || ps_value[1]>obstacle_detect_threshold;
       _is_in_tan_direc     = ps_value[5]> 80;
       //printf("ps5=%+.3f \n",ps_value[5]);

       target_distance = calculate_point2point_distance(currentX,currentZ,targetX,targetZ);

       // if current_state==BOUNDARY_TANGENT_DIRECTION ==> false.
       if(current_state==BOUNDARY_TANGENT_DIRECTION) _left_hit_point = false;
       _left_hit_point      = _left_hit_point?true: hitpoint_distance > hitpoint_reach_threshhold + threshold_offset; 
       _reached_hit_point   = _left_hit_point && hitpoint_distance < hitpoint_reach_threshhold;
       _reached_leave_point = calculate_point2point_distance(currentX,currentZ,min_obst_leaveX,min_obst_leaveZ) < leave_point_reach_threshhold;
       _is_reached_goal     =  target_distance < 0.07;
       //printf("target_distance= %.3f\n\n",target_distance);
      
      //printf("hitpoint_distance=%+.3f\n",hitpoint_distance);
      // update the angle of the bot
      //angle_accumulator += update_current_angle(left_motor_speed,right_motor_speed);
      /*
       * Enter here functions to send actuator commands, like:
       * wb_motor_set_position(my_actuator, 10.0);
       */
      // Define flow graphs here
      switch(current_state){
       case CALCULATE_TARGET_ANGLE: {
       
          // find the target angle in radian
          // can wrap the code below into a function later.
          double slope_toward_target = calculate_line_slope(originX,originZ,targetX,targetZ);
          theta_offset = atan(slope_toward_target);
          if(slope_toward_target*(targetX-originX) < 0 || (targetX-originX==0 && targetZ-originZ<0) ) { // (dx < 0 && slope > 0) || (dx > 0 && slope < 0) or simply is backwards
            theta_offset += PI;
          }
          else if (targetX-originX < 0) {
            // printf("if is satisfied\n");
            theta_offset += 2*PI;
          }
          // else no operation.

          theta_offset -= heading_angle;
          theta_offset = theta_offset<0?theta_offset+2*PI:theta_offset;
          /*printf("slope_toward_target=%+.3f \n",slope_toward_target);
          printf("theta_offset=%+.3f \n",theta_offset);*/

          /* 
          targetTheta = atan2((targetZ-currentZ),(targetX-currentX));
          printf("find goal angle from start point\n");
          printf("current angle = %+.4f\n",currentTheta);
          printf("target angle = %+.4f\n",targetTheta );
          */
          break;
       }
       case ROTATE_HEAD_TOWARD_GOAL: {
          steps_needed = rotate_head(left_motor, right_motor, theta_offset);
          rotation_steps = rotation_steps + 1; 
          //printf("STEPS=%d",rotation_steps);
          break;          
       }
       case MOVE_TOWARD_TARGET: {
       
         // moving toward target
          //printf("move toward target\n");
          left_motor_speed = MAX_SPEED;
          right_motor_speed = MAX_SPEED;
          wb_motor_set_velocity(left_motor, MAX_SPEED);
          wb_motor_set_velocity(right_motor, MAX_SPEED);
          break;

       }
       case BOUNDARY_TANGENT_DIRECTION: {
          hitpointX = currentX;
          hitpointZ = currentZ;
          min_obst_leaveX = hitpointX;
          min_obst_leaveZ = hitpointZ;
         // do head rotation
          //printf("head rotation toward boundary tangent line\n");
          left_motor_speed = MAX_SPEED;
          right_motor_speed = -MAX_SPEED;
          wb_motor_set_velocity(left_motor, MAX_SPEED);
          wb_motor_set_velocity(right_motor, -MAX_SPEED);
          break;
       
       }
       case OBTSACLE_FULL_SCAN: {        
        target_distance = calculate_point2point_distance(currentX, currentZ, targetX, targetZ);
        double minimum_distance = calculate_point2point_distance(min_obst_leaveX, min_obst_leaveZ, targetX, targetZ);
        //printf("target_distance= %.3f\n\n",target_distance);
        //printf("minimum_distance= %.3f\n\n",minimum_distance);
        if( minimum_distance > target_distance) {
          min_obst_leaveX = currentX;
          min_obst_leaveZ = currentZ;
        }

       }
       case GOTO_LEAVE_POINT: {
          if(front_wall){
              //printf("trun right in place\n");
              left_motor_speed = MAX_SPEED;
              right_motor_speed = -MAX_SPEED;
              wb_motor_set_velocity(left_motor, left_motor_speed);
              wb_motor_set_velocity(right_motor, right_motor_speed);
          }else{
            if(left_wall){
              //printf("drive forward\n");
              left_motor_speed = MAX_SPEED;
              right_motor_speed = MAX_SPEED;
              wb_motor_set_velocity(left_motor, left_motor_speed);
              wb_motor_set_velocity(right_motor, right_motor_speed);      
            }else{
              //printf("turn left\n");
              left_motor_speed = MAX_SPEED/8;
              right_motor_speed = MAX_SPEED;
              wb_motor_set_velocity(left_motor, left_motor_speed);
              wb_motor_set_velocity(right_motor, right_motor_speed);      
            }
            if(left_corner){
              //printf("getting far from the wall by driving right\n");
              left_motor_speed = MAX_SPEED;
              right_motor_speed = MAX_SPEED/8;
              wb_motor_set_velocity(left_motor, left_motor_speed);
              wb_motor_set_velocity(right_motor, right_motor_speed); 
            }
          }
          break;
       }
      }

       // Just state transition and flow here
      switch(current_state){
        case CALCULATE_TARGET_ANGLE :{          
          current_state = ROTATE_HEAD_TOWARD_GOAL;
          break;
        }
        case ROTATE_HEAD_TOWARD_GOAL :{
          //printf("theta_offset=%.3f\n",theta_offset);
          //printf("steps_needed=%d\n",steps_needed);
          //printf("steps=%d\n",rotation_steps);
          if(_is_faced_target){
            rotation_steps = -1;
            theta_offset = 0;
            angle_accumulator = 0;
            current_state = MOVE_TOWARD_TARGET;
          }
          break;
        }
        case MOVE_TOWARD_TARGET :{          
          if(_is_reached_goal) {
            printf("target reached!\n");
            printf("total distance walked = %.3f\n", total_distance_walked);
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);  
            current_state = END; 
          }
          if(_meet_hit_point) {
            current_state = BOUNDARY_TANGENT_DIRECTION;
          }
          break;
        }
        case BOUNDARY_TANGENT_DIRECTION:{
          if(_is_in_tan_direc){
            //printf("is tangent\n");
            current_state = OBTSACLE_FULL_SCAN;
          }
          break;
        }
        case OBTSACLE_FULL_SCAN: {
          //printf("leave point in \n\tx=%+.3f,\n\tz=%+.3f\n\n",min_obst_leaveX,min_obst_leaveZ);
          //printf("located in \n\tx=%+.3f,\n\tz=%+.3f\n\n",currentX,currentZ);
          if(_reached_hit_point) {
            current_state = GOTO_LEAVE_POINT;
          }
          break;
        }
        case GOTO_LEAVE_POINT: {
          if(_reached_leave_point) {
            current_state = CALCULATE_TARGET_ANGLE;
          } 
          break;
        }
        /*case BOUNDARY_WALL_FOLLWING :{
          if(_reached_M_line && _got_near_target)current_state = ROTATE_HEAD_TOWARD_GOAL; break;
        }*/
        case END :{
          goto terminating_process;
        }
      } 
       
    };
    terminating_process:
    /* Enter your cleanup code here */
  
    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

  return 0;
}

double calculate_line_slope(double origin_x,double origin_z,double target_x,double target_z){
  return (target_x-origin_x)/(target_z-origin_z);
}

int rotate_head(WbDeviceTag left_motor,WbDeviceTag right_motor, double theta_offset){
    double phi1 = ROTATION_WHEEL_AV; // dot{phi_1}
    double phi2 = -ROTATION_WHEEL_AV; // dot{phi_2}
    wb_motor_set_velocity(left_motor, phi1);
    wb_motor_set_velocity(right_motor, phi2);
    //printf("theta_offset=%.3f \n",theta_offset); 

    // returns how much steps of time is needed before the robot stops rotating.
    return (theta_offset<0.8?DEG_45_STEPS:DEG_90_STEPS);
    // 0.8 radian approximately= 45 degrees.
}

/*double update_current_angle(double phil, double phir) {
   double change_angle_rate = (RADIUS_MM*phil)/(2*AXLE_LENGTH_MM) - (RADIUS_MM*(phir))/(2*AXLE_LENGTH_MM);
   double result = change_angle_rate*TIME_STEP*ANGULAR_VELOCITY_MULTIPLIER/1000;
   return result;
   //fabs(result)>3.14?(result>0?1:-1)*(fabs(result)-3.14):result;
}*/

double calculate_point2point_distance(double currentX, double currentY, double targetX, double targetY) {
  return sqrt((targetY-currentY)*(targetY-currentY)+(targetX-currentX)*(targetX-currentX));
}



