/*
 * File:          bug2_controller.c
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
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/utils/ansi_codes.h>
#include <stdio.h>
#include <stdlib.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 8
#define MAX_SPEED 6.28
#define NUM_OF_SENSORS 8
#define AXLE_LENGTH 0.052
#define RADIUS 0.0205 

/*
 * Define states here
 */
#define CALCULATE_TARGET_ANGLE     1
#define ROTATE_HEAD_TOWARD_GOAL    2
#define MOVE_TOWARD_TARGET         3
#define BOUNDARY_TANGENT_DIRECTION 4
#define BOUNDARY_WALL_FOLLWING     5
#define END                        6
/*
 * Define functions here
 */
 double calculate_point2point_distance(double, double, double, double);
 double calculate_point_distance_to_target(double,double,double,double);
 double calculate_distance_from_M_line(double,double,double,double, double);

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
   int i;
   // bot state 
   int current_state;
   double left_motor_speed = 0;
   double right_motor_speed = 0;
   double currentVX = 0.0;
   double previuseVX = 0.0;
   double currentVZ = 0.0;
   double previuseVZ = 0.0;
   double distance_from_target;
   double M_Slope;
   // bot location
   double currentX = 0;
   double currentZ = 0.4;
   double total_distance_walked = 0.0;
   double previousX = currentX;
   double previousZ = currentZ;
   double currentTheta = 3.14;
   // target location
   double targetX = 0.4;
   double targetZ = -0.4;
   double targetTheta = 0.0;
   // define transition conditions here
   bool _is_faced_target = false;
   bool _meet_hit_point  = false;
   bool _is_in_tan_direc = false;
   bool _reached_M_line  = false;
   bool _got_near_target = false;
   bool _is_reached_goal = false;
   bool _init_duration_done = false;
   // define threshhold values here
   double      goal_angle_threshhold = 0.05;
   double M_line_distance_threshhold = 0.001;
   double      goal_reach_threshhold = 0.1;
   // define distance sensor values here
   bool left_wall;
   bool left_corner;
   bool front_wall;
   int counter;
   
    
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
    WbDeviceTag ps[NUM_OF_SENSORS];
    char ps_names[NUM_OF_SENSORS][4] = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };
    double proximity_sensor_value[NUM_OF_SENSORS];
    for (i = 0; i < NUM_OF_SENSORS ; i++) {
       ps[i] = wb_robot_get_device(ps_names[i]);
       wb_distance_sensor_enable(ps[i], TIME_STEP);
    }
   
    WbDeviceTag left_motor =  wb_robot_get_device( "left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor,  INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor,  0);
    wb_motor_set_velocity(right_motor, 0);
    
    // Get the accelerometer and enable it.
    WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(accelerometer, TIME_STEP);
    
    // Get and enable gyro
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);
    
    // get and enable GPS device
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);

    // enable the devices
    WbDeviceTag compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, TIME_STEP);


  // initiate robot memory here
  // calculate distance to goal
   distance_from_target = calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ);
  // Define start state here
   current_state = CALCULATE_TARGET_ANGLE;
  // Define M-line slope
  M_Slope = (targetX-currentX)/(targetZ-currentZ);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
      // Read gyro sensor data : vel[0], vel[1], vel[2]
      const double *vel = wb_gyro_get_values(gyro);
      
      // read compass and rotate arrow accordingly
      const double *north = wb_compass_get_values(compass);
    
       // Get the acceleration vector, which is close the gravity vector.
      // acceleration[0..2] for ax...az respectively
      const double *acceleration = wb_accelerometer_get_values(accelerometer);
      
      // read GPS values
      const double *gps_values = wb_gps_get_values(gps);
      
      // get value of proximity sensors
      for (i = 0; i < NUM_OF_SENSORS; i++){
        proximity_sensor_value[i] = wb_distance_sensor_get_value(ps[i]);
      }  
  
      currentTheta = atan2(north[0], north[2]);
      //printf("direction angle = %+.4f deg\n",currentTheta);

    /* Process sensor data here */
    // update state values here
    
    // measure the angle
     //printf("delta theta = %+.4f\n",TIME_STEP*vel[1]*0.001);
     //printf("current angle = %+.4f rad\n",currentTheta);
     //printf("target angle = %+.4f\n",targetTheta );
     //printf("Thetax = %+.4f, Thetay = %+.4f, Thetaz = %+.4f\n",vel[0],vel[1],vel[2]);
     //currentTheta = currentTheta + TIME_STEP*(left_motor_speed==right_motor_speed?0:(left_motor_speed>right_motor_speed?-1:1))*fabs((RADIUS*left_motor_speed/AXLE_LENGTH)-(RADIUS*right_motor_speed/AXLE_LENGTH))*0.001;
     //currentTheta = currentTheta < 0 ? currentTheta + 6.28 : (currentTheta>6.28?currentTheta-6.28:currentTheta);
     
    // measure the location  
     //currentX = currentX + acceleration[0]*TIME_STEP*TIME_STEP*0.000001/2 + previuseVX*TIME_STEP*0.001;
     //currentZ = currentZ + acceleration[1]*TIME_STEP*TIME_STEP*0.000001/2 + previuseVZ*TIME_STEP*0.001;
     previousX = currentX;
     previousZ = currentZ;
     currentX = gps_values[0];
     currentZ = gps_values[2];
     total_distance_walked += calculate_point2point_distance(previousX, previousZ, currentX, currentZ);
     //printf("current location = (%+.4f, %+.4f)\n",currentX,currentZ);
     
    // measure and update velocity
     //previuseVX = currentVX;
     //previuseVZ = currentVZ;
     //currentVX = currentVX + acceleration[0]*TIME_STEP*0.001;
     //currentVZ = currentVZ + acceleration[1]*TIME_STEP*0.001;
     //printf("Ax = %+.4f, Ay = %+.4f, Az = %+.4f\n",acceleration[0],acceleration[1],acceleration[2]);
     //printf("current velocity = (%+.4f, %+.4f)\n",currentVX,currentVZ);
     //printf("previous velocity = (%+.4f, %+.4f)\n",previuseVX,previuseVZ);
     
    
    // use distance sensors
     left_wall = proximity_sensor_value[5] > 80;
     left_corner = proximity_sensor_value[6] > 80;
     front_wall = proximity_sensor_value[7] > 80;
     
     
    // check transition conditions here
     _is_faced_target = fabs(targetTheta - currentTheta) < goal_angle_threshhold;
     _meet_hit_point  = proximity_sensor_value[6]>80 || proximity_sensor_value[7]>80 || proximity_sensor_value[0]>80 || proximity_sensor_value[1]>80;
     _is_in_tan_direc = proximity_sensor_value[5] > 80;
     _reached_M_line  = calculate_distance_from_M_line(currentZ,currentX,-M_Slope,1,M_Slope*0.4) < M_line_distance_threshhold;
     _got_near_target = distance_from_target > calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ);
     _is_reached_goal = calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ) < goal_reach_threshhold;
     _init_duration_done = counter < 0;
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
     //printf("-----------------------------------------------\n");
     //printf("gps[0]=%+.4f , gps[1]=%+.4f , gps[2]=%+.4f \n",gps_values[0],gps_values[1],gps_values[2]);
     //printf("distance from line = %+.4f \n",calculate_distance_from_M_line(currentZ,currentX,-tan(targetTheta),1,tan(targetTheta)*0.4) );
     //printf("privious nearest point to target = %+.4f \n",distance_from_target);
     //printf("current distance to target = %+.4f \n",calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ));
     

     
     
     // Define flow graphs here
     if(current_state == CALCULATE_TARGET_ANGLE){
     
        //printf(">>>>>>>> CALCULATE_TARGET_ANGLE\n");
       // find the target angle in radian
        targetTheta = atan2((targetZ-currentZ),(targetX-currentX)) + 3.14;
        targetTheta = fabs(targetTheta) > 3.14 ? (targetTheta>0?1:-1)*(fabs(targetTheta)-3.14) : targetTheta;
        //printf("target angle = %+.4f\n",targetTheta );
        //targetTheta = targetTheta < 0 ? targetTheta + 6.28 : (targetTheta > 6.28 ? targetTheta - 6.28 : targetTheta);
        //printf("original theta angle = %+.4f\n",targetTheta);
        //printf("find goal angle from start point\n");
        //printf("current angle = %+.4f\n",currentTheta);
        //printf("target angle = %+.4f\n",targetTheta );
       
     }
     if(current_state == ROTATE_HEAD_TOWARD_GOAL){
       
        //printf(">>>>>>>> ROTATE_HEAD_TOWARD_GOAL\n");
       // do head rotation
        left_motor_speed = MAX_SPEED/8;
        right_motor_speed = -MAX_SPEED/8;
        //printf("head rotation toward goal\n");
        //printf("n[0]=%+.4f , n[1]=%+.4f , n[2]=%+.4f \n",north[0],north[1],north[2]);
        //printf("current angle = %+.4f\n",currentTheta);
        //printf("target angle = %+.4f\n",targetTheta );
     
     }
     if(current_state == MOVE_TOWARD_TARGET){
     
        //printf(">>>>>>>> MOVE_TOWARD_TARGET\n");
       // moving toward target
        //printf("move toward target\n");
        left_motor_speed = MAX_SPEED/2;
        right_motor_speed = MAX_SPEED/2;
     }
     if(current_state == BOUNDARY_TANGENT_DIRECTION){
     
     
       //printf(">>>>>>>> BOUNDARY_TANGENT_DIRECTION\n");
       // do head rotation
        //printf("head rotation toward boundary tangent line\n");
        left_motor_speed = MAX_SPEED/8;
        right_motor_speed = -MAX_SPEED/8;
     }
     if(current_state == BOUNDARY_WALL_FOLLWING){
         
         
        // printf(">>>>>>>> BOUNDARY_WALL_FOLLWING\n"); 
        // printf("_reached_M_line=%d, _got_near_target=%d, _init_duration_done=%d\n",_reached_M_line,_got_near_target,_init_duration_done); 
        // count down init duration
        counter -= 1;
        
        if(front_wall){
            //printf("trun right in place\n");
            left_motor_speed = MAX_SPEED;
            right_motor_speed = -MAX_SPEED;
        }else if(left_corner){
            //printf("getting far from the wall by driving right\n");
            left_motor_speed = MAX_SPEED;
            right_motor_speed = MAX_SPEED/8;
        }else{
          if(left_wall){
            //printf("drive forward\n");
            left_motor_speed = MAX_SPEED;
            right_motor_speed = MAX_SPEED;     
          }else{
            //printf("turn left\n");
            left_motor_speed = MAX_SPEED/8;
            right_motor_speed = MAX_SPEED;    
          }
          
        }
     }
     
      wb_motor_set_velocity(left_motor, left_motor_speed);
      wb_motor_set_velocity(right_motor, right_motor_speed); 
     
     
     // Just state transition and flow here
      switch(current_state){
        case CALCULATE_TARGET_ANGLE    :{current_state = ROTATE_HEAD_TOWARD_GOAL;break;}
        case ROTATE_HEAD_TOWARD_GOAL   :{if(_is_faced_target)current_state = MOVE_TOWARD_TARGET;break;}
        case MOVE_TOWARD_TARGET        :{current_state = _meet_hit_point ? BOUNDARY_TANGENT_DIRECTION:(_is_reached_goal ? END : current_state); distance_from_target = calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ); break;}
        case BOUNDARY_TANGENT_DIRECTION:{if(_is_in_tan_direc){current_state = BOUNDARY_WALL_FOLLWING; counter = 200;}break;}
        case BOUNDARY_WALL_FOLLWING    :{if(_is_reached_goal){current_state = END;}else if(_reached_M_line && _got_near_target && _init_duration_done){current_state = ROTATE_HEAD_TOWARD_GOAL;distance_from_target = calculate_point_distance_to_target(currentX,currentZ,targetX,targetZ);} break;}
        case END                       :{goto terminating_process;}
      } 
  }

  terminating_process:
  printf("process finished successfully!\n");
  printf("total distance walked = %.3f\n", total_distance_walked);
  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

double calculate_point_distance_to_target(double currentX, double currentY, double targetX, double targetY){
  return sqrt((targetY-currentY)*(targetY-currentY)+(targetX-currentX)*(targetX-currentX));
}

double calculate_point2point_distance(double currentX, double currentY, double targetX, double targetY) {
  return sqrt((targetY-currentY)*(targetY-currentY)+(targetX-currentX)*(targetX-currentX));
}

double calculate_distance_from_M_line(double currentZ,double currentX, double a, double b, double c){
  double top_part = fabs(a*currentZ+b*currentX+c);
  double frac_part = sqrt(a*a+b*b);
  return top_part/frac_part;
}




