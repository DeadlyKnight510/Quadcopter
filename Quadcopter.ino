#include "Configuration.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Angles
float angleX,angleY,angleZ = 0.0;

// RX Signals
int throttle=THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
  
// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

// Track loop time.
unsigned long prev_time = 0;


void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif
  imu_initialize();     // great
  motors_initialize();  // great
  leds_initialize();    // great
  rx_initialize();      // not working for long enough
  pid_initialize();     // good
  motors_arm();         // great
  //wait for IMU YAW  to settle before beginning??? ~20s
}

void loop() 
{
  imu_update();
  control_update();

  #ifdef DEBUG_OUTPUT
    debug_process();
  #endif
  prev_time = micros();
}
